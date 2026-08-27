"""Elevation Map - GPU Implementation (Step 5: CuPy Acceleration).

GPU-accelerated elevation map using CuPy ElementwiseKernels.
Falls back to CPU if CuPy not available.
"""
import numpy as np
from numpy.typing import NDArray
from typing import Tuple, Optional

try:
    import cupy as cp
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False
    cp = None

from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.kernels.fusion_kernel import (
    fusion_kernel, finalize_kernel, compile_kernels,
    IDX_ELEVATION, IDX_VARIANCE, IDX_IS_VALID,
    IDX_TRAVERSABILITY, IDX_TIME, IDX_UPPER_BOUND, IDX_IS_UPPER_BOUND
)


class ElevationMapGPU:
    """GPU-accelerated elevation map using CuPy.
    
    Provides the same interface as ElevationMapCPU but runs fusion on GPU.
    """
    
    # Layer indices (must match CPU)
    IDX_ELEVATION = 0
    IDX_VARIANCE = 1
    IDX_IS_VALID = 2
    IDX_TRAVERSABILITY = 3
    IDX_TIME = 4
    IDX_UPPER_BOUND = 5
    IDX_IS_UPPER_BOUND = 6
    
    layer_names = [
        "elevation", "variance", "is_valid", 
        "traversability", "time", "upper_bound", "is_upper_bound"
    ]
    
    def __init__(self, param: Parameter):
        if not CUPY_AVAILABLE:
            raise RuntimeError("CuPy not available. Install cupy-cuda12x or use ElevationMapCPU.")
        
        self.param = param
        self.resolution = param.resolution
        self.cell_n = param.cell_n
        self.true_cell_n = param.true_cell_n
        self.center_x = 0.0
        self.center_y = 0.0
        
        # Main map: (7, cell_n, cell_n) - float32 on GPU
        self.elevation_map = cp.zeros((7, self.cell_n, self.cell_n), dtype=cp.float32)
        
        # Initialize layers
        self.elevation_map[IDX_VARIANCE] += param.initial_variance
        self.elevation_map[IDX_TRAVERSABILITY] += 1.0
        
        # Accumulators for fusion (reset each frame)
        map_size = self.cell_n * self.cell_n
        self.new_elevation = cp.zeros(map_size, dtype=cp.float32)
        self.new_variance = cp.zeros(map_size, dtype=cp.float32)
        self.new_count = cp.zeros(map_size, dtype=cp.float32)
        
        # Normal map: (3, cell_n, cell_n) for surface normals
        self.normal_map = cp.zeros((3, self.cell_n, self.cell_n), dtype=cp.float32)
        
        # Compile kernels
        compile_kernels(param)
    
    # ==================== Layer Access Helpers ====================
    
    def get_layer(self, layer_name: str) -> cp.ndarray:
        """Get a layer by name (returns view, not copy)."""
        idx = self.layer_names.index(layer_name)
        return self.elevation_map[idx]
    
    def get_elevation(self) -> cp.ndarray:
        return self.elevation_map[IDX_ELEVATION]
    
    def get_variance(self) -> cp.ndarray:
        return self.elevation_map[IDX_VARIANCE]
    
    def get_validity(self) -> cp.ndarray:
        return self.elevation_map[IDX_IS_VALID]
    
    def get_traversability(self) -> cp.ndarray:
        return self.elevation_map[IDX_TRAVERSABILITY]
    
    def set_layer(self, layer_name: str, data: cp.ndarray):
        idx = self.layer_names.index(layer_name)
        self.elevation_map[idx] = data
    
    # ==================== Coordinate Transforms ====================
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices.
        
        Args:
            x, y: World coordinates (meters). X forward, Y left.
        Returns:
            (row, col) grid indices. row=Y, col=X.
        """
        col = int(round((x - self.center_x) / self.resolution + self.cell_n / 2))
        row = int(round((y - self.center_y) / self.resolution + self.cell_n / 2))
        return row, col
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        x = (col - self.cell_n / 2) * self.resolution + self.center_x
        y = (row - self.cell_n / 2) * self.resolution + self.center_y
        return x, y
    
    def is_inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.cell_n and 0 <= col < self.cell_n
    
    def get_cell_center_world(self, row: int, col: int) -> Tuple[float, float]:
        return self.grid_to_world(row, col)
    
    def get_valid_region_slice(self) -> Tuple[slice, slice]:
        border = (self.cell_n - self.true_cell_n) // 2
        s = slice(border, border + self.true_cell_n)
        return s, s
    
    # ==================== Fusion (GPU) ====================
    
    def point_noise(self, x: float, y: float, z: float) -> float:
        return self.param.sensor_noise_factor * (x*x + y*y + z*z)
    
    def _validate_points_gpu(self, points: cp.ndarray) -> cp.ndarray:
        """Validate points on GPU: distance, height limits."""
        dist = cp.linalg.norm(points, axis=1)
        height = points[:, 2]
        
        mask = (dist >= self.param.min_valid_distance) & \
               (dist <= self.param.max_ray_length) & \
               (height >= self.param.min_height) & \
               (height <= self.param.max_height)
        return mask
    
    def fuse_pointcloud(self, points: cp.ndarray, R: cp.ndarray, t: cp.ndarray):
        """Fuse point cloud into elevation map (GPU implementation).
        
        Args:
            points: (N, 3) array in sensor frame (CuPy array)
            R: (3, 3) rotation matrix (CuPy array)
            t: (3,) translation vector (CuPy array)
        """
        # Reset accumulators
        self.new_elevation.fill(0)
        self.new_variance.fill(0)
        self.new_count.fill(0)
        
        # Transform points to map frame: p_map = R @ p_sensor + t
        points_map = (R @ points.T + t.reshape(3, 1)).T  # (N, 3)
        
        # Filter valid points
        valid_mask = self._validate_points_gpu(points_map)
        points_valid = points_map[valid_mask]
        
        if len(points_valid) == 0:
            self._finalize_fusion()
            return
        
        # Flatten R and t for kernel
        R_flat = R.ravel()
        t_flat = t.ravel()
        
        # Launch fusion kernel - one thread per point
        num_points = len(points_valid)
        block_size = 256
        grid_size = (num_points + block_size - 1) // block_size
        
        fusion_kernel(
            points_valid[:, 0], points_valid[:, 1], points_valid[:, 2],
            R_flat, t_flat,
            self.cell_n, self.resolution,
            self.center_x, self.center_y,
            self.param.sensor_noise_factor, self.param.mahalanobis_thresh,
            self.param.outlier_variance, self.param.initial_variance,
            self.param.min_valid_distance, self.param.max_ray_length,
            self.param.min_height, self.param.max_height,
            self.elevation_map[IDX_ELEVATION].ravel(),
            self.elevation_map[IDX_VARIANCE].ravel(),
            self.elevation_map[IDX_IS_VALID].ravel(),
            self.elevation_map[IDX_TIME].ravel(),
            self.elevation_map[IDX_UPPER_BOUND].ravel(),
            self.elevation_map[IDX_IS_UPPER_BOUND].ravel(),
            self.new_elevation, self.new_variance, self.new_count,
            size=num_points
        )
        
        # Finalize: average accumulators into map
        self._finalize_fusion()
    
    def _finalize_fusion(self):
        """Average accumulated values into map (GPU)."""
        num_cells = self.cell_n * self.cell_n
        block_size = 256
        grid_size = (num_cells + block_size - 1) // block_size
        
        finalize_kernel(
            self.new_elevation, self.new_variance, self.new_count,
            self.elevation_map[IDX_ELEVATION].ravel(),
            self.elevation_map[IDX_VARIANCE].ravel(),
            self.elevation_map[IDX_IS_VALID].ravel(),
            size=num_cells
        )
    
    # ==================== Traversability ====================
    
    def update_traversability(self):
        """Compute analytical traversability from elevation + variance (GPU).
        
        Note: This uses CuPy's gradient and scipy.ndimage filters.
        For now, falls back to CPU implementation.
        """
        # Transfer to CPU for traversability computation (uses scipy)
        elev_cpu = self.get_elevation().get()
        var_cpu = self.get_variance().get()
        valid_cpu = self.get_validity().get()
        
        # Compute on CPU (same as ElevationMapCPU)
        from scipy.ndimage import maximum_filter, minimum_filter
        elev = elev_cpu
        var = var_cpu
        valid = valid_cpu > 0.5
        
        trav = np.ones_like(elev, dtype=np.float32)
        
        if not np.any(valid):
            self.elevation_map[IDX_TRAVERSABILITY] = cp.asarray(trav)
            return
        
        grad_y, grad_x = np.gradient(elev)
        grad_x = grad_x / self.resolution
        grad_y = grad_y / self.resolution
        slope = np.sqrt(grad_x**2 + grad_y**2)
        
        step_height = maximum_filter(elev, size=3) - minimum_filter(elev, size=3)
        roughness = var
        
        lethal = (slope > self.param.max_slope) | \
                 (step_height > self.param.max_step) | \
                 (roughness > self.param.max_roughness)
        
        difficult = (slope > self.param.max_slope * 0.5) | \
                    (step_height > self.param.max_step * 0.5)
        
        trav[lethal] = 0.0
        trav[difficult] = 0.3
        trav[~valid] = 0.0
        
        self.elevation_map[IDX_TRAVERSABILITY] = cp.asarray(trav)
    
    # ==================== Map Shifting (GPU) ====================
    
    def shift_map_xy(self, delta_pixel: Tuple[int, int]):
        """Shift map by integer pixels (GPU with cp.roll).
        
        Args:
            delta_pixel: (dx, dy) in WORLD coordinates (X forward, Y left)
        """
        dx, dy = delta_pixel
        # cp.roll axis=(1,2) expects (row_shift, col_shift) = (dy, dx)
        # CRITICAL: SWAP [dx, dy] -> [dy, dx]!
        shift_rows = -dy  # Negative: map moves opposite to robot
        shift_cols = -dx
        
        self.elevation_map = cp.roll(self.elevation_map, 
                                     shift=(shift_rows, shift_cols), 
                                     axis=(1, 2))
        
        # Pad new edges
        if shift_rows > 0:
            self.elevation_map[:, :shift_rows, :] = 0
            self.elevation_map[IDX_VARIANCE, :shift_rows, :] = self.param.initial_variance
        elif shift_rows < 0:
            self.elevation_map[:, shift_rows:, :] = 0
            self.elevation_map[IDX_VARIANCE, shift_rows:, :] = self.param.initial_variance
        
        if shift_cols > 0:
            self.elevation_map[:, :, :shift_cols] = 0
            self.elevation_map[IDX_VARIANCE, :, :shift_cols] = self.param.initial_variance
        elif shift_cols < 0:
            self.elevation_map[:, :, shift_cols:] = 0
            self.elevation_map[IDX_VARIANCE, :, shift_cols:] = self.param.initial_variance
    
    def move_to(self, position: cp.ndarray, R: cp.ndarray):
        """Move map center to new robot position (GPU)."""
        new_center = cp.array([position[0], position[1], position[2]], dtype=cp.float32)
        delta = new_center - cp.array([self.center_x, self.center_y, 0.0], dtype=cp.float32)
        
        # Convert to pixel shift
        delta_pixel = cp.round(delta[:2] / self.resolution).astype(cp.int32)
        
        # Update center
        self.center_x += float(delta_pixel[0] * self.resolution)
        self.center_y += float(delta_pixel[1] * self.resolution)
        
        # Shift map (opposite to robot movement)
        self.shift_map_xy((-int(delta_pixel[0]), -int(delta_pixel[1])))
    
    # ==================== GridMap Conversion Helpers ====================
    
    def internal_to_gridmap(self, arr: cp.ndarray) -> cp.ndarray:
        """Convert internal (row=Y, col=X) to GridMap column-major convention."""
        arr = arr.T
        arr = cp.flip(arr, axis=0)
        arr = cp.flip(arr, axis=1)
        return arr
    
    def to_gridmap_layers(self) -> dict:
        """Extract all layers in GridMap convention (for publishing)."""
        s_row, s_col = self.get_valid_region_slice()
        layers = {}
        for name in self.layer_names:
            idx = self.layer_names.index(name)
            layer_data = self.elevation_map[idx, s_row, s_col]
            layers[name] = self.internal_to_gridmap(layer_data)
        return layers
    
    def to_cpu(self) -> ElevationMapCPU:
        """Transfer GPU map to CPU ElevationMapCPU."""
        cpu_map = ElevationMapCPU(self.param)
        cpu_map.elevation_map = self.elevation_map.get()
        cpu_map.new_elevation = self.new_elevation.get()
        cpu_map.new_variance = self.new_variance.get()
        cpu_map.new_count = self.new_count.get()
        cpu_map.center_x = self.center_x
        cpu_map.center_y = self.center_y
        cpu_map.normal_map = self.normal_map.get()
        return cpu_map


# Factory function to get appropriate implementation
def create_elevation_map(param: Parameter, use_gpu: bool = True):
    """Create elevation map instance (GPU if available and requested)."""
    if use_gpu and CUPY_AVAILABLE:
        try:
            return ElevationMapGPU(param)
        except Exception as e:
            print(f"GPU initialization failed: {e}, falling back to CPU")
            return ElevationMapCPU(param)
    else:
        return ElevationMapCPU(param)