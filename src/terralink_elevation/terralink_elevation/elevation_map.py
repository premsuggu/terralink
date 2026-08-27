"""Elevation Map - CPU Implementation (Step 3: Core Data Structures).

This is the reference implementation that GPU kernels must match.
All operations use NumPy for clarity and testability.

Map Layout (7 layers, cell_n x cell_n):
Layer 0: elevation       - Height (m)
Layer 1: variance        - Height uncertainty (m^2)
Layer 2: is_valid        - 1.0 = measured, 0.0 = unknown
Layer 3: traversability  - 0-1 (1 = traversable)
Layer 4: time            - Seconds since last update
Layer 5: upper_bound     - Max height from ray tracing
Layer 6: is_upper_bound  - 1.0 = ray hit ceiling

Coordinate Convention:
- Internal: row=Y (vertical), col=X (horizontal)  [row, col] = [y, x]
- GridMap msg: column-major, Row→-X, Col→-Y
- World: X forward, Y left, Z up
"""
import numpy as np
from numpy.typing import NDArray
from typing import Tuple, Optional
from terralink_elevation.parameter import Parameter


class ElevationMapCPU:
    """CPU-based elevation map for development and testing."""
    
    # Layer indices (use these instead of magic numbers)
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
        self.param = param
        self.resolution = param.resolution
        self.cell_n = param.cell_n
        self.true_cell_n = param.true_cell_n
        self.center_x = 0.0
        self.center_y = 0.0
        
        # Main map: (7, cell_n, cell_n) - float32
        self.elevation_map = np.zeros((7, self.cell_n, self.cell_n), dtype=np.float32)
        
        # Initialize layers
        self.elevation_map[self.IDX_VARIANCE] += param.initial_variance  # variance
        self.elevation_map[self.IDX_TRAVERSABILITY] += 1.0               # traversability = 1.0 (traversable)
        
        # Accumulators for fusion (reset each frame)
        self.new_elevation = np.zeros((self.cell_n, self.cell_n), dtype=np.float32)
        self.new_variance = np.zeros((self.cell_n, self.cell_n), dtype=np.float32)
        self.new_count = np.zeros((self.cell_n, self.cell_n), dtype=np.float32)
        
        # Normal map: (3, cell_n, cell_n) for surface normals
        self.normal_map = np.zeros((3, self.cell_n, self.cell_n), dtype=np.float32)
    
    # ==================== Layer Access Helpers ====================
    
    def get_layer(self, layer_name: str) -> NDArray[np.float32]:
        """Get a layer by name (returns view, not copy)."""
        idx = self.layer_names.index(layer_name)
        return self.elevation_map[idx]
    
    def get_elevation(self) -> NDArray[np.float32]:
        """Get elevation layer (height in meters)."""
        return self.elevation_map[self.IDX_ELEVATION]
    
    def get_variance(self) -> NDArray[np.float32]:
        """Get variance layer (uncertainty in m^2)."""
        return self.elevation_map[self.IDX_VARIANCE]
    
    def get_validity(self) -> NDArray[np.float32]:
        """Get validity layer (1.0 = measured, 0.0 = unknown)."""
        return self.elevation_map[self.IDX_IS_VALID]
    
    def get_traversability(self) -> NDArray[np.float32]:
        """Get traversability layer (0-1, higher = more traversable)."""
        return self.elevation_map[self.IDX_TRAVERSABILITY]
    
    def set_layer(self, layer_name: str, data: NDArray[np.float32]):
        """Set a layer by name."""
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
        """Check if grid indices are inside map bounds."""
        return 0 <= row < self.cell_n and 0 <= col < self.cell_n
    
    def get_cell_center_world(self, row: int, col: int) -> Tuple[float, float]:
        """Get world coordinates of cell center."""
        return self.grid_to_world(row, col)
    
    def get_valid_region_slice(self) -> Tuple[slice, slice]:
        """Get slice for valid region (excludes 1-cell border)."""
        border = (self.cell_n - self.true_cell_n) // 2
        s = slice(border, border + self.true_cell_n)
        return s, s
    
    # ==================== Sensor Noise & Point Validation ====================
    
    def point_noise(self, x: float, y: float, z: float, sensor_pos: Optional[NDArray[np.float32]] = None) -> float:
        """Sensor noise variance: factor * distance_from_sensor^2.
        
        Args:
            x, y, z: Point in world coordinates
            sensor_pos: Sensor position in world coordinates (optional). If None, uses origin.
        """
        if sensor_pos is not None:
            dx = x - sensor_pos[0]
            dy = y - sensor_pos[1]
            dz = z - sensor_pos[2]
            dist_sq = dx*dx + dy*dy + dz*dz
        else:
            dist_sq = x*x + y*y + z*z
        return self.param.sensor_noise_factor * dist_sq
    
    def _validate_points(self, points: NDArray[np.float32]) -> NDArray[np.bool_]:
        """Validate points: distance, height limits."""
        dist = np.linalg.norm(points, axis=1)
        height = points[:, 2]
        
        mask = (dist >= self.param.min_valid_distance) & \
               (dist <= self.param.max_ray_length) & \
               (height >= self.param.min_height) & \
               (height <= self.param.max_height)
        return mask
    
    # ==================== Visibility Cleanup (Ray Tracing) ====================
    
    def _trace_ray_to_point(self, sensor_origin: NDArray[np.float32], point: NDArray[np.float32]):
        """Trace ray from sensor to point using 3D DDA algorithm.
        
        Marks cells along ray as free space (decreases validity, increases variance),
        but avoids clearing walls using normal check.
        
        Args:
            sensor_origin: (3,) sensor position in map frame (x, y, z)
            point: (3,) point in map frame (x, y, z)
        """
        # Ray direction
        ray = point - sensor_origin
        ray_length = np.linalg.norm(ray)
        if ray_length == 0:
            return
        
        ray_dir = ray / ray_length
        
        # DDA algorithm parameters
        step = self.resolution / np.sqrt(3)  # diagonal step to ensure coverage
        max_steps = int(ray_length / step)
        
        # Current position in world coordinates
        current = sensor_origin.copy()
        
        # Sensor origin grid position
        s_row, s_col = self.world_to_grid(sensor_origin[0], sensor_origin[1])
        
        for step_idx in range(max_steps):
            # Move along ray
            current += ray_dir * step
            
            # Get grid cell
            row, col = self.world_to_grid(current[0], current[1])
            if not self.is_inside(row, col):
                break
            
            # Skip if too close to endpoint (don't clear the hit point itself)
            remaining = ray_length - step_idx * step
            if remaining < self.resolution:
                break
            
            # Get cell height and normal
            cell_h = self.elevation_map[self.IDX_ELEVATION, row, col]
            cell_valid = self.elevation_map[self.IDX_IS_VALID, row, col]
            
            # Skip if cell was never valid
            if cell_valid < 0.5:
                continue
            
            # Compute normal at this cell
            normal = self._compute_normal(row, col)
            
            # Check if ray is grazing ground (not hitting wall)
            # Ray dot normal > cos(threshold) means ray hits wall perpendicularly
            # Ray dot normal < cos(threshold) means ray grazes ground
            if normal is not None:
                ray_dot_normal = abs(np.dot(ray_dir, normal))
                if ray_dot_normal > self.param.cleanup_cos_thresh:
                    # Ray hits wall perpendicularly - don't clear
                    continue
            
            # Check if ray passes ABOVE the cell height
            # Only clear cells where ray passes significantly above terrain
            if current[2] > cell_h + 0.01:
                # Mark as free space: decrease validity, increase variance
                self.elevation_map[self.IDX_IS_VALID, row, col] *= (1.0 - self.param.cleanup_step)
                self.elevation_map[self.IDX_VARIANCE, row, col] += self.param.outlier_variance * self.param.cleanup_step
                self.elevation_map[self.IDX_IS_VALID, row, col] = max(0.0, self.elevation_map[self.IDX_IS_VALID, row, col])
    
    def _compute_normal(self, row: int, col: int) -> Optional[NDArray[np.float32]]:
        """Compute surface normal at given cell using finite differences."""
        if not self.is_inside(row, col):
            return None
        
        # Use 3x3 neighborhood for gradient
        if row == 0 or row >= self.cell_n - 1 or col == 0 or col >= self.cell_n - 1:
            return None
        
        elev = self.elevation_map[self.IDX_ELEVATION]
        
        # Central differences
        dzdx = (elev[row, col+1] - elev[row, col-1]) / (2 * self.resolution)
        dzdy = (elev[row+1, col] - elev[row-1, col]) / (2 * self.resolution)
        
        # Normal = (-dzdx, -dzdy, 1) normalized
        normal = np.array([-dzdx, -dzdy, 1.0], dtype=np.float32)
        norm = np.linalg.norm(normal)
        if norm > 0:
            return normal / norm
        return None
    
    # ==================== Fusion ====================
    
    def _fuse_single_point(self, x: float, y: float, z: float, sensor_pos: Optional[NDArray[np.float32]] = None):
        """Fuse a single point into the map (Bayesian update)."""
        # Grid index
        row, col = self.world_to_grid(x, y)
        if not self.is_inside(row, col):
            return
        
        # Sensor noise variance
        v = self.point_noise(x, y, z, sensor_pos)
        
        # Prior from map
        map_h = self.elevation_map[self.IDX_ELEVATION, row, col]
        map_v = self.elevation_map[self.IDX_VARIANCE, row, col]
        
        # Mahalanobis outlier check
        if abs(map_h - z) > np.sqrt(map_v) * self.param.mahalanobis_thresh:
            # Outlier: directly increase map variance, preserve elevation
            self.elevation_map[self.IDX_VARIANCE, row, col] += self.param.outlier_variance
            return
        
        # Bayesian fusion
        new_h = (map_h * v + z * map_v) / (map_v + v)
        new_v = (map_v * v) / (map_v + v)
        
        # Accumulate (atomic in GPU, direct in CPU)
        self.new_elevation[row, col] += new_h
        self.new_variance[row, col] += new_v
        self.new_count[row, col] += 1.0
        
        # Mark valid, reset time
        self.elevation_map[self.IDX_IS_VALID, row, col] = 1.0
        self.elevation_map[self.IDX_TIME, row, col] = 0.0
        self.elevation_map[self.IDX_UPPER_BOUND, row, col] = z
        self.elevation_map[self.IDX_IS_UPPER_BOUND, row, col] = 1.0
    
    def _finalize_fusion(self):
        """Average accumulated values into map."""
        mask = self.new_count > 0
        
        # Average valid cells
        self.elevation_map[self.IDX_ELEVATION, mask] = self.new_elevation[mask] / self.new_count[mask]
        self.elevation_map[self.IDX_VARIANCE, mask] = self.new_variance[mask] / self.new_count[mask]
        self.elevation_map[self.IDX_IS_VALID, mask] = 1.0
        
        # Reset invalid cells (only those that were never valid)
        invalid = ~mask & (self.elevation_map[self.IDX_IS_VALID] == 0)
        self.elevation_map[self.IDX_ELEVATION, invalid] = 0.0
        self.elevation_map[self.IDX_VARIANCE, invalid] = self.param.initial_variance
        self.elevation_map[self.IDX_IS_VALID, invalid] = 0.0
    
    def fuse_pointcloud(self, points: NDArray[np.float32], R: NDArray[np.float32], t: NDArray[np.float32]):
        """Fuse point cloud into elevation map (CPU reference implementation).
        
        Args:
            points: (N, 3) array in sensor frame
            R: (3, 3) rotation matrix
            t: (3,) translation vector (sensor origin in map frame)
        """
        # Reset accumulators
        self.new_elevation.fill(0)
        self.new_variance.fill(0)
        self.new_count.fill(0)
        
        # Sensor origin in map frame (translation vector)
        sensor_origin = t.copy()
        
        # Transform points to map frame first for height validation
        points_map = (R @ points.T + t.reshape(3, 1)).T  # (N, 3)
        
        # Validate points: distance from sensor (sensor frame) + height in map frame
        sensor_dist = np.linalg.norm(points, axis=1)
        map_height = points_map[:, 2]
        
        valid_mask = (sensor_dist >= self.param.min_valid_distance) & \
                     (sensor_dist <= self.param.max_ray_length) & \
                     (map_height >= self.param.min_height) & \
                     (map_height <= self.param.max_height)
        
        points_valid = points_map[valid_mask]
        
        if len(points_valid) == 0:
            self._finalize_fusion()
            return
        
        # Compute drift compensation error before fusion
        drift_error_sum = 0.0
        drift_error_cnt = 0
        
        if self.param.enable_drift_compensation:
            for px, py, pz in points_valid:
                row, col = self.world_to_grid(px, py)
                if self.is_inside(row, col) and self.elevation_map[self.IDX_IS_VALID, row, col] > 0.5:
                    map_h = self.elevation_map[self.IDX_ELEVATION, row, col]
                    map_v = self.elevation_map[self.IDX_VARIANCE, row, col]
                    # Only consider cells with low variance for drift compensation
                    if map_v < self.param.drift_compensation_variance_inlier:
                        drift_error_sum += pz - map_h
                        drift_error_cnt += 1
        
        # Fuse each valid point AND trace ray for visibility cleanup
        for px, py, pz in points_valid:
            self._fuse_single_point(px, py, pz, sensor_origin)
            # Trace ray for visibility cleanup (if enabled)
            if self.param.enable_visibility_cleanup:
                point_map = np.array([px, py, pz], dtype=np.float32)
                self._trace_ray_to_point(sensor_origin, point_map)
        
        # Finalize: average accumulators into map
        self._finalize_fusion()
        
        # Apply drift compensation after fusion
        if self.param.enable_drift_compensation and drift_error_cnt > self.param.min_height_drift_cnt:
            mean_error = drift_error_sum / drift_error_cnt
            if abs(mean_error) < self.param.max_drift:
                # Apply compensation to elevation layer
                self.elevation_map[self.IDX_ELEVATION] += mean_error * self.param.drift_compensation_alpha
    
    def _validate_points(self, points: NDArray[np.float32]) -> NDArray[np.bool_]:
        """Validate points: distance, height limits."""
        dist = np.linalg.norm(points, axis=1)
        height = points[:, 2]
        
        mask = (dist >= self.param.min_valid_distance) & \
               (dist <= self.param.max_ray_length) & \
               (height >= self.param.min_height) & \
               (height <= self.param.max_height)
        return mask
    
    # ==================== Traversability ====================
    
    def update_traversability(self):
        """Compute analytical traversability from elevation + variance."""
        elev = self.elevation_map[self.IDX_ELEVATION]
        var = self.elevation_map[self.IDX_VARIANCE]
        valid = self.elevation_map[self.IDX_IS_VALID] > 0.5
        
        # Initialize all as traversable
        trav = np.ones_like(elev, dtype=np.float32)
        
        # Unknown cells are never traversable
        trav[~valid] = 0.0
        
        if not np.any(valid):
            self.elevation_map[self.IDX_TRAVERSABILITY] = trav
            return
        
        # Slope (gradient magnitude)
        grad_y, grad_x = np.gradient(elev)  # Note: np.gradient returns (dy, dx)
        grad_x = grad_x / self.resolution
        grad_y = grad_y / self.resolution
        slope = np.sqrt(grad_x**2 + grad_y**2)
        
        # Step height (3x3 max - min)
        from scipy.ndimage import maximum_filter, minimum_filter
        step_height = maximum_filter(elev, size=3) - minimum_filter(elev, size=3)
        
        # Roughness = variance
        roughness = var
        
        # Classification
        lethal = (slope > self.param.max_slope) | \
                 (step_height > self.param.max_step) | \
                 (roughness > self.param.max_roughness)
        
        difficult = (slope > self.param.max_slope * 0.5) | \
                    (step_height > self.param.max_step * 0.5)
        
        trav[lethal] = 0.0
        trav[difficult] = 0.3
        # Unknown cells already set to 0.0 above
        
        self.elevation_map[self.IDX_TRAVERSABILITY] = trav
    
    # ==================== Map Shifting (Robot-Centric) ====================
    
    def shift_map_xy(self, delta_pixel: Tuple[int, int]):
        """Shift map by integer pixels (for robot-centric mapping).
        
        Args:
            delta_pixel: (dx, dy) in WORLD coordinates (X forward, Y left).
                         Positive dx shifts map in +X direction (right).
                         This is the RAW shift - move_to() should pass -delta_pixel.
        """
        dx, dy = delta_pixel
        # Map array: (layers, rows=Y, cols=X)
        # np.roll axis=(1,2) expects (row_shift, col_shift) = (dy, dx)
        # CRITICAL: SWAP [dx, dy] -> [dy, dx]!
        shift_rows = dy
        shift_cols = dx
        
        if shift_rows == 0 and shift_cols == 0:
            return
        
        self.elevation_map = np.roll(self.elevation_map, 
                                     shift=(shift_rows, shift_cols), 
                                     axis=(1, 2))
        
        # Pad new edges with zeros (elevation) and initial_variance
        # shift_rows > 0 means rows shifted down (positive Y), pad top
        if shift_rows > 0:
            self.elevation_map[:, :shift_rows, :] = 0
            self.elevation_map[self.IDX_VARIANCE, :shift_rows, :] = self.param.initial_variance
        elif shift_rows < 0:
            self.elevation_map[:, shift_rows:, :] = 0
            self.elevation_map[self.IDX_VARIANCE, shift_rows:, :] = self.param.initial_variance
        
        # shift_cols > 0 means cols shifted right (positive X), pad left
        if shift_cols > 0:
            self.elevation_map[:, :, :shift_cols] = 0
            self.elevation_map[self.IDX_VARIANCE, :, :shift_cols] = self.param.initial_variance
        elif shift_cols < 0:
            self.elevation_map[:, :, shift_cols:] = 0
            self.elevation_map[self.IDX_VARIANCE, :, shift_cols:] = self.param.initial_variance
    
    def move_to(self, position: NDArray, R: NDArray):
        """Move map center to new robot position.
        
        Args:
            position: (3,) new position in world frame
            R: (3, 3) rotation matrix
        """
        new_center = np.array([position[0], position[1], position[2]], dtype=np.float32)
        delta = new_center - np.array([self.center_x, self.center_y, 0.0])
        
        # Convert to pixel shift
        delta_pixel = np.round(delta[:2] / self.resolution).astype(int)
        
        # Update center
        self.center_x += delta_pixel[0] * self.resolution
        self.center_y += delta_pixel[1] * self.resolution
        
        # Shift map (opposite to robot movement)
        self.shift_map_xy((-delta_pixel[0], -delta_pixel[1]))
    
    # ==================== GridMap Conversion Helpers ====================
    
    def internal_to_gridmap(self, arr: NDArray) -> NDArray:
        """Convert internal (row=Y, col=X) to GridMap column-major convention.
        
        GridMap: column-major, Row→-X, Col→-Y
        Internal: row=Y, col=X
        """
        # Transpose: (H, W) -> (W, H) [now rows=X, cols=Y]
        arr = arr.T
        # Flip axis 0: row 0 becomes last row (X direction)
        arr = np.flip(arr, axis=0)
        # Flip axis 1: col 0 becomes last col (Y direction)
        arr = np.flip(arr, axis=1)
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
    
    def to_gridmap_msg(self):
        """Convert to GridMap message (placeholder for Step 10)."""
        raise NotImplementedError("Implemented in Step 10")


# Alias for easy import
ElevationMap = ElevationMapCPU