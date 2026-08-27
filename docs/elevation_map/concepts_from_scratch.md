# Elevation Mapping: Concepts from Scratch

**Goal**: After reading this, you should understand every algorithm, data structure, and design decision in `elevation_mapping_cupy` well enough to reimplement it yourself.

---

## Table of Contents

1. [What is 2.5D Elevation Mapping?](#1-what-is-25d-elevation-mapping)
2. [Why GPU? Why CuPy?](#2-why-gpu-why-cupy)
3. [Core Data Structures](#3-core-data-structures)
4. [Bayesian Height Fusion (The Heart of It)](#4-bayesian-height-fusion-the-heart-of-it)
5. [Sensor Noise Modeling](#5-sensor-noise-modeling)
6. [Outlier Rejection: Mahalanobis Distance](#6-outlier-rejection-mahalanobis-distance)
7. [Drift Compensation](#7-drift-compensation)
8. [Visibility Cleanup (Ray Tracing)](#8-visibility-cleanup-ray-tracing)
9. [Traversability Estimation](#9-traversability-estimation)
10. [Map Shifting & Robot-Centric Mapping](#10-map-shifting--robot-centric-mapping)
11. [Coordinate Systems & Conventions](#11-coordinate-systems--conventions)
12. [Kernel Architecture](#12-kernel-architecture)
13. [Learning Resources](#13-learning-resources)

---

## 1. What is 2.5D Elevation Mapping?

### 1.1 The Core Idea

A **2.5D elevation map** represents terrain height as a function of (x, y):

```
z = f(x, y)
```

Unlike full 3D (voxel grids, point clouds), we assume **exactly one height per (x, y) cell**. No overhangs, no caves, no vertical walls with multiple heights at same XY.

**Why 2.5D for ground robots?**
- Robots move on continuous surfaces
- Single height per cell = traversability analysis is simple
- Memory: O(N²) vs O(N³) for 3D
- Perfect for "can I drive here?" questions

### 1.2 From Point Cloud to Grid

**Input**: Stream of 3D points (x, y, z) from RGB-D camera or LiDAR, in sensor frame.

**Process per point**:
```
1. Transform point from sensor frame → map frame (using robot pose)
2. Compute grid cell indices: cell_x = floor((x - origin_x) / resolution)
3. Fuse point's z into that cell's height estimate
4. Update cell's variance (uncertainty)
```

### 1.3 Multi-Layer Grid Map

Each cell stores **7 layers** (not just height):

| Layer | Index | Meaning | Range |
|-------|-------|---------|-------|
| elevation | 0 | Height (m) | ℝ |
| variance | 1 | Uncertainty (m²) | [0, ∞) |
| is_valid | 2 | Has measurement? | {0, 1} |
| traversability | 2 | Drivability score | [0, 1] |
| time | 4 | Seconds since update | [0, ∞) |
| upper_bound | 5 | Max height from rays | ℝ |
| is_upper_bound | 6 | Ray hit ceiling? | {0, 1} |

---

## 2. Why GPU? Why CuPy?

### 2.1 The Compute Problem

At 10 Hz with 100k points/frame:
- 1M points/second
- Each point → transform + cell lookup + atomic updates
- CPU: ~10-50 ms/frame (single-threaded)
- GPU: ~1-2 ms/frame (10,000 parallel threads)

### 2.2 CuPy = NumPy on CUDA

```python
import cupy as cp

# Same API as NumPy, runs on GPU
a = cp.zeros((1000, 1000))  # GPU memory
b = cp.random.randn(1000, 1000)
c = a + b  # Parallel element-wise

# Custom CUDA kernels via ElementwiseKernel
kernel = cp.ElementwiseKernel(
    in_params='raw T x, raw T y',
    out_params='raw T z',
    operation='z = x + y'
)
```

### 2.3 Memory Management

```python
# Use managed memory (unified CPU/GPU)
pool = cp.cuda.MemoryPool(cp.cuda.malloc_managed)
cp.cuda.set_allocator(pool.malloc)

# Zero-copy with ROS 2 via ros2_numpy
points_gpu = ros2_numpy.pointcloud2_to_cupy(msg)  # No copy!
```

---

## 3. Core Data Structures

### 3.1 Parameter Class (`parameter.py`)

```python
@dataclass
class Parameter:
    # Map geometry
    resolution: float = 0.04      # 4 cm/cell
    map_length: float = 8.0       # 8m x 8m
    cell_n: int = None            # Computed: round(map_length/resolution) + 2
    
    # Sensor model
    sensor_noise_factor: float = 0.05   # noise = factor * (x²+y²+z²)
    mahalanobis_thresh: float = 2.0     # Outlier threshold (σ)
    outlier_variance: float = 0.01      # Variance added for outliers
    
    # Drift compensation
    enable_drift_compensation: bool = True
    max_drift: float = 0.10
    drift_compensation_alpha: float = 1.0
    
    # Traversability
    dilation_size: int = 2
    traversability_inlier: float = 0.1
    
    # Backend
    use_chainer: bool = True       # True = Chainer, False = PyTorch
    weight_file: str = "weights.dat"
```

**Key method**:
```python
def update(self):
    """Call after changing resolution/map_length!"""
    self.cell_n = int(round(self.map_length / self.resolution)) + 2  # +2 border
    self.true_cell_n = round(self.map_length / self.resolution)
    self.true_map_length = self.true_cell_n * self.resolution
```

### 3.2 ElevationMap Class (`elevation_mapping.py`)

```python
class ElevationMap:
    def __init__(self, param: Parameter):
        self.param = param
        self.resolution = param.resolution
        self.cell_n = param.cell_n
        
        # Main map: [7 layers, cell_n, cell_n] on GPU
        self.elevation_map = cp.zeros((7, self.cell_n, self.cell_n), dtype=cp.float32)
        
        # Layer 0: elevation
        # Layer 1: variance (initialized to initial_variance)
        # Layer 2: is_valid (0 or 1)
        # Layer 3: traversability
        # Layer 4: time
        # Layer 5: upper_bound
        # Layer 6: is_upper_bound
        
        self.elevation_map[1] += param.initial_variance
        self.elevation_map[3] += 1.0  # traversability starts at 1.0 (traversable)
        
        # Normal map: [3, cell_n, cell_n] for surface normals
        self.normal_map = cp.zeros((3, self.cell_n, self.cell_n), dtype=cp.float32)
        
        # Compile all CUDA kernels
        self.compile_kernels()
```

---

## 4. Bayesian Height Fusion (The Heart of It)

### 4.1 The Problem

We have:
- **Prior**: Current map estimate `h_map ± σ_map` (from previous frames)
- **Measurement**: New point `z ± σ_sensor` (from current frame)

We want the **posterior**: Best estimate combining both.

### 4.2 Bayes for Gaussians

If prior ~ N(μ₁, σ₁²) and measurement ~ N(μ₂, σ₂²):

```
Posterior mean:    μ = (μ₁/σ₁² + μ₂/σ₂²) / (1/σ₁² + 1/σ₂²)
                   = (μ₁·σ₂² + μ₂·σ₁²) / (σ₁² + σ₂²)

Posterior variance: σ² = 1 / (1/σ₁² + 1/σ₂²)
                    = (σ₁²·σ₂²) / (σ₁² + σ₂²)
```

**Intuition**: Weight by inverse variance (precision). More certain → more weight.

### 4.3 In the Kernel (`custom_kernels.py:187-204`)

```cuda
// Prior from map
float map_h = map[get_map_idx(idx, 0)];  // elevation
float map_v = map[get_map_idx(idx, 1)];  // variance

// Measurement
float z = point_z;
float v = point_noise(x, y, z);  // σ_sensor²

// Bayesian fusion
float new_h = (map_h * v + z * map_v) / (map_v + v);
float new_v = (map_v * v) / (map_v + v);

// Atomic accumulation (multiple points → same cell)
atomicAdd(&newmap[get_map_idx(idx, 0)], new_h);
atomicAdd(&newmap[get_map_idx(idx, 1)], new_v);
atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0f);  // count

// After all points: average_map_kernel divides by count
```

### 4.4 Why This Works

- **Recursive**: Each frame's posterior becomes next frame's prior
- **Consistent**: Properly handles varying sensor noise (near vs far points)
- **Uncertainty-aware**: High variance cells update slowly; low variance cells are stable

---

## 5. Sensor Noise Modeling

### 5.1 The Noise Model (`custom_kernels.py:66-71`)

```cuda
__device__ float point_noise(float16 x, float16 y, float16 z) {
    // Noise grows quadratically with distance from sensor
    return ${sensor_noise_factor} * (x * x + y * y + z * z);
}
```

**Why squared distance?**
- Depth cameras: noise ∝ range² (structured light / ToF physics)
- Near points: low noise (high precision)
- Far points: high noise (low precision)
- Points at sensor origin (0,0,0): would have 0 noise → avoid with `min_valid_distance`

### 5.2 In the Parameter File

```yaml
sensor_noise_factor: 0.05  # Tune per sensor!
min_valid_distance: 0.3    # Ignore points closer than 30cm
```

**Tuning**: Compare sensor specs. RealSense D435: ~0.5% of range at 1m → noise ≈ 0.005 * range².

---

## 6. Outlier Rejection: Mahalanobis Distance

### 6.1 The Idea

If a new measurement is **too far** from current estimate given uncertainty, it's likely an outlier (dynamic object, sensor glitch).

### 6.2 Mahalanobis Distance

For 1D (height only):

```
D_M = |z - h_map| / σ_map
```

If `D_M > threshold` → outlier.

In the kernel (`custom_kernels.py:184-186`):

```cuda
if (abs(map_h - z) > (map_v * ${mahalanobis_thresh})) {
    // Outlier: increase variance, don't fuse height
    atomicAdd(&map[get_map_idx(idx, 1)], ${outlier_variance});
}
```

### 6.3 Why Not Just Fuse?

If we fused a dynamic object (person walking):
- Height estimate corrupted
- Variance decreases (false confidence)
- Map "learns" the person as static obstacle

**Mahalanobis rejects** → variance increases → cell becomes "less certain" → future measurements can correct it.

### 6.4 Edge Sharpening (`custom_kernels.py:188-190`)

```cuda
if (${enable_edge_shaped} && (num_points > ${wall_num_thresh}) 
    && (z < map_h - map_v * ${mahalanobis_thresh} / num_points)) {
    // Don't fuse points BELOW a sharp wall
    // Prevents "bleeding" wall height into ground
    continue;
}
```

**Problem**: At wall edges, ground points mix with wall points → wall height "smears" onto ground.

**Solution**: If many points already in cell (wall), reject new points significantly **below** current height.

---

## 7. Drift Compensation

### 7.1 The Problem

Robot odometry drifts → map frame shifts relative to world → same physical location gets different map cells over time → "ghost" obstacles, height errors.

### 7.2 The Insight

On **flat, traversable terrain**, height should be constant. Any systematic height error = drift.

### 7.3 Algorithm (`elevation_mapping.py:368-379`)

```python
# 1. Find inlier points on flat ground
error_counting_kernel:
    For each point:
        if valid AND |z - h_map| < mahalanobis_thresh * σ_map 
           AND σ_map < outlier_variance/2 
           AND traversability > traversability_inlier:
            error += (z - h_map)
            error_cnt += 1

# 2. Compute mean error
mean_error = error / error_cnt

# 3. Apply correction if significant pose change
if |mean_error| < max_drift AND (pos_change > pos_thresh OR rot_change > rot_thresh):
    elevation_map[0] += mean_error * drift_compensation_alpha
    additive_mean_error += mean_error
```

### 7.4 Parameters

```yaml
enable_drift_compensation: true
max_drift: 0.10              # Max 10cm correction per update
drift_compensation_alpha: 1.0  # How aggressive (0-1)
position_noise_thresh: 0.1   # Trigger if robot moved >10cm
orientation_noise_thresh: 0.1 # Trigger if robot rotated >0.1 rad
```

---

## 8. Visibility Cleanup (Ray Tracing)

### 8.1 The Problem

Sensor sees point at (x,y,z). **All cells along the ray from sensor to that point are FREE SPACE**. But we only mark the endpoint as occupied. The ray cells remain "unknown" or keep old values.

### 8.2 Ray Tracing in Kernel (`custom_kernels.py:209-269`)

```cuda
// For each point, trace ray from sensor to point
float16 ray_x, ray_y, ray_z;
float16 ray_length = ray_vector(t_sensor, point, ray_x, ray_y, ray_z);
ray_length = min(ray_length, ${max_ray_length});

int last_nidx = -1;
for (float16 s = ${ray_step}; s < ray_length; s += ${ray_step}) {
    // Step along ray
    nx = t_sensor.x + ray_x * s;
    ny = t_sensor.y + ray_y * s;
    nz = t_sensor.z + ray_z * s;
    nidx = get_idx(nx, ny);
    
    if (last_nidx == nidx) continue;  // Same cell
    last_nidx = nidx;
    
    // Skip if near endpoint
    if (distance < 0.1) continue;
    
    // If cell was valid but not recently updated:
    if (nmap_valid > 0.5 && non_updated_t > 0.5) {
        // Check if ray penetrates surface
        if (nmap_h > nz + 0.01 - min(nmap_v, 1.0) * 0.05) {
            // Check normal alignment (don't clear vertical walls)
            float product = ray · normal;
            if (fabs(product) < ${cleanup_cos_thresh}) continue;
            
            // Mark as free: decrease validity, increase variance
            atomicAdd(&map[get_map_idx(nidx, 2)], -${cleanup_step});
            atomicAdd(&map[get_map_idx(nidx, 1)], ${outlier_variance});
        }
    }
}
```

### 8.3 Key Parameters

```yaml
enable_visibility_cleanup: true
max_ray_length: 3.0        # Max ray trace distance (m)
ray_step: resolution/√2    # Step size (diagonal of cell)
cleanup_step: 0.05         # How much to reduce validity per ray
cleanup_cos_thresh: 0.2    # Cosine threshold (ray vs normal)
```

**Why cosine threshold?** Ray hitting ground at grazing angle (≈90° from normal) = valid free space. Ray hitting vertical wall (≈0° from normal) = don't clear!

---

## 9. Traversability Estimation

### 9.1 Two-Stage Pipeline

**Stage 1: Dilation** (`dilation_filter_kernel`)
- Input: `is_valid + is_upper_bound` (binary mask)
- Dilate by `dilation_size` cells
- Output: Smooth binary traversable region

**Stage 2: CNN** (`traversability_filter.py`)
- Input: Dilated upper_bound map (200x200 → resized to network input)
- 3 Conv layers + output
- Output: Traversability per cell [0, 1]

### 9.2 CNN Architecture (`traversability_filter.py`)

```python
# Chainer version (default)
class TraversabilityFilter(chainer.Chain):
    def __init__(self, w1, w2, w3, w_out):
        super().__init__()
        with self.init_scope():
            self.conv1 = L.Convolution2D(None, 4, 3, 1, 1, initialW=w1)
            self.conv2 = L.Convolution2D(None, 4, 3, 1, 1, initialW=w2)
            self.conv3 = L.Convolution2D(None, 4, 3, 1, 1, initialW=w3)
            self.conv_out = L.Convolution2D(None, 1, 1, 1, 0, initialW=w_out)
    
    def forward(self, x):
        h = F.relu(self.conv1(x))
        h = F.relu(self.conv2(h))
        h = F.relu(self.conv3(h))
        return F.sigmoid(self.conv_out(h))
```

**Weights**: Loaded from `config/weights.dat` (pickle file with conv1-3 weights).

### 9.3 Why CNN?

- Learns "looks like ground" from data
- Handles texture, not just geometry
- Generalizes to unseen terrain types

### 9.4 Integration (`elevation_mapping.py:397-409`)

```python
# 1. Dilation
self.dilation_filter_kernel(
    self.elevation_map[5],  # upper_bound
    self.elevation_map[2] + self.elevation_map[6],  # valid + is_upper
    self.traversability_input,
    self.traversability_mask_dummy,
)

# 2. CNN
traversability = self.traversability_filter(self.traversability_input)

# 3. Write back (with 3-cell border)
self.elevation_map[3][3:-3, 3:-3] = traversability.reshape(...)
```

---

## 10. Map Shifting & Robot-Centric Mapping

### 10.1 The Problem

Robot moves → map must move with it. But map is fixed grid in world coordinates.

### 10.2 Shift on GPU (`elevation_mapping.py:236-258`)

```python
def shift_map_xy(self, delta_pixel):
    """
    delta_pixel: [dx, dy] in WORLD coords (X forward, Y left)
    Map array: (layers, height=rows=Y, width=cols=X)
    cp.roll axis=(1,2) expects [row_shift, col_shift] = [dy, dx]
    SWAP: [dx, dy] → [dy, dx]
    """
    shift_value = cp.array([delta_pixel[1], delta_pixel[0]], dtype=cp.int32)
    
    with self.map_lock:
        self.elevation_map = cp.roll(self.elevation_map, shift_value, axis=(1, 2))
        
        # Pad new edges
        self.pad_value(self.elevation_map, shift_value, value=0.0)           # elevation
        self.pad_value(self.elevation_map, shift_value, idx=1, value=self.initial_variance)  # variance
        self.plugin_manager.reset_layers()  # Invalidate plugin caches
```

### 10.3 The Axis Swap Bug (Fixed!)

**Bug**: Original code did `cp.roll(map, [dx, dy])` but axis=(1,2) = (rows=Y, cols=X).

**Fix** (`elevation_mapping.py:247-250`):
```python
# delta_pixel = [dx, dy] in world (X, Y)
# axis=(1, 2) = (rows=Y, cols=X)
# Need [dy, dx] for correct shift
shift_value = cp.array([delta_pixel[1], delta_pixel[0]], dtype=cp.int32)
```

### 10.4 Move to Absolute Pose (`elevation_mapping.py:190-206`)

```python
def move_to(self, position, R):
    """Shift map to center on robot position with rotation R."""
    self.base_rotation = cp.asarray(R, dtype=self.data_type)
    position = cp.asarray(position)
    delta = position - self.center
    delta_pixel = cp.around(delta[:2] / self.resolution)
    delta_xy = delta_pixel * self.resolution
    self.center[:2] += delta_xy
    self.center[2] += delta[2]
    self.shift_map_xy(-delta_pixel)   # Negative: map moves opposite to robot
    self.shift_map_z(-delta[2])
```

---

## 11. Coordinate Systems & Conventions

### 11.1 Frames

| Frame | Description |
|-------|-------------|
| `map` | World frame, fixed |
| `base_link` | Robot body frame |
| `camera_depth_optical_frame` | Sensor frame (Z forward, X right, Y down) |

### 11.2 Internal Convention (`elevation_mapping.py:790-813`)

```
elevation_mapping_cupy:  Row=Y, Col=X (row-major, axis 1=rows=Y, axis 2=cols=X)
grid_map (ROS):          Row→-X, Col→-Y (see GridMapMath.cpp)
```

**Transform for RViz** (`elevation_mapping.py:784-813`):

```python
def _transform_to_grid_map_coordinate_convention(self, m):
    # 1. Transpose: swap axes so Row=X, Col=Y
    m = m.T
    # 2. Flip axis 0: increasing row → decreasing X
    m = xp.flip(m, 0)
    # 3. Flip axis 1: increasing col → decreasing Y
    m = xp.flip(m, 1)
    return m
```

**Equivalent**: `rot90(m.T, k=2)` or `flip(flip(m.T, 0), 1)`

### 11.3 Sensor → Map Transform

```python
# Point in sensor frame: p_sensor = [x, y, z]
# Robot pose in map: R (3x3), t (3)
# Point in map frame:
p_map = R @ p_sensor + t

# Cell indices:
cx = int((p_map.x - center_x) / resolution + cell_n/2)
cy = int((p_map.y - center_y) / resolution + cell_n/2)
```

---

## 12. Kernel Architecture

### 12.1 ElementwiseKernel Pattern

All kernels use `cp.ElementwiseKernel` - one thread per element (point or cell).

```python
kernel = cp.ElementwiseKernel(
    in_params='raw T x, raw T y, raw T z',   # Inputs
    out_params='raw T out',                   # Outputs
    preamble='''__device__ float helper(...) { ... }''',  # Shared functions
    operation='out = helper(x, y, z)',        # Per-thread code
    name='my_kernel'
)

# Launch
kernel(x_gpu, y_gpu, z_gpu, out_gpu, size=N)
```

### 12.2 Kernel List

| Kernel | Purpose | Inputs | Outputs |
|--------|---------|--------|---------|
| `add_points_kernel` | Main fusion + ray tracing | Points, R, t, map | newmap (accumulators), map (updated) |
| `error_counting_kernel` | Drift compensation | Points, R, t, map | error sum, count |
| `average_map_kernel` | Finalize fusion | newmap | map (averaged) |
| `dilation_filter_kernel` | Traversability prep | upper_bound, mask | dilated map |
| `normal_filter_kernel` | Surface normals | elevation, mask | normal_map |
| `polygon_mask_kernel` | Polygon queries | Polygon vertices | Binary mask |

### 12.3 Preamble = Shared Device Functions

```python
def map_utils(...):
    return string.Template("""
    __device__ int get_x_idx(float x, float center) { ... }
    __device__ int get_y_idx(float y, float center) { ... }
    __device__ int get_idx(float x, float y, float cx, float cy) { ... }
    __device__ int get_map_idx(int idx, int layer_n) { ... }
    __device__ float point_noise(float x, float y, float z) { ... }
    __device__ bool is_valid(float x, float y, float z, ...) { ... }
    __device__ float ray_vector(...) { ... }
    """).substitute(params)
```

---

## 13. Learning Resources

### 13.1 Core Papers

| Topic | Paper |
|-------|-------|
| Elevation Mapping | Fankhauser et al., "Probabilistic Terrain Mapping for Mobile Robots" (RSS 2018) |
| GPU Elevation Mapping | Miki et al., "Elevation Mapping for Legged Robots on GPU" (RA-L 2022) |
| Bayesian Fusion | Thrun et al., "Probabilistic Robotics" Ch. 6 (Kalman/Bayes filters) |
| Grid Maps | Fankhauser et al., "Universal Grid Map Library" (IROS 2018) |

### 13.2 Code References

| Component | File |
|-----------|------|
| Main fusion kernel | `kernels/custom_kernels.py:136-288` |
| Drift compensation | `elevation_mapping.py:368-379` |
| Ray tracing | `kernels/custom_kernels.py:209-269` |
| Map shifting | `elevation_mapping.py:236-258` |
| Traversability CNN | `traversability_filter.py` |
| ROS 2 node | `elevation_mapping_node.py` |

### 13.3 Concepts to Master

| Concept | Where to Practice |
|---------|-------------------|
| Bayesian filtering | Implement 1D Kalman filter from scratch |
| CUDA kernels | Write `cp.ElementwiseKernel` for image blur |
| Coordinate transforms | Implement `transform_point(sensor, robot, map)` |
| Mahalanobis distance | 1D: `abs(x - μ)/σ`; ND: `sqrt((x-μ)ᵀ Σ⁻¹ (x-μ))` |
| Ray-grid intersection | Bresenham 3D or DDA algorithm |

### 13.4 Reimplement From Scratch Checklist

- [ ] 2D grid with multiple layers (NumPy → CuPy)
- [ ] Point cloud → grid fusion with Bayesian update
- [ ] Sensor noise model (range²)
- [ ] Mahalanobis outlier rejection
- [ ] Ray tracing for free space
- [ ] Map shifting with `cp.roll` (watch axis order!)
- [ ] Drift compensation on flat ground
- [ ] Simple CNN for traversability (PyTorch/Chainer)
- [ ] GridMap ROS 2 publisher with coordinate transform
- [ ] Parameter management with YAML overrides

---

## Appendix: Key Equations Reference

### Bayesian Fusion
```
μ_post = (μ₁/σ₁² + μ₂/σ₂²) / (1/σ₁² + 1/σ₂²)
σ²_post = 1 / (1/σ₁² + 1/σ₂²)
```

### Sensor Noise
```
σ²_sensor = k · (x² + y² + z²)  where k = sensor_noise_factor
```

### Mahalanobis Distance (1D)
```
D_M = |z - μ_map| / σ_map
```

### Ray Step
```
ray_step = resolution / √2  (diagonal step)
```

### Map Shift (World → Grid)
```
delta_pixel = round(delta_world / resolution)
shift_grid = [delta_pixel_y, delta_pixel_x]  # axis swap!
```

### Coordinate Transform (Internal → GridMap)
```
m_gridmap = flip(flip(m_internal.T, axis=0), axis=1)
```

---

*This document covers the mathematical and algorithmic foundations. Read alongside the source files for implementation details.*