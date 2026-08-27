# Elevation Mapping GPU ROS 2 - Code Deep Dive

**Repository**: `src/d1/elevation_mapping_gpu_ros2/` (iit-DLSLab fork)

This document provides a **line-by-line code walkthrough** of the elevation_mapping_cupy package. Read alongside the actual source files to own the code.

---

## 1. Repository Structure

```
elevation_mapping_gpu_ros2/
├── elevation_mapping_cupy/              # MAIN PACKAGE - GPU elevation mapping
│   ├── elevation_mapping_cupy/
│   │   ├── elevation_mapping.py         # Core ElevationMap class (1289 lines)
│   │   ├── elevation_mapping_node.py    # ROS 2 Node (982 lines)
│   │   ├── parameter.py                 # Parameter dataclass (293 lines)
│   │   ├── kernels/
│   │   │   ├── custom_kernels.py        # CUDA ElementwiseKernels (712 lines)
│   │   │   ├── custom_image_kernels.py
│   │   │   └── custom_semantic_kernels.py
│   │   ├── map_initializer.py           # Map initialization (cubic/linear interp)
│   │   ├── traversability_filter.py     # CNN traversability (Chainer/PyTorch)
│   │   ├── traversability_polygon.py    # Polygon safety checking
│   │   ├── gridmap_utils.py             # GridMap message encoding
│   │   └── plugins/                     # Plugin system (erosion, inpainting, etc.)
│   ├── scripts/
│   │   ├── elevation_mapping_node.py    # Entry point
│   │   └── synthetic_pointcloud_tf_publisher.py
│   ├── launch/
│   │   ├── elevation_mapping.launch.py
│   │   ├── synthetic_depth_demo.launch.py
│   │   └── ...
│   ├── config/
│   │   ├── core/weights.dat             # Traversability CNN weights
│   │   ├── core/plugin_config.yaml
│   │   └── elevation_mapping.yaml       # ROS params
│   └── test/
│
├── elevation_map_msgs/                  # Custom messages
│   ├── msg/ElevationMap.msg
│   ├── msg/ElevationGrid.msg
│   └── srv/
│
├── plane_segmentation_ros2/             # Plane extraction (empty in this fork)
│
└── sensor_processing/
    └── semantic_sensor/                 # DINO semantic segmentation
```

---

## 2. Core Data Structures

### 2.1 Parameter Class (`parameter.py:13-293`)

**Purpose**: Single source of truth for all algorithm parameters. Uses `simple_parsing.Serializable` for YAML serialization.

**Key Parameters** (with defaults from `parameter.py:133-219`):

```python
# Map geometry
resolution: float = 0.04          # 4cm/cell
map_length: float = 8.0           # 8m x 8m map
cell_n: int = None                # Computed: round(map_length/resolution) + 2

# Sensor noise model
sensor_noise_factor: float = 0.05  # noise = factor * (x²+y²+z²)
mahalanobis_thresh: float = 2.0    # Outlier threshold (std deviations)
outlier_variance: float = 0.01     # Variance added for outliers

# Drift compensation
enable_drift_compensation: bool = True
max_drift: float = 0.10            # Max height correction per update
drift_compensation_alpha: float = 1.0
position_noise_thresh: float = 0.1  # Trigger drift comp if pose change > this
orientation_noise_thresh: float = 0.1

# Traversability
dilation_size: int = 2             # Dilation before CNN
traversability_inlier: float = 0.1 # Min traversability for drift comp
checker_layer: str = "traversability"
safe_thresh: float = 0.5           # Polygon unsafe if traversability < this
max_unsafe_n: int = 20             # Max unsafe cells in polygon

# Visibility cleanup (ray tracing)
enable_visibility_cleanup: bool = True
max_ray_length: float = 2.0
cleanup_step: float = 0.01
cleanup_cos_thresh: float = 0.5

# Multi-floor
enable_overlap_clearance: bool = True
overlap_clear_range_xy: float = 4.0
overlap_clear_range_z: float = 2.0

# CNN backend
use_chainer: bool = True           # False = PyTorch (faster but +2GB VRAM)
weight_file: str = "config/weights.dat"
plugin_config_file: str = "config/plugin_config.yaml"
```

**Critical Method** (`parameter.py:275-283`):
```python
def update(self):
    """Must call after changing resolution/map_length!"""
    self.cell_n = int(round(self.map_length / self.resolution)) + 2  # +2 border
    self.true_cell_n = round(self.map_length / self.resolution)
    self.true_map_length = self.true_cell_n * self.resolution
```

---

### 2.2 ElevationMap Class (`elevation_mapping.py:91-1289`)

**The core GPU-accelerated elevation mapping engine.**

#### 2.2.1 Map Layout (`elevation_mapping.py:108-119`)

```python
# Shape: (7 layers, cell_n, cell_n) - CuPy array on GPU
self.elevation_map = xp.zeros((7, self.cell_n, self.cell_n), dtype=self.data_type)
self.layer_names = [
    "elevation",      # 0: Height (m)
    "variance",       # 1: Height variance
    "is_valid",       # 2: 1.0=valid, 0.0=invalid
    "traversability", # 3: CNN output (0-1, higher=more traversable)
    "time",           # 4: Time since last update
    "upper_bound",    # 5: Max height from ray tracing
    "is_upper_bound", # 6: 1.0=upper bound valid
]
```

**Coordinate Convention** (CRITICAL - `elevation_mapping.py:790-813`):
- **Internal**: Row=Y, Col=X (row-major, axis 1=rows=Y, axis 2=cols=X)
- **GridMap msg**: Row→-X, Col→-Y (flipped!)
- **Transform**: `m.T` → `flip(axis=0)` → `flip(axis=1)` (equiv. rot90 k=2)

---

### 2.3 CUDA Kernels (`kernels/custom_kernels.py`)

All kernels are `cp.ElementwiseKernel` - one thread per point/cell.

#### 2.3.1 `add_points_kernel` (`custom_kernels.py:136-288`)

**The main fusion kernel** - runs per point, updates map cells.

```python
# Inputs: point (x,y,z in sensor frame), R (3x3), t (3), map, center
# Outputs: newmap (accumulator), updated map (atomic)

# Per-point flow:
1. Transform point to map frame: p_map = R @ p_sensor + t
2. Compute cell index: idx = get_idx(x, y, center_x, center_y)
3. Validate: is_valid() checks distance, height limits, ramped filter
4. If valid & inside map bounds:
   a. Mahalanobis outlier check: |map_h - z| > map_v * mahalanobis_thresh
   b. If outlier: atomicAdd variance += outlier_variance
   c. Else: Bayesian fusion (weighted by variance):
       new_h = (map_h * v + z * map_v) / (map_v + v)
       new_v = (map_v * v) / (map_v + v)
       atomicAdd newmap[idx, 0] += new_h
       atomicAdd newmap[idx, 1] += new_v
       atomicAdd newmap[idx, 2] += 1 (count)
       map[idx, 2] = 1 (is_valid)
       map[idx, 4] = 0 (reset time)
       map[idx, 5] = new_h (upper_bound)
       map[idx, 6] = 0
5. Visibility cleanup (ray tracing from sensor to point):
   - Step along ray at resolution/√2 increments
   - For each cell: if valid & not updated recently & ray penetrates:
     atomicAdd validity -= cleanup_step, variance += outlier_variance
```

**Key Parameters** (substituted at compile time):
- `ray_step = resolution / √2` (diagonal step)
- `enable_edge_sharpen`, `enable_visibility_cleanup` as int (0/1)

---

#### 2.3.2 `error_counting_kernel` (`custom_kernels.py:291-356`)

**Computes height drift** for drift compensation.

```python
# Runs per point, accumulates error where:
# - Cell is valid
# - |z - map_h| < map_v * mahalanobis_thresh (inlier)
# - map_v < outlier_variance/2 (low variance)
# - traversability > traversability_inlier (traversable)
# error += (z - map_h) for each inlier
# error_cnt++ per inlier
# mean_error = error / error_cnt (computed on host)
```

---

#### 2.3.3 `average_map_kernel` (`custom_kernels.py:359-402`)

**Finalizes fused measurements** - runs per cell.

```python
# For each cell:
if new_cnt > 0:
    if new_v/new_cnt > max_variance:  # Too uncertain
        map_h = 0; map_v = initial_variance; is_valid = 0
    else:
        map_h = new_h/new_cnt; map_v = new_v/new_cnt; is_valid = 1
else:
    map_h = 0; map_v = initial_variance; is_valid = 0
```

---

#### 2.3.4 `dilation_filter_kernel` (`custom_kernels.py:405-467`)

**Morphological dilation** - fills invalid cells from neighbors.

```python
# For each invalid cell (mask < 0.5):
#   Search dilation_size neighborhood (Chebyshev distance)
#   Find nearest valid cell
#   Copy its height value
#   Set mask = 1
```

---

#### 2.3.5 `normal_filter_kernel` (`custom_kernels.py:470-530`)

**Computes surface normals** from elevation gradients.

```python
# For valid cells:
#   dzdx = h(x+1) - h(x)
#   dzdy = h(y+1) - h(y)
#   Normal = (-dzdx/res, -dzdy/res, 1) normalized
#   → stored in normal_map[3, H, W]
```

---

### 2.4 ROS 2 Node (`elevation_mapping_node.py:87-982`)

#### 2.4.1 Initialization (`elevation_mapping_node.py:88-129`)

```python
def __init__(self):
    # Load param files from package share
    weight_file = get_package_share_directory("elevation_mapping_cupy") + "/config/core/weights.dat"
    plugin_config_file = ... + "/config/core/plugin_config.yaml"
    
    self.param = Parameter(use_chainer=False, weight_file=..., plugin_config_file=...)
    self.initialize_ros()      # TF buffer, ROS params
    self.set_param_values_from_ros()  # Override Parameter with ROS params
    self.initialize_elevation_mapping()  # Creates ElevationMap instance
    self.register_subscribers()  # PointCloud2 + Image (synced with CameraInfo)
    self.register_publishers()   # GridMap publishers (configurable)
    self.register_timers()       # pose_update (10Hz), variance, time
    self.register_services()     # masked_replace, save_map, load_map
```

#### 2.4.2 Subscriber Registration (`elevation_mapping_node.py:285-349`)

**Supports multiple subscribers** configured via YAML:

```yaml
# config/elevation_mapping.yaml example:
subscribers:
  front_lidar:
    topic_name: "/front_lidar/points"
    data_type: "pointcloud"
    channels: ["intensity", "semantic"]
  downward_cam:
    topic_name: "/camera/depth/image_raw"
    camera_info_topic_name: "/camera/depth/camera_info"
    data_type: "image"
    channels: ["rgb", "grass"]
```

**PointCloud2** (`elevation_mapping_node.py:341-348`):
- Uses `QoSPresetProfiles.SENSOR_DATA` (BEST_EFFORT)
- Callback: `pointcloud_callback`

**Image** (`elevation_mapping_node.py:297-329`):
- Uses `message_filters.ApproximateTimeSynchronizer` with CameraInfo
- Callback: `image_callback`

---

#### 2.4.3 PointCloud2 Callback (`elevation_mapping_node.py:853-936`)

```python
def pointcloud_callback(self, msg, sub_key):
    self._last_t = msg.header.stamp
    
    # Parse channels from config
    additional_channels = self.param.subscriber_cfg[sub_key].get("channels", [])
    channels = ["x", "y", "z"] + additional_channels
    
    # Extract points (handles multiple PointCloud2 formats)
    if additional_channels:
        pts = rnp.numpify(msg)  # ros2_numpy → structured array
        # ... complex parsing for xyz + channels ...
    else:
        pts = _pointcloud2_xyz_f32(msg)  # Fast path: just xyz
    
    # Get transform: sensor_frame → map_frame
    if msg.header.frame_id == self.map_frame:
        R, t = I, 0
    else:
        transform = self.safe_lookup_transform(map_frame, sensor_frame, stamp)
        R, t = quaternion_to_matrix(transform.rotation), transform.translation
    
    # Fuse into map
    self._map.input_pointcloud(pts, channels, R, t_np, 0, 0)
    self._pointcloud_process_counter += 1
```

---

#### 2.4.4 Image Callback (`elevation_mapping_node.py:804-851`)

```python
def image_callback(self, camera_msg, camera_info_msg, sub_key):
    self._last_t = camera_msg.header.stamp
    
    # Convert image: cv_bridge → cv2 → list of channels [H,W]
    semantic_img = self.cv_bridge.imgmsg_to_cv2(camera_msg, "passthrough")
    if len(semantic_img.shape) == 3:
        semantic_img = [semantic_img[:,:,i] for i in range(semantic_img.shape[2])]
    
    # Camera intrinsics
    K = np.array(camera_info_msg.k).reshape(3,3)
    D = np.array(camera_info_msg.d)
    
    # Transform: camera_frame → map_frame
    transform = self.safe_lookup_transform(map_frame, camera_frame, stamp)
    R, t = quaternion_to_matrix(transform.rotation), transform.translation
    
    # Resolve channels (from config or ChannelInfo topic)
    channels = self.resolve_image_channels(sub_key)
    
    # Project image to map
    self._map.input_image(semantic_img, channels, R, t_np, K, D, 
                          distortion_model, height, width)
```

---

#### 2.4.5 Pose Update Timer (`elevation_mapping_node.py:938-955`)

**Runs at 10Hz** (`update_pose_fps` param). Moves map to follow robot.

```python
def pose_update(self):
    transform = self.safe_lookup_transform(map_frame, base_frame, self._last_t)
    t = transform.translation
    q = transform.rotation
    trans = [t.x, t.y, t.z]
    rot = quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
    
    self._map.move_to(trans, rot)  # Shifts map on GPU!
    self._map_t = t
    self._map_q = q
```

---

#### 2.4.6 GridMap Publishing (`elevation_mapping_node.py:422-462`)

```python
def publish_map(self, key):
    gm = GridMap()
    gm.header.frame_id = self.map_frame
    gm.header.stamp = self._last_t
    gm.info.resolution = self._map.resolution
    gm.info.length_x = (cell_n - 2) * resolution  # Exclude border
    gm.info.length_y = gm.info.length_x
    gm.info.pose.position = self._map_t (or center)
    gm.info.pose.orientation = identity (neutral for RViz)
    
    gm.layers = []
    gm.basic_layers = config["basic_layers"]  # e.g., ["elevation"]
    
    for layer in config["layers"]:
        self._map.get_map_with_name_ref(layer, self._map_data)
        # NOTE: No flip needed after coordinate convention fix in elevation_mapping.py
        gm.data.append(self._numpy_to_multiarray(self._map_data, "gridmap_column"))
    
    self._publishers_dict[key].publish(gm)
```

---

## 3. Key Algorithms In-Depth

### 3.1 Bayesian Height Fusion (`elevation_mapping.py:380-390`, `kernels/custom_kernels.py:187-204`)

**The core insight**: Each measurement has uncertainty (variance). Fuse optimally:

```
Prior: map_h ± map_v (current map estimate)
Measurement: z ± v (sensor noise = sensor_noise_factor * range²)

Posterior mean: (map_h/v + z/v) / (1/v + 1/map_v) = (map_h*v + z*map_v) / (map_v + v)
Posterior variance: 1 / (1/v + 1/map_v) = (map_v * v) / (map_v + v)
```

**Implemented atomically on GPU** per cell with `atomicAdd`.

---

### 3.2 Drift Compensation (`elevation_mapping.py:368-379`, `elevation_mapping_node.py:958`)

**Problem**: Robot odometry drifts → map accumulates height errors.

**Solution**: 
1. `error_counting_kernel` finds inlier points on flat, traversable terrain
2. Computes `mean_error = Σ(z - map_h) / N`
3. If `|mean_error| < max_drift` and pose change > thresholds:
   `map[0] += mean_error * drift_compensation_alpha`
4. Runs at `update_variance_fps` (default 10Hz)

---

### 3.3 Visibility Cleanup (Ray Tracing) (`kernels/custom_kernels.py:209-269`)

**Purpose**: Mark cells as "free" if ray from sensor passes through them.

```python
# For each point, trace ray from sensor (t) to point (x,y,z):
for s in [ray_step, 2*ray_step, ..., ray_length]:
    nx, ny, nz = t + ray_dir * s
    nidx = get_idx(nx, ny)
    
    if cell valid & not updated recently:
        # Check if ray penetrates surface
        if map_h > nz + 0.01 - min(map_v, 1.0)*0.05:
            # Check normal alignment (avoid marking vertical walls as free)
            if |ray · normal| < cleanup_cos_thresh:
                validity -= cleanup_step
                variance += outlier_variance
```

---

### 3.4 Traversability CNN (`elevation_mapping.py:144-147`, `traversability_filter.py`)

**Architecture**: Small CNN (3 conv layers + output) - runs on dilated upper_bound map.

```python
# Input: dilated upper_bound map (binary: 1=valid+upper, 0=invalid)
# Output: traversability per cell (0-1)

# Chainer version (default):
get_filter_chainer(w1, w2, w3, w_out)  # 3 conv layers

# PyTorch version (use_chainer=False):
get_filter_torch(w1, w2, w3, w_out)    # Faster, +2GB VRAM

# Weights loaded from config/weights.dat (pickle)
# w1: (4, 1, 3, 3), w2: (4, 1, 3, 3), w3: (4, 1, 3, 3), w_out: (1, 12, 1, 1)
```

---

### 3.5 Map Shifting (`elevation_mapping.py:236-258`)

**Critical for UAV movement** - shifts entire map on GPU.

```python
def shift_map_xy(self, delta_pixel):
    # delta_pixel = [dx, dy] in WORLD coords (X forward, Y left)
    # Map array: (layers, height=rows=Y, width=cols=X)
    # cp.roll axis=(1,2) expects [row_shift, col_shift] = [dy, dx]
    # SWAP: [dx, dy] → [dy, dx]
    shift_value = cp.array([delta_pixel[1], delta_pixel[0]], dtype=cp.int32)
    
    with self.map_lock:
        self.elevation_map = cp.roll(self.elevation_map, shift_value, axis=(1, 2))
        # Pad new edges with 0 (elevation) / initial_variance (variance)
        self.pad_value(self.elevation_map, shift_value, value=0.0)
        self.pad_value(self.elevation_map, shift_value, idx=1, value=self.initial_variance)
        self.plugin_manager.reset_layers()
```

**BUG FIX**: Original had axis swap bug (rolled [x,y] instead of [y,x]). Fixed in v2.1.0 with explicit swap.

---

## 4. Multi-Layer GridMap Output

### 4.1 Standard Layers (always available)

| Layer | Index | Description | Range |
|-------|-------|-------------|-------|
| elevation | 0 | Height (m) | ℝ |
| variance | 1 | Height uncertainty | [0, max_variance] |
| is_valid | 2 | 1=measured, 0=unknown | {0,1} |
| traversability | 3 | CNN output | [0,1] |
| time | 4 | Seconds since update | [0, ∞) |
| upper_bound | 5 | Max height from rays | ℝ |
| is_upper_bound | 6 | 1=ray hit ceiling | {0,1} |

### 4.2 Plugin Layers (dynamic)

Loaded from `config/core/plugin_config.yaml`:
- `min_filter` - Minimum filter
- `smooth` - Gaussian smoothing  
- `inpaint` - Inpainting for holes
- `robot_centric_elevation` - Relative to robot
- Custom semantic layers (rgb, grass, tree, people...)

---

## 5. Services

### 5.1 `masked_replace` (`elevation_mapping_node.py:464-475`)

```python
# Request: GridMap with layers + mask layer
# Action: Replace map cells where mask=1 with incoming layer values
# Use case: Human correction, multi-map fusion
```

### 5.2 `save_map` / `load_map` (`elevation_mapping_node.py:477-567`)

```python
# Saves TWO bags:
#   fused.bag - processed layers (elevation, traversability, ...)
#   fused_raw.bag - raw layers (all 7 base + plugins)
# Load restores full map state + robot pose
```

---

## 6. Configuration Files

### 6.1 `config/elevation_mapping.yaml` (ROS params)

```yaml
elevation_mapping_node:
  ros__parameters:
    use_chainer: false
    resolution: 0.05
    map_length: 20.0
    map_frame: "map"
    base_frame: "base_link"
    corrected_map_frame: "map_corrected"
    
    subscribers:
      pointcloud:
        topic_name: "/camera/depth/points"
        data_type: "pointcloud"
        channels: []
    
    publishers:
      elevation_map:
        fps: 2.0
        layers: ["elevation", "variance", "traversability", "is_valid"]
        basic_layers: ["elevation"]
```

---

## 7. Launch Files

### 7.1 `launch/synthetic_depth_demo.launch.py`

```python
# No Gazebo needed - generates synthetic pointcloud + TF
# Usage:
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py
ros2 run elevation_mapping_cupy elevation_mapping_node.py
```

### 7.2 `launch/elevation_mapping.launch.py`

```python
# Main launch file - loads params from elevation_mapping.yaml
# Usage with real sensor:
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py
```

---

## 8. Build & Run Commands

```bash
# Build
cd /home/prem/terralink
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash

# Run synthetic demo (no hardware)
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py &
ros2 run elevation_mapping_cupy elevation_mapping_node.py

# RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/elevation_mapping.rviz
```

---

## 9. Key Files Quick Reference

| File | Lines | Purpose |
|------|-------|---------|
| `elevation_mapping.py` | 1289 | Core ElevationMap class - all GPU logic |
| `elevation_mapping_node.py` | 982 | ROS 2 node - subscribers, publishers, services |
| `parameter.py` | 293 | Parameter dataclass with defaults & YAML loading |
| `kernels/custom_kernels.py` | 712 | CUDA ElementwiseKernels (fusion, ray trace, normals) |
| `traversability_filter.py` | ~200 | CNN for traversability (Chainer/PyTorch) |
| `map_initializer.py` | ~100 | Cubic/linear interpolation for map init |
| `gridmap_utils.py` | ~150 | GridMap message encoding/decoding |
| `plugins/plugin_manager.py` | ~200 | Dynamic plugin system |

---

## 10. Testing

```bash
# Unit tests (no ROS)
cd src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -v

# Key test files:
# test_map_shifting.py      - Verifies shift_map_xy axis swap fix
# test_gridmap_layout.py    - GridMap coordinate convention
# test_map_services.py      - save/load/masked_replace
# test_parameter.py         - Parameter serialization
```

---

## 11. Integration with TerraLink UGV

### Required Components (not yet implemented)

1. **UAV Description**: Add RGB-D camera (RealSense D435i) to `my_uav` in d3
2. **Costmap Converter Node**: GridMap → Nav2 OccupancyGrid
   ```python
   # Subscribe: /elevation_mapping_node/elevation_map (GridMap)
   # Extract: traversability layer
   # Publish: /global_costmap (OccupancyGrid)
   ```
3. **Nav2 Config**: Use traversability as static layer
4. **Launch File**: Combine UAV + mapping + UGV Nav2

---

*Read this alongside the source files. Every section references exact line numbers.*