# Direction 1: Geometric Approach - 2.5D Elevation Mapping

## Overview

Direction 1 uses physical height measurements from the terrain to distinguish traversable paths from obstacles. Instead of relying on visual appearance (color/texture), this approach maps the actual 3D geometry of the ground surface using a 2.5D elevation grid.

**Key Insight**: A gradual height increase = ramp (traversable). A sudden height spike = wall (obstacle). This is immune to optical illusions like shadows or color changes.

---

## 1. Core Concepts

### 1.1 What is 2.5D Elevation Mapping?

A 2.5D elevation map represents terrain height as a function of (x, y) coordinates: `z = f(x, y)`. Unlike full 3D mapping, it assumes a single height value per (x, y) cell - no overhangs or caves. This is perfect for ground robot navigation where the robot moves on a continuous surface.

```
Traditional 2D Occupancy Grid:     2.5D Elevation Grid:
[0 0 1 1 0]                        [0.0 0.0 1.5 2.0 0.0]
[0 0 1 1 0]                        [0.0 0.1 1.6 2.1 0.0]
[0 0 0 0 0]                        [0.0 0.0 0.1 0.2 0.0]
  0=free, 1=occupied                 Height in meters
```

### 1.2 Why Elevation Mapping for UAV-UGV?

- **UAV Sensor**: Downward-facing RGB-D camera or 3D LiDAR provides depth (Z) for each pixel
- **UGV Need**: Knows if a slope is drivable (ramp) vs blocked (wall/cliff)
- **Advantage over 2D**: Can traverse ramps, avoid cliffs, detect negative obstacles (holes)

---

## 2. Sensor Requirements

| Sensor Type | Pros | Cons | Typical Use |
|-------------|------|------|-------------|
| **RGB-D Camera** (Intel RealSense, ZED) | Color + depth, lightweight, ~$150-500 | Limited range (5-10m), sunlight interference | Indoor/outdoor UAV |
| **3D LiDAR** (Velodyne, Livox, Ouster) | Long range (50-100m), works in dark, accurate | Heavy (500g-2kg), expensive ($1000-5000), power hungry | Outdoor UAV, high accuracy |
| **Stereo Camera** (ZED, Bumblebee) | Passive, works outdoors, gives color | Compute heavy, needs texture, calibration drift | Outdoor UAV |

### 2.1 UAV Mounting Considerations

- **Downward-facing (nadir)**: Best for elevation mapping, covers area directly below
- **Forward-facing**: Needed for obstacle avoidance during flight
- **Gimbal stabilization**: Critical - UAV vibration ruins depth data
- **Altitude**: Higher = larger coverage but lower resolution

---

## 3. Elevation Mapping Pipeline

### 3.1 Data Flow Overview

```
UAV Sensor (RGB-D / LiDAR)
        │
        ▼
Point Cloud (x, y, z, intensity, color)
        │
        ▼
[Preprocessing] Voxel Grid Filter → Outlier Removal → Ground/Non-Ground Separation
        │
        ▼
[Elevation Map Update] Fuse points into 2.5D grid (cell = max/min/mean height)
        │
        ▼
[Layer Generation] Elevation, Variance, Traversability, Color, Semantic layers
        │
        ▼
[UGV Consumption] Traversability layer → Nav2 Costmap → Global/Local Planner
```

### 3.2 Key Algorithms

#### A. Point Cloud to Grid Fusion

For each incoming point (x, y, z):
1. **Cell lookup**: `cell_x = floor((x - origin_x) / resolution)`, `cell_y = floor((y - origin_y) / resolution)`
2. **Height update**: Store max z (for obstacle detection) or mean z (for ground)
3. **Variance tracking**: Track height variance per cell for traversability
4. **Time decay**: Older measurements fade (for dynamic environments)

```python
# Simplified fusion logic
def update_elevation_map(grid, points, resolution, origin):
    for pt in points:
        cx = int((pt.x - origin.x) / resolution)
        cy = int((pt.y - origin.y) / resolution)
        if 0 <= cx < grid.width and 0 <= cy < grid.height:
            # Use maximum height for conservative obstacle detection
            grid.elevation[cy, cx] = max(grid.elevation[cy, cx], pt.z)
            grid.count[cy, cx] += 1
            # Update running variance for traversability
            update_variance(grid, cy, cx, pt.z)
```

#### B. Traversability Analysis

From elevation grid, compute traversability cost for each cell:

```python
def compute_traversability(elevation_grid, variance_grid, robot_params):
    """
    robot_params: max_slope, max_step_height, max_roughness
    Returns: cost grid (0=free, 100=lethal, 255=unknown)
    """
    cost = np.zeros_like(elevation_grid, dtype=np.uint8)
    
    # Gradient = slope
    grad_x = np.gradient(elevation_grid, axis=1) / resolution
    grad_y = np.gradient(elevation_grid, axis=0) / resolution
    slope = np.sqrt(grad_x**2 + grad_y**2)  # rad
    
    # Step height = difference to neighbors
    step_height = maximum_filter(elevation_grid, size=3) - minimum_filter(elevation_grid, size=3)
    
    # Roughness = local variance
    roughness = variance_grid
    
    # Classify
    lethal = (slope > robot_params.max_slope) | \
             (step_height > robot_params.max_step) | \
             (roughness > robot_params.max_roughness)
    cost[lethal] = 100
    
    difficult = (slope > robot_params.max_slope * 0.5) | \
                (step_height > robot_params.max_step * 0.5)
    cost[difficult] = 50  # High cost but traversable
    
    return cost
```

---

## 4. elevation_mapping_gpu_ros2 Architecture

The [iit-DLSLab/elevation_mapping_gpu_ros2](https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2) repository provides a **GPU-accelerated** elevation mapping implementation using CuPy (CUDA Python).

### 4.1 Package Structure

```
elevation_mapping_gpu_ros2/
├── elevation_mapping_cupy/        # Main GPU elevation mapping node
│   ├── elevation_mapping_cupy/    # Python package
│   │   ├── elevation_mapping.py   # Core GPU kernel
│   │   ├── map_operations.py      # Shift, rotate, crop
│   │   └── layer_management.py    # Multi-layer grid map
│   ├── scripts/
│   │   ├── elevation_mapping_node.py    # ROS 2 node entry point
│   │   └── synthetic_pointcloud_tf_publisher.py  # Test data generator
│   ├── launch/
│   ├── config/
│   ├── test/
│   └── CMakeLists.txt / package.xml
│
├── elevation_map_msgs/            # Custom ROS 2 messages
│   ├── msg/ElevationMap.msg       # Multi-layer grid map
│   ├── msg/ElevationGrid.msg      # Single layer
│   └── srv/
│
├── plane_segmentation_ros2/       # Plane extraction from elevation map
│
├── sensor_processing/
│   └── semantic_sensor/           # Semantic segmentation integration
│
└── docs/
```

### 4.2 Core: elevation_mapping_cupy

**Key Innovation**: Uses CuPy (NumPy API on CUDA) for 10-100x speedup over CPU.

#### Main Node: `elevation_mapping_node.py`

```python
class ElevationMapping(Node):
    def __init__(self):
        # Parameters
        self.map_length_x = 20.0  # meters
        self.map_length_y = 20.0
        self.resolution = 0.05    # 5cm per cell
        self.min_height = -2.0
        self.max_height = 2.0
        
        # GPU Map (CuPy array)
        self.map_gpu = cupy.zeros((layers, height, width), dtype=cupy.float32)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'pointcloud', self.pointcloud_callback, 
            qos_profile_sensor_data)
        
        # Publishers
        self.elevation_map_pub = self.create_publisher(
            ElevationMap, 'elevation_map', 10)
    
    def pointcloud_callback(self, msg):
        # 1. Convert ROS PointCloud2 → CuPy array (zero-copy via ros2_numpy)
        points_gpu = ros2_numpy.pointcloud2_to_cupy(msg)
        
        # 2. Transform to map frame (GPU kernel)
        points_map = transform_points_gpu(points_gpu, self.tf_map_sensor)
        
        # 3. Fuse into elevation map (GPU kernel - THE FAST PART)
        self.fuse_pointcloud_gpu(points_map)
        
        # 4. Publish updated map
        self.publish_elevation_map()
```

#### GPU Fusion Kernel (Simplified)

```python
# In elevation_mapping.py - runs on GPU
def fuse_pointcloud_kernel(elevation_map, points, params):
    """
    elevation_map: [layers, H, W] CuPy array
    points: [N, 3] CuPy array (x, y, z)
    """
    # Each thread handles one point
    idx = cuda.grid(1)
    if idx >= points.shape[0]:
        return
    
    x, y, z = points[idx]
    
    # World to grid coordinates
    cx = int((x - origin_x) / resolution)
    cy = int((y - origin_y) / resolution)
    
    if 0 <= cx < width and 0 <= cy < height:
        # Atomic max for elevation layer (layer 0)
        cuda.atomic.max(elevation_map[0, cy, cx], z)
        
        # Update variance layer (layer 1) - Welford's algorithm
        update_variance_atomic(elevation_map[1], cy, cx, z)
        
        # Update color/semantic layers if available
        ...
```

### 4.3 Multi-Layer Grid Map

The elevation map publishes **multiple synchronized layers**:

| Layer Index | Name | Description | UGV Use |
|-------------|------|-------------|---------|
| 0 | `elevation` | Height (m) | Slope/step calculation |
| 1 | `variance` | Height variance | Roughness/uncertainty |
| 2 | `traversability` | Computed cost (0-100) | Direct Nav2 costmap |
| 3 | `color_r/g/b` | RGB from camera | Visualization |
| 4 | `semantic` | Class IDs (road=0, grass=1...) | Semantic cost weighting |
| 5 | `observation_count` | Points per cell | Confidence weighting |

### 4.4 Map Operations (GPU Accelerated)

Critical for UAV movement - map must follow UAV:

```python
# map_operations.py - all on GPU
def shift_map_xy(map_gpu, dx_cells, dy_cells):
    """Shift map by integer cells (UAV moved)"""
    return cupy.roll(map_gpu, shift=(-dy_cells, -dx_cells), axis=(1, 2))

def rotate_map(map_gpu, angle_rad, center):
    """Rotate map (UAV yaw changed)"""
    # Affine transform on GPU
    return cupyx.scipy.ndimage.rotate(map_gpu, angle_rad, axes=(1, 2), reshape=False)

def crop_map(map_gpu, new_bounds):
    """Crop to new bounds"""
    return map_gpu[:, y_min:y_max, x_min:x_max]
```

**Critical Bug Fixed**: Original `shift_map_xy` had **axis swap bug** - rolled (x, y) instead of (y, x). Fixed in v2.1.0.

---

## 5. Integration with UGV Nav2 Stack

### 5.1 Elevation Map → Nav2 Costmap

The UGV needs a standard `nav2_costmap_2d` costmap. Conversion pipeline:

```
ElevationMap (multi-layer, grid_map_msgs/msg/GridMap)
        │
        ▼
[GridMap → Costmap Converter Node]
        │
        ▼
nav2_costmap_2d / global_costmap
        │
        ▼
Nav2 Planner (Navfn / Smac)
```

### 5.2 Conversion Node (Pseudo-code)

```python
class ElevationToCostmap(Node):
    def __init__(self):
        self.sub = self.create_subscription(
            GridMap, 'elevation_map', self.map_callback, 10)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, 'global_costmap', 10)
        
        # Robot traversability params
        self.max_slope = 0.35      # ~20 degrees
        self.max_step = 0.15       # 15cm step
        self.max_roughness = 0.05  # 5cm variance
    
    def map_callback(self, grid_map_msg):
        # Extract layers
        elevation = gridmap_to_numpy(grid_map_msg, 'elevation')
        variance = gridmap_to_numpy(grid_map_msg, 'variance')
        traversability = gridmap_to_numpy(grid_map_msg, 'traversability')
        
        # If traversability layer exists, use it directly
        if traversability is not None:
            costmap = traversability.astype(np.uint8)
        else:
            # Compute from elevation + variance
            costmap = compute_traversability(elevation, variance, self)
        
        # Publish as OccupancyGrid
        self.publish_costmap(costmap, grid_map_msg.info)
```

### 5.3 Nav2 Configuration for Elevation Maps

```yaml
# nav2_params.yaml additions for elevation-based navigation
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["elevation_layer", "inflation_layer"]
      elevation_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: elevation_map
        elevation_map:
          topic: /elevation_map
          data_type: "GridMap"
          traversability_layer: "traversability"  # or compute from elevation
          max_slope: 0.35
          max_step: 0.15
          min_obstacle_height: 0.10
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## 6. Building and Running Direction 1

### 6.1 Dependencies

```bash
# ROS 2 Humble/Jazzy
# CUDA Toolkit 11.8+ (for CuPy)
# CuPy: pip install cupy-cuda11x  (match your CUDA version)

# ROS 2 packages needed:
sudo apt install ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
                 ros-humble-nav2-costmap-2d ros-humble-ros2-numpy \
                 python3-scipy python3-numpy python3-opencv
```

### 6.2 Build

```bash
# In terralink workspace
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON
```

### 6.3 Run Simulation

```bash
# Terminal 1: Launch UAV with depth camera in Gazebo
ros2 launch elevation_mapping_cupy synthetic_demo.launch.py

# Terminal 2: Run elevation mapping node
ros2 run elevation_mapping_cupy elevation_mapping_node.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/elevation_mapping.rviz
```

### 6.4 Run Tests

```bash
# Unit tests (no ROS required)
cd src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -v

# Integration tests (requires ROS)
colcon test --packages-select elevation_mapping_cupy \
    --event-handlers console_direct+
```

---

## 7. UAV-UGV Integration Architecture (Direction 1)

### 7.1 Complete Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        UAV (my_uav)                             │
│  ┌──────────────┐    ┌──────────────────┐    ┌───────────────┐  │
│  │ RGB-D Camera │───▶│ PointCloud2      │───▶│ Elevation     │  │
│  │ (Depth + RGB)│    │ /camera/depth    │    │ Mapping Node  │  │
│  └──────────────┘    └──────────────────┘    │ (GPU/CuPy)    │  │
│                                                └───────┬───────┘  │
│                                                        │          │
│                                                GridMap msg       │
│                                                /elevation_map    │
└────────────────────────────────────────────────────┼────────────┘
                                                     │
                                                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                        UGV (my_bot)                             │
│  ┌──────────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │ Costmap Converter│───▶│ Nav2 Global  │───▶│ Nav2 Local   │  │
│  │ (GridMap→Costmap)│    │ Costmap      │    │ Planner      │  │
│  └──────────────────┘    └──────────────┘    └──────────────┘  │
│                                                         │        │
│                                                         ▼        │
│                                                ┌──────────────┐  │
│                                                │ Diff Drive   │  │
│                                                │ Controller   │  │
│                                                └──────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 7.2 ROS 2 Topics & Services

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/my_uav/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | UAV → Mapping | Raw depth point cloud |
| `/elevation_map` | `grid_map_msgs/msg/GridMap` | Mapping → UGV | Multi-layer elevation map |
| `/global_costmap` | `nav_msgs/msg/OccupancyGrid` | Converter → Nav2 | Traversability costmap |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Client → Nav2 | Navigation goals |
| `/odom` | `nav_msgs/msg/Odometry` | UGV → Nav2 | Robot pose |

---

## 8. Advantages & Limitations

### 8.1 Advantages

| Advantage | Explanation |
|-----------|-------------|
| **Physical accuracy** | Measures actual geometry, not appearance |
| **Ramp/cliff detection** | Can distinguish 15° ramp (OK) from 45° wall (blocked) |
| **Negative obstacles** | Detects holes, ditches, drop-offs |
| **No lighting dependency** | Works in darkness, shadows, glare |
| **Quantitative** | Slope, step height, roughness in metric units |
| **GPU acceleration** | 10-100x faster than CPU for large maps |

### 8.2 Limitations

| Limitation | Mitigation |
|------------|------------|
| **Requires depth sensor** | Adds weight/cost/power to UAV |
| **Range limited** | RGB-D: ~10m; LiDAR: ~100m but heavier |
| **Sunlight interference** | RGB-D structured light fails in direct sun; use LiDAR or stereo |
| **Compute on UAV** | Needs onboard GPU (Jetson Orin, Xavier) or offboard |
| **2.5D assumption** | Cannot represent overhangs, multi-level structures |
| **Dynamic objects** | Moving objects leave "ghost" elevations; need temporal filtering |

---

## 9. Comparison with Other Directions

| Aspect | Direction 1: Elevation | Direction 2: Semantic | Direction 3: OpenCV PRM |
|--------|------------------------|----------------------|------------------------|
| **Sensor** | RGB-D / 3D LiDAR | RGB Camera | RGB Camera |
| **Compute** | High (GPU) | Very High (GPU + ML) | Low (CPU) |
| **Accuracy** | Metric geometry | Semantic classes | Color threshold |
| **Robustness** | High (physics-based) | Medium (ML generalization) | Low (lighting sensitive) |
| **Info richness** | Slope, step, roughness | Class labels (road, mud, grass) | Binary obstacle/free |
| **UGV action** | Avoid steep/rough | Avoid semantic classes | Avoid "non-gray" |
| **Maturity** | Production (ANYmal, Spot) | Research → Production | Simulation only |

---

## 10. References & Resources

### 10.1 Key Papers
- **Elevation Mapping**: Fankhauser et al., "Probabilistic Terrain Mapping for Mobile Robots" (2018)
- **GPU Acceleration**: Miki et al., "Elevation Mapping for Legged Robots on GPU" (2022)
- **Traversability**: Wermelinger et al., "Navigation Planning for Legged Robots" (2016)

### 10.2 Code References
- **Main Repo**: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- **Grid Map Library**: https://github.com/ANYbotics/grid_map
- **Nav2 Costmap2D**: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d
- **CuPy**: https://cupy.dev/

### 10.3 ROS 2 Packages Used
- `grid_map_msgs` - Multi-layer grid map messages
- `grid_map_rviz_plugin` - RViz visualization
- `nav2_costmap_2d` - Costmap implementation
- `ros2_numpy` / `ros2_numpy_cupy` - Zero-copy PointCloud2 ↔ NumPy/CuPy
- `tf2_ros` - Coordinate transforms

---

## 11. Implementation Checklist for TerraLink

- [ ] Integrate `elevation_mapping_cupy` into d1 workspace
- [ ] Create `elevation_to_costmap` converter node
- [ ] Define UGV traversability parameters (max_slope, max_step, robot_radius)
- [ ] Update UAV description with RGB-D camera (RealSense D435i or similar)
- [ ] Create launch file for full UAV-UGV stack (Direction 1)
- [ ] Test in Gazebo with elevation-enabled world
- [ ] Benchmark GPU vs CPU performance
- [ ] Document parameter tuning guide

---

*This documentation covers Direction 1 from first principles. Any engineer reading this should understand the complete pipeline from sensor to UGV navigation.*