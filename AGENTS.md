# AGENTS.md - Terralink UAV-UGV Project

## Project Overview

**TerraLink**: ROS 2 Humble collaborative UAV-UGV navigation framework for unstructured environments (construction sites, disaster response). UAV provides global overhead awareness; UGV executes ground tasks with payload capacity.

**Three Technical Directions** (each in separate `src/d<N>/` folder) - **REFERENCE ONLY, DO NOT MODIFY**:

| Direction | Approach | Folder | Status |
|-----------|----------|--------|--------|
| **1. Geometric** | 2.5D Elevation Mapping (RGB-D/3D LiDAR → GPU elevation grid → traversability costmap) | `src/d1/elevation_mapping_gpu_ros2/` | **Working (synthetic demo @ 10 Hz)** |
| **2. Semantic** | AI Vision to Costmap (RGB camera → YOLOv8-seg → semantic classes → Nav2 costmap) | `src/d2/semantic_nav/` | Skeleton package, needs implementation |
| **3. Baseline** | OpenCV Color Filtering + PRM (RGB camera → color threshold → binary grid → PRM waypoints) | `src/d3/my_bot/` | **Working in simulation** |

---

## ACTIVE: From-Scratch Implementation (`emap`)

**Goal**: Build our own elevation mapping package from scratch, referencing but NOT modifying the Direction 1 reference code (`src/d1/`) or the earlier from-scratch attempt (`src/terralink_elevation/`, now legacy/reference-only).

**Environment note**: this machine has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge` for ROS 2 Humble, NOT Gazebo Classic (`gazebo_ros`) - `terralink_elevation` and `src/d3` depend on Classic and won't run here as-is. `emap` targets Ignition/`ros_gz`. Also: `fuel.gazebosim.org` (Ignition Fuel) connects but stalls on downloads in this sandbox - don't add runtime/setup dependencies on it; vendor external assets locally instead (GitHub raw is reliable).

| Component | Package | Location | Status |
|-----------|---------|----------|--------|
| **Elevation Mapping** | `emap` | `src/emap/` | **Active - step 1 (UAV in Gazebo) complete, see `docs/work-docs/emap/`** |
| **Elevation Mapping (legacy)** | `terralink_elevation` | `src/terralink_elevation/` | Reference only, not under active development |
| **Semantic Vision** | `terralink_semantic` | `src/terralink_semantic/` | Not started |
| **Navigation Core** | `terralink_nav` | `src/terralink_nav/` | Not started |

**Key Principles**:
- ✅ New packages in `src/` (not `src/d<N>/`)
- ✅ Reference `src/d1/` for algorithms, NEVER modify it
- ✅ Each step verified with unit tests in `tests/elevation_mapping/`
- ✅ Concepts documented in `docs/work-logs/elevation_mapping/stepXX_*.md`
- ✅ Modular, testable, learning-focused

---

## Workspace Structure

```
terralink/
├── AGENTS.md                    # This file
├── README.md                    # Project overview
├── docs/                        # Technical documentation
│   ├── directions.md            # Three directions summary
│   ├── direction1_elevation_mapping.md    # Direction 1 deep dive
│   ├── direction2_semantic_vision.md      # Direction 2 deep dive
│   ├── direction3_opencv_prm.md           # Direction 3 deep dive
│   ├── PS.md                    # Problem statement
│   ├── elevation_map/           # Direction 1 reference docs
│   │   ├── README.md            # Quick start + density optimization
│   │   ├── concepts_from_scratch.md      # All concepts from scratch
│   │   ├── d1_elevation_mapping.md       # Complete technical guide
│   │   ├── code_deep_dive.md             # Line-by-line code walkthrough
│   │   └── integration_guide.md          # GridMap → Nav2 costmap
│   ├── uav_ugv_nav/             # Direction 3 reference docs
│   │   ├── overview.md
│   │   ├── technical_concepts.md
│   │   ├── simulation_workflow.md
│   │   ├── code_implementation.md
│   │   └── integration_guide.md
│   └── work-logs/               # OUR implementation logs
│       └── elevation_mapping/   # Step-by-step work logs
│           ├── IMPLEMENTATION_PLAN.md
│           ├── step01_project_skeleton.md
│           ├── step02_parameter_system.md
│           └── ... (one per step)
└── src/
    ├── d1/                      # Direction 1: Elevation Mapping (REFERENCE)
    │   └── elevation_mapping_gpu_ros2/    # iit-DLSLab repo (multi-package)
    │       ├── elevation_mapping_cupy/    # Main GPU mapping node (Python/CuPy)
    │       ├── elevation_map_msgs/        # Custom messages
    │       ├── plane_segmentation_ros2/   # Plane extraction
    │       └── sensor_processing/         # Semantic sensor integration
    │
    ├── d2/                      # Direction 2: Semantic Vision (REFERENCE)
    │   └── semantic_nav/        # Our ROS 2 package (C++/ONNX Runtime)
    │       ├── src/             # Nodes: segmentation, costmap converter, launcher
    │       ├── include/         # Headers
    │       ├── config/          # YAML params, class costs
    │       ├── launch/          # Launch files
    │       ├── models/          # ONNX models (yolov8n-seg.onnx)
    │       └── scripts/         # Export, benchmark scripts
    │
    ├── d3/                      # Direction 3: OpenCV PRM Baseline (WORKING)
    │   ├── my_bot/              # Main UAV-UGV package
    │   │   ├── src/             # waypoints_server, waypoints_client, nav2_handler
    │   │   ├── includes/        # GridSpace PRM implementation
    │   │   ├── config/          # nav2_params.yaml, twist_mux.yaml, etc.
    │   │   ├── launch/          # launch_sim.launch.py, navigation_launch.py
    │   │   ├── description/     # URDF/SDF/xacro (robot, UAV, cameras)
    │   │   └── worlds/          # Gazebo worlds
    │   └── tutorial_interfaces/ # GetWaypoints service definition
    │
    └── terralink_elevation/     # OUR from-scratch elevation mapping
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── elevation_mapping.yaml
        ├── launch/
        │   ├── elevation_mapping.launch.py
        │   └── elevation_nav_simulation.launch.py
        ├── terralink_elevation/
        │   ├── __init__.py
        │   ├── parameter.py              # Parameter dataclass
        │   ├── elevation_map.py          # Core ElevationMap class
        │   ├── elevation_mapping_node.py # ROS 2 node
        │   ├── costmap_converter.py      # GridMap → OccupancyGrid
        │   ├── kernels/
        │   │   ├── __init__.py
        │   │   ├── fusion_kernel.py      # Main fusion + ray tracing
        │   │   ├── drift_kernel.py       # Error counting
        │   │   └── utils.py              # Shared device functions
        │   └── utils/
        │       ├── __init__.py
        │       ├── coord_transform.py    # Internal ↔ GridMap coords
        │       └── gridmap_utils.py      # GridMap message encoding
        ├── scripts/
        │   └── synthetic_pointcloud.py   # Test data generator
        └── test/
            ├── test_parameter.py
            ├── test_elevation_map.py
            ├── test_kernels.py
            └── test_integration.py
```

---

## Build & Install

### Direction 3 (Baseline - Working)

```bash
# From repo root
colcon build --packages-select my_bot tutorial_interfaces
source install/local_setup.bash
```

### Direction 1 (Elevation Mapping) - **WORKING**

```bash
# Build elevation_mapping_cupy (Python/CuPy package)
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON

# Requires: CUDA Toolkit 12.x, CuPy 13.6.0 (pip install cupy-cuda12x==13.6.0)
# Python deps: simple_parsing, chainer, ros2_numpy, transforms3d>=0.4.2
# ROS deps: ros-humble-grid-map-msgs, ros-humble-grid-map-rviz-plugin, 
#           ros-humble-nav2-costmap-2d, ros-humble-ros2-numpy
```

### Direction 2 (Semantic Vision)

```bash
# Build semantic_nav (C++/ONNX Runtime)
colcon build --packages-select semantic_nav --cmake-args -DCMAKE_BUILD_TYPE=Release

# Requires: ONNX Runtime with CUDA/TensorRT EP
# ROS deps: ros-humble-nav2-costmap-2d, ros-humble-nav2-msgs, 
#           ros-humble-image-transport, ros-humble-cv-bridge
```

### Build All Directions (Independent)

```bash
# Each direction builds independently - no cross-dependencies
colcon build --packages-select my_bot tutorial_interfaces
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy
colcon build --packages-select semantic_nav
```

### Build Our Custom Elevation Mapping (terralink_elevation)

```bash
# From repo root
colcon build --packages-select terralink_elevation \
    --cmake-args -DBUILD_TESTING=ON

# Requires: CUDA Toolkit 12.x, CuPy 13.6.0 (pip install cupy-cuda12x==13.6.0)
# Python deps: simple_parsing, ros2_numpy, transforms3d>=0.4.2, scipy, numpy==1.24.2
# ROS deps: ros-humble-grid-map-msgs, ros-humble-grid-map-rviz-plugin, 
#           ros-humble-nav2-costmap-2d, ros-humble-ros2-numpy
```

---

## How to Run (Direction 1 - Elevation Mapping)

### Synthetic Demo (No Hardware/Gazebo) - **WORKING @ 10 Hz, 95-98% Coverage**

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Terminal 1: Synthetic pointcloud + TF + Elevation mapping
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

# Terminal 2 (optional): RViz visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
```

**Expected Output**:
```
[elevation_mapping_node]: Initialized map with length: 8.0, resolution: 0.04, cells: 202
```
GridMap publishing at **~10 Hz** on `/elevation_mapping_node/elevation_map` with layers: `elevation`, `variance`, `traversability`.
**Valid cells: 95-98%** (optimized from 26% baseline)

### Key Config Changes (Applied)

| Parameter | File | Value | Purpose |
|-----------|------|-------|---------|
| `front_only` | `scripts/synthetic_pointcloud_tf_publisher.py` | `false` | 360° FOV |
| World grid | `scripts/synthetic_pointcloud_tf_publisher.py` | 321×321 (0.05m) | 4x denser points |
| `update_pose_fps` | `config/core/core_param.yaml` | `0.0` | Fixed map center |
| `time_variance` | `config/core/core_param.yaml` | `0.00001` | 10x slower variance |
| `time_interval` | `config/core/core_param.yaml` | `0.5` | Less frequent updates |
| `enable_visibility_cleanup` | `config/core/core_param.yaml` | `false` | No false invalidations |

### Verification

```bash
ros2 topic hz /elevation_mapping_node/elevation_map
# average rate: 10.0 Hz
# Valid cells: 95-98%
```

### Build Fixes Applied

- Monkey patch for `distutils.msvccompiler` in `elevation_mapping_node.py`
- `setuptools==69.5.1` for build compatibility
- `numpy==1.24.2` for chainer compatibility

---

## How to Run (Our Custom Elevation Mapping - terralink_elevation)

### Synthetic Demo (No Hardware/Gazebo)

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Terminal 1: Synthetic pointcloud + TF + Elevation mapping
ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=false

# Terminal 2 (optional): RViz visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix terralink_elevation)/share/terralink_elevation/rviz/synthetic_demo.rviz
```

**Expected Output**:
```
[terralink_elevation]: Initialized map with length: 10.0, resolution: 0.05, cells: 202
```
GridMap publishing at **~10 Hz** on `/terralink_elevation/elevation_map` with layers: `elevation`, `variance`, `traversability`.

### Verification

```bash
ros2 topic hz /terralink_elevation/elevation_map
# average rate: 10.0 Hz
# Valid cells: 90%+ (target)
```

### Full Simulation (Gazebo + UAV + UGV + Nav2)

```bash
source install/local_setup.bash
ros2 launch terralink_elevation elevation_nav_simulation.launch.py
```

**Launch Sequence:**
| Time | Event |
|------|-------|
| 0s | Gazebo starts loading elevation-enabled world |
| 5s | UGV (my_bot) spawns |
| 10s | UAV with RGB-D camera spawns |
| ~12s | Elevation mapping starts receiving pointcloud |
| ~15s | Costmap converter publishes `/elevation_costmap` |
| ~18s | Nav2 starts with elevation-based costmap |
| 20s | Goal sent, robot navigates avoiding steep/rough terrain |

---

## How to Run (Direction 3 - Current Working Baseline)

### Full Simulation (Gazebo + UAV + UGV + Nav2 + PRM)

```bash
source install/local_setup.bash
ros2 launch my_bot launch_sim.launch.py
```

**Launch Sequence:**
| Time | Event |
|------|-------|
| 0s | Gazebo starts loading `roomWithObstacles.world` |
| 5s | Robot (my_bot) spawns via `TimerAction` |
| 15s | UAV (my_uav) spawns via `TimerAction` (staggered) |
| ~17s | Camera publishes, PRM warmup (`warming > 2`) |
| ~23s | `"[Sim] PRM graph generation complete!"` (1000 nodes) |
| 25s | `waypoints_client` timer fires, requests path |
| ~26s | Nav2 begins navigation |

**Verification Checklist:**
- [ ] Gazebo opens with maze world
- [ ] Robot and UAV visible
- [ ] Console shows PRM building progress
- [ ] `"[Sim] PRM graph generation complete!"` appears
- [ ] `"[Sim] Received valid path! Beginning waypoint navigation."` appears
- [ ] Robot moves toward goal

### Navigation Only (With Existing Map)

```bash
source install/local_setup.bash
ros2 launch my_bot navigation_launch.py
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
```

### Single Executable Testing

```bash
ros2 run my_bot waypoints_server      # Test PRM server alone
ros2 service list | grep waypoints    # Verify service available
```

---

## Key Executables

| Executable | Package | Description |
|------------|---------|-------------|
| `waypoints_server` | my_bot | UAV image → OpenCV color filter → PRM graph → GetWaypoints service |
| `waypoints_client` | my_bot | Requests path, tracks odom, publishes waypoints to `/waypoints` |
| `nav2_handler` | my_bot | Forwards `/waypoints` → Nav2 `/goal_pose` |
| `elevation_mapping_node.py` | elevation_mapping_cupy | GPU elevation mapping (Direction 1) - **Working @ 10 Hz** |
| `synthetic_pointcloud_tf_publisher.py` | elevation_mapping_cupy | Synthetic data generator (Direction 1) |
| `semantic_segmentation_node` | semantic_nav | YOLOv8-seg ONNX inference (Direction 2) |
| `costmap_converter_node` | semantic_nav | Semantic classes → Nav2 costmap (Direction 2) |

---

## Critical Workflow Order (Direction 3)

1. **Build first**: `colcon build --packages-select my_bot tutorial_interfaces`
2. **Launch simulation**: `ros2 launch my_bot launch_sim.launch.py`
   - Launches: Gazebo → Robot → UAV → waypoints_server → waypoints_client → Nav2
   - **DO NOT** run `navigation_launch.py` separately - `/odom` and costmap topics won't exist

---

## Package Details

### Direction 3: `src/d3/my_bot/`

**Source Files:**
- `src/waypoints_server.cpp` - Image callback, PRM building, service handler
- `src/waypoints_client.cpp` - Path request, odom tracking, waypoint publishing (**needs stuck detection fix**)
- `src/nav2_handler.cpp` - Simple `/waypoints` → `/goal_pose` forwarder
- `includes/processImage.h` - GridSpace class declaration (PRM)
- `includes/processImage.cpp` - GridSpace implementation (A*, line-of-sight, coord conversion)

**Config:**
- `config/nav2_params.yaml` - Full Nav2 stack config (AMCL, DWB, Navfn, costmaps)
- `config/twist_mux.yaml` - Velocity multiplexer (nav priority 10, joy 100)
- `config/gazebo_params.yaml` - Gazebo physics/sensors

**Launch:**
- `launch/launch_sim.launch.py` - Full stack (Gazebo, spawns, Nav2, PRM nodes)
- `launch/navigation_launch.py` - Nav2 only (requires map)
- `launch/spawn_uav.launch.py` - UAV spawn at 15s (SDF file)
- `launch/rsp.launch.py` / `rsp_uav.launch.py` - Robot state publishers

**Description:**
- `description/uav.sdf` - UAV model (spawned with `-file` flag)
- `description/camera_uav.xacro` - Downward RGB camera config
- `description/robot.urdf.xacro` - Ground robot (diff drive, LiDAR)

### Direction 3: `src/d3/tutorial_interfaces/`

**Service:** `srv/GetWaypoints.srv`
```
geometry_msgs/Point start
geometry_msgs/Point goal
---
bool valid
geometry_msgs/PoseStamped[] waypoints
```

---

## ROS Topics (Direction 3)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| UAV → Server | `/my_uav/camera_uav/image_raw` | sensor_msgs/msg/Image | Overhead RGB camera |
| Server ↔ Client | `waypoints_service` | tutorial_interfaces/srv/GetWaypoints | Start+goal → waypoints |
| Client → Handler | `/waypoints` | geometry_msgs/msg/PoseStamped | Waypoint sequence |
| Handler → Nav2 | `/goal_pose` | geometry_msgs/msg/PoseStamped | Nav2 navigation goal |
| UGV → Client | `/odom` | nav_msgs/msg/Odometry | Robot pose for arrival detection |
| Joystick → Mux | `/cmd_vel_joy` | geometry_msgs/msg/Twist | Optional teleop |

---

## Services

### `/waypoints_service` (tutorial_interfaces/srv/GetWaypoints)

- **Request**: start{x,y,z}, goal{x,y,z} (world coordinates, meters)
- **Response**: valid<bool>, waypoints[PoseStamped[]] (world coordinates)
- **PRM Warmup**: First ~2 camera frames (`warming > 2` check)
- **Before warmup**: Returns `valid=false`, logs `"PRM graph is still generating"`
- **After warmup**: Returns valid path instantly

---

## Codegen / Generated Code

- `tutorial_interfaces` generates C++ headers for `GetWaypoints` service during `colcon build`
- **Do not manually edit** `src/d3/tutorial_interfaces/srv/GetWaypoints.srv` without rebuild
- Direction 1 uses `elevation_map_msgs` (custom GridMap-based messages)
- Direction 2 uses standard `sensor_msgs/Image`, `nav_msgs/OccupancyGrid`

---

## Testing

### Lint (All Packages)

```bash
colcon test --packages-select my_bot tutorial_interfaces semantic_nav elevation_mapping_cupy \
    --event-handlers console_direct+ --return-code 0
```

### Direction 3 Integration Test

```bash
# 1. Launch simulation
ros2 launch my_bot launch_sim.launch.py

# 2. Verify logs
# [Sim] PRM graph generation complete!
# [Sim] Received valid path! Beginning waypoint navigation.

# 3. Manual service test
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
# Should return valid=true with waypoint array
```

### Direction 1 Tests

```bash
# Unit tests (no ROS) - pass with Chainer backend
cd src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest elevation_mapping_cupy/tests/test_parameter.py elevation_mapping_cupy/tests/test_repo_config_sanity.py -v

# Integration tests (requires ROS)
colcon test --packages-select elevation_mapping_cupy --event-handlers console_direct+
```

### Our Custom Elevation Mapping Tests (terralink_elevation)

```bash
# Unit tests (no ROS) - fast verification per step
cd tests/elevation_mapping
python -m pytest test_step01_skeleton.py -v
python -m pytest test_step02_parameters.py -v
python -m pytest test_step03_data_structures.py -v
python -m pytest test_step04_fusion_cpu.py -v
python -m pytest test_step05_fusion_gpu.py -v
python -m pytest test_step06_ray_tracing.py -v
python -m pytest test_step07_map_shifting.py -v
python -m pytest test_step08_drift_comp.py -v
python -m pytest test_step09_traversability.py -v

# Integration tests (requires ROS + GPU)
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select terralink_elevation --event-handlers console_direct+

# Run specific test
ros2 run terralink_elevation test_integration.py
```

---

## Known Quirks (Direction 3)

| Issue | Root Cause | Workaround/Fix |
|-------|-----------|----------------|
| PRM warmup delay | `warming > 2` needs 2 frames | Wait ~2s after UAV spawn |
| Robot gets stuck | No stuck detection in client | **Fix: Add watchdog + progress tracking** |
| Nav2 "out of bounds" | Costmap not ready during warmup | Normal, resolves after PRM complete |
| UAV spawn deadlock | Simultaneous spawn_entity calls | Fixed: TimerAction (5s robot, 15s UAV) |
| Camera QoS mismatch | Default QoS vs Gazebo plugin | Fixed: `rclcpp::SensorDataQoS()` |
| No path retry | Client warns but doesn't retry | **Fix: Implement request_new_path()** |
| Fixed color threshold | Hardcoded 100-180 gray range | Only works in simulation |

---

## Direction 3 Fix Plan (Priority)

1. **HIGH**: Add stuck detection to `waypoints_client.cpp`
   - 5s watchdog timer
   - Progress tracking via odom
   - Auto re-plan on stall

2. **HIGH**: Add path retry logic
   - `request_new_path()` method
   - Call on invalid response or stall

3. **MEDIUM**: Orientation in waypoints
   - PRM returns directed edges → compute yaw
   - Update `nav2_handler` to use it

4. **MEDIUM**: Adaptive color thresholding
   - HSV color space
   - Floor color learning from first frames

---

## Documentation Reference

| Document | Purpose |
|----------|---------|
| `docs/directions.md` | Three directions summary |
| `docs/direction1_elevation_mapping.md` | Direction 1: Complete technical guide |
| `docs/direction2_semantic_vision.md` | Direction 2: Complete technical guide |
| `docs/direction3_opencv_prm.md` | Direction 3: Complete technical guide |
| `docs/elevation_map/README.md` | Direction 1: Quick start + index |
| `docs/elevation_map/code_deep_dive.md` | Direction 1: Line-by-line code walkthrough |
| `docs/elevation_map/integration_guide.md` | Direction 1: GridMap → Nav2 costmap |
| `docs/uav_ugv_nav/overview.md` | Direction 3: Architecture + data flow + coord systems |
| `docs/uav_ugv_nav/technical_concepts.md` | Direction 3: PRM, Nav2, TF, A*, OpenCV (beginner-friendly) |
| `docs/uav_ugv_nav/simulation_workflow.md` | Direction 3: Launch, verify, troubleshoot, debug |
| `docs/uav_ugv_nav/code_implementation.md` | Direction 3: Line-by-line code walkthrough |
| `docs/uav_ugv_nav/integration_guide.md` | Direction 3: Extending, sensor fusion, real robot deployment |

---

## Development Guidelines

### Modularity Rules

1. **Each direction is independent** - No cross-directory dependencies
2. **Build separately** - Use `--packages-select` for each direction
3. **Separate namespaces** - Direction 1: `elevation_mapping`, Direction 2: `semantic_nav`, Direction 3: `my_bot`
4. **Shared interfaces only** - Common: `geometry_msgs`, `nav_msgs`, `sensor_msgs`

### Adding New Direction

1. Create `src/d4/new_approach/`
2. Add package(s) with unique names
3. Document in `docs/direction4_*.md`
4. Update this AGENTS.md

### Code Style

- C++17, ROS 2 Humble conventions
- No raw pointers - use `shared_ptr`/`unique_ptr`
- `rclcpp::SensorDataQoS()` for sensor subscriptions
- Parameters via YAML + `declare_parameter()`

---

## From-Scratch Implementation Guidelines (TerraLink Custom)

### Package Creation Rules

1. **New packages in `src/` root** - NOT in `src/d<N>/` (e.g., `src/terralink_elevation/`)
2. **Reference `src/d1/` for algorithms** - Study, understand, NEVER copy-paste
3. **Each step = testable module** - Unit test before integration
4. **Document everything** - Work-logs in `docs/work-logs/` with concept explanations
5. **CPU-first, GPU-second** - Verify logic on NumPy, then port to CuPy

### Testing Discipline

1. **Unit test per step** - `tests/elevation_mapping/test_stepXX_*.py`
2. **No ROS in unit tests** - Pure algorithm verification
3. **GPU vs CPU numerical match** - Verify kernel output matches CPU reference
4. **Integration test with ROS** - After all unit tests pass

### Documentation Standards

1. **Concept-first explanations** - Math, intuition, then code
2. **Reference line numbers** - Link to `src/d1/...` exact lines
3. **Work-log per step** - What, why, how, pitfalls
4. **Examples** - Synthetic data, expected outputs

### Code Architecture

```
terralink_elevation/
├── parameter.py              # Single source of truth (dataclass + YAML)
├── elevation_map.py          # Core logic (clean, documented)
├── kernels/                  # CuPy ElementwiseKernels (GPU)
│   ├── fusion_kernel.py      # Main fusion + ray tracing
│   ├── drift_kernel.py       # Error counting
│   └── utils.py              # Shared device functions
├── utils/                    # Helpers (no GPU)
│   ├── coord_transform.py    # Internal ↔ GridMap coords
│   └── gridmap_utils.py      # GridMap message encoding
├── elevation_mapping_node.py # ROS 2 node (thin wrapper)
├── costmap_converter.py      # GridMap → OccupancyGrid
└── test/                     # Unit tests per module
```

---
 
## Validation Results: Gaussian Bump Ground Truth
 
**Test Setup**: Synthetic Gaussian bump ground truth (1m height, 2m sigma) with horizontal LiDAR simulation
 
| Config | Resolution | Bump | Sigma | **RMSE** | **MAE** | **Coverage** | **Rel RMSE** |
|--------|------------|------|-------|----------|---------|--------------|--------------|
| **SteepBump** | 0.1m | 2.0m | 1.0m | **0.089m** | 0.069m | **45.1%** | **4.4%** ⭐ Best |
| **ShallowWide** | 0.1m | 0.5m | 3.0m | **0.095m** | 0.074m | 45.2% | 9.2% |
| **Default** | 0.1m | 1.0m | 2.0m | **0.093m** | 0.072m | 45.2% | 9.3% |
| **HighRes** | 0.05m | 1.0m | 2.0m | 0.239m | 0.209m | 36.0% | 23.9% |
 
### Key Findings:
1. **Accuracy is good**: RMSE ~9-10cm for typical configurations (Default, ShallowWide, SteepBump)
2. **Coverage improved**: 45% valid cells (fixed sensor simulation)
3. **SteepBump** has best relative accuracy (4.4% relative RMSE) - steeper features are easier to resolve
4. **HighRes** performs worse due to sensor noise scaling with resolution
 
**Validation Test Location**: `tests/elevation_mapping/test_validation_gaussian.py`
 
Run: `PYTHONPATH=/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages python3 tests/elevation_mapping/test_validation_gaussian.py`
 
---
 
## Quick Reference Commands
 
```bash
# Build Direction 3 (baseline)
colcon build --packages-select my_bot tutorial_interfaces
 
# Build Direction 1 (elevation) - WORKING
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON
 
# Build Direction 2 (semantic)
colcon build --packages-select semantic_nav --cmake-args -DCMAKE_BUILD_TYPE=Release
 
# Build Our Custom Elevation Mapping (terralink_elevation)
colcon build --packages-select terralink_elevation \
    --cmake-args -DBUILD_TESTING=ON
 
# Run Direction 1 - Synthetic Demo (WORKING @ 10 Hz)
source install/local_setup.bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false
 
# Run Our Custom Elevation Mapping - Synthetic Demo
source install/local_setup.bash
ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=false
 
# Run Direction 3 simulation
source install/local_setup.bash
ros2 launch my_bot launch_sim.launch.py
 
# Run Full Simulation (UAV + UGV + Elevation Mapping + Nav2)
source install/local_setup.bash
ros2 launch terralink_elevation elevation_nav_simulation.launch.py
 
# Check topics
ros2 topic list | grep -E "(camera|waypoint|goal|odom|elevation)"
 
# Check services
ros2 service list | grep waypoints
 
# View PRM graph building logs
ros2 log get waypoints_server | grep -E "(Building|complete|Valid)"
 
# RViz for Direction 1
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
 
# RViz for Our Custom Elevation Mapping
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix terralink_elevation)/share/terralink_elevation/rviz/synthetic_demo.rviz
```
---

## Important insutructions for agents
- Make sure the codebase is well maintained and follows best practices interms of modularity, readability, documentation, and performance.
- ALWAYS propose a plan for implementation before starting any task and get approval before proceeding.
- Include all the technical details in the implementation plan, including any dependencies, libraries, or frameworks that will be used. You can also include important code snippets or pseudocode to illustrate your approach.
- Every time a task is completed, ensure that the code is properly tested with unit tests and integration tests.
- NEVER create test or debug related files in root directory. All test files should be placed in the tests/ directory.
- NEVER push code to git unless explicitly instructed to do so.
- After every task, update the documentation to reflect any changes made to the codebase.
- Provide high-level documentation and detailed explanations (including technical details) separately in the docs/ in a very modular way.
- Provide references to code snippets in the documentation wherever necessary.
- NEVER assume anything and always ask for clarifications if any requirements or details are unclear.
- ALWAYS use best practices for code quality, including but not limited to code reviews, static analysis, and adherence to coding standards.
- Provide detailed EXPLANATIONS for all the concepts required to understand the codebase in form of documentation from SCRATCH and in a simple way such that beginners can also understand the concepts properly.
- For concept explanations use `docs/learning/` and explain the detailed explanation for all concepts here including any pre-requisite concepts required to understand the codebase. 
- Use coding examples wherever necessary to explain the concepts in a simple way.
- If you lack priviliges to perform any task, inform the user and provide a detailed explanation of the steps the user to perform to complete the task.
- When asked to debug or solve any issue related to simulation and incase you need to run the simulation run it headlessly and use the output logs for further analysis. Do not run any simulation in GUI mode unless neccessary. But at the same time u need to make sure things are working perfectly for simulation as well.

---
 
*This AGENTS.md reflects the modular three-direction architecture. Each direction in `src/d<N>/` builds and runs independently. Direction 3 is the current working baseline.*

