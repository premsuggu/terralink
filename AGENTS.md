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

**Goal**: Build our own elevation mapping package from scratch, referencing but NOT modifying the Direction 1 reference code (`src/d1/`). An earlier from-scratch attempt, `src/terralink_elevation/` (Gazebo Classic-based, never verified working), was removed entirely once `emap` fully superseded it - see git history if the old code is ever needed for reference.

**Environment note**: this machine has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge` for ROS 2 Humble, NOT Gazebo Classic (`gazebo_ros`) - `src/d3` depends on Classic and won't run here as-is. `emap` targets Ignition/`ros_gz`. Also: `fuel.gazebosim.org` (Ignition Fuel) connects but stalls on downloads in this sandbox - don't add runtime/setup dependencies on it; vendor external assets locally instead (GitHub raw is reliable).

| Component | Package | Location | Status |
|-----------|---------|----------|--------|
| **Elevation Mapping** | `emap` | `src/emap/` | **Active - see `docs/work-docs/emap/`** |
| **Navigation Core** | `nav` | `src/nav/` | **Active - see `docs/work-docs/nav/`** |
| **Semantic Vision** | `terralink_semantic` | `src/terralink_semantic/` | Not started |

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
    └── nav/                       # OUR navigation core (active)
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── config/
        │   ├── nav2_params.yaml
        │   └── bridge.yaml
        ├── launch/
        │   └── nav_sim.launch.py
        ├── models/nav_ugv/        # UGV SDF model
        ├── worlds/room_maze.world
        ├── resource/
        └── nav/
            ├── prm_planner.py         # PRM path planning
            ├── autopilot.py           # UAV autonomous flight logic
            ├── uav_autopilot_node.py  # UAV autopilot ROS node
            ├── planner_node.py        # PRM planner ROS node
            ├── waypoint_follower.py   # UGV waypoint following
            └── walkability.py         # Traversability analysis
See `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for the step-by-step build history.

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

### Build `nav` (our navigation core package)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav
```
See `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` for full build/run/test instructions - it's kept current as the package evolves.

---

### Build `emap` (our active elevation mapping package)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select emap
```
See `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for full build/run/test instructions - it's kept current as the package evolves, unlike this section.

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

## How to Run (`emap` - our active elevation mapping)

```bash
source /opt/ros/humble/setup.bash
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump
```
Full launch args, verification steps, and expected output are documented (and kept current) in `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` and its per-step docs - refer there rather than this file for exact commands/output, which will otherwise drift out of date as `emap` evolves.

---

## How to Run (`nav` - our navigation core)

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true
```

**Launch Arguments:**
| Arg | Default | Description |
|-----|---------|-------------|
| `headless` | `true` | Run gz sim server-only (no GUI) |
| `autonomous_uav` | `false` | UAV flies patrol waypoints to build map |
| `goal_x` | `1.7` | UGV goal X (world frame) |
| `goal_y` | `-0.5` | UGV goal Y (world frame) |

**Launch Sequence (see `nav_sim.launch.py` docstring for full rationale):**
| Time | Event |
|------|-------|
| 0s | Gazebo starts (`room_maze.world`) |
| 5s | Bridges, TFs, elevation mapping, UAV autopilot start |
| 9s | Nav2, PRM planner, UGV waypoint follower start |

**Verification Checklist:**
- [ ] Gazebo loads `room_maze.world`
- [ ] UAV (`iris_quad`) and UGV (`nav_ugv`) visible
- [ ] `/elevation_mapping_node/elevation_map` publishing GridMap
- [ ] Nav2 costmap receives UGV lidar scans (no TF drops)
- [ ] PRM planner finds path through traversable cells
- [ ] UGV follows waypoints to goal

Full details in `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` and per-step docs.

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
| `planner_node` | nav | PRM planner ROS node |
| `uav_autopilot_node` | nav | UAV autopilot ROS node |
| `waypoint_follower` | nav | UGV waypoint following |

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

### `nav` Tests (our navigation core)

```bash
cd tests/nav
python3 -m pytest -v
```
All algorithm-level tests (PRM planner, autopilot, walkability) live here and require no ROS/Gazebo - see `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` for what each covers.

---

### `emap` Tests (our active elevation mapping)

```bash
cd tests/emap
python3 -m pytest -v
```
All algorithm-level tests (fusion, GPU fusion, drift compensation, traversability, map shifting) live here and require no ROS/Gazebo - see `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for what each covers.

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

1. **New packages in `src/` root** - NOT in `src/d<N>/` (e.g., `src/emap/`)
2. **Reference `src/d1/` for algorithms** - Study, understand, NEVER copy-paste
3. **Each step = testable module** - Unit test before integration
4. **Document everything** - Work-logs in `docs/work-logs/` with concept explanations

### Testing Discipline

1. **Unit test per step** - `tests/emap/test_*.py`
2. **No ROS in unit tests** - Pure algorithm verification
3. **Integration test with ROS** - After all unit tests pass

### Documentation Standards

1. **Concept-first explanations** - Math, intuition, then code
2. **Reference line numbers** - Link to `src/d1/...` exact lines
3. **Work-log per step** - What, why, how, pitfalls
4. **Examples** - Synthetic data, expected outputs

See the earlier directory tree (`emap/` under "ACTIVE: From-Scratch Implementation") for the current code architecture.

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
 
# Build emap (our active elevation mapping)
colcon build --packages-select emap

# Build nav (our navigation core)
colcon build --packages-select nav

# Run Direction 1 - Synthetic Demo (WORKING @ 10 Hz)
source install/local_setup.bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

# Run emap
source install/local_setup.bash
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump

# Run nav
source install/local_setup.bash
ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true

# Run Direction 3 simulation
source install/local_setup.bash
ros2 launch my_bot launch_sim.launch.py
 
# Check topics
ros2 topic list | grep -E "(camera|waypoint|goal|odom|elevation)"
 
# Check services
ros2 service list | grep waypoints
 
# View PRM graph building logs
ros2 log get waypoints_server | grep -E "(Building|complete|Valid)"
 
# RViz for Direction 1
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
 
# RViz for emap (or pass launch_rviz:=true to the launch file above)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix emap)/share/emap/rviz/elevation_mapping.rviz

# RViz for nav (or use launch_rviz:=true in nav_sim.launch.py if added)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav)/share/nav/rviz/nav.rviz
```
---


## Agent Guidelines

### Code Quality & Maintenance
- Ensure the codebase is well-maintained and follows best practices in terms of modularity, readability, documentation, and performance.
- Always use best practices for code quality, including but not limited to:
    - Code reviews before merging changes
    - Static analysis tools for linting and type checking
    - Adherence to established coding standards and conventions
- Use coding examples wherever necessary to explain concepts in a simple and understandable way.

### Planning & Implementation Process
- **Before starting any task:** Always propose a detailed implementation plan and obtain explicit approval before proceeding.
- **Implementation plan must include:**
    - Clear description of the task objectives and scope
    - All technical details (algorithms, data structures, design patterns)
    - Complete list of dependencies, libraries, and frameworks that will be used
    - Important code snippets or pseudocode to illustrate the approach
    - Estimated timeline and potential risks or challenges
    - After each implementation step, provide a summary of the changes made and any relevant code snippets or examples to illustrate the modifications.

### Testing & Quality Assurance
- After completing any task, ensure the code is properly tested with:
    - Unit tests for individual components and functions
    - Integration tests to verify interactions between modules
    - All tests should pass before considering a task complete
- **Important:** Never create test or debug related files in the root directory. All test files must be placed in the `tests/` directory with appropriate subdirectories matching the source structure.

### Version Control & Deployment
- **Never push code to git independently.** Only push code to git after obtaining explicit approval from the user.
- Wait for user confirmation at each milestone before committing changes to the repository.

### Documentation Standards
- After each completed task, update the documentation to reflect all changes made to the codebase.
- Provide documentation in a modular way:
    - High-level documentation explaining overall concepts and workflows
    - Detailed explanations with in-depth technical details
    - All documentation should be placed in the `docs/` directory with clear organization
    - Explanatory docs and work logs shall be maintained separately in modular way.
- Always include references to relevant code snippets in the documentation.
- Provide both conceptual explanations and practical examples to help users understand the codebase better. Provide clear explanations such that even a new developer can understand the codebase and its functionality.
- I have created separate directories for GNN and TabNet documentation in the `docs/` folder. This is where we will maintain all the relevant documentation for these components.

### Communication & Clarification
- Never assume anything about requirements or implementation details.
- Always ask for clarifications if any requirements, specifications, or details are unclear.
- Request explicit approval before making architectural decisions or significant changes.

### Dependency Management
- Never install unnecessary packages or libraries in the system without explicit user permission.
- If you have installed packages without prior permission, immediately inform the user with:
    - A list of packages that were installed
    - Reasons why they were needed
    - A request for permission to keep or remove them
- Follow up accordingly based on user feedback.

### Privilege & Capability Limitations
- If you lack the necessary privileges to perform any task, inform the user immediately.
- Provide a detailed explanation of:
    - What task requires elevated privileges
    - Why those privileges are needed
    - Step-by-step instructions for the user to complete the task manually

---