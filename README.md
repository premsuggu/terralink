# TerraLink - Collaborative UAV-UGV Navigation Framework

ROS 2 Humble heterogeneous robotics framework for unstructured environments (construction sites, disaster response). UAV provides global overhead awareness; UGV executes ground tasks with payload capacity.

## Three Technical Directions

| Direction | Approach | Sensor | Status | Folder |
|-----------|----------|--------|--------|--------|
| **1. Geometric** | 2.5D Elevation Mapping (RGB-D/3D LiDAR → GPU elevation grid → traversability costmap) | RGB-D / 3D LiDAR | **Working** (synthetic demo) | `src/d1/elevation_mapping_gpu_ros2/` |
| **2. Semantic** | AI Vision to Costmap (RGB camera → YOLOv8-seg → semantic classes → Nav2 costmap) | RGB Camera | Skeleton | `src/d2/semantic_nav/` |
| **3. Baseline** | OpenCV Color Filtering + PRM (RGB camera → color threshold → binary grid → PRM waypoints) | RGB Camera | Working in Gazebo | `src/d3/my_bot/` |

---

## ACTIVE: `emap` - From-Scratch Elevation Mapping (Ignition Gazebo)

**Package**: `emap` (`src/emap/`) - the current, active from-scratch elevation-mapping rebuild. Supersedes `terralink_elevation` below as the line of active work (that package is kept as a reference, untouched).

**Why a rebuild**: this environment has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge`, not Gazebo Classic (`gazebo_ros`) which `terralink_elevation` and `src/d3` depend on. `emap` targets the stack that's actually installed here.

**Status**: Step 1 complete - a physically-simulated, ROS 2-controllable quadrotor (`iris_quad`, geometry from the open-source PX4 Iris model, flown with Ignition's native multicopter plugins) spawns and flies in Gazebo.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch emap uav_sim.launch.py          # headless by default; headless:=false for GUI
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.6}}"   # climb
```

Roadmap and per-step write-ups: `docs/work-docs/emap/`.

---

## LEGACY: Direction 1 Custom - From-Scratch Elevation Mapping (terralink_elevation)

Kept as reference only - no longer under active development, see `emap` above.

**Package**: `terralink_elevation` (`src/terralink_elevation/`) - Our from-scratch implementation referencing but NOT modifying the reference implementation.

**Purpose**: Learn and validate 2.5D elevation mapping concepts from scratch, referencing but NOT copying the reference implementation in `src/d1/elevation_mapping_gpu_ros2/`.

**Status**: Core algorithm implemented and validated (RMSE ~9cm), synthetic demo working at 10Hz, coordinate transformation for downward-facing camera needs refinement.

### Run Custom Synthetic Demo (No Hardware)

```bash
source /opt/ros/humble/setup.bash
source /home/prem/terralink/install/local_setup.bash

# Terminal 1: Synthetic pointcloud + TF + Elevation mapping node + RViz
ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=true
```

**Expected**: GridMap at 2Hz on `/elevation_mapping_node/elevation_map` with layers: `elevation`, `variance`, `is_valid`, `traversability`. RViz shows rainbow elevation map + UAV marker + TF display.

### Key Implementation Details

| Component | Status | Notes |
|-----------|--------|-------|
| Bayesian Fusion (CPU) | ✅ Working | Mahalanobis outlier rejection, sensor noise model |
| Map Shifting | ✅ Working | UAV-centric, correct coordinate conventions |
| Drift Compensation | ✅ Working | Bias correction from low-variance cells |
| Visibility Cleanup | ✅ Working | Ray tracing for downward-facing camera |
| Traversability Analysis | ✅ Working | Slope/step/roughness classification |
| GPU Acceleration (CuPy) | ⏸️ Deferred | Kernel compilation issues, CPU validated first |
| Downward Camera Coord Transform | ⚠️ Needs Fix | Elevation values currently zero |

### Key Fixed Issues

| Issue | Fix |
|-------|-----|
| TF transform direction | Fixed: `msg.header.frame_id → map_frame` (sensor→map) |
| QoS mismatch | Fixed: Both pub/sub use `sensor_data` QoS |
| RViz intensity bounds | Fixed: `Max Intensity: 1.0` for 0-0.4m range |
| Coordinate conventions | Fixed: `move_to` signature matches reference |
| Downward camera transform | Added: pitch=-90° in TF + point cloud transform |

### Validation Results

| Config | RMSE | Rel RMSE | Coverage |
|--------|------|----------|----------|
| Default (1m bump, σ=2m) | 0.093m | 9.3% | 45% |
| SteepBump (2m bump, σ=1m) | **0.088m** | **4.4%** | 45% |
| ShallowWide (0.5m bump, σ=3m) | 0.096m | 19% | 45% |

---

## Quick Start

### Prerequisites (One-time)

```bash
# ROS 2 Humble + build tools
sudo apt update && sudo apt install -y \
    ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Direction 1: Elevation Mapping (GPU)
# 1. Install CUDA Toolkit 12.x (for libnvrtc.so.12)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update && sudo apt-get install -y cuda-toolkit-12-0

# 2. Python deps (CuPy, Chainer, etc.)
pip install cupy-cuda12x==13.6.0 simple_parsing chainer ros2_numpy
pip install --upgrade transforms3d

# 3. ROS deps
sudo apt install -y \
    ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
    ros-humble-nav2-costmap-2d ros-humble-ros2-numpy \
    python3-scipy python3-numpy python3-opencv
```

### Build All Directions

```bash
cd /home/prem/terralink
source /opt/ros/humble/setup.bash

# Build independently (no cross-dependencies)
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy --cmake-args -DBUILD_TESTING=ON
colcon build --packages-select my_bot tutorial_interfaces
colcon build --packages-select semantic_nav --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select terralink_elevation --cmake-args -DBUILD_TESTING=ON

source install/local_setup.bash
```

---

## Run Commands

### Direction 1: Reference Elevation Mapping (Working)

```bash
# Synthetic demo (no Gazebo)
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

# RViz visualization
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
```

### Custom Terralink Elevation Mapping (Our Implementation)

```bash
# Synthetic demo with RViz
ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=true

# Headless
ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=false
```

### Direction 3: OpenCV PRM Baseline (Gazebo)

```bash
source /opt/ros/humble/setup.bash
source /home/prem/terralink/install/local_setup.bash

# Full simulation (Gazebo + UAV + UGV + Nav2 + PRM)
ros2 launch my_bot launch_sim.launch.py
```

**Timeline**: 0s Gazebo → 5s Robot → 15s UAV → ~23s PRM complete → 25s Nav2 starts

---

## Project Structure

```
terralink/
├── README.md                    # This file
├── AGENTS.md                    # Agent instructions
├── docs/
│   ├── SETUP.md                 # One-time environment setup per direction
│   ├── RUN.md                   # Runtime commands per direction
│   ├── directions.md            # Three directions summary
│   ├── elevation_map/           # Direction 1 deep docs
│   │   ├── README.md
│   │   ├── d1_elevation_mapping.md
│   │   ├── code_deep_dive.md
│   │   └── integration_guide.md
│   ├── uav_ugv_nav/             # Direction 3 legacy docs
│   └── segmentation_map/        # Direction 2 (future)
├── launch/
│   ├── elevation_mapping.launch.py      # Custom synthetic demo
│   └── elevation_gazebo_simulation.launch.py  # Gazebo sim (optional)
├── src/
│   ├── d1/elevation_mapping_gpu_ros2/    # Direction 1 reference (iit-DLSLab)
│   ├── d2/semantic_nav/                  # Direction 2 skeleton
│   ├── d3/                               # Direction 3 baseline
│   │   ├── my_bot/
│   │   └── tutorial_interfaces/
│   └── terralink_elevation/              # OUR from-scratch implementation
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/elevation_mapping.yaml
│       ├── launch/
│       ├── rviz/elevation_mapping.rviz
│       ├── scripts/
│       ├── terralink_elevation/
│       └── terralink_elevation.egg-info
└── tests/elevation_mapping/    # Unit tests for custom implementation
```

---

## References

- **Elevation Mapping Repo**: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- **Grid Map Library**: https://github.com/ANYbotics/grid_map
- **CuPy**: https://cupy.dev/
- **Nav2**: https://github.com/ros-planning/navigation2
- **Grid Map RViz Plugin**: https://github.com/ANYbotics/grid_map_rviz_plugin