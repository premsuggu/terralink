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

**Package**: `emap` (`src/emap/`) - the current, active from-scratch elevation-mapping rebuild. An earlier from-scratch attempt, `terralink_elevation` (Gazebo Classic-based, never verified working), has been removed now that `emap` fully supersedes it - see git history if ever needed for reference.

**Why a rebuild**: this environment has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge`, not Gazebo Classic (`gazebo_ros`) which `src/d3` depends on. `emap` targets the stack that's actually installed here.

**Status**: see `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for the current, up-to-date step-by-step status (kept current there, unlike this line).

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch emap uav_sim.launch.py          # headless by default; headless:=false for GUI
```

Roadmap and per-step write-ups: `docs/work-docs/emap/`.

### Controlling the UAV

**The drone does not fly on its own.** It's velocity-controlled: Gazebo's `MulticopterVelocityControl` plugin (see `step01_uav_gazebo_deployment.md`) constantly asks "what body-frame velocity was I just told to fly at?" and holds that - with nothing publishing to `/cmd_vel`, the commanded velocity is zero, so it just sits on the ground under gravity. This is normal for a velocity-controlled vehicle (a real drone does the same with no stick input) - it's not a bug, and nothing else in the simulation is supposed to move it by itself.

To actually fly it, publish `geometry_msgs/msg/Twist` messages to `/cmd_vel` (linear x/y/z in m/s, angular z for yaw rate, all in the drone's own body frame):

```bash
# One-shot: climb, then hold that velocity until told otherwise
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.6}}"

# Hover (zero velocity) once at altitude
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.0}}"

# Continuous manual control from the keyboard (ros-humble-teleop-twist-keyboard,
# already installed) - a terminal UI that publishes /cmd_vel as you press keys
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Is this critical right now? No.** Manual `/cmd_vel` commands are enough to test the sensor/TF/mapping pipeline (exactly how steps 1-2 were verified) - none of the mapping algorithm work (steps 3-4, pure CPU code) depends on the drone moving by itself at all. Real autonomy - flying a survey pattern, or a planner publishing `/cmd_vel` on the UAV's behalf - is a separate, later concern once the mapping pipeline itself is wired into a live node and there's an actual map worth flying to complete.

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
colcon build --packages-select emap

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

### `emap` (Our Active Elevation Mapping)

```bash
# With RViz
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump

# Headless, no RViz
ros2 launch emap uav_sim.launch.py headless:=true
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
├── src/
│   ├── d1/elevation_mapping_gpu_ros2/    # Direction 1 reference (iit-DLSLab)
│   ├── d2/semantic_nav/                  # Direction 2 skeleton
│   ├── d3/                               # Direction 3 baseline
│   │   ├── my_bot/
│   │   └── tutorial_interfaces/
│   └── emap/                             # OUR active from-scratch elevation mapping
│       ├── package.xml
│       ├── config/elevation_mapping.yaml
│       ├── launch/, models/, worlds/, rviz/
│       └── emap/                         # elevation_map, fusion(+gpu), drift, traversability, ...
└── tests/emap/                  # Unit tests for emap
```

---

## References

- **Elevation Mapping Repo**: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- **Grid Map Library**: https://github.com/ANYbotics/grid_map
- **CuPy**: https://cupy.dev/
- **Nav2**: https://github.com/ros-planning/navigation2
- **Grid Map RViz Plugin**: https://github.com/ANYbotics/grid_map_rviz_plugin