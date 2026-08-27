# SETUP.md - Complete Environment Setup for TerraLink

## Overview

This document covers **one-time environment setup** for all TerraLink directions. Run the sections relevant to your direction(s).

---

## Common Prerequisites (All Directions)

```bash
# 1. ROS 2 Humble (Ubuntu 22.04)
source /opt/ros/humble/setup.bash

# 2. Build tools
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

# 3. Initialize rosdep (if not done)
sudo rosdep init
rosdep update
```

---

## Direction 1: Elevation Mapping (2.5D GPU)

### Location
`src/d1/elevation_mapping_gpu_ros2/` (iit-DLSLab fork)

### System Dependencies

```bash
# ROS 2 packages
sudo apt install -y \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-nav2-costmap-2d \
    ros-humble-ros2-numpy \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-rosbag2-py

# Python scientific stack
sudo apt install -y \
    python3-scipy \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-yaml \
    python3-shapely

# CuPy (GPU acceleration) - MATCH YOUR CUDA VERSION
# Check: nvcc --version
# CUDA 11.8:
pip install cupy-cuda11x
# CUDA 12.1:
pip install cupy-cuda12x
# CUDA 12.2+:
pip install cupy-cuda12x

# Optional: ONNX Runtime for semantic sensor
pip install onnxruntime-gpu
```

### Verify GPU Setup

```bash
# Check CUDA
nvcc --version
nvidia-smi

# Test CuPy import
python3 -c "import cupy; print(cupy.cuda.runtime.getDeviceCount())"
# Should print >= 1
```

### Build Direction 1

```bash
cd /home/prem/terralink

# Install ROS deps via rosdep
rosdep install --from-paths src/d1/elevation_mapping_gpu_ros2 --ignore-src -r -y

# Build elevation_map_msgs first (dependency)
colcon build --packages-select elevation_map_msgs

# Build main package with tests
colcon build --packages-select elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON

# Source workspace
source install/local_setup.bash
```

### Verify Build

```bash
# Check packages found
colcon list | grep elevation

# Run unit tests (no ROS needed)
cd src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -v

# Check executables
ros2 pkg executables elevation_mapping_cupy
```

---

## Direction 2: Semantic Vision (AI Segmentation)

### Location
`src/d2/semantic_nav/` (skeleton package - needs implementation)

### System Dependencies

```bash
# ROS 2 packages
sudo apt install -y \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-rclcpp-components \
    ros-humble-vision-msgs

# ONNX Runtime with CUDA/TensorRT
# Option A: pip (easier)
pip install onnxruntime-gpu

# Option B: Build from source with TensorRT (better performance)
# See: https://onnxruntime.ai/docs/build/eps.html#tensorrt

# PyTorch (for model export)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Ultralytics (YOLO)
pip install ultralytics
```

### Build Direction 2

```bash
cd /home/prem/terralink

# Install deps
rosdep install --from-paths src/d2/semantic_nav --ignore-src -r -y

# Build (Release mode for performance)
colcon build --packages-select semantic_nav \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/local_setup.bash
```

### Prepare Models

```bash
# Export YOLOv8-seg to ONNX (run once)
cd /home/prem/terralink/src/d2/semantic_nav/scripts
python export_yolo_to_onnx.py \
    --model yolov8n-seg.pt \
    --output ../models/yolov8n-seg.onnx \
    --imgsz 512 \
    --simplify
```

---

## Direction 3: OpenCV PRM Baseline (UAV-UGV Simulation)

### Location
`src/d3/my_bot/` + `src/d3/tutorial_interfaces/`

### System Dependencies

```bash
# ROS 2 packages (most in base Humble desktop)
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-twist-mux \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    python3-pip

# Python deps
pip install transforms3d
```

### Build Direction 3

```bash
cd /home/prem/terralink

# Install deps
rosdep install --from-paths src/d3 --ignore-src -r -y

# Build
colcon build --packages-select my_bot tutorial_interfaces

source install/local_setup.bash
```

### Verify Build

```bash
# Check packages
colcon list | grep -E "(my_bot|tutorial_interfaces)"

# Check executables
ros2 pkg executables my_bot
ros2 interface show tutorial_interfaces/srv/GetWaypoints
```

---

## Full Workspace Build (All Directions)

```bash
cd /home/prem/terralink

# Install all ROS deps at once
rosdep install --from-paths src --ignore-src -r -y

# Build all (independent, can run in parallel)
colcon build --packages-select \
    elevation_map_msgs elevation_mapping_cupy \
    my_bot tutorial_interfaces \
    semantic_nav \
    --cmake-args -DBUILD_TESTING=ON

# Or build individually for cleaner logs:
# colcon build --packages-select elevation_map_msgs elevation_mapping_cupy --cmake-args -DBUILD_TESTING=ON
# colcon build --packages-select my_bot tutorial_interfaces
# colcon build --packages-select semantic_nav --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/local_setup.bash
```

---

## Environment Persistence

Add to `~/.bashrc` for auto-sourcing:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# TerraLink workspace
source /home/prem/terralink/install/local_setup.bash

# DDS fix for testing
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then:
```bash
source ~/.bashrc
```

---

## Troubleshooting Setup

| Issue | Solution |
|-------|----------|
| `colcon build` fails on CuPy | Check CUDA version matches `pip install cupy-cudaXX` |
| `rosdep` fails | `rosdep update` then retry |
| Gazebo fails to start | `export GAZEBO_MODEL_DATABASE_URI=""` |
| DDS discovery issues | `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| Import errors in Python nodes | Ensure `source install/local_setup.bash` after build |
| RViz GridMap plugin not found | `sudo apt install ros-humble-grid-map-rviz-plugin` |

---

## Directory Structure After Setup

```
terralink/
├── install/                    # Built artifacts (after colcon build)
├── src/
│   ├── d1/
│   │   └── elevation_mapping_gpu_ros2/
│   │       ├── elevation_mapping_cupy/
│   │       ├── elevation_map_msgs/
│   │       └── ...
│   ├── d2/
│   │   └── semantic_nav/
│   │       ├── models/yolov8n-seg.onnx   # After export
│   │       └── ...
│   └── d3/
│       ├── my_bot/
│       └── tutorial_interfaces/
├── docs/
│   ├── SETUP.md               # This file
│   ├── RUN.md                 # Runtime commands
│   └── elevation_map/         # Direction 1 deep docs
└── AGENTS.md                  # Agent instructions
```

---

*Each direction is independent - build only what you need.*