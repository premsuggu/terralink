# RUN.md - Runtime Commands for All TerraLink Directions

## Overview

This document contains **copy-pasteable runtime commands** for each direction. Run `SETUP.md` first for one-time environment setup.

**Each direction is independent** - run only what you need.

---

## Common Prerequisites (Every Session)

```bash
# Source ROS 2 + workspace (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
source /home/prem/terralink/install/local_setup.bash

# DDS fix (helps with discovery in VMs/Docker)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

---

## Direction 1: Elevation Mapping (2.5D GPU)

**Package**: `elevation_mapping_cupy` in `src/d1/elevation_mapping_gpu_ros2/`

**Status**: ✅ **Working @ 10 Hz, 95-98% coverage** (density optimized)

### 1A. Synthetic Demo (No Hardware/Gazebo) - **RECOMMENDED**

```bash
# Single launch - runs both synthetic publisher + elevation mapping
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

# Optional: RViz visualization (Terminal 2)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
```

**Expected Output**:
```
[elevation_mapping_node]: Initialized map with length: 8.0, resolution: 0.04, cells: 202
```
GridMap publishing at **~10 Hz** on `/elevation_mapping_node/elevation_map` with layers: `elevation`, `variance`, `traversability`.
**Valid cells: 95-98%** (optimized from 26% baseline)

### 1B. Verify Runtime

```bash
# Check publish rate
ros2 topic hz /elevation_mapping_node/elevation_map
# average rate: 10.0 Hz

# Check topic info
ros2 topic info /elevation_mapping_node/elevation_map
# Type: grid_map_msgs/msg/GridMap, Publisher count: 1

# Echo single message
ros2 topic echo /elevation_mapping_node/elevation_map --once
```

### 1C. Key Config (Density Optimized)

| Parameter | File | Value | Effect |
|-----------|------|-------|--------|
| `front_only` | `scripts/synthetic_pointcloud_tf_publisher.py` | `false` | 360° FOV |
| World grid | `scripts/synthetic_pointcloud_tf_publisher.py` | 321×321 (0.05m) | 4x denser |
| `update_pose_fps` | `config/core/core_param.yaml` | `0.0` | Fixed map |
| `time_variance` | `config/core/core_param.yaml` | `0.00001` | 10x slower |
| `time_interval` | `config/core/core_param.yaml` | `0.5` | Less frequent |
| `enable_visibility_cleanup` | `config/core/core_param.yaml` | `false` | No false inval |

### 1D. Run Tests

```bash
# Unit tests (no ROS)
cd /home/prem/terralink/src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -v

# Specific test files
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test_map_shifting.py -v
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test_gridmap_layout.py -v

# Integration tests (requires ROS + build with -DBUILD_TESTING=ON)
cd /home/prem/terralink
colcon test --packages-select elevation_mapping_cupy --event-handlers console_direct+
```

### 1E. Key Topics to Monitor

```bash
# Input PointCloud (synthetic)
ros2 topic echo /camera/depth/points --once
ros2 topic hz /camera/depth/points

# Output GridMap (multi-layer)
ros2 topic echo /elevation_mapping_node/elevation_map --once
ros2 topic hz /elevation_mapping_node/elevation_map

# Specific layers via RViz GridMap display
```

### 1F. Services

```bash
# Save map to bag
ros2 service call /elevation_mapping_node/save_map \
  grid_map_msgs/srv/ProcessFile "{file_path: '/tmp/my_map', topic_name: 'elevation_map'}"

# Load map from bag
ros2 service call /elevation_mapping_node/load_map \
  grid_map_msgs/srv/ProcessFile "{file_path: '/tmp/my_map', topic_name: 'elevation_map'}"

# Masked replace (human correction)
ros2 service call /elevation_mapping_node/masked_replace \
  grid_map_msgs/srv/SetGridMap "{map: ...}"
```

### 1D. Run Tests

```bash
# Unit tests (no ROS)
cd /home/prem/terralink/src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -v

# Specific test files
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test_map_shifting.py -v
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test_gridmap_layout.py -v

# Integration tests (requires ROS + build with -DBUILD_TESTING=ON)
cd /home/prem/terralink
colcon test --packages-select elevation_mapping_cupy --event-handlers console_direct+
```

### 1E. Key Topics to Monitor

```bash
# Input PointCloud
ros2 topic echo /camera/depth/points --once
ros2 topic hz /camera/depth/points

# Output GridMap (multi-layer)
ros2 topic echo /elevation_mapping_node/elevation_map --once
ros2 topic hz /elevation_mapping_node/elevation_map

# Specific layers via RViz GridMap display
```

### 1F. Services

```bash
# Save map to bag
ros2 service call /elevation_mapping_node/save_map \
  grid_map_msgs/srv/ProcessFile "{file_path: '/tmp/my_map', topic_name: 'elevation_map'}"

# Load map from bag
ros2 service call /elevation_mapping_node/load_map \
  grid_map_msgs/srv/ProcessFile "{file_path: '/tmp/my_map', topic_name: 'elevation_map'}"

# Masked replace (human correction)
ros2 service call /elevation_mapping_node/masked_replace \
  grid_map_msgs/srv/SetGridMap "{map: ...}"
```

---

## Direction 2: Semantic Vision (AI Segmentation)

**Package**: `semantic_nav` in `src/d2/semantic_nav/`

> **Status**: Skeleton package - nodes need implementation

### 2A. Build & Export Model

```bash
# Build (if not done)
cd /home/prem/terralink
colcon build --packages-select semantic_nav --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash

# Export YOLOv8-seg to ONNX
cd src/d2/semantic_nav/scripts
python export_yolo_to_onnx.py \
    --model yolov8n-seg.pt \
    --output ../models/yolov8n-seg.onnx \
    --imgsz 512 \
    --simplify
```

### 2B. Run Nodes (When Implemented)

```bash
# Terminal 1: Semantic segmentation (YOLOv8-seg ONNX)
ros2 run semantic_nav semantic_segmentation_node

# Terminal 2: Costmap converter (semantic → Nav2 costmap)
ros2 run semantic_nav costmap_converter_node

# Terminal 3: Launcher (composable node container)
ros2 run semantic_nav semantic_nav_launcher
```

### 2C. Verify Topics

```bash
# Input: UAV camera
ros2 topic hz /my_uav/camera/image_raw

# Output: Semantic segmentation (mono8 class IDs)
ros2 topic hz /semantic_segmentation

# Output: Nav2 costmap
ros2 topic hz /semantic_costmap
```

---

## Direction 3: OpenCV PRM Baseline (UAV-UGV Simulation)

**Packages**: `my_bot` + `tutorial_interfaces` in `src/d3/`

### 3A. Full Simulation (Gazebo + UAV + UGV + Nav2 + PRM)

```bash
# Build (if not done)
cd /home/prem/terralink
colcon build --packages-select my_bot tutorial_interfaces
source install/local_setup.bash

# Launch full stack
ros2 launch my_bot launch_sim.launch.py
```

**Expected Timeline**:
| Time | Event |
|------|-------|
| 0s | Gazebo loads `roomWithObstacles.world` |
| 5s | Robot (my_bot) spawns |
| 15s | UAV (my_uav) spawns |
| ~17s | Camera publishes, PRM warmup starts |
| ~23s | `"[Sim] PRM graph generation complete!"` |
| 25s | Waypoints client requests path |
| 26s+ | Robot navigates via Nav2 |

### 3B. Verification Checklist

```bash
# Check topics
ros2 topic list | grep -E "(camera|waypoint|goal|odom|scan)"

# Check services
ros2 service list | grep waypoints

# Monitor PRM building logs
ros2 log get waypoints_server | grep -E "(Building|complete|Valid)"

# Manual service call (after PRM ready)
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
```

**Success indicators**:
- [ ] Gazebo opens with maze world
- [ ] Robot and UAV visible
- [ ] Console shows PRM building progress
- [ ] `"[Sim] PRM graph generation complete!"` appears
- [ ] `"[Sim] Received valid path! Beginning waypoint navigation."` appears
- [ ] Robot moves toward goal

### 3C. Navigation Only (With Existing Map)

```bash
# Launch Nav2 stack (requires map)
ros2 launch my_bot navigation_launch.py

# Manual path request
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
```

### 3D. Single Executable Testing

```bash
# Test PRM server alone
ros2 run my_bot waypoints_server

# Test client alone (after server running)
ros2 run my_bot waypoints_client

# Verify service
ros2 service list | grep waypoints
```

---

## Quick Debug Commands (All Directions)

```bash
# Check built packages
colcon list | grep -E "(elevation|my_bot|semantic|tutorial)"

# Check TF tree
ros2 run tf2_tools view_frames

# Check node info
ros2 node info /waypoints_server
ros2 node info /elevation_mapping_node

# Parameter dump
ros2 param dump /elevation_mapping_node
ros2 param dump /waypoints_server

# Topic rates
ros2 topic hz /elevation_mapping_node/elevation_map
ros2 topic hz /my_uav/camera_uav/image_raw
ros2 topic hz /odom

# List all topics/services
ros2 topic list
ros2 service list
```

---

## Common Issues Quick Fix

| Issue | Direction | Fix |
|-------|-----------|-----|
| CuPy import error | 1 | `pip install cupy-cuda11x` (match `nvcc --version`) |
| `ros2 launch` not found | All | `source /opt/ros/humble/setup.bash` |
| Package not found | All | `source install/local_setup.bash` |
| Gazebo hangs on start | 1,3 | `export GAZEBO_MODEL_DATABASE_URI=""` |
| DDS discovery fails | All | `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| RViz shows no map | 1 | Check topic: `/elevation_mapping_node/elevation_map` |
| PRM never completes | 3 | Wait for UAV spawn (15s) + 2 camera frames |
| Robot stuck mid-path | 3 | Known issue - needs stuck detection fix |

---

## Running Multiple Directions Simultaneously

> **Not recommended** - each direction uses different UAV sensors/topics. Run one at a time.

```bash
# If needed, use different ROS_DOMAIN_ID
export ROS_DOMAIN_ID=1  # Direction 1
export ROS_DOMAIN_ID=3  # Direction 3
```

---

*Copy-paste commands directly. Each section is independent.*