# Learning Resources - TerraLink UAV Elevation Mapping

**Purpose**: Understand every library, tool, and concept used in the project.  
**Read Order**: Start from top, each builds on previous.  
**Reference**: Exact versions used in this project.

---

## Table of Contents

1. [ROS 2 Humble Fundamentals](01_ros2_fundamentals.md)
2. [Python for Robotics](02_python_robotics.md)
3. [CuPy - NumPy on GPU](03_cupy_gpu.md)
4. [GridMap Library](04_gridmap.md)
5. [Gazebo Simulation](05_gazebo.md)
6. [Coordinate Frames & TF2](06_coordinate_frames.md)
7. [PointCloud2 & ros2_numpy](07_pointcloud_ros2_numpy.md)
8. [Colcon Build System](08_colcon.md)
9. [RViz Visualization](09_rviz.md)
10. [Testing with pytest](10_pytest.md)

---

## Quick Reference: Versions Used

| Library | Version | Install Command |
|---------|---------|-----------------|
| ROS 2 | Humble | `sudo apt install ros-humble-desktop` |
| Python | 3.10 | System default (Ubuntu 22.04) |
| CuPy | 13.6.0 (CUDA 12.x) | `pip install cupy-cuda12x==13.6.0` |
| NumPy | 1.24.2 | `pip install numpy==1.24.2` |
| grid_map_msgs | Humble | `sudo apt install ros-humble-grid-map-msgs` |
| grid_map_rviz_plugin | Humble | `sudo apt install ros-humble-grid-map-rviz-plugin` |
| ros2_numpy | Latest | `pip install ros2_numpy` |
| transforms3d | >=0.4.2 | `pip install --upgrade transforms3d` |
| simple_parsing | Latest | `pip install simple_parsing` |
| SciPy | Latest | `pip install scipy` |
| Gazebo | 11 (Humble default) | `sudo apt install ros-humble-gazebo-ros-pkgs` |

---

## How to Use These Docs

1. **Read sequentially** - Each doc assumes knowledge from previous
2. **Run code snippets** - Copy-paste into Python terminal to verify
3. **Reference during implementation** - Come back when you see unfamiliar code
4. **Ask questions** - If something isn't clear, flag it for clarification

---

## Learning Path for This Project

```
Week 1 (Basics):
├── 01_ros2_fundamentals.md     ← Start here
├── 02_python_robotics.md
├── 08_colcon.md

Week 2 (Core Libraries):
├── 03_cupy_gpu.md              ← Critical for GPU kernels
├── 04_gridmap.md               ← Our output message type
├── 06_coordinate_frames.md     ← TF2, frames, transforms
├── 07_pointcloud_ros2_numpy.md ← Sensor data handling

Week 3 (Simulation & Viz):
├── 05_gazebo.md                ← UAV + world setup
├── 09_rviz.md                  ← Visualizing elevation maps

Week 4 (Testing):
├── 10_pytest.md                ← Unit testing per step
```

---

## Key Concepts You'll Encounter

| Concept | Where It Appears | Learning Doc |
|---------|------------------|--------------|
| `rclpy.Node` | Every ROS 2 node | 01_ros2_fundamentals |
| `create_subscription` | PointCloud2 subscriber | 01_ros2_fundamentals |
| `create_publisher` | GridMap publisher | 01_ros2_fundamentals |
| `tf2_ros.Buffer` | Coordinate transforms | 06_coordinate_frames |
| `cp.ElementwiseKernel` | GPU fusion kernel | 03_cupy_gpu |
| `GridMap` message | Publishing elevation map | 04_gridmap |
| `PointCloud2` message | Input from camera | 07_pointcloud_ros2_numpy |
| `ros2_numpy.numpify` | Zero-copy conversion | 07_pointcloud_ros2_numpy |
| `colcon build` | Building packages | 08_colcon |
| `QoSPresetProfiles.SENSOR_DATA` | Sensor subscription QoS | 01_ros2_fundamentals |

---

## Common Confusion Points (Preemptive FAQ)

| Confusion | Clarification |
|-----------|---------------|
| "Is CuPy just NumPy on GPU?" | Yes, almost identical API. `import cupy as cp` instead of `import numpy as np` |
| "Why GridMap not OccupancyGrid?" | GridMap supports **multiple layers** (elevation, variance, traversability) in one message |
| "What's the difference: map frame vs base_link?" | `map` = world-fixed, `base_link` = moves with robot. TF connects them. |
| "Why Bayesian fusion not just averaging?" | Accounts for sensor uncertainty (near points more reliable than far) |
| "What's ray tracing for?" | Marks cells as FREE along sensor ray, not just the endpoint |
| "Why shift map instead of growing it?" | Fixed memory, follows robot, infinite world coverage |

---

## Getting Help

- **ROS 2 Docs**: https://docs.ros.org/en/humble/
- **CuPy Docs**: https://docs.cupy.dev/
- **GridMap Docs**: https://github.com/ANYbotics/grid_map
- **Gazebo ROS**: http://gazebosim.org/docs/latest/ros_wrappers
- **This Project's Reference Code**: `src/d1/elevation_mapping_gpu_ros2/`

---

## Next Step

Start with **[01_ros2_fundamentals.md](01_ros2_fundamentals.md)** - covers nodes, topics, services, parameters, QoS, and the ROS 2 Python API (rclpy).