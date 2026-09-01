# Elevation Mapping Documentation Index

## Overview

This directory contains comprehensive documentation for **Direction 1: 2.5D Elevation Mapping** (Geometric Approach).

**Status**: ✅ **Working** - Synthetic demo runs at 10 Hz with Chainer backend, **95-98% map coverage**

---

## Documents

| File | Purpose | Audience |
|------|---------|----------|
| [`concepts_from_scratch.md`](concepts_from_scratch.md) | **Start here** - All concepts from scratch with math, algorithms, learning resources | Anyone wanting to understand/reimplement |
| [`d1_elevation_mapping.md`](d1_elevation_mapping.md) | Complete technical guide from first principles | Engineers new to elevation mapping |
| [`code_deep_dive.md`](code_deep_dive.md) | Line-by-line code walkthrough with exact references | Developers integrating/modifying the repo |
| [`integration_guide.md`](integration_guide.md) | How to connect elevation map → Nav2 costmap | System integrators |

---

## Quick Start (Verified Working)

### Prerequisites (One-time)

```bash
# 1. CUDA Toolkit 12.x (for libnvrtc.so.12)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update && sudo apt-get install -y cuda-toolkit-12-0

# 2. Python dependencies
pip install cupy-cuda12x==13.6.0 simple_parsing chainer ros2_numpy
pip install --upgrade transforms3d

# 3. ROS 2 packages
sudo apt install -y \
    ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
    ros-humble-nav2-costmap-2d ros-humble-ros2-numpy \
    python3-scipy python3-numpy python3-opencv
```

### Build

```bash
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

### Run Synthetic Demo (No Gazebo)

```bash
source /opt/ros/humble/setup.bash
source /home/prem/terralink/install/local_setup.bash

# Terminal 1: Synthetic pointcloud + TF + Elevation mapping
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false

# Terminal 2 (optional): RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix elevation_mapping_cupy)/share/elevation_mapping_cupy/rviz/synthetic_demo.rviz
```

**Expected Output**:
```
[elevation_mapping_node]: Initialized map with length: 8.0, resolution: 0.04, cells: 202
```
GridMap publishing at **~10 Hz** on `/elevation_mapping_node/elevation_map` with layers: `elevation`, `variance`, `traversability`.

### Verify

```bash
ros2 topic hz /elevation_mapping_node/elevation_map
# average rate: 10.0 Hz
# Valid cells: 95-98% (was 26% baseline)
```

---

## Final Working Configuration (Density Optimized)

### Key Changes Applied

| Parameter | File | Before | After | Effect |
|-----------|------|--------|-------|--------|
| World grid | `scripts/synthetic_pointcloud_tf_publisher.py` | 161×161 (0.1m) | **321×321 (0.05m)** | 4x source points |
| Sensor FOV | `scripts/synthetic_pointcloud_tf_publisher.py` | `front_only: true` | **`front_only: false`** | 360° coverage |
| Map center | `config/core/core_param.yaml` | `update_pose_fps: 10.0` | **`update_pose_fps: 0.0`** | Fixed map, no drift |
| Variance growth | `config/core/core_param.yaml` | `time_variance: 0.0001` | **`time_variance: 0.00001`** | 10x slower |
| Time interval | `config/core/core_param.yaml` | `time_interval: 0.1` | **`time_interval: 0.5`** | Less frequent |
| Visibility cleanup | `config/core/core_param.yaml` | `enable_visibility_cleanup: true` | **`false`** | No false invalidations |

### Results

| Metric | Baseline | Optimized | Change |
|--------|----------|-----------|--------|
| **Valid cells** | 26% | **95-98%** | **+3.7x** |
| No points in air | ✅ | ✅ | Same |
| Ground plane accuracy | 0.0m | 0.0m ±0.01m | Same |
| Map center drift | Yes | **Fixed** | ✅ |
| Publish rate | 10 Hz | 10 Hz | Same |
| Long-term stability | Degrades | **Stable 60s+** | ✅ |

---

## Backend: Chainer (Not PyTorch)

The synthetic demo uses **Chainer backend** because PyTorch was installed without CUDA support.

**Config**: `config/core/core_param.yaml:67`
```yaml
use_chainer: true  # If false, uses PyTorch (requires CUDA-enabled torch)
```

---

## Fixed Issues (Applied)

| Issue | Fix Applied |
|-------|-------------|
| `libnvrtc.so.12` missing | Install CUDA toolkit 12.x |
| CuPy version mismatch | `pip install cupy-cuda12x==13.6.0` with numpy 1.24.x |
| PyTorch not CUDA-enabled | Force Chainer backend (`use_chainer: true`) |
| `transforms3d` numpy 2.x crash | `pip install --upgrade transforms3d` |
| `ros2_numpy` missing | `pip install ros2_numpy` |
| `chainer` missing | `pip install chainer` |
| `distutils.msvccompiler` missing (Linux) | Monkey patch in `elevation_mapping_node.py` |
| `setuptools` version conflict | Downgrade to `setuptools==69.5.1` |

---

## Code Entry Points

| Entry Point | File | Description |
|-------------|------|-------------|
| ROS Node | `elevation_mapping_cupy/scripts/elevation_mapping_node.py` | Main executable |
| Core Class | `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py` | `ElevationMap` (1289 lines) |
| Parameters | `elevation_mapping_cupy/elevation_mapping_cupy/parameter.py` | `Parameter` dataclass |
| Kernels | `elevation_mapping_cupy/elevation_mapping_cupy/kernels/custom_kernels.py` | CUDA ElementwiseKernels |
| Config | `elevation_mapping_cupy/config/core/core_param.yaml` | ROS params |
| Launch | `elevation_mapping_cupy/launch/synthetic_depth_demo.launch.py` | Demo launch |

---

## Integration Checklist for TerraLink

- [x] Elevation mapping node running (synthetic demo)
- [x] **Density optimized: 95-98% coverage, stable**
- [ ] Install cuDNN/NCCL for speed (optional)
- [ ] Create `elevation_to_costmap` converter (GridMap → Nav2 OccupancyGrid)
- [ ] Add RGB-D camera to UAV description (d3/my_bot/description/)
- [ ] Create combined launch (UAV + mapping + UGV Nav2)
- [ ] Test in Gazebo with elevation-enabled world
- [ ] Benchmark GPU vs CPU performance

---

## Test Status

```bash
# Unit tests (no ROS needed)
cd src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest elevation_mapping_cupy/tests/test_parameter.py elevation_mapping_cupy/tests/test_repo_config_sanity.py -v
# → 5 passed, 3 config failures (ROS1-style substitutions in old configs)

# CUDA kernel smoke test (requires CUDA-enabled PyTorch - skipped)
```

---

## References

- **Original Repo**: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- **Grid Map Library**: https://github.com/ANYbotics/grid_map
- **CuPy**: https://cupy.dev/
- **Key Paper**: Miki et al., "Elevation Mapping for Legged Robots on GPU" (2022)