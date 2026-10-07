# SETUP.md - Environment Setup for TerraLink (`emap` + `nav`)

One-time environment setup. For day-to-day commands see `RUN.md`.

> Earlier versions of this file also covered the Direction 1 / 2 / 3 reference packages (`src/d1`, `src/d2`, `src/d3`). Those were removed from the tree (last present in commit `8a86dab`); only `emap` and `nav` remain.

---

## 1. System prerequisites

```bash
# ROS 2 Humble on Ubuntu 22.04 (WSL2 is what this project was developed on)
source /opt/ros/humble/setup.bash

sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions python3-rosdep build-essential cmake git

sudo rosdep init   # once
rosdep update
```

## 2. Simulator and bridges (Ignition Gazebo Fortress, NOT Gazebo Classic)

`emap` and `nav` use `gz sim` (Ignition Fortress) with `ros_gz_sim` / `ros_gz_bridge`. Gazebo Classic (`gazebo_ros`) is not used.

```bash
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

WSL2 note: software rendering is used for cameras (`LIBGL_ALWAYS_SOFTWARE=1`, already set by the launch files). Ignition Fuel (`fuel.gazebosim.org`) stalls on downloads in the sandbox, so every external model is vendored under `src/emap/models/`.

## 3. ROS packages

```bash
sudo apt install -y \
    ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-cv-bridge ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard
```
(`nav` needs Nav2 for low-level driving; `grid_map_*` is the message format of `/elevation_map`.)

## 4. Python packages

```bash
pip install numpy scipy opencv-python        # core (scipy also available as python3-scipy)
pip install octomap-python                   # nav/voxel_map.py - bounded 3D verification map (PyPI binding of OctoMap)
pip install pytest                           # tests
# Optional, GPU fusion only (emap `use_gpu_fusion`): needs a CUDA toolkit + a GPU
pip install cupy-cuda12x        # developed with 13.6.0; 14.2.0 is what is installed now
```
CuPy is optional - `emap` falls back to the CPU fusion path automatically, and all unit tests run without a GPU.
The PyPI package is `octomap-python`; the module imported in code is `octomap`.

## 5. Build

```bash
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y     # optional: resolves package.xml deps
colcon build --packages-select emap nav
source install/local_setup.bash
```

## 6. Persist the environment

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /home/prem/terralink/install/local_setup.bash' >> ~/.bashrc
echo 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4' >> ~/.bashrc    # helps DDS discovery in VMs/WSL
```

## 7. Verify

```bash
colcon list                                     # should list exactly: emap, nav
(cd tests/nav && python3 -m pytest -q)          # 382 tests, needs ROS sourced for the node tests
for f in tests/emap/test_*.py; do python3 -m pytest -q $f; done   # run file by file (see RUN.md)
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `colcon build` can't find `emap` when building `nav` | Build both: `colcon build --packages-select emap nav` |
| Import errors in Python nodes | `source install/local_setup.bash` after every build |
| `import octomap` fails | `pip install octomap-python`; confirm with `python3 -c "import octomap"` |
| `import cupy` fails / no GPU | Ignore - CPU fusion is the default path |
| Gazebo hangs on start (WSL2) | Software rendering is already forced; make sure no stale `gz`/`ruby` processes are running |
| DDS discovery issues | `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| RViz GridMap plugin not found | `sudo apt install ros-humble-grid-map-rviz-plugin` |
| Nav2 not installed | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |

## Directory structure

```
terralink/
├── src/
│   ├── emap/   # elevation mapping (UAV, Ignition Gazebo)
│   └── nav/    # UGV navigation (planner, follower, voxel verification)
├── tests/      # emap/ and nav/
├── docs/       # see AGENTS.md "Documentation Reference"
└── media/      # videos, figures, report
```
