# Step 1: Project Skeleton & Build System - Work Log

**Date**: 2026-08-22  
**Goal**: Create buildable ROS 2 package `terralink_elevation` with CuPy support  
**Reference**: `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/` structure

---

## Concept: ROS 2 Package Structure

A ROS 2 Python package (ament_python) needs:
1. **package.xml** - Dependencies, metadata, build type
2. **setup.py** - Python package installation, entry points
3. **setup.cfg** - Package metadata
4. **CMakeLists.txt** - Minimal, calls ament_python
5. **Python source** - In `terralink_elevation/` subdirectory
6. **Resource marker** - `resource/terralink_elevation` for ament index

---

## Reference Code Analysis

From `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/`:

| File | Purpose | Our Adaptation |
|------|---------|----------------|
| `package.xml` | ROS deps + ament_python | Simplified, our deps |
| `setup.py` | Entry points: `elevation_mapping_node`, `scripts` | Our entry points |
| `CMakeLists.txt` | `ament_python_install_package` + install dirs | Same pattern |
| `config/elevation_mapping.yaml` | ROS params | Our params |
| `launch/*.launch.py` | Launch files | Our launch files |

---

## Final Package Structure (After Reorganization)

```
terralink/                              # Workspace root
├── config/                             # Config files (root level)
│   └── elevation_mapping.yaml
├── launch/                             # Launch files (root level)
│   ├── elevation_mapping.launch.py
│   └── elevation_mapping_sim.launch.py
├── rviz/                               # RViz configs (root level)
│   └── elevation_mapping.rviz
├── worlds/                             # Gazebo worlds (root level)
│   ├── gaussian_bump.world
│   └── construction_site.world
├── tests/                              # Tests (root level)
│   └── elevation_mapping/
│       ├── test_parameter.py
│       ├── test_elevation_map.py
│       ├── test_kernels.py
│       └── test_integration.py
└── src/
    └── terralink_elevation/            # Our package
        ├── CMakeLists.txt
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── terralink_elevation
        ├── scripts/
        │   └── synthetic_pointcloud.py
        └── terralink_elevation/        # Python package
            ├── __init__.py
            ├── parameter.py
            ├── elevation_map.py
            ├── elevation_mapping_node.py
            ├── kernels/
            │   ├── __init__.py
            │   ├── fusion_kernel.py
            │   ├── drift_kernel.py
            │   └── utils.py
            └── utils/
                ├── __init__.py
                ├── coord_transform.py
                └── gridmap_utils.py
```

**Key Changes from Original Plan:**
- Tests moved to `tests/elevation_mapping/` (root level)
- Config, launch, rviz, worlds moved to root level directories
- Package source only contains Python code and resource files

---

## Implementation Details

### 1. package.xml

**Key dependencies** (from reference + our needs):
```xml
<depend>rclpy</depend>
<depend>grid_map_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>ros2_numpy</depend>
<depend>simple_parsing</depend>  <!-- For YAML parameter loading -->
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>std_msgs</depend>
```

### 2. setup.py Entry Points
```python
entry_points={
    'console_scripts': [
        'elevation_mapping_node = terralink_elevation.elevation_mapping_node:main',
        'synthetic_pointcloud = terralink_elevation.scripts.synthetic_pointcloud:main',
    ],
}
```

### 3. CMakeLists.txt
```cmake
ament_python_install_package(${PROJECT_NAME})
# Install from workspace root (../../ from package source)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../worlds DESTINATION share/${PROJECT_NAME})
```

### 4. config/elevation_mapping.yaml
```yaml
terralink_elevation:
  ros__parameters:
    # Map geometry
    resolution: 0.05
    map_length: 20.0
    min_height: -2.0
    max_height: 5.0
    
    # Sensor
    sensor_noise_factor: 0.05
    min_valid_distance: 0.3
    
    # Outlier rejection
    mahalanobis_thresh: 2.0
    outlier_variance: 0.01
    
    # Drift compensation
    enable_drift_compensation: true
    max_drift: 0.10
    position_noise_thresh: 0.2
    
    # Visibility cleanup
    enable_visibility_cleanup: true
    max_ray_length: 10.0
    cleanup_step: 0.05
    cleanup_cos_thresh: 0.3
    
    # Traversability
    max_slope: 0.35
    max_step: 0.15
    max_roughness: 0.05
    
    # Timing
    update_pose_fps: 10.0
    update_variance_fps: 5.0
    publish_fps: 2.0
```

---

## Verification Steps

### 1. Build
```bash
cd /home/prem/terralink
colcon build --packages-select terralink_elevation --symlink-install
source install/local_setup.bash
```

### 2. Verify Package Found
```bash
ros2 pkg prefix terralink_elevation
# Should show: /home/prem/terralink/install/terralink_elevation
```

### 3. Verify Node Starts
```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages:$PYTHONPATH timeout 5 python3 -m terralink_elevation.elevation_mapping_node
# Should show: "ElevationMappingNode initialized", "Map: 20.0x20.0m, 0.05m resolution, 402x402 cells"
```

### 4. Verify Launch Files
```bash
ros2 launch terralink_elevation elevation_mapping.launch.py --show-args
```

---

## Common Pitfalls & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| "Package not found" | Missing `resource/` marker | Create `resource/terralink_elevation` file |
| "ModuleNotFoundError: terralink_elevation" | `setup.py` packages wrong | Use `find_packages(exclude=['test'])` |
| "Entry point not found" | `setup.py` entry_points wrong | Check `console_scripts` mapping |
| "Config file not found" | Not installed | Use `${CMAKE_SOURCE_DIR}/../../config` in CMakeLists.txt |
| Build succeeds but import fails | Missing `--symlink-install` | Rebuild with `--symlink-install` |
| `ModuleNotFoundError: rclpy` | Not sourcing ROS | `source /opt/ros/humble/setup.bash` |

---

## Next Steps (Step 2)

After skeleton works:
1. Implement `parameter.py` with dataclass + YAML loading ✓ (done)
2. Add parameter validation
3. Test parameter serialization round-trip
4. Create `test_step02_parameters.py`

---

## Files Created

| File | Lines | Purpose |
|------|-------|---------|
| `package.xml` | ~40 | ROS dependencies, metadata |
| `setup.py` | ~35 | Python package config, entry points |
| `setup.cfg` | ~10 | Metadata |
| `CMakeLists.txt` | ~25 | ament_python build, install dirs from root |
| `resource/terralink_elevation` | 1 | Ament index marker |
| `config/elevation_mapping.yaml` | ~40 | ROS parameters (root level) |
| `launch/elevation_mapping.launch.py` | ~30 | Node launch (root level) |
| `launch/elevation_mapping_sim.launch.py` | ~50 | Gazebo + node launch (root level) |
| `rviz/elevation_mapping.rviz` | ~40 | RViz config (root level) |
| `worlds/gaussian_bump.world` | ~35 | Simple test world (root level) |
| `worlds/construction_site.world` | ~35 | Realistic test world (root level) |
| `terralink_elevation/__init__.py` | 1 | Package marker |
| `terralink_elevation/parameter.py` | ~80 | Parameter dataclass + YAML loading |
| `terralink_elevation/elevation_map.py` | ~200 | CPU elevation map implementation |
| `terralink_elevation/elevation_mapping_node.py` | ~200 | ROS 2 node |
| `terralink_elevation/utils/gridmap_utils.py` | ~50 | GridMap message encoding |
| `terralink_elevation/utils/coord_transform.py` | ~40 | Coordinate transform helpers |
| `terralink_elevation/kernels/fusion_kernel.py` | ~20 | Placeholder for GPU kernel |
| `terralink_elevation/kernels/drift_kernel.py` | ~10 | Placeholder for GPU kernel |
| `terralink_elevation/kernels/utils.py` | ~20 | Placeholder for GPU utils |
| `scripts/synthetic_pointcloud.py` | ~120 | Synthetic data generator |
| `tests/elevation_mapping/test_parameter.py` | ~30 | Parameter tests |
| `tests/elevation_mapping/test_elevation_map.py` | ~40 | Elevation map tests |
| `tests/elevation_mapping/test_kernels.py` | ~15 | Kernel placeholders |
| `tests/elevation_mapping/test_integration.py` | ~15 | Integration test placeholders |

---

## Time Spent

- Design & planning: ~15 min
- Implementation: ~45 min
- Reorganization (tests/config to root): ~15 min
- Verification: ~15 min
- Documentation: ~20 min
**Total**: ~110 min

---

## Notes for Future Steps

- Keep `terralink_elevation/` Python package flat (no deep nesting)
- Use absolute imports: `from terralink_elevation.parameter import Parameter`
- All kernels in `kernels/`, utilities in `utils/`
- Tests in `tests/elevation_mapping/` mirror source structure
- Config, launch, rviz, worlds at workspace root for easy access