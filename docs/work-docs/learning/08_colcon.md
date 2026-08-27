# Colcon Build System

**Goal**: Build ROS 2 packages with colcon (the standard build tool).  
**Time to Read**: ~15 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md)

---

## 1. What is Colcon?

Colcon is the **standard build tool for ROS 2**. It:
- Handles CMake (C++) and ament_python (Python) packages
- Manages dependencies between packages
- Supports parallel builds
- Installs to `install/` directory with proper environment setup

---

## 2. Workspace Structure

```
terralink/                 # Workspace root
├── src/                   # Source packages (git repos, your code)
│   ├── terralink_elevation/
│   ├── d1/
│   ├── d2/
│   └── d3/
├── build/                 # Build artifacts (intermediate)
├── install/               # Installed packages (source this!)
├── log/                   # Build logs
└── tests/                 # Test files (optional)
```

---

## 3. Building Packages

### 3.1 Build Everything
```bash
cd /home/prem/terralink
colcon build
```

### 3.2 Build Specific Packages (Recommended)
```bash
# Build only what you need
colcon build --packages-select terralink_elevation

# Build with dependencies
colcon build --packages-select terralink_elevation --packages-up-to terralink_elevation

# Build from specific package onward
colcon build --packages-from terralink_elevation
```

### 3.3 Build Types

| Package Type | Build System | CMakeLists.txt / setup.py |
|--------------|--------------|---------------------------|
| C++ | ament_cmake | CMakeLists.txt |
| Python | ament_python | setup.py + setup.cfg |
| Mixed | ament_cmake + ament_python | Both |

**Our package**: Python (ament_python)

---

## 4. Python Package Build (ament_python)

### 4.1 Required Files
```
terralink_elevation/
├── package.xml          # Dependencies, metadata
├── setup.py             # Python package config
├── setup.cfg            # Package metadata
├── CMakeLists.txt       # Minimal (just calls ament_python)
└── terralink_elevation/ # Python source
```

### 4.2 CMakeLists.txt (Minimal for Python)
```cmake
cmake_minimum_required(VERSION 3.8)
project(terralink_elevation)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# This installs the Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch/config files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
```

### 4.3 setup.py
```python
from setuptools import find_packages, setup

package_name = 'terralink_elevation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', ['launch/elevation_mapping.launch.py',
                                                'launch/elevation_mapping_sim.launch.py']),
        # Config files
        ('share/' + package_name + '/config', ['config/elevation_mapping.yaml']),
        # Worlds
        ('share/' + package_name + '/worlds', ['worlds/gaussian_bump.world',
                                                'worlds/construction_site.world']),
        # RViz configs
        ('share/' + package_name + '/rviz', ['rviz/elevation_mapping.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='UAV Elevation Mapping for TerraLink',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevation_mapping_node = terralink_elevation.elevation_mapping_node:main',
            'synthetic_pointcloud = terralink_elevation.scripts.synthetic_pointcloud:main',
        ],
    },
)
```

### 4.4 setup.cfg
```ini
[metadata]
name = terralink_elevation
version = 0.0.1

[options]
packages = find:
zip_safe = True

[options.packages.find]
where = .
```

### 4.5 package.xml (Key Parts)
```xml
<package format="3">
  <name>terralink_elevation</name>
  <version>0.0.1</version>
  <description>UAV Elevation Mapping for TerraLink</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>grid_map_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>ros2_numpy</depend>
  <depend>simple_parsing</depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 5. Installing & Sourcing

### 5.1 After Build
```bash
# Build
colcon build --packages-select terralink_elevation

# Source the install space (REQUIRED!)
source install/local_setup.bash
# Or for zsh: source install/local_setup.zsh
# Or for fish: source install/local_setup.fish
```

### 5.2 Verify Install
```bash
# Check package is found
ros2 pkg prefix terralink_elevation
# Should print: /home/prem/terralink/install/terralink_elevation

# Check executable
ros2 run terralink_elevation elevation_mapping_node --help

# Check launch files
ros2 launch terralink_elevation elevation_mapping.launch.py --show-args
```

---

## 6. Common Build Commands

| Command | Purpose |
|---------|---------|
| `colcon build` | Build all packages |
| `colcon build --packages-select pkg` | Build only pkg + deps |
| `colcon build --packages-up-to pkg` | Build pkg and its dependencies |
| `colcon build --symlink-install` | Symlink Python files (edit without rebuild) |
| `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | Release build (C++) |
| `colcon build --event-handlers console_direct+` | Show output live |
| `colcon test --packages-select pkg` | Run tests |
| `colcon test --result-dir test_results` | Save test results |

---

## 7. Symlink Install (Development Mode)

**Essential for Python development** - edit code, runs immediately without rebuild.

```bash
# Build with symlinks
colcon build --packages-select terralink_elevation --symlink-install
source install/local_setup.bash

# Now edit terralink_elevation/elevation_map.py
# Changes take effect IMMEDIATELY on next ros2 run
```

---

## 8. Handling Dependencies

### 8.1 ROS Dependencies (package.xml)
```xml
<depend>rclpy</depend>
<depend>grid_map_msgs</depend>
```
→ Installed via `rosdep` or `apt install ros-humble-<name>`

### 8.2 Python Dependencies (pip)
```bash
# In package.xml (for rosdep)
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-cupy</exec_depend>  # Not in rosdep, manual

# Manual install
pip install cupy-cuda12x==13.6.0
pip install numpy==1.24.2
pip install simple_parsing
pip install transforms3d
pip install scipy
```

### 8.3 rosdep (Auto-install ROS deps)
```bash
# Initialize (once)
rosdep init
rosdep update

# Install dependencies for workspace
cd /home/prem/terralink
rosdep install --from-paths src --ignore-src -r -y
```

---

## 9. Troubleshooting Build Issues

| Error | Solution |
|-------|----------|
| "Package 'X' not found" | `source /opt/ros/humble/setup.bash` first |
| "ament_python not found" | `sudo apt install ros-humble-ament-python` |
| "ModuleNotFoundError" at runtime | `colcon build --symlink-install && source install/local_setup.bash` |
| "CMake error" | Check `CMakeLists.txt` syntax, dependencies in `find_package` |
| "Import error: terralink_elevation" | Check `setup.py` packages=find_packages(), `__init__.py` exists |
| Old code running | Always `source install/local_setup.bash` after build |

---

## 10. Our Build Commands

```bash
# Build our package
cd /home/prem/terralink
colcon build --packages-select terralink_elevation --symlink-install
source install/local_setup.bash

# Build reference (Direction 1) - separate
colcon build --packages-select elevation_map_msgs elevation_mapping_cupy \
    --cmake-args -DBUILD_TESTING=ON

# Build baseline (Direction 3) - separate
colcon build --packages-select my_bot tutorial_interfaces

# Run tests
colcon test --packages-select terralink_elevation --event-handlers console_direct+
```

---

## Next: [09_rviz.md](09_rviz.md) - RViz visualization