# Gazebo Simulation

**Goal**: Set up Gazebo with UAV, depth camera, and custom test worlds.  
**Time to Read**: ~25 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md)

---

## 1. What is Gazebo?

Gazebo is a **physics-based 3D robot simulator**. It simulates:
- Rigid body dynamics (gravity, collisions, joints)
- Sensors (cameras, LiDAR, IMU, GPS)
- Environments (worlds with terrain, objects, lighting)
- ROS 2 integration via `gazebo_ros` plugins

**We use it for**: UAV flight, depth camera simulation, terrain testing

---

## 2. Key Concepts

### 2.1 Models (SDF/URDF)
- **SDF** (Simulation Description Format): Gazebo native, more features
- **URDF** (Unified Robot Description Format): ROS standard, converted to SDF
- Our UAV: `d3/my_bot/description/uav.sdf` (SDF)

### 2.2 Worlds
- `.world` files define the environment
- Contains: ground plane, lighting, models, physics settings
- We'll create: `gaussian_bump.world`, `construction_site.world`

### 2.3 Plugins (ROS 2 ↔ Gazebo Bridge)
```xml
<!-- In SDF/URDF -->
<plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/my_uav</namespace>
    <remapping>~/out:=/camera/image_raw</remapping>
  </ros>
  <camera_name>camera</camera_name>
  <update_rate>30</update_rate>
  <image_width>640</image_width>
  <image_height>480</image_height>
  <format>R8G8B8</format>
</plugin>
```

### 2.4 Sensors We Need
| Sensor | Plugin | Output Topic |
|--------|--------|--------------|
| RGB Camera | `libgazebo_ros_camera.so` | `/camera/image_raw` |
| Depth Camera | `libgazebo_ros_depth_camera.so` | `/camera/depth/image_raw` |
| PointCloud | `libgazebo_ros_pointcloud.so` | `/camera/depth/points` |
| IMU | `libgazebo_ros_imu_sensor.so` | `/imu` |

---

## 3. Using Existing UAV Model (d3/my_bot)

### 3.1 Current UAV (d3/my_bot/description/uav.sdf)
```bash
# Check existing model
cat src/d3/my_bot/description/uav.sdf
```

**Likely has**: Basic quadrotor model, maybe RGB camera
**Needs**: Depth camera (RealSense D435i equivalent) for PointCloud2

### 3.2 Adding Depth Camera to UAV
```xml
<!-- In uav.sdf, add to model -->
<link name="camera_link">
  <pose>0 0 -0.05 0 0 0</pose>  <!-- Below UAV body -->
  <inertial>...</inertial>
  
  <sensor name="depth_camera" type="depth">
    <pose>0 0 0 0 0 0</pose>  <!-- Relative to camera_link -->
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.22</horizontal_fov>  <!-- 70 deg -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R_FLOAT32</format>  <!-- Depth format -->
      </image>
      <clip>
        <near>0.1</near>
        <far>20.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <namespace>/my_uav</namespace>
        <remapping>~/out:=/camera/depth/image_raw</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
    </plugin>
  </sensor>
  
  <!-- Also add pointcloud plugin -->
  <sensor name="pointcloud_sensor" type="gpu_ray">
    <pose>0 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-0.61</min_angle>
          <max_angle>0.61</max_angle>
        </horizontal>
        <vertical>
          <samples>480</samples>
          <resolution>1</resolution>
          <min_angle>-0.46</min_angle>
          <max_angle>0.46</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>20.0</max>
      </range>
    </ray>
    <plugin name="pointcloud_plugin" filename="libgazebo_ros_pointcloud.so">
      <ros>
        <namespace>/my_uav</namespace>
        <remapping>~/out:=/camera/depth/points</remapping>
      </ros>
      <frame_name>camera_depth_optical_frame</frame_name>
    </plugin>
  </sensor>
</link>
```

---

## 4. Camera Frames (Critical for TF)

```
UAV base_link
    │
    └── camera_link (physical mount)
            │
            └── camera_depth_optical_frame (sensor data frame)
                    Z forward (out of lens)
                    X right
                    Y down
```

**TF Tree Needed**:
```
map → base_link (from UAV odometry/mocap)
base_link → camera_link (static, from URDF/SDF)
camera_link → camera_depth_optical_frame (static, ROS convention)
```

---

## 5. Creating Test Worlds

### 5.1 Gaussian Bump World (Simple)
```xml
<!-- worlds/gaussian_bump.world -->
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="gaussian_bump">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <physics name="default_physics" default="1" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Ground plane with Gaussian bump -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://media/materials/textures/flat.png</uri>
              <size>20 20 1</size>  <!-- 20x20m, 1m max height -->
              <pos>0 0 0</pos>
              <!-- Use script to generate heightmap, or use mesh -->
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://media/materials/textures/flat.png</uri>
              <size>20 20 1</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Alternative: Use mesh for precise Gaussian bump -->
    <model name="gaussian_terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://terrains/gaussian_bump/meshes/gaussian_bump.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://terrains/gaussian_bump/meshes/gaussian_bump.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Spawn UAV -->
    <include>
      <uri>model://my_uav</uri>
      <pose>0 0 5 0 0 0</pose>  <!-- 5m above ground -->
    </include>
  </world>
</sdf>
```

### 5.2 Generating Gaussian Bump Mesh (Python)
```python
# scripts/generate_gaussian_terrain.py
import numpy as np
import trimesh

def create_gaussian_bump(size=20.0, resolution=0.1, peak_height=1.0, sigma=2.0):
    """Generate Gaussian bump terrain mesh."""
    x = np.arange(-size/2, size/2, resolution)
    y = np.arange(-size/2, size/2, resolution)
    X, Y = np.meshgrid(x, y)
    
    # Gaussian bump at center
    Z = peak_height * np.exp(-(X**2 + Y**2) / (2 * sigma**2))
    
    # Create mesh
    vertices = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    
    # Create faces (triangulate grid)
    faces = []
    nx, ny = len(x), len(y)
    for iy in range(ny - 1):
        for ix in range(nx - 1):
            a = iy * nx + ix
            b = iy * nx + ix + 1
            c = (iy + 1) * nx + ix
            d = (iy + 1) * nx + ix + 1
            faces.append([a, b, d])
            faces.append([d, c, a])
    
    mesh = trimesh.Trimesh(vertices=vertices, faces=np.array(faces))
    mesh.export('gaussian_bump.dae')
    return mesh

if __name__ == '__main__':
    create_gaussian_bump()
```

### 5.3 Construction Site World (Realistic)
```xml
<!-- worlds/construction_site.world -->
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="construction_site">
    <include><uri>model://sun</uri></include>
    
    <physics name="default_physics" default="1" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Base ground with roughness -->
    <model name="rough_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>model://terrains/construction_site/textures/roughness.png</uri>
              <size>30 30 0.5</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>model://terrains/construction_site/textures/roughness.png</uri>
              <size>30 30 0.5</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Add individual objects: ramps, steps, debris, holes -->
    <include>
      <uri>model://construction_ramp_15deg</uri>
      <pose>5 0 0 0 0 0</pose>
    </include>
    
    <include>
      <uri>model://construction_step_30cm</uri>
      <pose>-5 0 0 0 0 0</pose>
    </include>
    
    <include>
      <uri>model://debris_pile</uri>
      <pose>0 5 0 0 0 0</pose>
    </include>
    
    <include>
      <uri>model://trench</uri>
      <pose>0 -5 0 0 0 0</pose>
    </include>
    
    <!-- UAV at 10m altitude -->
    <include>
      <uri>model://my_uav</uri>
      <pose>0 0 10 0 0 0</pose>
    </include>
  </world>
</sdf>
```

---

## 6. Launch Files

### 6.1 Simulation Launch
```python
# launch/elevation_mapping_sim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('terralink_elevation')
    
    # World argument
    world_arg = DeclareLaunchArgument(
        'world', default_value='gaussian_bump',
        description='World to load: gaussian_bump or construction_site'
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_share, 'worlds', 
                LaunchConfiguration('world')
            ] + '.world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn UAV (from d3/my_bot)
    spawn_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('my_bot'),
                'launch', 'spawn_uav.launch.py'
            ])
        ]),
        launch_arguments={
            'x': '0', 'y': '0', 'z': '5',  # 5m altitude for gaussian_bump
        }.items()
    )
    
    # Elevation mapping node
    elevation_node = Node(
        package='terralink_elevation',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'elevation_mapping.yaml')]
    )
    
    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'elevation_mapping.rviz')],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([
        world_arg,
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        gazebo,
        spawn_uav,
        elevation_node,
        rviz,
    ])
```

---

## 7. Running Simulation

```bash
# Build
colcon build --packages-select terralink_elevation my_bot
source install/local_setup.bash

# Run Gaussian bump test
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=gaussian_bump

# Run Construction site test
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=construction_site

# With RViz
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=construction_site launch_rviz:=true
```

---

## 8. Verifying Sensor Output

```bash
# Check topics
ros2 topic list | grep camera
# Should see:
# /my_uav/camera/depth/image_raw
# /my_uav/camera/depth/points
# /my_uav/camera/depth/camera_info

# Check PointCloud2
ros2 topic hz /my_uav/camera/depth/points
ros2 topic echo /my_uav/camera/depth/points --once | head -30

# Check TF
ros2 run tf2_tools view_frames.py
# Should show: map -> base_link -> camera_link -> camera_depth_optical_frame
```

---

## 9. Common Issues

| Issue | Solution |
|-------|----------|
| "Model not found" | `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/terralink_elevation/models` |
| No PointCloud2 | Check depth camera plugin, topic remapping, `<frame_name>` |
| UAV falls | Check physics, mass, rotor thrust in SDF |
| TF missing | Add `robot_state_publisher` for UAV, static transforms for camera |
| Slow simulation | Reduce physics step size, use simpler collision meshes |

---

## 10. Reference: d3/my_bot Launch

```bash
# Check how d3 spawns UAV
cat src/d3/my_bot/launch/spawn_uav.launch.py
cat src/d3/my_bot/description/uav.sdf
cat src/d3/my_bot/description/camera_uav.xacro
```

---

## Next: [06_coordinate_frames.md](06_coordinate_frames.md) - TF2 and coordinate transformations