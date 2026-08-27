# ROS 2 Humble Fundamentals

**Goal**: Understand the core ROS 2 concepts used in every node we write.  
**Time to Read**: ~30 minutes  
**Prerequisites**: Basic Python, command line

---

## 1. What is ROS 2?

ROS 2 (Robot Operating System 2) is a **middleware** for robotics - it handles communication between different parts of a robot system (nodes). Think of it as a message bus with standardized message types.

**Key Ideas**:
- **Nodes**: Independent processes that do one thing (e.g., "elevation mapping", "camera driver")
- **Topics**: Named message buses (publish/subscribe) - async, many-to-many
- **Services**: Request/response - synchronous, one-to-one
- **Parameters**: Configuration values (YAML files + runtime changes)
- **TF2**: Coordinate frame transformations (robot pose, sensor positions)

---

## 2. Core Concepts

### 2.1 Nodes
A node is a single executable that performs a specific task.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')  # Unique name in ROS graph
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)  # Blocks, processes callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**In our project**: `ElevationMappingNode` class in `elevation_mapping_node.py`

### 2.2 Topics (Publish/Subscribe)
**Publisher** sends messages, **Subscriber** receives them. Decoupled - they don't know about each other.

```python
# Publisher
self.pub = self.create_publisher(
    GridMap,           # Message type
    '/elevation_map',  # Topic name
    10                 # Queue size (QoS history depth)
)
self.pub.publish(gridmap_msg)

# Subscriber
self.sub = self.create_subscription(
    PointCloud2,       # Message type
    '/camera/depth/points',  # Topic name
    self.pointcloud_callback,  # Callback function
    QoSPresetProfiles.SENSOR_DATA.value  # QoS profile
)
```

**QoS (Quality of Service)** - Critical for sensors:
- `SENSOR_DATA` = BEST_EFFORT + KEEP_LAST(1) - drops old data, low latency
- `SYSTEM_DEFAULT` = RELIABLE + KEEP_LAST(10) - guarantees delivery

### 2.3 Services (Request/Response)
Synchronous communication - client waits for response.

```python
# Service definition (in .srv file):
# Request:
#   geometry_msgs/Point start
#   geometry_msgs/Point goal
# Response:
#   bool valid
#   geometry_msgs/PoseStamped[] waypoints

# Server
self.srv = self.create_service(
    GetWaypoints,      # Service type
    'waypoints_service',  # Service name
    self.get_waypoints_callback
)

# Client
self.client = self.create_client(GetWaypoints, 'waypoints_service')
future = self.client.call_async(request)
rclpy.spin_until_future_complete(self, future)
response = future.result()
```

### 2.4 Parameters
Configuration loaded from YAML, overridable at runtime.

```python
# Declare with default
self.declare_parameter('map_resolution', 0.05)
self.declare_parameter('map_length', 20.0)

# Get value
resolution = self.get_parameter('map_resolution').get_parameter_value().double_value

# Or use Parameter object (cleaner)
from rclpy.parameter import Parameter
param = self.get_parameter('map_resolution')
resolution = param.value  # Automatically typed
```

**YAML Config** (`config/elevation_mapping.yaml`):
```yaml
elevation_mapping_node:
  ros__parameters:
    map_resolution: 0.05
    map_length: 20.0
    update_pose_fps: 10.0
```

---

## 3. Message Types We'll Use

| Message | Package | Purpose |
|---------|---------|---------|
| `sensor_msgs/msg/PointCloud2` | sensor_msgs | 3D point cloud from depth camera |
| `grid_map_msgs/msg/GridMap` | grid_map_msgs | Multi-layer elevation map (our output) |
| `geometry_msgs/msg/TransformStamped` | geometry_msgs | TF2 transforms |
| `nav_msgs/msg/Odometry` | nav_msgs | Robot pose + velocity |
| `std_msgs/msg/Header` | std_msgs | Timestamp + frame_id (on every message) |

### PointCloud2 Structure
```
Header (stamp, frame_id)
height, width (organized vs unorganized)
fields[] (x, y, z, intensity, rgb, etc.)
is_bigendian, point_step, row_step
data[] (raw bytes - use ros2_numpy to decode)
```

### GridMap Structure
```
Header (stamp, frame_id)
info: resolution, length_x, length_y, pose (position + orientation)
layers[] (string names: "elevation", "variance", "traversability")
data[] (Float32MultiArray per layer, column-major order!)
```

---

## 4. Coordinate Frames & TF2 (Critical!)

ROS 2 uses **TF2** to track coordinate frames over time.

### Common Frames in Our Project
| Frame | Description |
|-------|-------------|
| `map` | World-fixed frame (origin = where UAV started) |
| `base_link` | UAV body center |
| `camera_depth_optical_frame` | Depth camera sensor (Z forward, X right, Y down) |
| `odom` | Odometry frame (drifts over time) |

### Transform Lookup
```python
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

# In node __init__:
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# In callback (async, non-blocking):
try:
    transform = self.tf_buffer.lookup_transform(
        'map',                    # Target frame
        'camera_depth_optical_frame',  # Source frame
        msg.header.stamp,         # Time (use message timestamp)
        timeout=rclpy.duration.Duration(seconds=0.1)
    )
except TransformException as e:
    self.get_logger().warn(f'TF lookup failed: {e}')
    return
```

### Transform to Matrix
```python
import numpy as np
from tf_transformations import quaternion_matrix  # or transforms3d

def transform_to_matrix(transform: TransformStamped):
    t = transform.transform.translation
    q = transform.transform.rotation
    # Returns 4x4 matrix
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat

# Usage:
mat = transform_to_matrix(transform)
R = mat[:3, :3]  # 3x3 rotation
t = mat[:3, 3]   # 3x1 translation
# Point in map frame: p_map = R @ p_sensor + t
```

---

## 5. Callback Groups & Executors (Advanced)

By default, all callbacks run in a single thread. For heavy processing (GPU fusion), use **callback groups**:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# In node __init__:
self.cb_group = MutuallyExclusiveCallbackGroup()

# Subscriber with callback group:
self.sub = self.create_subscription(
    PointCloud2, 'topic', self.callback, 
    QoSPresetProfiles.SENSOR_DATA.value,
    callback_group=self.cb_group
)

# In main():
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()  # Runs callbacks in parallel threads
```

---

## 6. Running ROS 2 Commands

```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/local_setup.bash  # After colcon build

# List nodes
ros2 node list

# List topics
ros2 topic list
ros2 topic hz /topic_name      # Publishing rate
ros2 topic echo /topic_name    # Print messages

# List services
ros2 service list
ros2 service call /service_name pkg/srv/Type "{field: value}"

# List parameters
ros2 param list
ros2 param get /node_name param_name
ros2 param set /node_name param_name value

# TF tree
ros2 run tf2_tools view_frames.py  # Generates frames.pdf

# Record/playback bags
ros2 bag record /topic1 /topic2
ros2 bag play recording.bag
```

---

## 7. Package Structure (Python)

```
my_package/
├── CMakeLists.txt          # Build config (ament_cmake)
├── package.xml             # Dependencies, metadata
├── config/
│   └── params.yaml         # Parameter defaults
├── launch/
│   └── my_launch.py        # Launch file
├── my_package/             # Python package (same name as folder)
│   ├── __init__.py
│   ├── my_node.py          # Node implementation
│   └── utils.py
├── test/
│   └── test_my_node.py     # Unit tests
└── resource/
    └── my_package          # Marker file for ament
```

**Key Files**:

**package.xml**:
```xml
<package format="3">
  <name>terralink_elevation</name>
  <version>0.0.1</version>
  <description>UAV Elevation Mapping</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>grid_map_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>ros2_numpy</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**CMakeLists.txt** (for Python packages):
```cmake
cmake_minimum_required(VERSION 3.8)
project(terralink_elevation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 8. Common Patterns in Our Code

### Safe TF Lookup (with timeout)
```python
def safe_lookup_transform(self, target_frame, source_frame, time):
    try:
        return self.tf_buffer.lookup_transform(
            target_frame, source_frame, time,
            timeout=rclpy.duration.Duration(seconds=0.05)
        )
    except TransformException as e:
        self.get_logger().warn(f'TF {source_frame} -> {target_frame}: {e}')
        return None
```

### Parameter Loading from YAML + ROS Override
```python
def load_params(self):
    # 1. Load from YAML file
    param_file = self.get_parameter('param_file').value
    with open(param_file, 'r') as f:
        yaml_params = yaml.safe_load(f)
    
    # 2. Declare all params (with YAML defaults)
    for key, value in yaml_params.items():
        self.declare_parameter(key, value)
    
    # 3. ROS 2 command-line overrides automatically applied
```

### Timer for Periodic Tasks
```python
# 10 Hz pose update
self.pose_timer = self.create_timer(
    1.0 / 10.0,  # seconds
    self.pose_update_callback
)
```

---

## 9. Debugging Tips

| Problem | Solution |
|---------|----------|
| "No messages received" | Check topic name: `ros2 topic list`, QoS mismatch |
| "TF lookup failed" | Check `ros2 run tf2_tools view_frames.py`, frame names |
| "Node dies silently" | Check `ros2 node list`, run with `--ros-args --log-level DEBUG` |
| "Import error" | `colcon build --packages-select pkg_name && source install/local_setup.bash` |
| "Old code running" | Always `source install/local_setup.bash` after build |

---

## 10. Practice Exercise

Before moving on, verify you can:

1. Create a minimal node that publishes a counter on `/counter` topic
2. Create a subscriber that prints received messages
3. Add a parameter `publish_rate` with default 1.0 Hz
4. Build with `colcon build` and run with `ros2 run`
5. Change parameter at runtime: `ros2 param set /node publish_rate 5.0`

```python
# Solution template - try writing it yourself first!
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterNode(Node):
    def __init__(self):
        super().__init__('counter_node')
        self.declare_parameter('publish_rate', 1.0)
        rate = self.get_parameter('publish_rate').value
        self.pub = self.create_publisher(Int32, '/counter', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_cb)
        self.count = 0
    
    def timer_cb(self):
        msg = Int32()
        msg.data = self.count
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {self.count}')
        self.count += 1

def main():
    rclpy.init()
    rclpy.spin(CounterNode())
    rclpy.shutdown()
```

---

## Next: [02_python_robotics.md](02_python_robotics.md) - Python patterns for robotics (NumPy, arrays, type hints)