# Coordinate Frames & TF2

**Goal**: Master coordinate transformations - essential for mapping sensor data to world frame.  
**Time to Read**: ~25 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md), [02_python_robotics.md](02_python_robotics.md)

---

## 1. Why Coordinate Frames Matter

**The Problem**: 
- Camera sees points in **its own frame** (pixels → 3D rays)
- Map exists in **world frame** (fixed, map origin)
- Robot moves → frames change over time

**The Solution**: TF2 tracks all frame relationships over time.

```
Point in camera frame:     p_cam = [x, y, z]  (meters from lens)
Transform to map:          p_map = R @ p_cam + t
Where:                     R = 3x3 rotation, t = 3x1 translation
From TF2:                  map → camera_depth_optical_frame
```

---

## 2. Standard ROS 2 Frames (REP-105)

| Frame | Description | Convention |
|-------|-------------|------------|
| `map` | World-fixed, origin at startup | Z up, X forward, Y left |
| `odom` | Odometry frame (drifts) | Z up, X forward, Y left |
| `base_link` | Robot body center | Z up, X forward, Y left |
| `camera_link` | Camera physical mount | Z forward, X right, Y down |
| `camera_depth_optical_frame` | Depth sensor data | Z forward, X right, Y down |
| `camera_rgb_optical_frame` | RGB sensor data | Z forward, X right, Y down |

**Optical Frame Convention (REP-103)**:
```
Z = forward (out of lens)
X = right
Y = down

This is DIFFERENT from robot base_link (Z up)!
```

---

## 3. TF2 in ROS 2

### 3.1 Static Transforms (Constant)
```python
# In launch file - camera mount on UAV
from launch_ros.actions import Node

static_tf_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.1', '0', '-0.05',   # x, y, z translation (base_link -> camera_link)
        '0', '0', '0',         # roll, pitch, yaw (radians)
        'base_link',           # parent frame
        'camera_link'          # child frame
    ]
)

# Optical frame (ROS convention)
static_tf_optical = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0', '0', '0',
        '-1.5708', '0', '-1.5708',  # -90°, 0, -90° = ROS optical convention
        'camera_link',
        'camera_depth_optical_frame'
    ]
)
```

### 3.2 Dynamic Transforms (Changing)
Published by robot localization, odometry, or motion capture.

```python
# Published by robot_localization or similar
# map → odom (infrequent, corrects drift)
# odom → base_link (high rate, from wheel encoders/IMU)
```

---

## 4. Transform Math

### 4.1 Representations
```python
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_euler

# 1. Translation + Quaternion (ROS standard)
t = np.array([1.0, 2.0, 0.5])  # x, y, z
q = np.array([0, 0, 0.707, 0.707])  # x, y, z, w (90° yaw)

# 2. 4x4 Homogeneous Matrix
def transform_to_matrix(t, q):
    mat = quaternion_matrix(q)  # 4x4
    mat[:3, 3] = t
    return mat

mat = transform_to_matrix(t, q)
# [[R11 R12 R13 tx]
#  [R21 R22 R23 ty]
#  [R31 R32 R33 tz]
#  [0   0   0   1]]

# 3. Extract rotation matrix (3x3) and translation (3,)
R = mat[:3, :3]
t = mat[:3, 3]
```

### 4.2 Transforming Points
```python
def transform_point(point_sensor, R, t):
    """
    point_sensor: (3,) in sensor frame
    Returns: (3,) in map frame
    """
    return R @ point_sensor + t

def transform_points(points_sensor, R, t):
    """
    points_sensor: (N, 3) in sensor frame
    Returns: (N, 3) in map frame
    """
    # Vectorized: (3,3) @ (3,N) + (3,1) -> (3,N) -> (N,3)
    return (R @ points_sensor.T + t.reshape(3, 1)).T
```

### 4.3 Inverse Transform
```python
def inverse_transform(R, t):
    """Returns (R_inv, t_inv) such that p_sensor = R_inv @ p_map + t_inv"""
    R_inv = R.T  # Rotation inverse = transpose
    t_inv = -R_inv @ t
    return R_inv, t_inv

# Usage: map point -> sensor frame
R_inv, t_inv = inverse_transform(R, t)
p_sensor = R_inv @ p_map + t_inv
```

---

## 5. TF2 Lookup in Code

### 5.1 Basic Lookup
```python
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def get_transform(self, target_frame, source_frame, time=None):
        if time is None:
            time = rclpy.time.Time()
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,      # Where we want to go
                source_frame,      # Where data comes from
                time,              # When (use message timestamp!)
                timeout=Duration(seconds=0.1)
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f'TF {source_frame} -> {target_frame}: {e}')
            return None
```

### 5.2 Convert to Matrix
```python
from tf_transformations import quaternion_matrix

def transform_to_matrix(transform):
    """geometry_msgs/TransformStamped -> 4x4 numpy matrix"""
    t = transform.transform.translation
    q = transform.transform.rotation
    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat

# Usage in callback:
def pointcloud_callback(self, msg):
    # Get transform at message timestamp
    transform = self.get_transform('map', msg.header.frame_id, msg.header.stamp)
    if transform is None:
        return
    
    mat = transform_to_matrix(transform)
    R = mat[:3, :3]
    t = mat[:3, 3]
    
    # Now transform points...
```

---

## 6. Our Project's Frame Tree

```
map (world origin, fixed)
    │
    └── base_link (UAV body, moves with UAV)
            │
            ├── camera_link (physical camera mount, static)
            │       │
            │       └── camera_depth_optical_frame (depth data, static)
            │               Z forward, X right, Y down
            │
            └── camera_rgb_optical_frame (RGB data, static)
```

### Static Transforms Needed
```yaml
# In elevation_mapping.yaml or launch file
static_transforms:
  base_to_camera:
    parent: "base_link"
    child: "camera_link"
    translation: [0.1, 0, -0.05]  # Forward 10cm, down 5cm
    rotation_rpy: [0, 0, 0]
  
  camera_to_optical:
    parent: "camera_link"
    child: "camera_depth_optical_frame"
    translation: [0, 0, 0]
    rotation_rpy: [-1.5708, 0, -1.5708]  # ROS optical convention
```

---

## 7. Timestamp Synchronization (Critical!)

**Rule**: Always use the **message timestamp** for TF lookup, not `now()`.

```python
# WRONG - uses current time, data may be stale
transform = tf_buffer.lookup_transform('map', 'camera', rclpy.time.Time())

# CORRECT - uses when sensor actually captured data
transform = tf_buffer.lookup_transform('map', 'camera', msg.header.stamp)

# For synchronized sensors (Image + CameraInfo):
def image_callback(self, image_msg, camera_info_msg):
    # Both have same timestamp (from ApproximateTimeSynchronizer)
    transform = self.get_transform('map', image_msg.header.frame_id, image_msg.header.stamp)
```

---

## 8. Debugging TF

```bash
# Visualize TF tree (generates frames.pdf)
ros2 run tf2_tools view_frames.py
# Open frames.pdf - shows all frames, parents, frequencies

# Check specific transform
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame

# Monitor transform at rate
ros2 topic echo /tf_static  # Static transforms
ros2 topic echo /tf         # Dynamic transforms
```

### Common TF Errors
| Error | Cause | Fix |
|-------|-------|-----|
| "Lookup would require extrapolation into future" | Using `now()` instead of `msg.header.stamp` | Use message timestamp |
| "Frame X does not exist" | Missing static transform publisher | Add static_transform_publisher |
| "Transform cache empty" | TF listener not running, or buffer too small | Increase buffer: `Buffer(cache_time=Duration(seconds=10))` |
| Wrong rotation | Optical frame convention mismatch | Verify -90°, 0, -90° for camera_link → optical |

---

## 9. Practice Exercise

```python
# Verify your understanding - compute by hand, then code:

# 1. Point in camera_depth_optical_frame: [1, 0, 2] (1m right, 2m forward)
# 2. Camera mounted: 0.1m forward, 0.05m down from base_link, no rotation
# 3. UAV at: map position [10, 5, 3], yaw 45° (0.785 rad)

# Expected: Point in map frame?
# Step 1: camera_optical -> camera_link (rotate -90, 0, -90)
# Step 2: camera_link -> base_link (translate -0.1, 0, +0.05)
# Step 3: base_link -> map (rotate 45° yaw, translate 10, 5, 3)

# Write code to compute this using transform_to_matrix and matrix multiplication
```

---

## Next: [07_pointcloud_ros2_numpy.md](07_pointcloud_ros2_numpy.md) - PointCloud2 message handling