# PointCloud2 & ros2_numpy

**Goal**: Handle PointCloud2 messages efficiently with zero-copy GPU transfer.  
**Time to Read**: ~20 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md), [02_python_robotics.md](02_python_robotics.md), [03_cupy_gpu.md](03_cupy_gpu.md)

---

## 1. PointCloud2 Message Structure

```python
from sensor_msgs.msg import PointCloud2

# Fields in message:
msg.header          # stamp, frame_id
msg.height          # 1 = unorganized, >1 = organized (image-like)
msg.width           # Number of points per row
msg.fields[]        # PointField: name, offset, datatype, count
msg.is_bigendian    # False (x86)
msg.point_step      # Bytes per point
msg.row_step        # Bytes per row (point_step * width)
msg.data            # Raw bytes (uint8[])
msg.is_dense        # True if no NaN/Inf
```

### Common Field Layouts
```python
# XYZ only (12 bytes/point)
fields = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]
point_step = 12

# XYZ + RGB (16 bytes/point)
fields = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),  # Packed RGB
]
point_step = 16

# XYZ + Intensity (16 bytes/point)
fields = [...]
PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
```

---

## 2. ros2_numpy - Zero-Copy Conversion

**Key insight**: `ros2_numpy` converts PointCloud2 → NumPy/CuPy **without copying data** (memory view).

```python
import ros2_numpy as rnp

# NumPy (CPU)
points_np = rnp.numpify(pointcloud_msg)
# Returns structured array: dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ...]
# Access: points_np['x'], points_np['y'], points_np['z']

# CuPy (GPU) - ZERO COPY!
points_cp = rnp.pointcloud2_to_cupy(pointcloud_msg)
# Returns CuPy structured array on GPU
# Access: points_cp['x'], points_cp['y'], points_cp['z']
```

### Extracting XYZ as Float32 Array
```python
import numpy as np
import cupy as cp

# From NumPy structured array
xyz_np = np.column_stack([
    points_np['x'], points_np['y'], points_np['z']
]).astype(np.float32)  # Shape: (N, 3)

# From CuPy structured array (on GPU!)
xyz_cp = cp.column_stack([
    points_cp['x'], points_cp['y'], points_cp['z']
]).astype(cp.float32)  # Shape: (N, 3)
```

### Fast Path: XYZ Only (Reference Code Pattern)
```python
def _pointcloud2_xyz_f32(msg):
    """Fast extraction for XYZ-only PointCloud2 (from reference)."""
    import ros2_numpy as rnp
    points = rnp.numpify(msg)
    # Structured array -> (N, 3) float32
    return np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)

# GPU version
def _pointcloud2_xyz_f32_cupy(msg):
    import ros2_numpy as rnp
    points = rnp.pointcloud2_to_cupy(msg)
    return cp.column_stack([points['x'], points['y'], points['z']]).astype(cp.float32)
```

---

## 3. Handling Additional Channels

```python
def extract_pointcloud_channels(msg, channel_names):
    """
    Extract XYZ + additional channels from PointCloud2.
    
    Args:
        msg: PointCloud2 message
        channel_names: List of additional channel names (e.g., ['intensity', 'rgb'])
    
    Returns:
        dict with 'xyz' (N,3) and each channel (N,)
    """
    import ros2_numpy as rnp
    
    # Try CuPy first (GPU)
    try:
        points = rnp.pointcloud2_to_cupy(msg)
        use_cupy = True
    except:
        points = rnp.numpify(msg)
        use_cupy = False
    
    # XYZ always present
    xyz = cp.column_stack([points['x'], points['y'], points['z']]).astype(cp.float32) \
          if use_cupy else \
          np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
    
    result = {'xyz': xyz}
    
    # Additional channels
    for ch in channel_names:
        if ch in points.dtype.names:
            arr = points[ch]
            result[ch] = arr.astype(cp.float32) if use_cupy else arr.astype(np.float32)
        else:
            self.get_logger().warn(f"Channel '{ch}' not in PointCloud2")
            result[ch] = None
    
    return result
```

---

## 4. Organized vs Unorganized PointClouds

| Type | Height | Width | Use Case |
|------|--------|-------|----------|
| **Unorganized** | 1 | N | LiDAR, sparse points |
| **Organized** | H | W | Depth camera (matches image pixels) |

```python
# Organized: can reshape to (H, W, 3)
if msg.height > 1:
    xyz_image = xyz.reshape(msg.height, msg.width, 3)
    # Easy pixel access: xyz_image[v, u] = [x, y, z]
else:
    # Unorganized: just (N, 3)
    pass
```

---

## 5. PointCloud2 Filtering (Pre-Fusion)

Filter before fusion to reduce GPU work:

```python
def filter_points(xyz, min_dist=0.3, max_dist=20.0, max_height=5.0, min_height=-2.0):
    """
    Filter points by distance and height.
    
    Args:
        xyz: (N, 3) array
        min_dist: Ignore points too close (sensor noise)
        max_dist: Ignore points too far (noise)
        max_height: Max Z in sensor frame
        min_height: Min Z in sensor frame
    Returns:
        Filtered xyz (M, 3), mask (N,)
    """
    # Distance from sensor origin
    dist = cp.linalg.norm(xyz, axis=1)
    
    # Height in sensor frame (Z)
    height = xyz[:, 2]
    
    # Combined mask
    mask = (dist >= min_dist) & (dist <= max_dist) & \
           (height >= min_height) & (height <= max_height)
    
    return xyz[mask], mask
```

---

## 6. Complete Callback Pattern (From Reference)

```python
def pointcloud_callback(self, msg, sub_key):
    self._last_t = msg.header.stamp
    
    # 1. Parse channels from config
    additional_channels = self.param.subscriber_cfg[sub_key].get("channels", [])
    # e.g., ['intensity', 'semantic']
    
    # 2. Extract points (handles multiple formats)
    if additional_channels:
        # Complex parsing for XYZ + channels
        pts = rnp.numpify(msg)
        # ... parse structured array ...
    else:
        # Fast path: just XYZ
        pts = _pointcloud2_xyz_f32(msg)  # (N, 3) float32
    
    # 3. Get transform: sensor_frame -> map_frame
    if msg.header.frame_id == self.map_frame:
        R, t = np.eye(3), np.zeros(3)
    else:
        transform = self.safe_lookup_transform(self.map_frame, msg.header.frame_id, msg.header.stamp)
        R, t = quaternion_to_matrix(transform.rotation), transform.translation
    
    # 4. Convert to CuPy (zero-copy if already on GPU)
    pts_cp = cp.asarray(pts)  # (N, 3)
    R_cp = cp.asarray(R)      # (3, 3)
    t_cp = cp.asarray(t)      # (3,)
    
    # 5. Fuse into elevation map
    self._map.input_pointcloud(pts_cp, R_cp, t_cp)
    
    self._pointcloud_process_counter += 1
```

---

## 7. Image + CameraInfo Synchronization

For RGB-D cameras, depth image + camera info arrive separately. Synchronize them:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Subscribers
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/camera/depth/camera_info')
        
        # Synchronizer (approximate time, queue=10, slop=0.1s)
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.depth_image_callback)
        
        self.bridge = CvBridge()
    
    def depth_image_callback(self, depth_msg, info_msg):
        # Convert to OpenCV
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # depth_img: (H, W) float32 (meters) or uint16 (mm)
        
        # Camera intrinsics
        K = np.array(info_msg.k).reshape(3, 3)  # [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        D = np.array(info_msg.d)  # Distortion coefficients
        
        # Project to pointcloud (using depth + K)
        # ... or use depth_image_proc/point_cloud_xyz node ...
```

---

## 8. Creating PointCloud2 (For Testing)

```python
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

def create_pointcloud2(xyz, frame_id='camera_depth_optical_frame', stamp=None):
    """
    Create PointCloud2 from (N, 3) float32 array.
    
    Args:
        xyz: (N, 3) numpy/cupy array
        frame_id: TF frame
        stamp: ROS Time (default: now)
    """
    if stamp is None:
        stamp = rclpy.time.Time().to_msg()
    
    # Structured array
    dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    structured = np.zeros(xyz.shape[0], dtype=dtype)
    structured['x'] = xyz[:, 0]
    structured['y'] = xyz[:, 1]
    structured['z'] = xyz[:, 2]
    
    # Convert to PointCloud2
    msg = rnp.msgify(PointCloud2, structured, stamp=stamp, frame_id=frame_id)
    return msg

# Usage in tests:
xyz_test = np.random.randn(1000, 3).astype(np.float32)
msg = create_pointcloud2(xyz_test)
pub.publish(msg)
```

---

## 9. Performance Tips

| Tip | Impact |
|-----|--------|
| Use `rnp.pointcloud2_to_cupy()` | Zero-copy to GPU |
| Filter points BEFORE fusion | Reduces kernel launches |
| Pre-allocate CuPy arrays | Avoids allocation overhead |
| Use organized pointclouds when possible | Enables 2D image operations |
| Channel selection in config | Only parse needed channels |

---

## 10. Debugging PointCloud2

```bash
# Check topic
ros2 topic info /camera/depth/points
ros2 topic hz /camera/depth/points

# Inspect message
ros2 topic echo /camera/depth/points --once | head -50

# Check fields
ros2 interface show sensor_msgs/msg/PointCloud2

# Visualize in RViz
# Add PointCloud2 display, topic: /camera/depth/points
# Style: Points, Size: 0.01, Color: z-axis (height)
```

```python
# Python debug
def debug_pointcloud(msg):
    print(f"Frame: {msg.header.frame_id}")
    print(f"Size: {msg.width}x{msg.height}")
    print(f"Point step: {msg.point_step}, Row step: {msg.row_step}")
    print(f"Fields: {[f.name for f in msg.fields]}")
    print(f"Dense: {msg.is_dense}, Bigendian: {msg.is_bigendian}")
    print(f"Data size: {len(msg.data)} bytes")
    
    pts = rnp.numpify(msg)
    print(f"Parsed: {pts.shape}, dtype: {pts.dtype}")
    print(f"X range: [{pts['x'].min():.2f}, {pts['x'].max():.2f}]")
    print(f"Y range: [{pts['y'].min():.2f}, {pts['y'].max():.2f}]")
    print(f"Z range: [{pts['z'].min():.2f}, {pts['z'].max():.2f}]")
```

---

## Next: [08_colcon.md](08_colcon.md) - Colcon build system