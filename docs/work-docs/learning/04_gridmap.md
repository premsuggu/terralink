# GridMap Library

**Goal**: Understand GridMap message format and ROS 2 integration.  
**Time to Read**: ~20 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md), [02_python_robotics.md](02_python_robotics.md)

---

## 1. What is GridMap?

GridMap is a **multi-layer 2D grid map** library from ANYbotics. Unlike OccupancyGrid (single layer), GridMap stores **multiple synchronized layers** per cell.

**Used by**: ANYmal, Spot, TerraLink elevation mapping

### Why GridMap for Elevation Mapping?
| Layer | Purpose |
|-------|---------|
| `elevation` | Height (m) - the main data |
| `variance` | Uncertainty (m²) - sensor noise, fusion quality |
| `traversability` | 0-1 cost - derived from slope/roughness |
| `color_r/g/b` | RGB from camera - visualization |
| `semantic` | Class IDs - road, grass, obstacle |
| `observation_count` | Points per cell - confidence |

All layers share **same geometry** (resolution, size, origin, pose).

---

## 2. GridMap Message Structure

```python
from grid_map_msgs.msg import GridMap

msg = GridMap()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'map'

# Geometry (shared by all layers)
msg.info.resolution = 0.05           # meters/cell
msg.info.length_x = 20.0             # meters (X direction)
msg.info.length_y = 20.0             # meters (Y direction)
msg.info.pose.position.x = 0.0       # Map center in world frame
msg.info.pose.position.y = 0.0
msg.info.pose.position.z = 0.0
msg.info.pose.orientation.w = 1.0    # No rotation (for RViz)

# Layer names (order matters!)
msg.layers = ['elevation', 'variance', 'traversability']

# Data per layer (Float32MultiArray)
msg.data = []  # One entry per layer
```

### Float32MultiArray Layout
```python
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

layer_data = Float32MultiArray()
layer_data.layout = MultiArrayLayout()
layer_data.layout.dim = [
    MultiArrayDimension(label='column_index', size=width, stride=width*height),
    MultiArrayDimension(label='row_index', size=height, stride=height),
]
layer_data.data = []  # Flattened column-major!
```

---

## 3. CRITICAL: Column-Major Ordering!

**GridMap uses COLUMN-MAJOR (Fortran) order**, not row-major (C/NumPy)!

```
NumPy (row-major):     GridMap (column-major):
arr[row, col]          data[col * height + row]

Memory layout:         Memory layout:
[0,0] [0,1] [0,2]      [0,0] [1,0] [2,0] [0,1] [1,1] [2,1] ...
[1,0] [1,1] [1,2]      
[2,0] [2,1] [2,2]      

# Conversion:
# NumPy (H, W) -> GridMap column-major flat list
flat = arr.T.ravel()  # Transpose then flatten
# OR
flat = np.ascontiguousarray(arr.T).flatten()
```

### Visual Example
```python
import numpy as np

# 2x3 grid (2 rows, 3 cols)
arr = np.arange(6).reshape(2, 3)
print(arr)
# [[0 1 2]
#  [3 4 5]]

# NumPy row-major flat: [0, 1, 2, 3, 4, 5]
print(arr.ravel())  # [0 1 2 3 4 5]

# GridMap column-major flat: [0, 3, 1, 4, 2, 5]
print(arr.T.ravel())  # [0 3 1 4 2 5]

# GridMap expects: column 0 (rows 0,1), column 1 (rows 0,1), column 2 (rows 0,1)
```

---

## 4. Coordinate Convention (Internal vs GridMap)

### Internal (Our Code / Elevation Mapping)
```
Row = Y (vertical, increases downward)
Col = X (horizontal, increases rightward)
Array: [layer, row, col] = [layer, y, x]
```

### GridMap Message (ROS Standard)
```
From GridMapMath.cpp:
- Row index → -X direction (increasing row = decreasing X)
- Col index → -Y direction (increasing col = decreasing Y)

Transform:
1. Transpose: swap axes so Row=X, Col=Y
2. Flip axis 0: increasing row → decreasing X  
3. Flip axis 1: increasing col → decreasing Y

Equivalent: rot90(transpose, k=2) or flip(flip(T, 0), 1)
```

### Code for Conversion
```python
def internal_to_gridmap(internal_arr: np.ndarray) -> np.ndarray:
    """
    internal_arr: (H, W) where H=rows=Y, W=cols=X
    Returns: (H, W) in GridMap column-major convention
    """
    # Transpose: (H, W) -> (W, H)  [now rows=X, cols=Y]
    arr = internal_arr.T
    # Flip axis 0: row 0 becomes last row (X direction)
    arr = np.flip(arr, axis=0)
    # Flip axis 1: col 0 becomes last col (Y direction)
    arr = np.flip(arr, axis=1)
    return arr

# For 3D (layers, H, W):
def internal_to_gridmap_3d(internal_arr: np.ndarray) -> np.ndarray:
    # Apply to each layer
    return np.array([internal_to_gridmap(layer) for layer in internal_arr])
```

---

## 5. Creating GridMap Message (Complete Example)

```python
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

def create_gridmap_msg(elevation_map, variance_map, traversability_map, 
                       resolution, center_x, center_y, stamp, frame_id='map'):
    """
    elevation_map: (H, W) numpy array
    variance_map: (H, W) numpy array
    traversability_map: (H, W) numpy array
    """
    H, W = elevation_map.shape
    
    msg = GridMap()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    
    # Geometry
    msg.info.resolution = resolution
    msg.info.length_x = W * resolution
    msg.info.length_y = H * resolution
    msg.info.pose.position.x = center_x
    msg.info.pose.position.y = center_y
    msg.info.pose.position.z = 0.0
    msg.info.pose.orientation.w = 1.0
    
    # Layers
    msg.layers = ['elevation', 'variance', 'traversability']
    
    # Convert each layer to column-major
    for layer_arr in [elevation_map, variance_map, traversability_map]:
        # Convert to GridMap convention
        gm_arr = internal_to_gridmap(layer_arr)
        
        # Flatten column-major
        flat_data = gm_arr.ravel().astype(np.float32).tolist()
        
        # Create Float32MultiArray
        layer_msg = Float32MultiArray()
        layer_msg.layout = MultiArrayLayout()
        layer_msg.layout.dim = [
            MultiArrayDimension(label='column_index', size=W, stride=W*H),
            MultiArrayDimension(label='row_index', size=H, stride=H),
        ]
        layer_msg.data = flat_data
        
        msg.data.append(layer_msg)
    
    # Basic layer (for RViz default display)
    msg.basic_layers = ['elevation']
    
    return msg
```

---

## 6. Reading GridMap Message

```python
def extract_layer(gridmap_msg: GridMap, layer_name: str) -> np.ndarray:
    """Extract a layer from GridMap as 2D numpy array (row-major)."""
    if layer_name not in gridmap_msg.layers:
        return None
    
    idx = gridmap_msg.layers.index(layer_name)
    layer_msg = gridmap_msg.data[idx]
    
    # Get dimensions
    layout = layer_msg.layout
    if len(layout.dim) >= 2:
        # GridMap: dim[0]=column_index (width), dim[1]=row_index (height)
        width = layout.dim[0].size
        height = layout.dim[1].size
    else:
        # Fallback
        size = len(layer_msg.data)
        width = height = int(np.sqrt(size))
    
    # Data is column-major flat list
    flat = np.array(layer_msg.data, dtype=np.float32)
    
    # Reshape column-major: (height, width) in GridMap convention
    gm_arr = flat.reshape(height, width, order='F')
    
    # Convert back to internal (row-major) convention
    return gridmap_to_internal(gm_arr)

def gridmap_to_internal(gm_arr: np.ndarray) -> np.ndarray:
    """Convert GridMap column-major to internal row-major."""
    # Reverse of internal_to_gridmap
    arr = np.flip(gm_arr, axis=1)
    arr = np.flip(arr, axis=0)
    return arr.T
```

---

## 7. RViz Visualization

### 7.1 RViz GridMap Display
1. Add **GridMap** display
2. Topic: `/elevation_map`
3. Layer: `elevation` (or `traversability`, `variance`)
4. Color scheme: `Rainbow`, `Viridis`, `Grayscale`
5. Min/Max: Set manually or auto

### 7.2 Common RViz Issues

| Issue | Fix |
|-------|-----|
| Map appears rotated | Check coordinate transform (internal ↔ GridMap) |
| Map offset from robot | Check `msg.info.pose.position` = map center |
| No data visible | Check layer name matches `msg.layers`, min/max range |
| Flickering | Increase `msg.info.pose.orientation.w = 1.0` (no rotation) |

---

## 8. GridMap in Our Reference Code

### Reference Implementation
- `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/elevation_mapping_cupy/gridmap_utils.py`
- `elevation_mapping.py:784-813` - `_transform_to_grid_map_coordinate_convention`
- `elevation_mapping_node.py:384-462` - `publish_map`

### Key Reference Code (elevation_mapping.py:784-813)
```python
def _transform_to_grid_map_coordinate_convention(self, m):
    """
    Transform from elevation_mapping_cupy convention to grid_map convention.
    elevation_mapping_cupy:  Row=Y, Col=X
    grid_map (ROS):          Row→-X, Col→-Y
    """
    m = m.T                    # Transpose: Row=X, Col=Y
    m = xp.flip(m, axis=0)     # Flip X axis
    m = xp.flip(m, axis=1)     # Flip Y axis
    return m
```

---

## 9. Testing GridMap Conversion

```python
def test_gridmap_conversion():
    # Create test data
    H, W = 10, 15
    elevation = np.arange(H*W).reshape(H, W).astype(np.float32)
    variance = np.full((H, W), 0.01, dtype=np.float32)
    traversability = np.ones((H, W), dtype=np.float32)
    
    # Create message
    from rclpy.time import Time
    msg = create_gridmap_msg(elevation, variance, traversability,
                            resolution=0.05, center_x=0.0, center_y=0.0,
                            stamp=Time(seconds=0).to_msg())
    
    # Extract and verify
    elev_back = extract_layer(msg, 'elevation')
    var_back = extract_layer(msg, 'variance')
    trav_back = extract_layer(msg, 'traversability')
    
    # Should match original (within float precision)
    assert np.allclose(elevation, elev_back, atol=1e-6)
    assert np.allclose(variance, var_back, atol=1e-6)
    assert np.allclose(traversability, trav_back, atol=1e-6)
    print("GridMap round-trip test PASSED!")

test_gridmap_conversion()
```

---

## 10. ROS 2 Packages Needed

```bash
# Ubuntu packages
sudo apt install ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin

# Python (if needed for standalone)
pip install gridmap  # Not typically needed, msgs are enough
```

---

## Next: [05_gazebo.md](05_gazebo.md) - Gazebo Simulation for UAV