# Python for Robotics

**Goal**: Python patterns and libraries used in robotics (NumPy, type hints, arrays).  
**Time to Read**: ~20 minutes  
**Prerequisites**: Basic Python

---

## 1. NumPy - The Foundation

NumPy is **the** library for numerical computing in Python. All robotics math uses NumPy arrays.

### 1.1 Arrays vs Lists
```python
import numpy as np

# List - slow, flexible
lst = [1, 2, 3, 4, 5]

# NumPy array - fast, fixed type, vectorized operations
arr = np.array([1, 2, 3, 4, 5], dtype=np.float32)

# Operations apply to ALL elements (no loops!)
arr * 2           # [2, 4, 6, 8, 10]
arr + 10          # [11, 12, 13, 14, 15]
arr ** 2          # [1, 4, 9, 16, 25]
np.sqrt(arr)      # [1, 1.414, 1.732, 2, 2.236]
```

### 1.2 2D Arrays (Matrices/Images/Grids)
```python
# 3x4 grid (rows=3, cols=4)
grid = np.zeros((3, 4), dtype=np.float32)
# [[0, 0, 0, 0],
#  [0, 0, 0, 0],
#  [0, 0, 0, 0]]

grid[1, 2] = 5.0  # Row 1, Col 2
# [[0, 0, 0, 0],
#  [0, 0, 5, 0],
#  [0, 0, 0, 0]]

# Slicing
grid[0, :]      # Row 0 (all cols)
grid[:, 1]      # Col 1 (all rows)
grid[0:2, 1:3]  # Rows 0-1, Cols 1-2
```

### 1.3 Coordinate Convention (CRITICAL!)
```python
# NumPy: [row, col] = [y, x]
# grid[y, x]  where y=row (vertical), x=col (horizontal)

# Visual:
#     x (col) →
# y ↓ [0,0] [0,1] [0,2]
# ( [1,0] [1,1] [1,2]
# r [2,0] [2,1] [2,2]

# GridMap message uses DIFFERENT convention!
# See 04_gridmap.md for details
```

### 1.4 Common Operations
```python
# Create grids
np.zeros((H, W))           # All zeros
np.ones((H, W))            # All ones
np.full((H, W), fill_val)  # All fill_val
np.eye(N)                  # Identity matrix

# Random
np.random.randn(H, W)      # Normal distribution (mean=0, std=1)
np.random.uniform(0, 1, (H, W))

# Math
np.gradient(arr, axis=0)   # Derivative along rows (y)
np.gradient(arr, axis=1)   # Derivative along cols (x)
np.sqrt(arr)
np.clip(arr, min, max)     # Clamp values

# Statistics
arr.mean(), arr.std(), arr.min(), arr.max()

# Linear algebra
R @ t        # Matrix @ vector (rotation + translation)
np.linalg.inv(R)           # Inverse rotation
```

### 1.5 Broadcasting (Vectorized Math)
```python
# Instead of loops, use broadcasting
points = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])  # (N, 3)
translation = np.array([10, 20, 30])                   # (3,)

# This works! translation added to each row
transformed = points + translation
# [[11, 22, 33], [14, 25, 36], [17, 28, 39]]

# Rotation matrix @ points (N, 3)
R = np.eye(3)
rotated = (R @ points.T).T  # (3,3) @ (3,N) -> (3,N) -> (N,3)
```

---

## 2. Type Hints (Modern Python)

Type hints make code readable and catch bugs early.

```python
from typing import Tuple, List, Optional
import numpy as np
from numpy.typing import NDArray

# Basic types
def add(a: int, b: int) -> int:
    return a + b

# NumPy arrays with shape and dtype
def process_grid(grid: NDArray[np.float32]) -> NDArray[np.float32]:
    # grid.shape == (H, W)
    return grid * 2

# With shape annotation (Python 3.9+)
def transform_points(
    points: NDArray[np.float32],   # Shape: (N, 3)
    R: NDArray[np.float32],        # Shape: (3, 3)
    t: NDArray[np.float32]         # Shape: (3,)
) -> NDArray[np.float32]:          # Shape: (N, 3)
    return (R @ points.T + t.reshape(3, 1)).T

# Optional (can be None)
def find_transform(frame: str) -> Optional[NDArray[np.float32]]:
    if frame not in known_frames:
        return None
    return known_frames[frame]
```

---

## 3. Dataclasses (Clean Data Containers)

Used for parameters, configuration, structured data.

```python
from dataclasses import dataclass, field
from typing import List
import numpy as np

@dataclass
class MapParameters:
    resolution: float = 0.05
    map_length: float = 20.0
    cell_n: int = field(init=False)  # Computed, not in __init__
    
    def __post_init__(self):
        self.cell_n = int(round(self.map_length / self.resolution)) + 2

@dataclass
class PointCloud:
    points: NDArray[np.float32]  # (N, 3) - x, y, z
    intensities: Optional[NDArray[np.float32]] = None  # (N,)
    frame_id: str = "camera_depth_optical_frame"
    stamp: float = 0.0

# Usage
params = MapParameters(resolution=0.04, map_length=10.0)
print(params.cell_n)  # 252 (computed)
```

---

## 4. Path Handling (pathlib)

Modern, cross-platform path handling.

```python
from pathlib import Path

# Get package share directory (ROS 2)
from ament_index_python.packages import get_package_share_directory

pkg_share = Path(get_package_share_directory('terralink_elevation'))
config_file = pkg_share / 'config' / 'elevation_mapping.yaml'
launch_file = pkg_share / 'launch' / 'elevation_mapping.launch.py'

# Read YAML
import yaml
with open(config_file, 'r') as f:
    params = yaml.safe_load(f)
```

---

## 5. Common Patterns in Our Code

### 5.1 Array Shape Conventions
```python
# Point cloud: (N, 3) - N points, 3 coordinates (x, y, z)
points: NDArray[np.float32]  # shape (N, 3)

# Elevation map: (layers, H, W) - 7 layers, height rows, width cols
elevation_map: NDArray[np.float32]  # shape (7, cell_n, cell_n)
# Layer 0: elevation, Layer 1: variance, etc.

# GridMap message: column-major per layer!
# Must transpose when converting: grid.T or np.ascontiguousarray(grid.T)
```

### 5.2 Coordinate Transforms
```python
def sensor_to_map(points_sensor: NDArray, R: NDArray, t: NDArray) -> NDArray:
    """
    points_sensor: (N, 3) in sensor frame
    R: (3, 3) rotation matrix
    t: (3,) translation vector
    Returns: (N, 3) in map frame
    """
    # p_map = R @ p_sensor + t
    # Using broadcasting: (3,3) @ (3,N) + (3,1) -> (3,N) -> transpose
    return (R @ points_sensor.T + t.reshape(3, 1)).T
```

### 5.3 Grid Index Calculation
```python
def world_to_grid(x: float, y: float, center_x: float, center_y: float, 
                  resolution: float, cell_n: int) -> Tuple[int, int]:
    """
    Convert world coordinates to grid indices.
    Grid center = (cell_n/2, cell_n/2) corresponds to world (center_x, center_y)
    """
    col = int(round((x - center_x) / resolution + cell_n / 2))
    row = int(round((y - center_y) / resolution + cell_n / 2))
    return row, col  # Note: row=y, col=x !

def grid_to_world(row: int, col: int, center_x: float, center_y: float,
                  resolution: float, cell_n: int) -> Tuple[float, float]:
    x = (col - cell_n / 2) * resolution + center_x
    y = (row - cell_n / 2) * resolution + center_y
    return x, y
```

---

## 6. Performance Tips

| Slow (Loop) | Fast (Vectorized) |
|-------------|-------------------|
| `for p in points: process(p)` | `process(points)` - operates on whole array |
| `math.sin(x)` | `np.sin(arr)` |
| `list.append()` in loop | Pre-allocate: `np.zeros(N)` then assign |
| `cp.asnumpy(gpu_arr)` | Keep on GPU, use CuPy operations |

---

## 7. Debugging NumPy Arrays

```python
import numpy as np

arr = np.random.randn(100, 100)

# Shape & dtype
print(f"Shape: {arr.shape}, Dtype: {arr.dtype}")  # (100, 100), float64

# Statistics
print(f"Min: {arr.min():.3f}, Max: {arr.max():.3f}, Mean: {arr.mean():.3f}")

# Check for NaN/Inf
print(f"Has NaN: {np.isnan(arr).any()}")
print(f"Has Inf: {np.isinf(arr).any()}")

# Visualize slice
print(arr[50, 45:55])  # Row 50, cols 45-54

# Save for inspection
np.save('debug_array.npy', arr)
# Later: arr = np.load('debug_array.npy')
```

---

## Next: [03_cupy_gpu.md](03_cupy_gpu.md) - CuPy: NumPy API on NVIDIA GPU