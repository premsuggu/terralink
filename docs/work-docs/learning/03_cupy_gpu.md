# CuPy - NumPy on NVIDIA GPU

**Goal**: Understand CuPy for GPU-accelerated elevation mapping.  
**Time to Read**: ~25 minutes  
**Prerequisites**: [02_python_robotics.md](02_python_robotics.md) - NumPy basics

---

## 1. What is CuPy?

CuPy is **NumPy-compatible array library that runs on NVIDIA GPU (CUDA)**.

```python
import cupy as cp

# Almost identical to NumPy!
a = cp.zeros((1000, 1000), dtype=cp.float32)  # On GPU
b = cp.random.randn(1000, 1000, dtype=cp.float32)
c = a + b                                     # Parallel on GPU
d = cp.sqrt(c)                                # Element-wise on GPU
```

**Why GPU for elevation mapping?**
- 100,000 points/frame × 10 Hz = 1M points/sec
- Each point: transform → cell lookup → atomic updates
- CPU: ~50ms/frame (single-threaded)
- GPU: ~2ms/frame (10,000 parallel threads)

---

## 2. Key Differences from NumPy

| NumPy | CuPy |
|-------|------|
| `import numpy as np` | `import cupy as cp` |
| `np.array([1,2,3])` | `cp.array([1,2,3])` |
| Runs on CPU | Runs on GPU (CUDA) |
| Single-threaded | Massively parallel |
| `np.float32` | `cp.float32` (same) |
| `arr.nbytes` | `arr.nbytes` (same) |
| **No** `.numpy()` method | **Has** `.get()` to copy to CPU |

### Memory Transfer (CPU ↔ GPU)
```python
import numpy as np
import cupy as cp

# CPU → GPU (copy)
cpu_arr = np.array([1, 2, 3], dtype=np.float32)
gpu_arr = cp.asarray(cpu_arr)      # Copy to GPU
# or
gpu_arr = cp.array(cpu_arr)        # Same

# GPU → CPU (copy)
cpu_result = gpu_arr.get()         # Copy back to NumPy
# or
cpu_result = cp.asnumpy(gpu_arr)   # Same

# Zero-copy with ROS 2 (ros2_numpy):
import ros2_numpy as rnp
gpu_points = rnp.pointcloud2_to_cupy(pointcloud_msg)  # Direct to GPU!
```

---

## 3. CuPy Array Operations

### 3.1 Creation
```python
import cupy as cp

# Basic
cp.zeros((H, W), dtype=cp.float32)
cp.ones((H, W), dtype=cp.float32)
cp.full((H, W), 5.0, dtype=cp.float32)
cp.eye(N, dtype=cp.float32)

# Random (on GPU!)
cp.random.randn(H, W, dtype=cp.float32)   # Normal
cp.random.uniform(0, 1, (H, W), dtype=cp.float32)

# From existing
cp.asarray(numpy_array)
cp.array(numpy_array)
```

### 3.2 Math (Same as NumPy)
```python
# All these run on GPU in parallel
a = cp.random.randn(1000, 1000)
b = cp.random.randn(1000, 1000)

c = a + b          # Element-wise add
c = a * b          # Element-wise multiply
c = a @ b          # Matrix multiply
c = cp.sqrt(a)     # Element-wise sqrt
c = cp.sin(a)      # Element-wise sin
c = cp.clip(a, 0, 1)

# Reductions
c = a.sum()        # Scalar
c = a.mean(axis=0) # Row means
c = a.max(axis=1)  # Column maxes
```

### 3.3 Indexing/Slicing (Same as NumPy)
```python
grid = cp.zeros((7, 200, 200), dtype=cp.float32)

grid[0, 100, 100] = 1.5      # Single cell
grid[0, 50:60, 50:60] = 2.0  # Slice
grid[0, :, 100] = 3.0        # Column
grid[1] = cp.full((200, 200), 0.01)  # Entire layer
```

---

## 4. The Power: Custom CUDA Kernels

**This is where 100x speedup comes from** - write CUDA C++ code embedded in Python.

### 4.1 ElementwiseKernel (One Thread Per Element)

```python
import cupy as cp

# Each thread processes ONE element
kernel = cp.ElementwiseKernel(
    in_params='raw float32 x, raw float32 y',   # Input arrays
    out_params='raw float32 z',                  # Output array
    operation='z = x + y',                       # Per-thread code
    name='add_kernel'
)

# Launch: one thread per output element
x = cp.random.randn(1000, dtype=cp.float32)
y = cp.random.randn(1000, dtype=cp.float32)
z = cp.zeros(1000, dtype=cp.float32)
kernel(x, y, z)  # z = x + y on GPU
```

### 4.2 Kernel for Point Cloud Fusion (Our Use Case)

```python
fusion_kernel = cp.ElementwiseKernel(
    # Inputs: point coordinates + transform + map params
    in_params='''
        raw float32 px, raw float32 py, raw float32 pz,  # Point in sensor frame
        raw float32 R, raw float32 t,                     # 3x3 rotation, 3x1 translation
        int32 cell_n, float32 resolution, 
        float32 center_x, float32 center_y,
        float32 sensor_noise_factor, float32 mahalanobis_thresh,
        float32 outlier_variance, float32 min_valid_distance
    ''',
    # Outputs: accumulators + map layers
    out_params='''
        raw float32 elevation, raw float32 variance, raw float32 is_valid,
        raw float32 time, raw float32 upper_bound, raw float32 is_upper_bound,
        raw float32 new_elevation, raw float32 new_variance, raw float32 new_count
    ''',
    # Shared device functions (preamble)
    preamble='''
        __device__ int get_idx(float x, float y, float cx, float cy, float res, int n) {
            int col = int(round((x - cx) / res + n / 2.0f));
            int row = int(round((y - cy) / res + n / 2.0f));
            return row * n + col;  // Flattened index: row-major
        }
        
        __device__ float point_noise(float x, float y, float z, float factor) {
            return factor * (x*x + y*y + z*z);
        }
        
        __device__ bool is_valid_point(float x, float y, float z, float min_dist, float max_h) {
            float dist = sqrtf(x*x + y*y + z*z);
            return (dist >= min_dist) && (fabsf(z) <= max_h);
        }
    ''',
    # Per-thread operation
    operation='''
        // 1. Transform point to map frame
        float mx = R[0]*px + R[1]*py + R[2]*pz + t[0];
        float my = R[3]*px + R[4]*py + R[5]*pz + t[1];
        float mz = R[6]*px + R[7]*py + R[8]*pz + t[2];
        
        // 2. Validate
        if (!is_valid_point(mx, my, mz, min_valid_distance, 100.0f)) return;
        
        // 3. Grid index
        int idx = get_idx(mx, my, center_x, center_y, resolution, cell_n);
        if (idx < 0 || idx >= cell_n * cell_n) return;
        
        // 4. Sensor noise variance
        float v = point_noise(px, py, pz, sensor_noise_factor);
        
        // 5. Prior from map
        float map_h = elevation[idx];
        float map_v = variance[idx];
        
        // 6. Mahalanobis outlier check
        if (fabsf(map_h - mz) > sqrtf(map_v) * mahalanobis_thresh) {
            // Outlier: increase variance only
            atomicAdd(&variance[idx], outlier_variance);
            return;
        }
        
        // 7. Bayesian fusion
        float new_h = (map_h * v + mz * map_v) / (map_v + v);
        float new_v = (map_v * v) / (map_v + v);
        
        // 8. Atomic accumulation (multiple points -> same cell)
        atomicAdd(&new_elevation[idx], new_h);
        atomicAdd(&new_variance[idx], new_v);
        atomicAdd(&new_count[idx], 1.0f);
        
        // 9. Mark valid, reset time
        is_valid[idx] = 1.0f;
        time[idx] = 0.0f;
        upper_bound[idx] = mz;
        is_upper_bound[idx] = 1.0f;
    ''',
    name='fuse_pointcloud_kernel'
)
```

### 4.3 Launching the Kernel

```python
def fuse_pointcloud(self, points_sensor: cp.ndarray, R: cp.ndarray, t: cp.ndarray):
    """
    points_sensor: (N, 3) on GPU
    R: (3, 3) on GPU
    t: (3,) on GPU
    """
    N = points_sensor.shape[0]
    
    # Flatten R and t for kernel
    R_flat = R.ravel()      # (9,)
    t_flat = t.ravel()      # (3,)
    
    # Allocate accumulators (zeroed each frame)
    self.new_elevation.fill(0)
    self.new_variance.fill(0)
    self.new_count.fill(0)
    
    # Launch kernel - one thread per point
    self.fusion_kernel(
        points_sensor[:, 0], points_sensor[:, 1], points_sensor[:, 2],
        R_flat, t_flat,
        self.cell_n, self.resolution, self.center_x, self.center_y,
        self.sensor_noise_factor, self.mahalanobis_thresh,
        self.outlier_variance, self.min_valid_distance,
        self.elevation_map[0], self.elevation_map[1], self.elevation_map[2],
        self.elevation_map[4], self.elevation_map[5], self.elevation_map[6],
        self.new_elevation, self.new_variance, self.new_count,
        size=N  # Number of threads = number of points
    )
    
    # After kernel: average accumulators into map
    self.average_kernel(self.new_elevation, self.new_variance, self.new_count,
                        self.elevation_map[0], self.elevation_map[1], self.elevation_map[2],
                        size=self.cell_n * self.cell_n)
```

### 4.4 Average Kernel (Per Cell, Not Per Point)

```python
average_kernel = cp.ElementwiseKernel(
    in_params='raw float32 new_h, raw float32 new_v, raw float32 new_cnt',
    out_params='raw float32 elevation, raw float32 variance, raw float32 is_valid',
    preamble='''
        __device__ float max_variance = 10.0f;
        __device__ float initial_variance = 1.0f;
    ''',
    operation='''
        if (new_cnt > 0) {
            float avg_v = new_v / new_cnt;
            if (avg_v > max_variance) {
                elevation = 0.0f;
                variance = initial_variance;
                is_valid = 0.0f;
            } else {
                elevation = new_h / new_cnt;
                variance = avg_v;
                is_valid = 1.0f;
            }
        } else {
            elevation = 0.0f;
            variance = initial_variance;
            is_valid = 0.0f;
        }
    ''',
    name='average_map_kernel'
)
```

---

## 5. Memory Management

### 5.1 Memory Pools (Avoid Allocation Overhead)
```python
import cupy as cp

# Use managed memory (unified CPU/GPU) - simpler
pool = cp.cuda.MemoryPool(cp.cuda.malloc_managed)
cp.cuda.set_allocator(pool.malloc)

# Or use default pool (faster for repeated allocations)
cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)

# Arrays automatically reused from pool
```

### 5.2 Streams (Async Operations)
```python
stream = cp.cuda.Stream()

# Async kernel launch
with stream:
    kernel(x, y, z)
    
# Async copy
with stream:
    cpu_result = gpu_arr.get()
    
# Synchronize when needed
stream.synchronize()
```

---

## 6. Debugging CuPy

```python
import cupy as cp

# Check if CUDA available
print(f"CUDA available: {cp.cuda.is_available()}")
print(f"Device: {cp.cuda.Device()}")  # Shows GPU name
print(f"Compute capability: {cp.cuda.Device().compute_capability}")

# Memory info
free, total = cp.cuda.Device().mem_info
print(f"GPU Memory: {free/1e9:.2f} GB free / {total/1e9:.2f} GB total")

# Synchronize to catch errors
cp.cuda.Device().synchronize()

# Profile kernel
from cupy import prof
with prof.time_range('fusion_kernel', 0):
    fusion_kernel(...)

# Print kernel PTX (compiled CUDA code)
print(fusion_kernel.code)
```

---

## 7. Common Gotchas

| Issue | Solution |
|-------|----------|
| `cupy.cuda.runtime.CUDARuntimeError: out of memory` | Reduce map size, use smaller batches, check for memory leaks |
| Kernel slower than NumPy | Kernel launch overhead ~10μs. Only worth it for >1000 elements |
| `TypeError: cannot convert` | Ensure dtypes match: `cp.float32` not `cp.float64` |
| Results differ from CPU | Floating point non-determinism on GPU. Use `cp.allclose(cpu, gpu.get(), rtol=1e-5)` |
| `cp.random` not reproducible | `cp.random.seed(42)` before random ops |

---

## 8. CuPy vs NumPy Verification Pattern

**Always verify GPU matches CPU during development:**

```python
def test_fusion_kernel():
    # 1. Run CPU version
    cpu_map = cpu_fusion(points, R, t, params)
    
    # 2. Run GPU version
    gpu_map = gpu_fusion(points, R, t, params)
    
    # 3. Compare (allow small numerical differences)
    cp.testing.assert_allclose(
        cpu_map, gpu_map.get(), 
        rtol=1e-5, atol=1e-6
    )
    print("GPU matches CPU!")
```

---

## 9. Installation Verification

```bash
# Check CUDA
nvcc --version
# Should show CUDA 12.x

# Check CuPy
python -c "import cupy as cp; print(cp.__version__); print(cp.cuda.is_available())"
# Should print: 13.6.0, True

# Test simple kernel
python -c "
import cupy as cp
a = cp.array([1,2,3])
b = cp.array([4,5,6])
print((a+b).get())  # Should print [5 7 9]
"
```

---

## Next: [04_gridmap.md](04_gridmap.md) - GridMap: Multi-layer grid maps for ROS 2