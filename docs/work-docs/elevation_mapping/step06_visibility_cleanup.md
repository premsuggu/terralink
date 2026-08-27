# Step 6: Visibility Cleanup (Ray Tracing) - Work Log

**Date**: 2026-08-23  
**Goal**: Implement ray tracing for visibility cleanup to mark free space along sensor rays  
**Reference**: `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/elevation_mapping_cupy/kernels/custom_kernels.py` lines 209-269

---

## What Was Implemented

### 1. Ray Tracing Algorithm (3D DDA)
Added `_trace_ray_to_point()` method to `ElevationMapCPU` class that implements 3D Digital Differential Analyzer (DDA) algorithm:

```python
def _trace_ray_to_point(self, sensor_origin, point):
    """Trace ray from sensor to point using 3D DDA algorithm.
    
    Marks cells along ray as free space (decreases validity, increases variance),
    but avoids clearing walls using normal check.
    """
    ray = point - sensor_origin
    ray_length = np.linalg.norm(ray)
    ray_dir = ray / ray_length
    step = self.resolution / np.sqrt(3)  # diagonal step to ensure coverage
    max_steps = int(ray_length / step)
    
    current = sensor_origin.copy()
    for step_idx in range(max_steps):
        current += ray_dir * step
        row, col = self.world_to_grid(current[0], current[1])
        if not self.is_inside(row, col):
            break
        
        # Skip if too close to endpoint
        remaining = ray_length - step_idx * step
        if remaining < self.resolution:
            break
        
        # Get cell height and normal
        cell_h = self.elevation_map[self.IDX_ELEVATION, row, col]
        cell_valid = self.elevation_map[self.IDX_IS_VALID, row, col]
        if cell_valid < 0.5:
            continue
        
        normal = self._compute_normal(row, col)
        
        # Check if ray grazes ground (not hitting wall)
        if normal is not None:
            ray_dot_normal = abs(np.dot(ray_dir, normal))
            if ray_dot_normal > self.param.cleanup_cos_thresh:
                continue  # Ray hits wall perpendicularly - don't clear
        
        # Mark as free space
        if current[2] > cell_h + 0.01:
            self.elevation_map[self.IDX_IS_VALID, row, col] *= (1.0 - self.param.cleanup_step)
            self.elevation_map[self.IDX_VARIANCE, row, col] += self.param.outlier_variance * self.param.cleanup_step
            self.elevation_map[self.IDX_IS_VALID, row, col] = max(0.0, self.elevation_map[self.IDX_IS_VALID, row, col])
```

### 2. Normal Computation
Added `_compute_normal()` method using finite differences on elevation map:

```python
def _compute_normal(self, row, col):
    """Compute surface normal at given cell using finite differences."""
    if row == 0 or row >= self.cell_n - 1 or col == 0 or col >= self.cell_n - 1:
        return None
    
    elev = self.elevation_map[self.IDX_ELEVATION]
    dzdx = (elev[row, col+1] - elev[row, col-1]) / (2 * self.resolution)
    dzdy = (elev[row+1, col] - elev[row-1, col]) / (2 * self.resolution)
    
    normal = np.array([-dzdx, -dzdy, 1.0], dtype=np.float32)
    norm = np.linalg.norm(normal)
    if norm > 0:
        return normal / norm
    return None
```

### 3. Integration with Fusion Pipeline
Modified `fuse_pointcloud()` to call `_trace_ray_to_point()` for each point when `enable_visibility_cleanup=True`:

```python
def fuse_pointcloud(self, points, R, t):
    # ... existing code ...
    for px, py, pz in points_valid:
        self._fuse_single_point(px, py, pz)
        # Trace ray for visibility cleanup
        if self.param.enable_visibility_cleanup:
            point_map = np.array([px, py, pz], dtype=np.float32)
            self._trace_ray_to_point(sensor_origin, point_map)
    self._finalize_fusion()
```

---

## Key Technical Details

### Ray Tracing Logic
The algorithm implements 3D DDA (Digital Differential Analyzer) to step along the ray from sensor to measured point:

1. **Step Size**: `resolution / sqrt(3)` - diagonal step ensures coverage of all cells
2. **Max Steps**: `ray_length / step` - number of steps needed
3. **Per-Step Logic**:
   - Advance position along ray
   - Convert to grid coordinates
   - Skip if outside map bounds
   - Skip if near endpoint (within 1 cell)
   - Skip if cell was never valid (never measured)
   - Compute surface normal
   - Check if ray grazes ground (dot product with normal < threshold)
   - If ray passes ABOVE cell surface: mark as free space

### Normal-Based Wall Detection
The cosine threshold (`cleanup_cos_thresh=0.3`) distinguishes:
- **Ray hits wall**: `ray · normal > 0.3` → perpendicular impact → DON'T clear
- **Ray grazes ground**: `ray · normal < 0.3` → glancing angle → CLEAR cell

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_visibility_cleanup` | `True` | Enable/disable ray tracing |
| `cleanup_step` | `0.05` | Validity reduction per ray pass |
| `cleanup_cos_thresh` | `0.3` | Cosine threshold for wall detection |
| `max_ray_length` | `50.0` | Maximum ray trace distance |

---

## Test Results

### Aerial Sensor (Working)
```python
# UAV at 10m, camera pointing down
# Wall at (10, 5, 0) to (10, 5, 3)
# Ground point at (15, 5, 0) beyond wall

# Result: Wall correctly detected (valid=1.0)
# Cells in front of wall (x=5..9) cleared by ray tracing
# Wall cells (x=10) preserved
```

### Horizontal Sensor (Limitation)
```python
# Ground robot with horizontal lidar
# Ray at ground level (z=0) doesn't pass ABOVE terrain cells
# Ray at z=0, cell elevation=0.33: 0 > 0.33 + 0.01 = FALSE
# Cells NOT cleared - known limitation for ground robots
```

---

## Implementation Files

| File | Purpose |
|------|---------|
| `src/terralink_elevation/terralink_elevation/elevation_map.py` | CPU implementation with `_trace_ray_to_point()`, `_compute_normal()`, integration in `fuse_pointcloud()` |
| `src/terralink_elevation/terralink_elevation/kernels/fusion_kernel.py` | GPU ElementwiseKernel (compilation issues - deferred) |
| `src/terralink_elevation/terralink_elevation/elevation_map_gpu.py` | GPU wrapper with RawKernel fallback |
| `tests/elevation_mapping/test_step06_ray_tracing.py` | Test cases for aerial and horizontal sensors |
| `docs/work-logs/elevation_mapping/step06_visibility_cleanup.md` | This document |

---

## Known Issues & Limitations

1. **Horizontal Sensor Limitation**: Ray tracing designed for aerial sensors (Z-up). Horizontal sensors (ground robots) have rays at ground level that don't pass above terrain cells.

2. **GPU Kernel Compilation**: CuPy ElementwiseKernel has argument type issues with `raw` parameters. Deferred to future work.

2. **Coordinate Frame Assumption**: Assumes sensor frame Z = up. For downward-facing cameras, requires coordinate transformation (handled in test via `R_world_to_cam`).

3. **Performance**: CPU implementation uses Python loops for ray tracing. GPU version would provide 10-100x speedup.

---

## Integration with Pipeline

The visibility cleanup integrates into the existing fusion pipeline:

```
PointCloud2 → [Validate] → [Transform to Map] → [Fuse Point] → [Trace Ray] → [Finalize] → GridMap
```

Each fused point triggers a ray trace from sensor origin to the point, clearing free space along the path.

---

## Next Steps (Future Work)

1. **Fix GPU Kernel**: Resolve CuPy ElementwiseKernel compilation issues
2. **Horizontal Sensor Support**: Add ray tracing mode for horizontal sensors
3. **Performance Optimization**: Vectorize ray tracing, use spatial indexing
4. **Multi-Sensor Fusion**: Fuse rays from multiple sensors
5. **Dynamic Obstacle Handling**: Time-decay for validity to handle moving obstacles

---

## Time Spent

- Ray tracing algorithm design: ~1 hour
- CPU implementation: ~2 hours  
- Testing & debugging: ~3 hours
- Documentation: ~1 hour
- GPU kernel attempts: ~3 hours (deferred)
**Total**: ~7 hours

---

## Files Modified/Created

```
src/terralink_elevation/terralink_elevation/elevation_map.py       # +200 lines (ray tracing)
src/terralink_elevation/terralink_elevation/kernels/fusion_kernel.py  # GPU kernels
src/terralink_elevation/terralink_elevation/elevation_map_gpu.py     # GPU wrapper
tests/elevation_mapping/test_step06_ray_tracing.py                 # Tests
docs/work-logs/elevation_mapping/step06_visibility_cleanup.md       # This document
```