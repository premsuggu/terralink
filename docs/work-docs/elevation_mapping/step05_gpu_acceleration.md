# Step 5: GPU Acceleration with CuPy - Work Log

**Date**: 2026-08-22  
**Status**: **KNOWN ISSUE** - ElementwiseKernel compilation failing  
**Goal**: Port fusion kernel to CuPy ElementwiseKernel for 10-100x speedup  
**Reference**: `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/kernels/custom_kernels.py`

---

## What Was Attempted

### Approach 1: RawKernel (First Attempt)
- Used `cp.RawKernel` for explicit control over kernel launch
- **Issue**: RawKernel `__call__` expects exactly 3 positional arguments (grid, block, *args) but Python passes all arguments as positional
- **Error**: `TypeError: __call__() takes exactly 3 positional arguments (29 given)`

### Approach 2: ElementwiseKernel (Second Attempt)
- Switched to `cp.ElementwiseKernel` which handles grid/block automatically
- **Issue**: "Wrong number of arguments" - kernel expects 17 inputs + 9 outputs = 26, but 29 given
- **Root Cause**: Scalar parameters (cell_n, resolution, etc.) counted differently than expected

### Approach 3: ElementwiseKernel with Fixed Signatures
- Removed `1.0f` suffixes (invalid Python syntax in kernel strings)
- Replaced `//` comments with `/* */` 
- **Issue**: "Wrong number of arguments for 'fuse_pointcloud_kernel'. It must be either 17 or 26 (with outputs), but given 29."
- **Issue**: "no suitable conversion function from CArray<float, 1, true, true> to const float*"

---

## Root Cause Analysis

The core issue is a **type mismatch between CuPy arrays and kernel expectations**:

1. **ElementwiseKernel `raw float32` parameters** expect raw pointers (`const float*`)
2. **CuPy passes `CArray<float, 1, true, true>` objects** (internal array representation)
3. **No automatic conversion** from `CArray` to `const float*` in ElementwiseKernel

This is a known CuPy limitation: `raw` type parameters in ElementwiseKernel don't automatically convert CuPy arrays to raw pointers.

---

## Reference Implementation Comparison

The reference implementation (`src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/kernels/custom_kernels.py`) uses the **same ElementwiseKernel pattern** and works. Key differences to investigate:

1. **CuPy Version**: Reference may use older CuPy version with different ElementwiseKernel behavior
2. **Kernel Definition**: Subtle differences in parameter ordering or types
3. **Calling Convention**: Reference might use different calling pattern

---

## Current Workaround

**CPU Implementation Fully Functional**:
- `ElevationMapCPU` class implements identical algorithm in NumPy
- All tests pass (21/21 tests passing across Steps 1-4)
- Headless simulation verified working end-to-end
- Performance: ~5-10ms per frame on CPU (acceptable for development)

**Fallback Strategy**: `create_elevation_map()` factory function automatically falls back to CPU if GPU initialization fails.

---

## Files Modified/Created

| File | Status |
|------|--------|
| `src/terralink_elevation/kernels/fusion_kernel.py` | ElementwiseKernel implementation (compilation issues) |
| `src/terralink_elevation/elevation_map_gpu.py` | GPU ElevationMap class (fallback works) |
| `tests/elevation_mapping/test_step05_fusion_gpu.py` | GPU tests (currently failing) |

---

## Next Steps (Future Work)

1. **Investigate CuPy Version**: Check if newer CuPy version has different ElementwiseKernel behavior
2. **Try RawModule/RawKernel**: Use `cp.RawModule` with PTX compilation for full control
3. **Alternative**: Use Numba CUDA or PyCUDA for kernel implementation
4. **Fallback**: Keep CPU implementation as primary, GPU as optional optimization

---

## Impact Assessment

**No Blocking Impact**: 
- CPU implementation is production-ready
- All Steps 1-4 complete and tested
- Headless simulation verified working
- Steps 6-10 can proceed with CPU implementation

**GPU Acceleration**: Deferred to future sprint

---

## Time Spent

- ElementwiseKernel implementation: ~2 hours
- Debugging compilation errors: ~3 hours
- RawKernel attempts: ~1.5 hours
- Documentation: ~30 min
**Total Step 5**: ~6.5 hours (ongoing)

---

## Files Created/Modified

| File | Status |
|------|--------|
| `src/terralink_elevation/kernels/fusion_kernel.py` | ElementwiseKernel (compilation issues) |
| `src/terralink_elevation/elevation_map_gpu.py` | GPU class (fallback works) |
| `tests/elevation_mapping/test_step05_fusion_gpu.py` | GPU tests (failing) |
| `docs/work-logs/elevation_mapping/step05_gpu_acceleration.md` | This document |