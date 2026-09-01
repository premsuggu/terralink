# Step 9: GPU-Accelerated Fusion

**Package**: `src/emap/`
**Goal**: Port the fusion kernel (step 4) to run on the GPU via CuPy, and verify its output matches the CPU reference numerically - the node's own docs already noted publish rate falling short of the configured 2Hz under this environment's CPU load.
**Status**: ✅ Complete and verified - GPU output matches the CPU reference exactly (within float tolerance) on a randomized point cloud, and measured ~25% faster at a realistic point-cloud size.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 14 (GPU acceleration with CuPy).

---

## 1. Environment setup (two real problems found and fixed)

Installing `cupy-cuda12x` (matching this machine's driver-reported CUDA 13.2, which is backward-compatible with CUDA 12.x wheels) succeeded, but running it immediately failed with:

```
RuntimeError: Failed to find CUDA headers. Please install CUDA toolkit headers
(e.g., pip install cupy-cuda12x[ctk]) or specify CUDA_PATH environment variable.
```

Only the NVIDIA *driver* was present in this WSL2 checkout - no CUDA *toolkit* (no `nvcc`, no `/usr/local/cuda`). CuPy needs the toolkit's headers to JIT-compile GPU kernels at runtime. Fixed with `python3 -m pip install --user "cupy-cuda12x[ctk]"` - the `[ctk]` extra pulls the needed NVRTC/runtime headers as ordinary pip packages (`nvidia-cuda-nvrtc-cu12`, etc.), so no system-level CUDA toolkit install or `sudo` was needed at all.

Separately, installing cupy pulled `numpy==2.2.6` into user site-packages (a real dependency of cupy, not optional), which - because user-site packages are checked before system ones - started shadowing the apt-installed `numpy` for every `python3` invocation. That broke the system's `scipy` (compiled against NumPy 1.x) with `AttributeError: _ARRAY_API not found` the moment anything imported it (including `traversability.py`'s `scipy.ndimage` filters). Fixed by upgrading user-site `scipy` too (`python3 -m pip install --user --upgrade scipy` → 1.15.3, which supports NumPy 2.x) - this shadows the broken system scipy with a compatible one, touching nothing system-wide.

Verified clean afterward:
- `python3 -c "import cupy as cp; a = cp.array([1,2,3]); print(a*2)"` → `[2 4 6]`, GPU name printed correctly (RTX 3050 Laptop GPU).
- `cd tests/emap && python3 -m pytest -q` → all 33 pre-existing tests still passed.
- `source /opt/ros/humble/setup.bash && python3 -c "import rclpy; import numpy; import scipy"` → no errors, confirming the numpy 2 upgrade didn't break the ROS stack.

## 2. The port

New file `emap/fusion_gpu.py`, `fuse_points_gpu(...)` - the exact same 6-step algorithm as `fusion.py`'s `fuse_points` (same outlier rule, same variance-weighted Bayesian combine, same reliance on a correct scatter-accumulate for repeated cell indices), reimplemented with `cupy` in place of `numpy`. First checked that CuPy's `add.at` behaves identically to NumPy's (the one primitive the whole algorithm depends on for correctness):

```python
>>> import cupy as cp
>>> a = cp.zeros(5)
>>> cp.add.at(a, cp.array([0, 0, 2]), cp.array([1.0, 1.0, 3.0]))
>>> a
array([2., 0., 3., 0., 0.])   # matches NumPy exactly
```

`ElevationMap` itself was deliberately left untouched (still plain NumPy) - `fuse_points_gpu` copies just the `elevation`/`variance` layers to the GPU once at the start of a call, does the whole batch of math there, and copies the result back once at the end. From the outside it has the identical contract as `fuse_points`: it mutates `emap`'s NumPy layers in place. Nothing about `ElevationMap`, `traversability.py` (still `scipy.ndimage`, CPU), or the `GridMap` encoding needed to change.

**Deliberately not done**: making `ElevationMap` fully GPU-resident, or porting `traversability.py` to `cupyx.scipy.ndimage`. The roadmap's own definition of this step is "port the fusion kernel, verify GPU output matches CPU reference numerically" - fusion is the actual per-point bottleneck (up to 76,800 points/frame from the 320x240 depth camera); `compute_traversability` runs once per whole cell-array (at most 400x400 cells) and scipy's CPU filters are already fast at that size. Widening scope would have touched every consumer of `ElevationMap` (tests, gridmap encoding, traversability) for no measured benefit.

## 3. Verification

- `tests/emap/test_fusion_gpu.py`: `pytest.importorskip("cupy")` at the top, so the suite still passes clean on a machine with no GPU. Two tests: the same hand-computed single-point scenario as `test_fusion.py`, run through the GPU path; and a 500-point randomized point cloud fused into two otherwise-identical fresh maps via `fuse_points` and `fuse_points_gpu`, asserting `elevation`/`variance`/`is_valid` match via `np.testing.assert_allclose`/`assert_array_equal`. Both pass.
- Honest benchmark (not shipped as a test - a one-off script) at the camera's worst-case point count (320x240 = 76,800 points) on the global map's actual size (40m/0.1m = 400x400 cells), averaged over 10 runs each (first GPU call excluded as warm-up, since it pays a one-time CUDA context/kernel-compile cost):

  ```
  CPU: mean=21.57ms  min=16.60ms  max=28.78ms
  GPU: mean=16.11ms  min=11.97ms  max=19.94ms
  ```

  GPU fusion is genuinely faster here (~25% lower mean latency), but modestly - not the order-of-magnitude speedup GPUs are sometimes assumed to give. This is honestly explained by Section 14's point: at this map's small scale, host↔device transfer overhead eats into a meaningful fraction of the total time, so the win is real but bounded. Reported as measured, not assumed.
- `colcon build --packages-select emap` succeeds; the node logs `fusion=GPU` at startup with `use_gpu_fusion: true` (the config default).

## Follow-ups for later steps

- If map sizes ever grow much larger (more cells, denser point clouds), the GPU's relative advantage should grow too, since the fixed transfer overhead becomes a smaller fraction of a larger compute cost - worth re-benchmarking if that happens.
- `use_gpu_fusion` degrades to the CPU path automatically (with one log line) on any checkout without cupy/a GPU - this was deliberately kept a **parameter**, not a hard dependency, so the package still runs everywhere step 1-8 already did.
