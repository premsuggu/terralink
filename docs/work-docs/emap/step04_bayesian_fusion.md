# Step 4: Bayesian Fusion (CPU)

**Package**: `src/emap/`
**Goal**: Teach the elevation map (step 3) to actually learn terrain height from a batch of 3D points, by combining each new measurement with what the cell already believes — not simply overwriting it — and reject measurements that are too inconsistent to trust.
**Status**: ✅ Complete and verified.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 9 (Combining two uncertain beliefs) — this document is the code walkthrough for exactly that idea, with the same numbers actually run through the real code.

---

## 1. What does "fusion" mean here?

Step 3 gave us an empty grid. This step gives it one function, `fuse_points`, that takes a batch of 3D points (imagine one camera frame's worth — step 2 already proved we can get a real point cloud, correctly located in the world, into ROS 2) and updates the map's `elevation` and `variance` layers accordingly. The key design decision, explained from scratch in `00_concepts.md` Section 9, is that a new measurement doesn't just replace whatever a cell currently holds — it's *combined* with the cell's existing belief, weighted by how much each side should be trusted.

## 2. Walkthrough: `emap/fusion.py`

`fuse_points(emap, points_xyz, sensor_origin, sensor_noise_factor, mahalanobis_thresh, outlier_variance, min_valid_distance)` runs each point through this pipeline:

### 2.1 How far away was each point, and is it too close to trust?

```python
offset = points_xyz - sensor_origin
range_sq = np.sum(offset * offset, axis=1)
far_enough = range_sq >= (min_valid_distance ** 2)
```
Real depth sensors are unreliable at very short range (near the lens, before the optics can properly focus). Anything closer than `min_valid_distance` is dropped outright before it can affect anything. We compute `range_sq` (distance *squared*) rather than distance, since that's all either this filter or the next step actually need — an unnecessary square root for every point in a 76,800-point cloud (step 2's camera resolution) adds up.

### 2.2 Measurement noise grows with distance

```python
measurement_variance = sensor_noise_factor * range_sq
```
A point measured 10m away is far less trustworthy than one measured 0.5m away — this one line is the entire "how noisy was this specific measurement" model, and it's what lets nearby, confident measurements dominate over noisy, far-away ones during fusion (Section 2.4).

### 2.3 Which cell, and is that cell even part of the map?

```python
row, col = emap.world_to_grid(x, y)
inside = emap.in_bounds(row, col)
```
Straight reuse of step 3's coordinate-transform functions — this step doesn't reinvent that math, it builds on it. Points that land outside the map are dropped, exactly like the too-close points in Section 2.1.

### 2.4 The actual fusion: outliers vs. inliers

For every point that survived the filters above, we look up what its cell currently believes (`prior_h`, `prior_v`), then split into two cases:

**Outliers** — `abs(prior_h - z) > prior_v * mahalanobis_thresh`: the measurement disagrees with a confident belief by more than the allowed margin (loosely, "more standard deviations than we're willing to accept" — `00_concepts.md` Section 9's "too surprising" idea, made concrete). These don't get to change the height at all, but the cell's variance is bumped up by `outlier_variance` — an acknowledgment that something unexpected just happened here, even though we don't trust *what* it says.

**Inliers** — everything else — get the real Bayesian update from `00_concepts.md` Section 9:
```python
new_h = (prior_h * v + z * prior_v) / (prior_v + v)
new_v = (prior_v * v) / (prior_v + v)
```

### 2.5 The one real NumPy gotcha this step is built around

Here's a mistake it would be very easy to make: with a dense point cloud, many points often land in the *same* cell within one batch. If you write the fused results in with ordinary indexing —
```python
variance_layer[rows, cols] += outlier_variance          # WRONG if rows/cols has repeats
elevation_layer[rows, cols] = new_h                      # WRONG if rows/cols has repeats
```
— NumPy does **not** add or assign once per repeated index. For repeated indices, plain assignment just keeps whichever one happened to be written *last*, silently discarding every other point that landed in that same cell. `+=` has the same problem for repeated indices in one statement. This isn't a hypothetical: with the resolution used in step 2's simulated camera, this happens on essentially every real update.

The fix used throughout `fuse_points` is `np.add.at`, which is specifically designed to correctly accumulate into repeated indices, one at a time:
```python
np.add.at(variance_layer, (row[is_outlier], col[is_outlier]), outlier_variance)
...
np.add.at(sum_h, (rows_in, cols_in), new_h)
np.add.at(sum_v, (rows_in, cols_in), new_v)
np.add.at(count, (rows_in, cols_in), 1.0)
```
For the normal (inlier) case, each point's *own* fused estimate is computed first (against the prior every point in this batch shares, not against each other — see Section 3), then summed per cell and divided by how many points landed there — i.e. a cell hit by several points in one batch ends up with the *average* of their individually-fused results, not the last one to happen to be processed.

## 3. A deliberate simplification vs. the GPU reference

The preferred reference, `src/d1/elevation_mapping_gpu_ros2/.../kernels/custom_kernels.py`, computes the same formula but on a GPU, where many threads might try to update the same cell at the exact same instant — a genuine race condition a single CPU function never has. Its workaround is a two-buffer trick (accumulate into a separate scratch array using GPU atomic operations, then a second pass divides by count). Our CPU version reaches the exact same *result* — each point's estimate computed against a shared prior, then averaged per cell — using `np.add.at` in one straightforward pass, because a single-threaded CPU function processes points one at a time and never needs to worry about two points racing to update the same memory simultaneously. Same math, simpler code, because the underlying hardware problem it was solving doesn't apply here.

## 4. Verification (numbers cross-checked before being written down)

Every expected number in `tests/emap/test_fusion.py` was computed independently (plain arithmetic written directly in each test, not by calling `fuse_points`) and then run against the real function to confirm they matched exactly, before being committed — the same discipline as step 3's worked example.

- **Single point, single cell** (`test_single_point_matches_hand_computed_bayesian_update`): a point at height 1.0, sensor 2m above it, `sensor_noise_factor=0.01` → measurement variance `v = 0.01 * 2² = 0.04`. Starting from the map's default belief (`elevation=0, variance=10`):
  ```
  new_h = (0*0.04 + 1.0*10) / (10 + 0.04) = 0.996016
  new_v = (10*0.04) / (10 + 0.04)          = 0.039841
  ```
  The real function produced exactly these values (to floating-point precision). Notice `new_h` landed close to the measurement (1.0), not the old belief (0) — because the measurement's variance (0.04) is far smaller than the prior's (10), i.e. far more trustworthy, matching Section 9's "lab thermometer" intuition.
- **Repeated measurement converges** (`test_repeated_measurement_converges`): fusing the same true height 5 times in a row, `variance` strictly shrinks and the error to the true height shrinks on every single iteration (checked in a loop, not just at the end) — direct evidence that combining more independent observations keeps increasing confidence, exactly as Section 9 describes.
- **Two points, one cell, averaged not overwritten** (`test_two_points_in_the_same_cell_are_averaged_not_overwritten`): two distinct points both landing in the map's center cell; the test computes each point's own fused estimate by hand and asserts the map holds their *average* — directly exercising the `np.add.at` behavior from Section 2.5, and would fail immediately if the code used plain fancy-index assignment instead.
- **Outlier rejection** (`test_outlier_is_rejected_but_still_raises_variance`): a measurement of 100 against a prior belief of 0 (with `variance=10`, `mahalanobis_thresh=2.0` → allowed disagreement is only 20) is correctly rejected: `elevation` stays exactly 0, `variance` increases by exactly `outlier_variance`, and (deliberately) `is_valid` is NOT set — an outlier doesn't count as confirming the cell.
- **Filtering** (`test_too_close_and_out_of_bounds_points_have_zero_effect`): a too-close point and an out-of-map point together leave every layer of the map bit-for-bit identical to before the call.

Run: `cd tests/emap && python3 -m pytest -v` (17 tests total now, including step 3's — all still pass, confirming this step didn't disturb the data structure it builds on).

## Follow-ups for later steps

- `fuse_points` currently updates cells "from scratch" every call — nothing here yet accounts for the UAV moving between updates (that's step 5, map shifting) or ages out stale measurements over time.
- The wall/edge-preservation special case and ray-based visibility cleanup from the reference kernel are real features we deliberately left out of this step (scope: "fusion + outlier rejection" only) — visibility cleanup is its own later step in the roadmap.
- `traversability` is still untouched at its step-3 default (1.0 everywhere) — step 7 computes it for real from slope/step/roughness, which itself depends on having real `elevation` values from this step to compute a slope *of*.
