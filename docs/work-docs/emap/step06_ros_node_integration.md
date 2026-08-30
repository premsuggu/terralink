# Step 6: ROS 2 Node Integration

**Package**: `src/emap/`
**Goal**: Wire steps 2-5 together into one live ROS 2 node - subscribe to the real point cloud, fuse it into a real `ElevationMap`, and publish the result as a `grid_map_msgs/GridMap`.
**Status**: ✅ Complete and verified - and this is the step where the whole pipeline (steps 1 through 6) was proven correct end to end for the first time, not just piece by piece.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 11 (GridMap) — this document is the code walkthrough plus the verification story for that idea and everything built in steps 1-5.

---

## 1. What this step actually is

Nothing here is a new mapping *concept* - it's the glue that finally makes steps 2 through 5 run together, live, instead of being individually unit-tested or checked with one-off scripts. Concretely: a real `rclpy.Node` that (1) listens for the depth camera's point cloud, (2) asks TF where the camera and the drone actually are right now, (3) calls `ElevationMap.move_to` and `fuse_points` with that real data, and (4) periodically publishes the resulting map.

A dependency blocker came up before any of this could be tested: `grid_map_msgs` (the message type step 4-5's plan always assumed we'd publish) wasn't installed in this environment - not a network or driver issue like earlier steps, just genuinely not installed, and installing it needs `sudo`, which needed you to run it. Once installed, everything below built and ran as designed.

## 2. Walkthrough: `emap/utils/gridmap_utils.py`

Covered conceptually in `00_concepts.md` Section 11 - the code itself:
```python
msg.layout.dim = [
    MultiArrayDimension(label="column_index", size=cols, stride=rows * cols),
    MultiArrayDimension(label="row_index", size=rows, stride=rows),
]
msg.data = arr.flatten(order="F").tolist()
```
`order="F"` (column-major) instead of NumPy's default `order="C"` (row-major) is the one line that matters most here, and it has to agree with the dimension metadata above it - change one without the other and the message still "looks fine" (right size, right type) but decodes into the wrong shape on the receiving end. Verified directly, not just asserted: encoding `[[0, 1, 2], [3, 4, 5]]` produces `[0, 3, 1, 4, 2, 5]` - exactly the hand-worked column-by-column order (column 0's two values, then column 1's, then column 2's).

## 3. Walkthrough: `emap/utils/tf_utils.py`

This is step 2's verification script's own math (quaternion → rotation matrix, then apply rotation + translation to a batch of points), promoted from a one-off script into real, reusable node code, because it was already proven correct there (points landing at the real ground height, down to fractions of a millimeter). Nothing new here except that it's now called every point cloud callback instead of once by hand.

## 4. Walkthrough: `emap/elevation_mapping_node.py`

The point cloud callback, in order:
1. **Look up `map_frame -> camera_frame`** - gives both the rotation+translation to transform the points themselves, AND (its translation alone) the sensor's position for `fuse_points`' distance-based noise model, so one lookup answers two needs.
2. **Look up `map_frame -> base_link`, separately** - used only to re-center the map (`move_to`). This is intentionally a *different* lookup from step 1: the map should follow the drone's *body*, not wobble around chasing the camera's fixed 8cm mounting offset from it.
3. Decode the cloud, transform it into `map_frame`, then `self._map.move_to(...)` followed by `fuse_points(...)` - literally just calling the already-tested functions from steps 4 and 5 with real, live data.
4. A `_lookup_transform` helper tries the message's own exact timestamp first, and falls back to the latest available transform if that's not ready yet - the TF buffer is always a little behind live sim time, and refusing perfectly good data over a few milliseconds of lag would be pure waste (same idea as `src/d1`'s own `safe_lookup_transform`).

## 5. Two real bugs found and fixed while verifying this step

Both were caught because verification here means *reading real published data back and checking it against ground truth* - the same discipline as every previous step - not just "the node started without crashing."

### 5.1 `inf` points from a depth camera that's too close to the ground

The very first test run produced 350 `RuntimeWarning: invalid value encountered in matmul` warnings and an elevation map that stayed entirely at its "never observed" defaults even after the drone had been commanded to move. The cause: at rest, the camera (mounted 8cm below the body) sits closer to the ground than its own 0.1m near-clip plane, so the simulated sensor reports **every single pixel** as "no return" - encoded as `+inf`, not `NaN`. `sensor_msgs_py.point_cloud2.read_points_numpy(..., skip_nans=True)` only filters `NaN`, not `inf` - so those `inf` points passed straight through into `transform_points`, where multiplying an infinite coordinate by the rotation matrix's exact-zero entries (this camera's fixed downward mount has several) produces `NaN` (`0 * inf = NaN` is standard IEEE float behavior). The fix - filter both explicitly, defensively, regardless of how a non-finite value could arise upstream:
```python
finite = np.all(np.isfinite(points_sensor_frame), axis=1)
points_sensor_frame = points_sensor_frame[finite]
```
This also matches why every previous step's verification always commanded the drone to altitude *before* inspecting camera data (steps 2 and this one both hit some version of "too close to the ground, camera sees nothing useful") - here it additionally revealed a real filtering gap worth fixing permanently, not just working around by flying higher.

### 5.2 A phantom "wrong" result that turned out to be two node processes racing on one topic

After fixing 5.1, an early check of the published map showed elevation values around **-3.78** under the drone - wildly wrong (should be ≈0, the real ground height, with the drone hovering ~3.8m up). Rather than assume the transform math was broken, it was checked directly: a standalone script performing the *exact* same TF lookup and transform the node uses returned the correct answer (z ≈ 0, to within microns) at that same moment. That ruled out the math. The actual cause, found with `ps aux`: **two separate `elevation_mapping_node` processes were running at once** - one launched before the fix in 5.1 was rebuilt (and therefore still corrupting its own map with the `inf`/`NaN` bug), one launched after - both publishing to the same `/elevation_map` topic, so a subscriber would land on whichever one's message happened to arrive first. Killing the stale process and re-verifying with exactly one instance running produced the correct result immediately (Section 6). The lesson, worth stating plainly: when a live-system result looks wrong, **check what's actually running** before doubting code that's already been independently verified - `ps aux | grep <node>` is as much a debugging tool here as reading the data itself.

## 6. Verification (the strongest yet - the whole pipeline, at once)

With exactly one clean set of processes running (`ign gazebo`, the bridge, and one `elevation_mapping_node`):

- `colcon build --packages-select emap` succeeds (confirms `grid_map_msgs` really is installed and importable).
- `ros2 topic list` shows `/elevation_map`; the message decodes with the expected 4 layers (`elevation`, `variance`, `is_valid`, `traversability`), matching `LAYER_INDEX`'s order automatically.
- **Hovering at z ≈ 4.42m**, directly above the map's origin: 1,989 of 40,000 cells were `is_valid`, and every one of them read elevation between **-3.5e-7 and +5.6e-7** - the real ground height (0.0), accurate to a fraction of a micron, with small, sane variance (0.0002-0.0037, consistent with many repeated confident measurements from steps 4's convergence behavior). Cells never observed remained at their exact step-3 defaults (elevation 0.0, variance 10.0) - confirmed directly on the raw published data, not assumed.
- **Commanded a lateral move** (`/cmd_vel` linear.x) to x ≈ 2.9m: the published map's `info.pose.position` moved to exactly `(2.9, 0.0)` (snapped to the 0.1m grid resolution, per step 5's whole-cell snapping), and freshly-fused cells after the move again read elevation within a fraction of a micron of the true ground height - proof that `move_to` is being correctly driven by live TF data end-to-end, not just passing its own unit tests in isolation.
- **One honestly-reported open item**: the configured `publish_rate_hz: 2.0` isn't fully achieved in practice - observed average was closer to 0.5-1 Hz with noticeable jitter. `top` during a run showed Gazebo's own process (software-rendering the depth camera at 10 Hz, per the earlier WSL2 GPU-driver workaround) consuming ~80% of a core, while the mapping node used ~20% - plausibly executor/scheduling contention rather than a bug in the mapping logic itself (the *data* published, checked above, is fully correct regardless of how often it arrives). Noted as a follow-up, not blocking this step's goal.

## Follow-ups for later steps

- Investigate the publish-rate shortfall (Section 6) if a steadier rate turns out to matter later - candidates include a multi-threaded executor, or simply lowering `publish_rate_hz`'s default to match what this environment can sustain.
- `traversability` is still untouched at its step-3 default (1.0 everywhere) in the live map too - step 7 computes it for real.
- No RViz visualization has been attempted yet (`ros-humble-grid-map-rviz-plugin` wasn't part of the install run) - the map has only been verified by reading its published data directly, which is a stronger correctness check than a visual glance would be, but a visual sanity check is still worth doing once available.
