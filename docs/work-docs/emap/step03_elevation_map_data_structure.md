# Step 3: Elevation Map Core Data Structure (CPU)

**Package**: `src/emap/`
**Goal**: Build the elevation map's actual grid data structure — the thing step 4 onward will fill in with real sensor measurements — as plain, fast, well-tested NumPy code with no ROS or Gazebo involved at all.
**Status**: ✅ Complete and verified.
**Read first**: [`00_concepts.md`](00_concepts.md), especially the new Section 8 (2.5D height grids and uncertainty) — this document is the code walkthrough for exactly the ideas introduced there.

---

## 1. Why does this step have no ROS or Gazebo in it at all?

Steps 1 and 2 were about the *simulation* — getting a UAV and a camera working. This step is about the *algorithm* — the actual data structure elevation mapping is built on. AGENTS.md's rule for this project is "CPU-first": get the core logic fully correct and unit-tested using plain, ordinary Python/NumPy, completely independent of any simulator, before connecting it to real (or simulated) sensor data in step 4, and before ever attempting a GPU version. This has a very practical benefit: the tests in this step run in about a tenth of a second (see Section 5) and need nothing installed beyond NumPy — no Gazebo, no ROS — because there is nothing simulation-shaped in this code to test.

## 2. Walkthrough: `emap/elevation_map.py`

### 2.1 One array, four named layers

```python
LAYER_INDEX = {
    "elevation": 0,
    "variance": 1,
    "is_valid": 2,
    "traversability": 3,
}
```
The map's actual data is one NumPy array of shape `(4, cell_n, cell_n)` — four stacked 2D grids, one per layer, all the same size. `LAYER_INDEX` is just a lookup table from a readable name to "which of the 4 stacked grids is this". The `ElevationMap.layer("elevation")` method uses it so that the rest of the code (and step 4's fusion logic, and everyone reading it) never has to remember "layer 0 means elevation" — it just asks for `"elevation"` by name.

Why one stacked array instead of four separate NumPy arrays (`self.elevation = ...`, `self.variance = ...`, etc.)? Because later steps need to do the same operation to *every* layer at once — e.g. step 5 ("map shifting") moves the whole map by one array-shift operation as the UAV flies, and a future GPU step moves the whole map onto the GPU with one array-copy. Doing that once to a single stacked array is simpler and faster than repeating it four times over four separate arrays that have to be kept in sync by hand.

### 2.2 What each layer means (recapping `00_concepts.md` Section 8)

- **`elevation`**: terrain height in meters at this cell. Starts at 0.
- **`variance`**: how uncertain that height estimate currently is (meters²). Starts at a large `initial_variance` (default 10.0) — "we have no idea yet" — so that step 4's very first real measurement of a cell immediately dominates this placeholder.
- **`is_valid`**: has this cell ever received a real measurement? Starts at 0 (false). This will matter once we need to distinguish "the ground here really is at height 0" from "we've simply never looked here so it defaulted to 0."
- **`traversability`**: 0 (impassable) to 1 (easy). Starts at 1.0 — optimistic by default, since we haven't measured any slope/roughness yet (that's step 7's job); the same "assume fine until proven otherwise" convention the `src/d1` reference uses.

### 2.3 Sizing the grid: `cell_n`

```python
self.cell_n = int(round(self.length / self.resolution))
```
If you ask for a 10 m map at 0.1 m resolution, that's exactly 100 cells — easy. But `length / resolution` isn't always a whole number (e.g. a 10 m map at 0.3 m resolution is 33.33... cells) — `round()` picks the nearest whole cell count (33) rather than `int()`/`floor()`, which would always silently round *down*, quietly making the map a bit smaller than what was actually asked for. `tests/emap/test_elevation_map.py::test_rounds_rather_than_truncates_cell_count` checks this directly.

## 3. Walkthrough: `emap/utils/coord_transform.py` — converting real positions to grid cells

The map needs to answer: "a sensor just measured a point at real-world position (x, y) — which cell of the array does that correspond to?" and the reverse. Two functions handle this, `world_to_grid` and `grid_to_world`.

### 3.1 The grid is *centered*, not corner-anchored

The map doesn't start at (0, 0) in one corner — it's centered on a point (`center_x`, `center_y`, currently always `(0, 0)` until step 5 introduces map-following). For a 10 m map, that means it covers from -5 m to +5 m in both X and Y, with the center point landing in the middle of the array, not the top-left corner. This matters later: as the UAV flies, the map will be re-centered on it (step 5) so it always covers the ground nearby, rather than being forced to cover a fixed, pre-decided rectangle of the world forever.

### 3.2 A worked example (real, verified output — not asserted)

For our 10 m / 0.1 m-resolution map (`cell_n = 100`), running the actual code on a few points gives:

```
world (0.0, 0.0)     -> grid (row=50,  col=50)     -> back to world (0.000, 0.000)
world (2.3, -1.7)    -> grid (row=33,  col=73)      -> back to world (2.300, -1.700)
world (-4.95, 4.95)  -> grid (row=100, col=0)       -> back to world (-5.000, 5.000)
```
Reading the math for the second row by hand: `col = round((2.3 - 0)/0.1 + 100/2) = round(23 + 50) = 73`. `row = round((-1.7 - 0)/0.1 + 50) = round(-17 + 50) = 33`. Converting back multiplies by `0.1` and undoes the `+50` shift, exactly recovering `(2.3, -1.7)`.

The third row is the interesting one. `(-4.95, 4.95)` is right at the very edge of this 10 m map (which spans -5.0 to +5.0). Its column comes out to exactly `0` (the first valid column) — but its row comes out to `100`, which is **one past the last valid row** (valid rows are `0..99` for a 100-cell grid; `100` doesn't exist). This isn't a bug — it's real, correct arithmetic for a point sitting exactly on the map's boundary, rounding to just outside it. It's *exactly* the situation `in_bounds()` exists to catch:
```
>>> emap.in_bounds(100, 0)
False
>>> emap.in_bounds(*emap.world_to_grid(-4.94, 4.94))   # 1cm further inside the map
True
```
The lesson: **never index the map array with the output of `world_to_grid` without checking `in_bounds()` first.** A real sensor will routinely report points beyond the map's edge (distant terrain, sky, whatever), and NumPy will not stop you from accidentally indexing out of range (or, worse, silently wrapping around to the *other* side of the array with a negative index) if you skip this check. Step 4's fusion code will filter every point through `in_bounds()` before touching the map.

### 3.3 Written to accept a single point *or* a whole array of points

Neither function has an "if this is an array do X, else do Y" branch anywhere — `np.asarray(x) - center_x`, for instance, works identically whether `x` is one float or a NumPy array of fifty thousand floats, because NumPy already applies arithmetic element-wise to arrays (this is called *vectorization*). This isn't a nice-to-have: step 4 will need to convert an entire point cloud — tens of thousands of points from one camera frame — into grid indices on every single sensor update. Doing that with a Python `for` loop calling a scalar version of this function once per point would be dramatically slower than calling it once with the whole array, and would only get worse as point clouds get denser. `test_vectorized_matches_scalar_calls_one_at_a_time` in the test suite confirms the array path and the one-at-a-time path agree exactly, not just that the array path "runs without crashing."

## 4. Verification

```
cd tests/emap
python3 -m pytest test_elevation_map.py -v
```
All 12 tests pass in about 0.1 seconds (no ROS, no Gazebo, nothing simulated — this is the "fast, isolated unit test" step 1/2's verifications couldn't be, since those steps were inherently about the simulator itself). What they check, and why each one matters:

- **Shape/defaults** (`test_cell_count_matches_length_and_resolution`, `test_rounds_rather_than_truncates_cell_count`, `test_default_layer_values`): the map is the size it claims to be, and every layer starts at the value Section 2.2 describes — the foundation everything else builds on.
- **`layer()` returns a live view** (`test_layer_returns_a_live_view_not_a_copy`): confirms in-place mutation actually works, which is how step 4 will write measurements into the map efficiently (rather than constantly rebuilding whole arrays).
- **`reset()`** (`test_reset_restores_defaults`): mutate every layer, reset, confirm everything's back to the Section 2.2 defaults.
- **Coordinate round-trip** (`test_map_center_is_the_middle_cell`, `test_round_trip_recovers_the_same_point`): world → grid → world recovers the original point (up to one cell's width, since `grid_to_world` returns a cell's center) — the same "invertibility" check both reference implementations relied on.
- **Vectorization is actually correct, not just fast** (`test_vectorized_matches_scalar_calls_one_at_a_time`): see Section 3.3.
- **Boundary handling** (`test_in_bounds_flags_points_outside_the_map`, `test_in_bounds_works_on_arrays_too`): includes the exact edge-of-map case worked through by hand in Section 3.2.

## Follow-ups for later steps

- Step 4 (Bayesian fusion) is the first real *consumer* of this data structure: it will take a point cloud (already transformed into the map's frame via the TF tree from step 2), convert every point to grid indices with `world_to_grid`, drop the ones that fail `in_bounds`, and update `elevation`/`variance`/`is_valid` for the rest using the uncertainty-combining approach Section 8 of `00_concepts.md` previews.
- `center_x`/`center_y` stay fixed at `(0, 0)` until step 5 ("map shifting") teaches the map to re-center itself on the UAV as it flies.
- `traversability` stays at its optimistic default of 1.0 everywhere until step 7 computes it from real slope/step/roughness.
