# Step 8: Persistent Global Map + Local Rolling Map

**Package**: `src/emap/`
**Goal**: Fix the mismatch between step 5's rolling map (which forgets ground the UAV flew away from) and what's actually needed for the project's real end goal - autonomous UGV navigation, which requires a map that never forgets terrain it's already seen.
**Status**: ✅ Complete and verified - proved live that a bump stays remembered in the global map long after the UAV has flown far enough away that the local map has genuinely forgotten it.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 13 (Local vs. global maps).

---

## 1. The report that started this step

While testing step 7's visualization, it looked like the ground itself - including the bump - was moving along with the UAV in RViz. Before assuming anything, this was checked with real, live numbers (not just reasoning about it): flying directly over the bump, its peak was found in the published map at array position `col=100` (elevation 1.4978m). After flying the UAV 4.4m sideways, the peak was found again - now at `col=56` (exactly 44 cells = 4.4m away, matching the UAV's own movement, cell for cell) - but converting *that* array position back into a real-world coordinate using the map's new center gave **exactly (0.000, 0.000)** both times. The bump's actual data was never moving; only its position *inside the array* was, because the array is a window that follows the UAV (step 5, by design).

So the underlying math was correct - but that didn't make the *behavior* correct for this project's actual purpose. A map that forgets everything outside a 20m window centered on the UAV is exactly the wrong shape of memory for a UGV path planner, which needs to remember terrain well outside its immediate sensor range. That's the real problem this step fixes - not a data bug, but an architecture mismatch with the stated goal (autonomous UGV navigation).

## 2. The fix: two maps, not one

`elevation_mapping_node.py` now keeps **two** `ElevationMap` instances:

- **`self._local_map`** - unchanged from step 6: 20m wide (configurable), re-centered on the UAV's body every point cloud (`move_to`, step 5). Kept for whatever future fast-local-reaction use it's suited for.
- **`self._global_map`** - new: 40m wide by default (configurable, `global_map_length`), centered at `(0, 0)` **forever**. `move_to` is never called on it, anywhere. That single omission is the entire difference - `ElevationMap`, `fuse_points`, and `compute_traversability` (steps 3, 4, 7) needed **zero** code changes to support this; they already work correctly on a map that's never told to move. A point that lands outside the global map's fixed extent is simply dropped by `fuse_points`'s existing `in_bounds` check (step 4) - already-tested behavior, not a new edge case.

Both maps are updated from the exact same sensor data every callback - `_fuse_and_update_traversability(emap, points_map_frame, sensor_origin)` is called once per map, pulled out as its own method specifically so the two maps can never accidentally receive different treatment from a copy-pasted-and-then-diverged version of the same logic.

**Publishing changed**: `/elevation_map` now serves the **global** map (the one that matters for navigation, and the one that behaves the way "static ground, permanently fills in as you explore" was always supposed to look). The local rolling map moved to a new topic, `/elevation_map_local` - kept, not deleted, in case a future local-costmap-style consumer wants it.

## 3. Why two maps instead of just making the one map bigger and calling it done

A simpler fix would have been: just stop calling `move_to` at all, and make the single existing map big enough to cover the whole test area. That would have solved the immediate complaint. It was **not** what was chosen, because of the actual stated future use: **autonomous UGV navigation**. Real navigation stacks (Nav2, this project's own referenced integration target) are built with exactly this local/global split for good reasons that will matter later even though nothing consumes the local map yet: a global map for planning a route across everywhere the robot has ever seen, and a separate, small, fast local map/costmap for immediate obstacle reactions right around the robot - which benefits from staying small and simple rather than being slowed down by a much larger structure. Building the local map's sibling now, while the local map already exists and is fully tested, costs almost nothing (one more `ElevationMap` instance, one more `move_to`-that-never-gets-called) and avoids having to retrofit this same split back in later once an actual planner needs it.

## 4. Verification

Every check below used real published data, the same discipline as every prior step - not just "it looks right now":

- `cd tests/emap && python3 -m pytest -v` - all 33 tests still pass unchanged (no core logic in `elevation_map.py`/`fusion.py`/`traversability.py` was touched).
- `colcon build --packages-select emap` succeeds; `ros2 topic list` shows both `/elevation_map` and `/elevation_map_local`.
- **The definitive live test**: flew directly over the bump (confirmed peak elevation ≈1.498m at world (0,0)), then flew the UAV to x≈12.26m - well past half the local map's 20m width (10m), i.e. far enough that the *local* map's window should have moved on and lost that area entirely. Reading both topics back at that point:
  ```
  GLOBAL (/elevation_map):  center=(0.00, 0.00)   size=40m
    max elevation 1.4976 at world (0.000, 0.000)
    cell at true bump location (0,0): elevation=1.4976, valid=True
  LOCAL (/elevation_map_local):  center=(12.00, 0.10)  size=20m
    true bump location (0,0) is OUTSIDE this map's current window
  ```
  The global map still holds the bump, correctly, at its true location, completely unaffected by how far away the UAV has since flown. The local map has genuinely moved on and no longer has any record of that location at all - exactly the local-forgets/global-remembers split this step set out to build, proven with the same live UAV, the same real sensor data, not two separate hypothetical scenarios.

## Follow-ups for later steps

- Nothing yet consumes `/elevation_map_local` - it exists for whenever a fast local-reaction/costmap use case actually needs it.
- `global_map_length` (40m) is sized for this project's current test worlds; a much larger or truly unbounded operating area would need a different strategy (e.g. tiled/chunked storage) rather than one single large dense array - not needed at this project's current scale.
- GPU acceleration (deferred until CPU correctness was solid) and drift compensation remain the last items on the original roadmap, and now apply to whichever map(s) need them once that work starts.
