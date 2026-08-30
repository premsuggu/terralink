# Step 5: Map Shifting (UAV-Centric)

**Package**: `src/emap/`
**Goal**: Let the fixed-size elevation map re-center itself on the UAV as it flies, instead of staying fixed at the origin forever, without corrupting any data that's still within view or mislabeling which edge is genuinely new.
**Status**: ✅ Complete and verified.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 10 (Why the map has to "follow" the robot) — this document is the code walkthrough for exactly that idea.

---

## 1. The one thing this step is most likely to get wrong

Our own project roadmap calls this step out by name as needing extra care: re-centering a 2D array is the classic place to accidentally swap rows and columns, or to blank the wrong edge. Both mistakes can look *almost* right in casual testing (the map still runs, still produces some numbers) and only reveal themselves later as corrupted-looking geometry. This document exists mainly to show the actual reasoning — worked through with real numbers — rather than just asserting the final formulas are correct.

## 2. A worked example, verified before being written down here

Everything below was actually run, not just reasoned about on paper (see `tests/emap/test_elevation_map.py::TestMapShifting` for the same checks as real, passing tests). Take a tiny, easy-to-read 10×10 map (`resolution=1.0`, so 1 cell = 1 meter) and fill every cell with a unique number — `row*100 + col` — purely so we can tell at a glance where each cell's data ends up:

```
[[  0   1   2   3   4   5   6   7   8   9]
 [100 101 102 103 104 105 106 107 108 109]
 [200 201 202 203 204 205 206 207 208 209]
 ...
 [500 501 502 503 504 505 506 507 508 509]
 ...
 [900 901 902 903 904 905 906 907 908 909]]
```
The map starts centered at world `(0, 0)`, so cell `(row=5, col=5)` — value `505` — is the center cell. Now move the UAV +1 meter in X only: `emap.move_to(1.0, 0.0)`. The result:

```
[[  1   2   3   4   5   6   7   8   9   0]
 [101 102 103 104 105 106 107 108 109   0]
 [201 202 203 204 205 206 207 208 209   0]
 ...
 [501 502 503 504 505 506 507 508 509   0]
 ...
 [901 902 903 904 905 906 907 908 909   0]]
```
Two things to check by hand, both directly testing "did the geometry actually stay correct":
- World point `(0, 0)` — the *old* center, value `505` — should still exist somewhere, just no longer at the array's center (since the center moved past it). `emap.world_to_grid(0.0, 0.0)` after the move returns `(row=5, col=4)`, and that cell holds `505.0` — correct: it's now one cell to the *left* of center, since the center moved one cell to the *right* of it.
- The *new* center, world point `(1, 0)`, should hold whatever used to be one cell further east under the old center — value `506`. `emap.world_to_grid(1.0, 0.0)` returns `(row=5, col=5)` (the array's actual center, as always), holding `506.0`. Correct.

And the **entire last column got reset to `0`** — that's `_reset_region` correctly blanking the one column of cells this 1-meter move actually brought into view for the first time.

## 3. Why the array shifts in the *opposite* direction of the UAV's motion

This is the part that reads backwards if you don't stop and think about it physically: the UAV moved **+1 in x**, but the array's *content* had to shift by **-1** (`np.roll(..., shift=-1, axis=col)`). Why the opposite sign? The map isn't a window painted onto fixed ground — it's the other way around: the array's cells are defined *relative to wherever the center currently is*. When the center moves +1 east, a specific fixed point in the world that used to be, say, 3 cells east of the (old) center is now only 2 cells east of the (new) center — every existing point's position *relative to center* decreases by 1 when the center itself moves +1. That's a shift of -1, matching what the code does.

## 4. Why the "which edge is new" logic is keyed off the *roll's* shift value, not the raw motion

This was the actual bug caught and fixed while building this step (not a hypothetical — an early version of this code got it backwards). It's tempting to reason "the UAV moved +x, so obviously the +x (high-column) edge is the new one" and blank based on the sign of the *raw motion*. But the code doesn't have "the raw motion" sitting around at the point it needs to decide what to blank — it has the *roll shift value*, which is the *negative* of the motion (Section 3). Keying the blanking logic off the wrong sign silently blanks legitimate, still-valid data on one side while leaving actual stale, wrapped-around garbage un-blanked on the other — exactly the kind of subtle, plausible-looking-until-you-check-the-numbers bug this step is designed to catch. The fix (and the reason `elevation_map.py`'s comments spell this out explicitly) is to base the blanking decision on `row_shift`/`col_shift` — the literal values passed to `np.roll` — not on `delta_row`/`delta_col`.

Confirmed by the worked example above: `move_to(1.0, 0.0)` computes `col_shift = -1` (negative), and a *negative* roll shift is what correctly identifies the array's **tail** (`array[..., col_shift:]`, i.e. the last column) as the freshly-exposed strip — which is exactly what the numbers above showed got reset. Moving in `-x` instead produces `col_shift = +1` (positive) and correctly blanks the *head* instead (`tests/emap/test_elevation_map.py::test_negative_x_shift_blanks_the_near_column` checks this directly).

## 5. Snapping to whole cells

```python
delta_col = int(round(delta_x / self.resolution))
...
self.center_x += delta_col * self.resolution   # NOT the raw delta_x
```
If the UAV moves 0.03m and the map only updates its center by that raw amount, after a few thousand small updates the center would no longer sit on a clean multiple of the resolution — and `world_to_grid`'s exact round-trip guarantee from step 3 depends on the grid always being aligned to a fixed lattice. Rounding the *requested* move to whole cells, and only ever moving the center by that rounded amount (not the original request), keeps the lattice perfectly aligned forever, at the cost of the map's center trailing the UAV's true position by less than half a cell at any moment — a deliberate, negligible tradeoff for exactness elsewhere.

## 6. A real edge case the simple version misses: shifting further than the whole map

If the UAV teleported 100m on a 10m-wide map, *nothing* in the old array is still relevant. The natural-looking code (`array[:, :shift, :] = default` for a shift bigger than the array) doesn't actually blank everything in that case — Python slicing silently clips an out-of-range slice down to an empty, no-op slice instead of "the whole array." `move_to` checks for this explicitly (`abs(delta_row) >= cell_n or abs(delta_col) >= cell_n`) and falls back to a full `reset()` — verified in `test_shift_larger_than_the_map_falls_back_to_a_full_reset` by checking the result against a genuinely brand-new map, cell for cell.

## 7. One deliberate difference from `src/d1`, and why

The preferred reference resets a freshly-exposed cell's `elevation`/`variance` to "unobserved" but leaves `traversability` at `0.0` there — different from the `1.0` a *brand-new* map starts with (`00_concepts.md`'s "assume traversable until shown otherwise" default). Nothing in the reference explains this asymmetry, and we can't be sure it's intentional rather than an artifact of how its padding helper happens to be structured. Rather than silently copy a difference we can't independently justify, `_reset_region` in `elevation_map.py` applies **the exact same defaults** to a freshly-exposed cell as `reset()` uses for the whole map — a newly-revealed cell is, after all, in exactly the same state of "we've simply never looked here" as a cell in a map that was just constructed. One rule, defined in one place, used everywhere a cell needs to become "unobserved" — simpler to explain, simpler to test, and it can't accidentally drift out of sync with itself the way two separately-maintained default lists could.

## 8. Verification

`cd tests/emap && python3 -m pytest -v` — 26 tests total now (12 from step 3, 5 from step 4, 9 new here), all passing. The new ones, and what each specifically rules out:

- **Pure x-shift / pure y-shift** (`test_pure_x_shift_moves_content_along_columns_not_rows`, `test_pure_y_shift_moves_content_along_rows_not_columns`): the exact numbers from Section 2, run as real assertions. Between the two, a transposed row/col bug cannot pass both.
- **Positive vs. negative x-shift blanking** (`test_positive_x_shift_blanks_the_far_column_not_the_near_one`, `test_negative_x_shift_blanks_the_near_column`): Section 4's reasoning, checked in both directions.
- **Diagonal shift** (`test_diagonal_shift_blanks_an_l_shaped_region_including_the_corner`): both a row-band and a column-band blanked at once, including their overlapping corner.
- **Sub-cell snapping** (`test_submeter_move_snaps_center_to_the_nearest_whole_cell`): Section 5.
- **True no-op and huge-shift-resets** (`test_moving_to_the_current_center_is_a_true_no_op`, `test_shift_larger_than_the_map_falls_back_to_a_full_reset`): the two "boundary of normal operation" cases, Section 6 among them.
- **Coordinate transform still exact after moving** (`test_coordinate_transform_is_still_correct_after_moving`): re-runs step 3's own "does the center map to the array's middle" check, now after the map has actually moved — ties this step's correctness back to the same public API a real caller (step 6's ROS node) will use, not just to internal array positions.

## Follow-ups for later steps

- `move_to` only handles X/Y re-centering. A Z (altitude) equivalent — shifting the whole map's height reference as the UAV's own altitude reference drifts — is a `src/d1` feature (`shift_map_z`) we haven't needed yet and isn't part of this step's scope.
- Step 6 (the live ROS node) will be the first real caller of `move_to`, invoking it every update with the UAV's live position from the TF tree step 2 already verified.
