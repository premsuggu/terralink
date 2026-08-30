"""Unit tests for emap.elevation_map.ElevationMap and emap.utils.coord_transform.

Pure NumPy - no ROS, no Gazebo, nothing simulated. These tests exist to prove
the *data structure itself* is correct before step 4 builds real sensor
fusion on top of it: wrong array shapes, an off-by-one in the coordinate
math, or a broken round-trip here would otherwise silently corrupt every
later step.
"""
import numpy as np
import pytest

from emap.elevation_map import ElevationMap
from emap.utils.coord_transform import in_bounds


@pytest.fixture
def emap():
    """A modest 10m x 10m map at 0.1m/cell (100x100 cells) - big enough to
    exercise center/edge/off-center cases without making the tests slow.
    """
    return ElevationMap(resolution=0.1, length=10.0, initial_variance=10.0)


class TestElevationMapShapeAndDefaults:
    def test_cell_count_matches_length_and_resolution(self, emap):
        # 10m / 0.1m per cell = 100 cells per side, exactly.
        assert emap.cell_n == 100
        assert emap.shape == (100, 100)

    def test_rounds_rather_than_truncates_cell_count(self):
        # 10.0 / 0.3 = 33.33... - this must round to 33, not silently
        # truncate to 33 via int() (which would happen to give the same
        # answer here) or, worse, floor to a smaller map than requested for
        # inputs where rounding and truncating actually disagree.
        m = ElevationMap(resolution=0.3, length=10.0)
        assert m.cell_n == round(10.0 / 0.3)

    def test_default_layer_values(self, emap):
        # Every cell starts "unobserved": zero height, the placeholder
        # variance, not-yet-valid, and optimistically traversable.
        assert np.all(emap.layer("elevation") == 0.0)
        assert np.all(emap.layer("variance") == 10.0)
        assert np.all(emap.layer("is_valid") == 0.0)
        assert np.all(emap.layer("traversability") == 1.0)

    def test_layer_returns_a_live_view_not_a_copy(self, emap):
        # Confirms map.layer(...) can be used to actually update the map in
        # place, which is exactly how step 4's fusion code will write into it.
        emap.layer("elevation")[5, 5] = 1.23
        assert emap.layer("elevation")[5, 5] == pytest.approx(1.23)

    def test_reset_restores_defaults(self, emap):
        # Mutate every layer, then confirm reset() puts them all back.
        emap.layer("elevation")[:] = 99.0
        emap.layer("variance")[:] = 0.0
        emap.layer("is_valid")[:] = 1.0
        emap.layer("traversability")[:] = 0.0

        emap.reset()

        assert np.all(emap.layer("elevation") == 0.0)
        assert np.all(emap.layer("variance") == 10.0)
        assert np.all(emap.layer("is_valid") == 0.0)
        assert np.all(emap.layer("traversability") == 1.0)


class TestCoordinateTransform:
    def test_map_center_is_the_middle_cell(self, emap):
        # The world point the map is centered on (0, 0) must land in the
        # middle of the array - this is the whole point of a "centered" grid.
        row, col = emap.world_to_grid(0.0, 0.0)
        assert row == emap.cell_n // 2
        assert col == emap.cell_n // 2

    @pytest.mark.parametrize(
        "x, y",
        [
            (0.0, 0.0),      # dead center
            (1.5, -2.3),     # generic off-center point
            (-4.9, 4.9),     # near the edge, still inside the 10m map
        ],
    )
    def test_round_trip_recovers_the_same_point(self, emap, x, y):
        # world -> grid -> world should land back on (x, y), up to the size
        # of one cell (0.1m here) - grid_to_world returns a cell's CENTER,
        # so it can only recover the original point that precisely.
        row, col = emap.world_to_grid(x, y)
        x2, y2 = emap.grid_to_world(row, col)
        assert x2 == pytest.approx(x, abs=emap.resolution / 2)
        assert y2 == pytest.approx(y, abs=emap.resolution / 2)

    def test_vectorized_matches_scalar_calls_one_at_a_time(self, emap):
        # This is the actual point of writing world_to_grid to accept arrays:
        # confirm it's not just "not crashing" on an array, but computing
        # exactly the same thing a plain Python loop over scalars would.
        rng = np.random.default_rng(0)
        xs = rng.uniform(-4.0, 4.0, size=50)
        ys = rng.uniform(-4.0, 4.0, size=50)

        rows_vec, cols_vec = emap.world_to_grid(xs, ys)
        rows_loop = [emap.world_to_grid(x, y)[0] for x, y in zip(xs, ys)]
        cols_loop = [emap.world_to_grid(x, y)[1] for x, y in zip(xs, ys)]

        assert list(rows_vec) == rows_loop
        assert list(cols_vec) == cols_loop

    def test_in_bounds_flags_points_outside_the_map(self, emap):
        # (0, 0) in world coords is the map's own center - always in bounds.
        # 1000m away is nowhere near this 10m-wide map - always out of bounds.
        row_in, col_in = emap.world_to_grid(0.0, 0.0)
        row_out, col_out = emap.world_to_grid(1000.0, 1000.0)

        assert emap.in_bounds(row_in, col_in) == True  # noqa: E712 (numpy bool)
        assert emap.in_bounds(row_out, col_out) == False  # noqa: E712

    def test_in_bounds_works_on_arrays_too(self):
        # A small, easy-to-reason-about grid: 10 cells, valid indices 0..9.
        rows = np.array([-1, 0, 5, 9, 10])
        cols = np.array([0, 0, 5, 9, 3])
        expected = np.array([False, True, True, True, False])
        assert np.array_equal(in_bounds(rows, cols, cell_n=10), expected)


@pytest.fixture
def small_map():
    """A 10x10 map at 1m/cell - deliberately tiny and round-numbered so a
    human can trace exactly which cell a value should end up in by hand,
    which is the whole point of the map-shifting tests below (they exist to
    catch an axis-swap/direction bug, and that's much easier to verify with
    "does the 7 end up here or there" than with a 100x100 grid of floats).
    """
    return ElevationMap(resolution=1.0, length=10.0, initial_variance=10.0)


def _mark_every_cell_uniquely(emap):
    """Fill `elevation` with `row*100 + col` (so every cell's original
    location can be read straight off its value) and mark every cell
    `is_valid` - so after a move, any cell showing `is_valid == 0` is
    unambiguously one `move_to` just blanked as newly-exposed.
    """
    rows, cols = np.meshgrid(np.arange(emap.cell_n), np.arange(emap.cell_n), indexing="ij")
    emap.layer("elevation")[:] = rows * 100 + cols
    emap.layer("is_valid")[:] = 1.0


class TestMapShifting:
    def test_pure_x_shift_moves_content_along_columns_not_rows(self, small_map):
        # This is the test that would fail immediately if rows/cols ever got
        # swapped: moving purely in X must only ever change which COLUMN a
        # world point's data lives in, never its row.
        _mark_every_cell_uniquely(small_map)

        small_map.move_to(1.0, 0.0)  # +1m in x, 0 in y

        # The point at world (0,0) - the OLD center, originally cell (5,5)
        # with value 505 - must still be findable, now one column further
        # from the new center (since the new center moved past it in +x).
        row, col = small_map.world_to_grid(0.0, 0.0)
        assert (row, col) == (5, 4)
        assert small_map.layer("elevation")[row, col] == 505.0

        # The new center (1, 0) must hold whatever used to be one cell
        # further east under the OLD center - value 506.
        row, col = small_map.world_to_grid(1.0, 0.0)
        assert (row, col) == (5, 5)
        assert small_map.layer("elevation")[row, col] == 506.0

    def test_pure_y_shift_moves_content_along_rows_not_columns(self, small_map):
        # The mirror image of the test above - moving purely in Y must only
        # change ROW, never column. Between this test and the previous one,
        # a transposed row/col bug cannot pass both.
        _mark_every_cell_uniquely(small_map)

        small_map.move_to(0.0, 1.0)  # +1m in y, 0 in x

        row, col = small_map.world_to_grid(0.0, 0.0)
        assert (row, col) == (4, 5)
        assert small_map.layer("elevation")[row, col] == 505.0

        row, col = small_map.world_to_grid(0.0, 1.0)
        assert (row, col) == (5, 5)
        assert small_map.layer("elevation")[row, col] == 605.0

    def test_positive_x_shift_blanks_the_far_column_not_the_near_one(self, small_map):
        # Moving in +x means the map now covers ground further east than
        # before - the genuinely NEW (never-before-seen) strip is the far
        # (high-index) column, not the near one. This is the exact "which
        # side gets blanked" detail that's easy to get backwards (the sign
        # of the roll shift is the NEGATIVE of the direction moved) - see
        # the comment above this logic in elevation_map.py.
        _mark_every_cell_uniquely(small_map)
        small_map.move_to(1.0, 0.0)

        is_valid = small_map.layer("is_valid")
        assert np.all(is_valid[:, -1] == 0.0), "the far column must be the freshly-blanked one"
        assert np.all(is_valid[:, :-1] == 1.0), "every other column should still be the old data"
        # And the freshly-blanked column must actually be reset to the same
        # defaults reset() uses elsewhere, not just is_valid=0.
        assert np.all(small_map.layer("elevation")[:, -1] == 0.0)
        assert np.all(small_map.layer("variance")[:, -1] == small_map.initial_variance)
        assert np.all(small_map.layer("traversability")[:, -1] == 1.0)

    def test_negative_x_shift_blanks_the_near_column(self, small_map):
        # The opposite direction: moving in -x exposes new ground on the
        # LOW-index (near/west) side instead.
        _mark_every_cell_uniquely(small_map)
        small_map.move_to(-1.0, 0.0)

        is_valid = small_map.layer("is_valid")
        assert np.all(is_valid[:, 0] == 0.0)
        assert np.all(is_valid[:, 1:] == 1.0)

    def test_diagonal_shift_blanks_an_l_shaped_region_including_the_corner(self, small_map):
        # Moving in both x and y at once must blank BOTH a row-band and a
        # column-band - including the corner cell where they overlap, which
        # is exactly the case a naive "only handle one axis" implementation
        # would get wrong.
        _mark_every_cell_uniquely(small_map)
        small_map.move_to(2.0, -3.0)  # +2 cells in x, -3 cells in y

        is_valid = small_map.layer("is_valid")
        # -3 in y blanks the first 3 rows (see the pure-y-shift direction
        # logic - moving in -y exposes new ground at the LOW row end).
        assert np.all(is_valid[:3, :] == 0.0)
        # +2 in x blanks the last 2 columns.
        assert np.all(is_valid[:, -2:] == 0.0)
        # Everything NOT in either blanked band must be untouched old data.
        assert np.all(is_valid[3:, :-2] == 1.0)

    def test_submeter_move_snaps_center_to_the_nearest_whole_cell(self, small_map):
        # 0.3m and -0.2m both round to "0 whole cells" at 1m resolution -
        # the center must not move at all, not creep by the raw sub-cell amount.
        small_map.move_to(0.3, -0.2)
        assert small_map.center_x == pytest.approx(0.0)
        assert small_map.center_y == pytest.approx(0.0)

        # 0.6m rounds UP to 1 whole cell.
        small_map.move_to(0.6, 0.0)
        assert small_map.center_x == pytest.approx(1.0)

    def test_moving_to_the_current_center_is_a_true_no_op(self, small_map):
        _mark_every_cell_uniquely(small_map)
        elevation_before = small_map.layer("elevation").copy()

        small_map.move_to(0.0, 0.0)  # already there

        assert np.array_equal(small_map.layer("elevation"), elevation_before)

    def test_shift_larger_than_the_map_falls_back_to_a_full_reset(self, small_map):
        # A jump of 100m on a 10m-wide map means literally nothing from the
        # old map is still in view - naive edge-slicing wouldn't blank
        # everything correctly here (see the comment in elevation_map.py),
        # so this must produce exactly what a brand new map looks like.
        _mark_every_cell_uniquely(small_map)

        small_map.move_to(100.0, 100.0)

        fresh = ElevationMap(resolution=1.0, length=10.0, initial_variance=10.0)
        assert np.array_equal(small_map.layer("elevation"), fresh.layer("elevation"))
        assert np.array_equal(small_map.layer("is_valid"), fresh.layer("is_valid"))

    def test_coordinate_transform_is_still_correct_after_moving(self, small_map):
        # The same "does the center map to the middle of the array" check
        # step 3 ran at construction time (TestCoordinateTransform in this
        # file) - now proven to still hold true after the map has actually
        # moved, tying this step's correctness to the same public API a
        # real caller (step 6's ROS node) will actually use.
        small_map.move_to(3.0, -4.0)
        row, col = small_map.world_to_grid(3.0, -4.0)
        assert (row, col) == (small_map.cell_n // 2, small_map.cell_n // 2)
