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
