"""Unit tests for emap.fusion.fuse_points.

Every expected number here was worked out independently by hand (written as
plain arithmetic in each test, not by calling fuse_points itself) and then
cross-checked against the real function before being committed - see
docs/work-docs/emap/step04_bayesian_fusion.md for the same numbers explained
step by step. Still pure NumPy - no ROS, no Gazebo.
"""
import numpy as np
import pytest

from emap.elevation_map import ElevationMap
from emap.fusion import fuse_points

# Shared, deliberately simple parameters used by every test below - a 10m map
# with 1m cells (cell_n=10) keeps the arithmetic easy to follow by hand.
RESOLUTION = 1.0
LENGTH = 10.0
INITIAL_VARIANCE = 10.0
SENSOR_NOISE_FACTOR = 0.01
MAHALANOBIS_THRESH = 2.0
OUTLIER_VARIANCE = 1.0
MIN_VALID_DISTANCE = 0.1


def new_map() -> ElevationMap:
    return ElevationMap(resolution=RESOLUTION, length=LENGTH, initial_variance=INITIAL_VARIANCE)


def fuse(emap, points_xyz, sensor_origin):
    """Thin wrapper binding the shared parameters above, so each test only
    has to specify what actually varies for it: the points and where the
    sensor was.
    """
    fuse_points(
        emap,
        np.asarray(points_xyz, dtype=np.float64),
        np.asarray(sensor_origin, dtype=np.float64),
        sensor_noise_factor=SENSOR_NOISE_FACTOR,
        mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE,
        min_valid_distance=MIN_VALID_DISTANCE,
    )


def test_single_point_matches_hand_computed_bayesian_update():
    """One point, one cell, starting from the map's default "no idea yet"
    prior (elevation=0, variance=INITIAL_VARIANCE). This is the simplest
    possible case and the one worked through step by step in
    step04_bayesian_fusion.md.
    """
    emap = new_map()
    point = [0.0, 0.0, 1.0]          # measured height z = 1.0
    sensor_origin = [0.0, 0.0, 3.0]  # sensor is 2m above the point (in Z)

    fuse(emap, [point], sensor_origin)

    # By hand: range^2 = (0-0)^2 + (0-0)^2 + (1.0-3.0)^2 = 4.0
    #          measurement variance v = sensor_noise_factor * range^2 = 0.01 * 4.0 = 0.04
    #          prior_h = 0, prior_v = 10 (the map's defaults)
    #          new_h = (prior_h*v + z*prior_v) / (prior_v + v) = (0*0.04 + 1.0*10) / 10.04
    #          new_v = (prior_v*v) / (prior_v + v) = (10*0.04) / 10.04
    v = SENSOR_NOISE_FACTOR * 4.0
    expected_h = (0.0 * v + 1.0 * INITIAL_VARIANCE) / (INITIAL_VARIANCE + v)
    expected_v = (INITIAL_VARIANCE * v) / (INITIAL_VARIANCE + v)

    row, col = emap.world_to_grid(0.0, 0.0)
    assert emap.layer("elevation")[row, col] == pytest.approx(expected_h, rel=1e-5)
    assert emap.layer("variance")[row, col] == pytest.approx(expected_v, rel=1e-5)
    assert emap.layer("is_valid")[row, col] == 1.0

    # The fused height moved from the prior (0) toward the measurement (1.0),
    # and the fused variance is smaller than either input on its own -
    # combining two independent estimates can only increase confidence.
    assert 0.0 < expected_h < 1.0
    assert expected_v < min(INITIAL_VARIANCE, v)


def test_repeated_measurement_converges():
    """Fusing the same true height several times in a row should make the
    map progressively more confident (shrinking variance) and progressively
    closer to the truth (shrinking error) - not just "eventually right", but
    monotonically improving on every single update.
    """
    emap = new_map()
    true_height = 2.0
    point = [0.0, 0.0, true_height]
    sensor_origin = [0.0, 0.0, 5.0]
    row, col = emap.world_to_grid(0.0, 0.0)

    previous_variance = INITIAL_VARIANCE
    previous_error = abs(0.0 - true_height)  # map starts at elevation=0
    for _ in range(5):
        fuse(emap, [point], sensor_origin)
        variance = emap.layer("variance")[row, col]
        error = abs(emap.layer("elevation")[row, col] - true_height)

        assert variance < previous_variance, "each repeated measurement should reduce uncertainty"
        assert error <= previous_error, "each repeated measurement should not move us further from the truth"
        previous_variance, previous_error = variance, error


def test_two_points_in_the_same_cell_are_averaged_not_overwritten():
    """The whole reason fuse_points uses np.add.at instead of plain fancy-
    index assignment: if two points from the same batch land in the same
    cell, both must count. Plain `array[rows, cols] = values` would silently
    keep only the second point's result and throw the first one away.
    """
    emap = new_map()
    sensor_origin = [0.0, 0.0, 3.0]
    # Two distinct points, close enough together to round to the same cell
    # at 1m resolution (both are within 0.5m of the map's center cell).
    point_a = [0.1, 0.1, 1.0]
    point_b = [-0.1, -0.1, 1.2]

    row_a, col_a = emap.world_to_grid(point_a[0], point_a[1])
    row_b, col_b = emap.world_to_grid(point_b[0], point_b[1])
    assert (row_a, col_a) == (row_b, col_b), "test setup assumption: both points land in one cell"

    fuse(emap, [point_a, point_b], sensor_origin)

    # Hand-compute each point's OWN fused estimate against the shared prior
    # (elevation=0, variance=10), exactly as fuse_points does internally,
    # then average the two - that average is what a single shared cell
    # should end up holding after both points landed in it in one batch.
    def fused_alone(point):
        offset = np.array(point) - np.array(sensor_origin)
        v = SENSOR_NOISE_FACTOR * float(np.sum(offset * offset))
        new_h = (0.0 * v + point[2] * INITIAL_VARIANCE) / (INITIAL_VARIANCE + v)
        new_v = (INITIAL_VARIANCE * v) / (INITIAL_VARIANCE + v)
        return new_h, new_v

    h_a, v_a = fused_alone(point_a)
    h_b, v_b = fused_alone(point_b)
    expected_h = (h_a + h_b) / 2.0
    expected_v = (v_a + v_b) / 2.0

    assert emap.layer("elevation")[row_a, col_a] == pytest.approx(expected_h, rel=1e-5)
    assert emap.layer("variance")[row_a, col_a] == pytest.approx(expected_v, rel=1e-5)
    assert emap.layer("is_valid")[row_a, col_a] == 1.0


def test_outlier_is_rejected_but_still_raises_variance():
    """A measurement wildly inconsistent with a confident prior must not be
    allowed to move the height - but it SHOULD make the cell less certain
    (something unexpected just happened here), by exactly `outlier_variance`.
    """
    emap = new_map()
    row, col = emap.world_to_grid(0.0, 0.0)

    # abs(prior_h - z) = abs(0 - 100) = 100, which is far bigger than
    # prior_v * mahalanobis_thresh = 10 * 2.0 = 20 -> this must be rejected.
    fuse(emap, [[0.0, 0.0, 100.0]], sensor_origin=[0.0, 0.0, 3.0])

    assert emap.layer("elevation")[row, col] == pytest.approx(0.0)  # unchanged
    assert emap.layer("variance")[row, col] == pytest.approx(INITIAL_VARIANCE + OUTLIER_VARIANCE)
    assert emap.layer("is_valid")[row, col] == 0.0  # an outlier does not "confirm" the cell


def test_too_close_and_out_of_bounds_points_have_zero_effect():
    """Two different reasons a point should be silently ignored: closer to
    the sensor than `min_valid_distance` (real depth sensors are unreliable
    at very short range), and simply outside the map entirely.
    """
    emap = new_map()
    elevation_before = emap.layer("elevation").copy()
    variance_before = emap.layer("variance").copy()
    is_valid_before = emap.layer("is_valid").copy()

    too_close = [0.0, 0.0, 2.95]      # 0.05m from sensor_origin below - under MIN_VALID_DISTANCE=0.1
    out_of_bounds = [1000.0, 1000.0, 5.0]  # nowhere near this 10m-wide map

    fuse(emap, [too_close, out_of_bounds], sensor_origin=[0.0, 0.0, 3.0])

    assert np.array_equal(emap.layer("elevation"), elevation_before)
    assert np.array_equal(emap.layer("variance"), variance_before)
    assert np.array_equal(emap.layer("is_valid"), is_valid_before)
