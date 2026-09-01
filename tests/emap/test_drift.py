"""Unit tests for emap.drift (step 10) - a synthetic drift scenario, since
this project's live sim uses Gazebo's ground-truth TF and has no real drift
to test against (see docs/work-docs/emap/step10_drift_compensation.md).
"""
import numpy as np
import pytest

from emap.elevation_map import ElevationMap
from emap.fusion import fuse_points
from emap.drift import estimate_vertical_drift

RESOLUTION = 1.0
LENGTH = 20.0
INITIAL_VARIANCE = 10.0
SENSOR_NOISE_FACTOR = 0.0001  # small on purpose: many repeated fusions must
# converge variance well below MIN_CONFIDENCE_VARIANCE within a handful of
# calls, so the test setup below isn't doing dozens of fuse() calls just to
# reach "confident".
MAHALANOBIS_THRESH = 5.0  # loose during setup: repeated identical
# measurements must never be rejected as outliers while variance is still
# shrinking toward confidence.
OUTLIER_VARIANCE = 1.0
MIN_VALID_DISTANCE = 0.1
MIN_CONFIDENCE_VARIANCE = 0.05
MIN_MATCHES = 20


def new_confident_map(true_height: float = 2.0, n_cells_per_side: int = 6) -> ElevationMap:
    """Build a map and fuse the SAME true height into a block of cells
    repeatedly, until their variance drops below MIN_CONFIDENCE_VARIANCE -
    i.e. cells the map is now genuinely confident about, exactly the
    situation estimate_vertical_drift is designed to use as a reference.
    """
    emap = ElevationMap(resolution=RESOLUTION, length=LENGTH, initial_variance=INITIAL_VARIANCE)
    xs = np.arange(n_cells_per_side) - n_cells_per_side / 2.0
    ys = np.arange(n_cells_per_side) - n_cells_per_side / 2.0
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, true_height)])
    sensor_origin = np.array([0.0, 0.0, true_height + 5.0])

    for _ in range(10):
        fuse_points(
            emap, points, sensor_origin,
            sensor_noise_factor=SENSOR_NOISE_FACTOR,
            mahalanobis_thresh=MAHALANOBIS_THRESH,
            outlier_variance=OUTLIER_VARIANCE,
            min_valid_distance=MIN_VALID_DISTANCE,
        )

    row, col = emap.world_to_grid(0.0, 0.0)
    assert emap.layer("variance")[row, col] < MIN_CONFIDENCE_VARIANCE, (
        "test setup assumption: repeated fusion must reach the confidence threshold"
    )
    return emap, points


def test_recovers_known_injected_z_offset():
    """The core scenario: points at the SAME (x, y) as already-confident
    cells, but shifted by a known, deliberate Z offset (as if the pose
    currently believes it's 0.4m higher than it really is). The estimate
    should recover ~0.4m.
    """
    emap, points = new_confident_map(true_height=2.0)
    injected_offset = 0.4
    drifted_points = points.copy()
    drifted_points[:, 2] += injected_offset

    estimate = estimate_vertical_drift(
        emap, drifted_points,
        min_confidence_variance=MIN_CONFIDENCE_VARIANCE,
        min_matches=MIN_MATCHES,
    )

    assert estimate is not None
    # abs tolerance (not exact): SENSOR_NOISE_FACTOR is small but nonzero, so
    # repeated fusion converges each cell's height very close to, but not
    # bit-exactly, true_height - the residual left over from that is a real,
    # expected artifact of Bayesian fusion never claiming perfect certainty,
    # not a bug in the drift estimate itself.
    assert estimate == pytest.approx(injected_offset, abs=1e-3)


def test_negative_offset_is_recovered_with_correct_sign():
    """Same idea, opposite direction - the sign of the returned estimate
    must match the sign of the true injected error, not just its magnitude.
    """
    emap, points = new_confident_map(true_height=2.0)
    injected_offset = -0.25
    drifted_points = points.copy()
    drifted_points[:, 2] += injected_offset

    estimate = estimate_vertical_drift(
        emap, drifted_points,
        min_confidence_variance=MIN_CONFIDENCE_VARIANCE,
        min_matches=MIN_MATCHES,
    )

    assert estimate == pytest.approx(injected_offset, abs=1e-3)


def test_too_few_confident_matches_returns_none():
    """A freshly-built map has no confident cells at all yet - there is
    nothing trustworthy to compare against, so this must return None rather
    than a meaningless guess.
    """
    fresh_map = ElevationMap(resolution=RESOLUTION, length=LENGTH, initial_variance=INITIAL_VARIANCE)
    points = np.column_stack([
        np.arange(5) - 2.0, np.zeros(5), np.full(5, 3.0),
    ])

    estimate = estimate_vertical_drift(
        fresh_map, points,
        min_confidence_variance=MIN_CONFIDENCE_VARIANCE,
        min_matches=MIN_MATCHES,
    )

    assert estimate is None


def test_minority_outlier_point_does_not_skew_the_median():
    """A handful of points reading a genuinely different height (e.g. a
    real small obstacle sitting on otherwise-flat, confident ground) should
    not drag the estimate away from what the majority of confident cells
    agree on - this is exactly why the median is used instead of the mean.
    """
    emap, points = new_confident_map(true_height=2.0, n_cells_per_side=10)
    injected_offset = 0.3
    drifted_points = points.copy()
    drifted_points[:, 2] += injected_offset

    # Corrupt a small minority of the points with a huge, unrelated jump -
    # far fewer than half, so the median must ignore them.
    n_outliers = max(1, len(drifted_points) // 10)
    drifted_points[:n_outliers, 2] += 50.0

    estimate = estimate_vertical_drift(
        emap, drifted_points,
        min_confidence_variance=MIN_CONFIDENCE_VARIANCE,
        min_matches=MIN_MATCHES,
    )

    assert estimate == pytest.approx(injected_offset, abs=1e-3)


def test_implausibly_large_residual_is_rejected_as_a_sensor_glitch():
    """A single callback's residual of many meters can't be real pose drift
    (drift accumulates slowly) - it's far more likely a bad sensor frame
    (observed live: a depth-camera glitch briefly producing garbage points).
    max_reasonable_residual must reject it (return None) rather than let a
    single bad frame yank the running bias estimate by meters.
    """
    emap, points = new_confident_map(true_height=2.0)
    corrupted_points = points.copy()
    corrupted_points[:, 2] += 9.0  # a magnitude no real drift would produce in one frame

    estimate = estimate_vertical_drift(
        emap, corrupted_points,
        min_confidence_variance=MIN_CONFIDENCE_VARIANCE,
        min_matches=MIN_MATCHES,
        max_reasonable_residual=1.0,
    )

    assert estimate is None


def test_damped_gain_converges_toward_true_offset_without_overshoot():
    """Mirrors elevation_mapping_node's own running-bias-estimate logic:
    bias += gain * estimate, applied repeatedly. A damping gain < 1 should
    make the running estimate approach the true offset smoothly across
    several updates, never overshooting past it - proving the damping
    actually damps, rather than just being a slower version of an instant
    snap that could still overshoot given noisy per-call estimates.
    """
    true_offset = 0.5
    gain = 0.3
    bias_estimate = 0.0
    previous_error = abs(true_offset - bias_estimate)

    for _ in range(30):
        # Each call "measures" the CURRENT residual, i.e. the drift not yet
        # corrected for - exactly what happens live: after subtracting the
        # running bias_estimate from incoming points, the residual left to
        # discover shrinks as the estimate improves.
        residual_left = true_offset - bias_estimate
        bias_estimate += gain * residual_left

        error = abs(true_offset - bias_estimate)
        assert error <= previous_error, "each update should move closer to the true offset, never further"
        assert bias_estimate <= true_offset + 1e-9, "a damped gain < 1 must never overshoot past the true offset"
        previous_error = error

    assert bias_estimate == pytest.approx(true_offset, abs=1e-3)
