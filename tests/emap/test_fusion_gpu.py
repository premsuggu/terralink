"""Unit tests for emap.fusion_gpu.fuse_points_gpu (step 9).

Skips cleanly (not a failure) on any machine without cupy/a GPU - the CPU
path (test_fusion.py) is the one every environment must pass; this file only
proves the GPU path computes the SAME thing when it's available at all,
which is the roadmap's own definition of done for this step ("verify GPU
output matches CPU reference numerically").
"""
import numpy as np
import pytest

cupy = pytest.importorskip("cupy")

from emap.elevation_map import ElevationMap
from emap.fusion import fuse_points
from emap.fusion_gpu import fuse_points_gpu

RESOLUTION = 1.0
LENGTH = 10.0
INITIAL_VARIANCE = 10.0
SENSOR_NOISE_FACTOR = 0.01
MAHALANOBIS_THRESH = 2.0
OUTLIER_VARIANCE = 1.0
MIN_VALID_DISTANCE = 0.1


def new_map() -> ElevationMap:
    return ElevationMap(resolution=RESOLUTION, length=LENGTH, initial_variance=INITIAL_VARIANCE)


def test_single_point_matches_hand_computed_bayesian_update():
    """Same scenario as test_fusion.py's equivalent CPU test, run through
    the GPU path instead - proves the GPU kernel reaches the same
    hand-computed answer, not just "the same answer as the CPU function"
    (which could both be wrong in the same way).
    """
    emap = new_map()
    point = [0.0, 0.0, 1.0]
    sensor_origin = [0.0, 0.0, 3.0]

    fuse_points_gpu(
        emap,
        np.asarray([point], dtype=np.float64),
        np.asarray(sensor_origin, dtype=np.float64),
        sensor_noise_factor=SENSOR_NOISE_FACTOR,
        mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE,
        min_valid_distance=MIN_VALID_DISTANCE,
    )

    v = SENSOR_NOISE_FACTOR * 4.0
    expected_h = (0.0 * v + 1.0 * INITIAL_VARIANCE) / (INITIAL_VARIANCE + v)
    expected_v = (INITIAL_VARIANCE * v) / (INITIAL_VARIANCE + v)

    row, col = emap.world_to_grid(0.0, 0.0)
    assert emap.layer("elevation")[row, col] == pytest.approx(expected_h, rel=1e-5)
    assert emap.layer("variance")[row, col] == pytest.approx(expected_v, rel=1e-5)
    assert emap.layer("is_valid")[row, col] == 1.0


def test_max_valid_range_rejects_clamped_far_clip_artifacts_on_gpu_too():
    """The far-clip clamp-artifact filter (see fusion.py's docstring) must
    behave identically on the GPU path - otherwise use_gpu_fusion:=true
    would silently reintroduce the exact live bug it was added to fix.
    """
    emap = new_map()
    sensor_origin = np.array([0.0, 0.0, 20.0])
    max_valid_range = 19.8
    row, col = emap.world_to_grid(0.0, 0.0)

    clamped_artifact = [[0.0, 0.0, 0.06]]  # range 19.94 - must be rejected
    fuse_points_gpu(
        emap, np.asarray(clamped_artifact, dtype=np.float64), sensor_origin,
        sensor_noise_factor=SENSOR_NOISE_FACTOR, mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE, min_valid_distance=MIN_VALID_DISTANCE,
        max_valid_range=max_valid_range,
    )
    assert emap.layer("is_valid")[row, col] == 0.0

    real_far_reading = [[0.0, 0.0, 0.5]]  # range 19.5 - must still be fused
    fuse_points_gpu(
        emap, np.asarray(real_far_reading, dtype=np.float64), sensor_origin,
        sensor_noise_factor=SENSOR_NOISE_FACTOR, mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE, min_valid_distance=MIN_VALID_DISTANCE,
        max_valid_range=max_valid_range,
    )
    assert emap.layer("is_valid")[row, col] == 1.0
    v = SENSOR_NOISE_FACTOR * (19.5 ** 2)
    expected_h = (0.0 * v + 0.5 * INITIAL_VARIANCE) / (INITIAL_VARIANCE + v)
    assert emap.layer("elevation")[row, col] == pytest.approx(expected_h, rel=1e-3)


def test_gpu_matches_cpu_on_random_point_cloud():
    """The real point of this file: feed IDENTICAL random input to both
    fuse_points (CPU) and fuse_points_gpu (GPU), on two otherwise-identical
    fresh maps, and assert every layer ends up numerically the same. This is
    the actual "port verified against the CPU reference" check.
    """
    rng = np.random.default_rng(42)
    n_points = 500
    # Points spread across most of the map (10m map, keep a margin so
    # rounding never pushes a point just outside in one map but not the
    # other) with heights that will trigger a realistic mix of fresh
    # updates, repeated-cell averaging, and outliers.
    xy = rng.uniform(-4.0, 4.0, size=(n_points, 2))
    z = rng.uniform(-2.0, 2.0, size=n_points)
    points = np.column_stack([xy, z])
    sensor_origin = np.array([0.0, 0.0, 5.0])

    emap_cpu = new_map()
    emap_gpu = new_map()

    fuse_points(
        emap_cpu, points, sensor_origin,
        sensor_noise_factor=SENSOR_NOISE_FACTOR,
        mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE,
        min_valid_distance=MIN_VALID_DISTANCE,
    )
    fuse_points_gpu(
        emap_gpu, points, sensor_origin,
        sensor_noise_factor=SENSOR_NOISE_FACTOR,
        mahalanobis_thresh=MAHALANOBIS_THRESH,
        outlier_variance=OUTLIER_VARIANCE,
        min_valid_distance=MIN_VALID_DISTANCE,
    )

    np.testing.assert_allclose(
        emap_cpu.layer("elevation"), emap_gpu.layer("elevation"), atol=1e-4,
        err_msg="GPU fusion's elevation layer diverged from the CPU reference",
    )
    np.testing.assert_allclose(
        emap_cpu.layer("variance"), emap_gpu.layer("variance"), atol=1e-4,
        err_msg="GPU fusion's variance layer diverged from the CPU reference",
    )
    np.testing.assert_array_equal(
        emap_cpu.layer("is_valid"), emap_gpu.layer("is_valid"),
        err_msg="GPU fusion marked a different set of cells valid than the CPU reference",
    )
