#!/usr/bin/env python3
"""Test Step 4: Bayesian Fusion (CPU)"""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_bayesian_fusion_single_point():
    """Test Bayesian fusion with a single point."""
    print("Test 1: Bayesian fusion - single point")
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05)
    param.update()
    m = ElevationMapCPU(param)

    # Single point at center, height=1.0
    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    m.fuse_pointcloud(points, R, t)
    center = m.cell_n // 2

    # First measurement: prior=0, var=1.0 -> posterior = measurement * (prior_var / (prior_var + meas_var))
    # sensor noise = 0.05 * (0^2+0^2+1^2) = 0.05
    # posterior_mean = (0*0.05 + 1*1.0) / (1.0 + 0.05) = 1/1.05 = 0.9524
    elev = m.elevation_map[m.IDX_ELEVATION, center, center]
    var = m.elevation_map[m.IDX_VARIANCE, center, center]
    assert abs(elev - 0.9524) < 0.01
    assert abs(var - 0.0476) < 0.01
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 1.0
    print("  PASSED")


def test_bayesian_fusion_multiple_measurements():
    """Test that repeated measurements reduce variance."""
    print("\nTest 2: Bayesian fusion - multiple measurements")
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05)
    param.update()
    m = ElevationMapCPU(param)

    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    # Fuse same point 5 times
    for _ in range(5):
        m.fuse_pointcloud(points, R, t)

    center = m.cell_n // 2
    elev = m.elevation_map[m.IDX_ELEVATION, center, center]
    var = m.elevation_map[m.IDX_VARIANCE, center, center]

    # With repeated measurements, elevation should converge to 1.0, variance -> 0
    assert abs(elev - 1.0) < 0.02
    assert var < 0.01
    print("  PASSED")


def test_bayesian_fusion_multiple_points_same_cell():
    """Test fusion of multiple points falling in same cell."""
    print("\nTest 3: Bayesian fusion - multiple points in same cell")
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05)
    param.update()
    m = ElevationMapCPU(param)

    # Three points at slightly different heights
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.1], [0.0, 0.0, 0.9]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    m.fuse_pointcloud(points, R, t)
    center = m.cell_n // 2

    elev = m.elevation_map[m.IDX_ELEVATION, center, center]
    # Average of 1.0, 1.1, 0.9 = 1.0 (Bayesian weighted by variance)
    assert abs(elev - 1.0) < 0.05
    print("  PASSED")


def test_outlier_rejection():
    """Test that outliers are rejected (elevation preserved, variance increased)."""
    print("\nTest 4: Outlier rejection")
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05, 
                      mahalanobis_thresh=2.0, outlier_variance=0.01,
                      max_height=200.0, max_ray_length=200.0)
    param.update()
    m = ElevationMapCPU(param)

    # Build up valid data first
    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)
    for _ in range(5):
        m.fuse_pointcloud(points, R, t)

    center = m.cell_n // 2
    elev_before = m.elevation_map[m.IDX_ELEVATION, center, center]
    var_before = m.elevation_map[m.IDX_VARIANCE, center, center]

    # Add outlier at z=100 (way above ground)
    points_outlier = np.array([[0.0, 0.0, 100.0]], dtype=np.float32)
    m.fuse_pointcloud(points_outlier, R, t)

    elev_after = m.elevation_map[m.IDX_ELEVATION, center, center]
    var_after = m.elevation_map[m.IDX_VARIANCE, center, center]

    # Elevation should be preserved
    assert abs(elev_after - elev_before) < 0.001
    # Variance should increase by outlier_variance
    assert abs(var_after - (var_before + 0.01)) < 0.001
    print("  PASSED")


def test_point_validation():
    """Test point validation (distance, height limits)."""
    print("\nTest 5: Point validation")
    param = Parameter(resolution=0.05, map_length=10.0, 
                      min_valid_distance=0.5, max_ray_length=5.0,
                      min_height=-1.0, max_height=5.0)
    param.update()
    m = ElevationMapCPU(param)

    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    # Too close (distance < 0.5)
    points_close = np.array([[0.0, 0.0, 0.3]], dtype=np.float32)
    m.fuse_pointcloud(points_close, R, t)
    center = m.cell_n // 2
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 0.0

    # Too far (distance > 5.0)
    points_far = np.array([[0.0, 0.0, 10.0]], dtype=np.float32)
    m.fuse_pointcloud(points_far, R, t)
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 0.0

    # Too high (z > max_height)
    points_high = np.array([[0.0, 0.0, 10.0]], dtype=np.float32)
    m.fuse_pointcloud(points_high, R, t)
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 0.0

    # Valid point
    points_valid = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    m.fuse_pointcloud(points_valid, R, t)
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 1.0
    print("  PASSED")


def test_coordinate_transform_in_fusion():
    """Test that coordinate transform works correctly in fusion."""
    print("\nTest 6: Coordinate transform in fusion")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)

    # Point 1m in +X, 2m in +Y, height=1.0
    points = np.array([[1.0, 2.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    m.fuse_pointcloud(points, R, t)

    center = m.cell_n // 2
    # Should be at row = center + 40 (2m/0.05), col = center + 20 (1m/0.05)
    expected_row = center + 40  # 2m / 0.05
    expected_col = center + 20  # 1m / 0.05

    assert m.elevation_map[m.IDX_IS_VALID, expected_row, expected_col] == 1.0
    # Bayesian fusion: prior=0/var=1, meas=1/var=0.3 -> posterior = 1/1.3 = 0.769
    elev = m.elevation_map[m.IDX_ELEVATION, expected_row, expected_col]
    assert abs(elev - 0.77) < 0.05  # ~0.769 due to Bayesian fusion
    print("  PASSED")


def test_multiple_cells():
    """Test fusion into multiple different cells."""
    print("\nTest 7: Multiple cells")
    param = Parameter(resolution=0.1, map_length=5.0)
    param.update()
    m = ElevationMapCPU(param)

    # Two points in different cells
    points = np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 2.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    m.fuse_pointcloud(points, R, t)

    center = m.cell_n // 2
    # First point at center
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 1.0
    # Second point at x=2.0 (20 cells right with resolution=0.1)
    assert m.elevation_map[m.IDX_IS_VALID, center, center + 20] == 1.0

    elev1 = m.elevation_map[m.IDX_ELEVATION, center, center]
    elev2 = m.elevation_map[m.IDX_ELEVATION, center, center + 20]
    # Bayesian fusion: point 1 at z=1, noise=0.05*1=0.05 -> posterior ~0.95
    # Point 2 at z=2, noise=0.05*4=0.2 -> posterior ~1.43
    assert abs(elev1 - 0.95) < 0.05
    assert abs(elev2 - 1.43) < 0.1
    print("  PASSED")


def test_sensor_noise_model():
    """Test that sensor noise increases with distance."""
    print("\nTest 8: Sensor noise model")
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05)
    param.update()
    m = ElevationMapCPU(param)

    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    # Close point (distance ~1.0) - low noise
    points_close = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    m.fuse_pointcloud(points_close, R, t)
    center = m.cell_n // 2
    var_close = m.elevation_map[m.IDX_VARIANCE, center, center]

    # Reset
    m2 = ElevationMapCPU(param)
    # Far point (distance ~5.0) - high noise
    points_far = np.array([[0.0, 0.0, 5.0]], dtype=np.float32)
    m2.fuse_pointcloud(points_far, R, t)
    var_far = m2.elevation_map[m2.IDX_VARIANCE, center, center]

    # Far point should have higher variance (noise factor * distance^2)
    assert var_far > var_close
    print("  PASSED")


def test_accumulator_reset():
    """Test that accumulators are properly reset each frame."""
    print("\nTest 9: Accumulator reset between frames")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)

    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    # Frame 1
    m.fuse_pointcloud(points, R, t)
    center = m.cell_n // 2
    elev1 = m.elevation_map[m.IDX_ELEVATION, center, center]

    # Frame 2 - same point
    m.fuse_pointcloud(points, R, t)
    elev2 = m.elevation_map[m.IDX_ELEVATION, center, center]

    # Elevation should converge (not double)
    assert elev2 > elev1
    assert elev2 < 1.0
    print("  PASSED")


def test_finalize_fusion_no_points():
    """Test finalize with no valid points."""
    print("\nTest 10: Finalize with no points")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)

    # Empty point cloud
    points = np.empty((0, 3), dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)

    m.fuse_pointcloud(points, R, t)

    # All cells should be invalid
    center = m.cell_n // 2
    assert m.elevation_map[m.IDX_IS_VALID, center, center] == 0.0
    assert m.elevation_map[m.IDX_ELEVATION, center, center] == 0.0
    assert m.elevation_map[m.IDX_VARIANCE, center, center] == m.param.initial_variance
    print("  PASSED")


if __name__ == '__main__':
    tests = [
        test_bayesian_fusion_single_point,
        test_bayesian_fusion_multiple_measurements,
        test_bayesian_fusion_multiple_points_same_cell,
        test_outlier_rejection,
        test_point_validation,
        test_coordinate_transform_in_fusion,
        test_multiple_cells,
        test_sensor_noise_model,
        test_accumulator_reset,
        test_finalize_fusion_no_points,
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")

    if failed > 0:
        sys.exit(1)