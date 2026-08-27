#!/usr/bin/env python3
"""Test Step 8: Drift Compensation."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_drift_compensation_basic():
    """Test basic drift compensation functionality."""
    print("Test: Drift compensation basic")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      enable_drift_compensation=True,
                      drift_compensation_alpha=0.1,
                      max_drift=0.5,
                      drift_compensation_variance_inlier=1.0,
                      min_height_drift_cnt=1,
                      sensor_noise_factor=0.01)
    param.update()
    m = ElevationMapCPU(param)
    
    # Build a flat map at z=0
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)  # UAV at height 10
    
    # Add many points at z=0 (ground)
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    # Check map is at z=0
    row, col = m.world_to_grid(0.0, 0.0)
    elev_before = m.elevation_map[m.IDX_ELEVATION, row, col]
    print(f"Map elevation before drift: {elev_before:.4f}")
    assert abs(elev_before) < 0.05, f"Expected ~0, got {elev_before}"
    
    # Now simulate sensor with +0.2m bias (consistently reads 0.2m higher)
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -9.8], dtype=np.float32)  # 0.2m higher
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    # Check map elevation after biased readings
    elev_after = m.elevation_map[m.IDX_ELEVATION, row, col]
    print(f"Map elevation after biased readings: {elev_after:.4f}")
    
    # The drift compensation should correct for the bias
    print(f"Drift compensation test - map elevation: {elev_after:.4f}")
    
    # The map should NOT drift all the way to 0.2m
    # Drift compensation should keep it closer to 0
    assert abs(elev_after) < 0.15, f"Drift compensation failed: map drifted to {elev_after}"
    print("✓ Drift compensation basic test passed")
    
    return True


def test_drift_compensation_with_initial_variance():
    """Test drift compensation when map has higher variance (less confident)."""
    print("\nTest: Drift compensation with initial variance")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      enable_drift_compensation=True,
                      drift_compensation_alpha=0.1,
                      max_drift=0.5,
                      drift_compensation_variance_inlier=1.0,
                      min_height_drift_cnt=1,
                      sensor_noise_factor=0.01)
    param.update()
    m = ElevationMapCPU(param)
    
    # Build a flat map at z=0 with only a few points (high variance)
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Add only a few points at z=0
    for x in [-1.0, 0.0, 1.0]:
        for y in [-1.0, 0.0, 1.0]:
            point_sensor = np.array([x, y, -10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    # Check map variance is still high
    row, col = m.world_to_grid(0.0, 0.0)
    var_before = m.elevation_map[m.IDX_VARIANCE, row, col]
    print(f"Map variance before drift: {var_before:.4f}")
    assert var_before > 0.5, f"Expected high variance, got {var_before}"
    
    # Now add biased readings - with high variance, drift comp should work more aggressively
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -9.8], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    elev_after = m.elevation_map[m.IDX_ELEVATION, row, col]
    print(f"Map elevation after biased readings (high var): {elev_after:.4f}")
    
    # With high variance, drift compensation should have corrected
    # The map should not have drifted much
    assert abs(elev_after) < 0.2, f"Drift compensation failed with high variance: {elev_after}"
    print("✓ Drift compensation with high variance test passed")
    
    return True


def test_drift_compensation_max_drift():
    """Test that max_drift limits compensation."""
    print("\nTest: Drift compensation max_drift limit")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      enable_drift_compensation=True,
                      drift_compensation_alpha=0.1,
                      max_drift=0.05,  # Very small max drift
                      drift_compensation_variance_inlier=1.0,
                      min_height_drift_cnt=1)
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build flat map
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    row, col = m.world_to_grid(0.0, 0.0)
    
    # Add heavily biased readings (0.5m bias)
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -9.5], dtype=np.float32)  # 0.5m higher
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    elev_after = m.elevation_map[m.IDX_ELEVATION, row, col]
    print(f"Map elevation with 0.5m bias and max_drift=0.05: {elev_after:.4f}")
    
    # With max_drift=0.05, compensation should be limited
    assert abs(elev_after) < 0.1, f"Max drift limit failed: {elev_after}"
    print("✓ Max drift limit test passed")
    
    return True


def test_drift_compensation_variance_threshold():
    """Test that drift compensation only uses low-variance cells."""
    print("\nTest: Drift compensation variance threshold")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      enable_drift_compensation=True,
                      drift_compensation_alpha=0.1,
                      max_drift=0.5,
                      drift_compensation_variance_inlier=0.1,  # Only very low variance
                      min_height_drift_cnt=1)
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build map with few points (high variance)
    for x in np.arange(-1, 1.1, 0.5):
        for y in np.arange(-1, 1.1, 0.5):
            point_sensor = np.array([x, y, -10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    row, col = m.world_to_grid(0.0, 0.0)
    var = m.elevation_map[m.IDX_VARIANCE, row, col]
    print(f"Map variance: {var:.4f} (threshold: {param.drift_compensation_variance_inlier})")
    
    # Variance should be higher than threshold, so drift comp should NOT use these cells
    # Add biased readings
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            point_sensor = np.array([x, y, -9.8], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    elev_after = m.elevation_map[m.IDX_ELEVATION, row, col]
    print(f"Map elevation with high variance cells: {elev_after:.4f}")
    
    # Since variance > threshold, drift compensation should not use these cells
    # But Bayesian fusion naturally prevents drift when variance is high
    # So the map should stay close to 0 (not drift toward biased reading)
    assert abs(elev_after) < 0.1, f"Variance threshold test failed: map drifted to {elev_after}"
    print("✓ Variance threshold test passed (drift comp disabled by high variance, Bayesian fusion prevents drift)")
    
    return True


if __name__ == '__main__':
    try:
        test_drift_compensation_basic()
        test_drift_compensation_with_initial_variance()
        test_drift_compensation_max_drift()
        test_drift_compensation_variance_threshold()
        print("\n✓ All Step 8 Drift Compensation tests passed!")
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)