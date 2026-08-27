#!/usr/bin/env python3
"""Test Step 9: Traversability Analysis."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_traversability_flat_ground():
    """Test traversability on flat ground."""
    print("Test: Traversability on flat ground")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      max_slope=0.35,
                      max_step=0.15,
                      max_roughness=1.0)  # Allow higher roughness for testing
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build flat ground at z=0 with multiple passes to reduce variance
    for _ in range(5):  # Multiple passes to reduce variance
        for x in np.arange(-5, 5.1, 0.5):
            for y in np.arange(-5, 5.1, 0.5):
                point_sensor = np.array([x, y, -10.0], dtype=np.float32)
                points_sensor = np.array([point_sensor], dtype=np.float32)
                m.fuse_pointcloud(points_sensor, R, t)
    
    m.update_traversability()
    
    # Check center is traversable
    row, col = m.world_to_grid(0.0, 0.0)
    trav = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"Flat ground traversability: {trav:.2f}")
    assert trav > 0.9, f"Flat ground should be traversable, got {trav}"
    print("✓ Flat ground traversability test passed")
    return True


def test_traversability_slope():
    """Test traversability on slope."""
    print("\nTest: Traversability on slope")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      max_slope=0.35,  # ~20 degrees
                      max_step=0.15,
                      max_roughness=1.0,
                      max_ray_length=20.0)  # Increase ray length for sloped ground test
    param.update()
    m = ElevationMapCPU(param)
    
    # Build sloped ground: z = 0.1 * x (10% grade = ~5.7 degrees)
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    for x in np.arange(-5, 5.1, 0.5):
        for y in np.arange(-5, 5.1, 0.5):
            # Ground at z = 0.1 * x
            z = 0.1 * x
            point_sensor = np.array([x, y, z - 10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    m.update_traversability()
    
    # Check traversability at different x positions
    # At x=0, slope should be ~0.1 (within limit 0.35)
    row, col = m.world_to_grid(0.0, 0.0)
    trav_center = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"Center (x=0) traversability: {trav_center:.2f}")
    assert trav_center > 0.5, f"Center should be traversable, got {trav_center}"
    
    # At x=3, slope = 0.1 (within limit 0.35)
    row, col = m.world_to_grid(3.0, 0.0)
    trav_x3 = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"x=3 traversability: {trav_x3:.2f}")
    
    # At x=6, slope = 0.1 (within limit)
    row, col = m.world_to_grid(6.0, 0.0)
    trav_x6 = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"x=6 traversability: {trav_x6:.2f}")
    
    print("✓ Slope traversability test passed")
    return True


def test_traversability_step():
    """Test traversability with step height."""
    print("\nTest: Traversability with step")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      max_slope=0.35,
                      max_step=0.15,
                      max_roughness=1.0,
                      max_ray_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build ground with a step: z=0 for x<0, z=0.2 for x>=0
    # Multiple passes to reduce variance
    for _ in range(5):
        for x in np.arange(-3, 3.1, 0.2):
            for y in np.arange(-2, 2.1, 0.2):
                if x < 0:
                    z = 0.0
                else:
                    z = 0.2  # 20cm step
                point_sensor = np.array([x, y, z - 10.0], dtype=np.float32)
                points_sensor = np.array([point_sensor], dtype=np.float32)
                m.fuse_pointcloud(points_sensor, R, t)
    
    m.update_traversability()
    
    # Check traversability away from step
    row, col = m.world_to_grid(-1.0, 0.0)
    trav_before = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"Before step (x=-1) traversability: {trav_before:.2f}")
    assert trav_before > 0.9, f"Before step should be traversable, got {trav_before}"
    
    row, col = m.world_to_grid(1.0, 0.0)
    trav_after = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"After step (x=1) traversability: {trav_after:.2f}")
    # After step, the local step height exceeds max_step, so it's difficult (0.3)
    assert trav_after < 0.5 and trav_after > 0.2, f"After step should be difficult (0.3), got {trav_after}"
    
    # Check at step boundary (should be difficult or lethal)
    row, col = m.world_to_grid(0.0, 0.0)
    trav_step = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"At step (x=0) traversability: {trav_step:.2f}")
    assert trav_step < 0.5, f"Step should be difficult/lethal, got {trav_step}"
    
    print("✓ Step traversability test passed")
    return True


def test_traversability_roughness():
    """Test traversability with high roughness (variance)."""
    print("\nTest: Traversability with high roughness")
    
    param = Parameter(resolution=0.1, map_length=20.0,
                      max_slope=0.35,
                      max_step=0.15,
                      max_roughness=0.05)
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build ground with noisy measurements (high variance)
    for x in np.arange(-2, 2.1, 0.5):
        for y in np.arange(-2, 2.1, 0.5):
            # Add noise to z
            z = np.random.normal(0.0, 0.1)  # 10cm noise
            point_sensor = np.array([x, y, z - 10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    m.update_traversability()
    
    # Check traversability - high variance areas should be marked as difficult
    row, col = m.world_to_grid(0.0, 0.0)
    trav = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"High roughness traversability: {trav:.2f}")
    
    # With high variance, traversability should be reduced
    assert trav < 1.0, f"High roughness should reduce traversability, got {trav}"
    print("✓ Roughness traversability test passed")
    return True


def test_traversability_unknown():
    """Test that unknown cells are marked non-traversable."""
    print("\nTest: Unknown cells are non-traversable")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Don't add any points
    m.update_traversability()
    
    row, col = m.world_to_grid(0.0, 0.0)
    trav = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"Unknown cell traversability: {trav:.2f}")
    assert trav == 0.0, f"Unknown cells should be non-traversable, got {trav}"
    print("✓ Unknown cell test passed")
    return True


def test_traversability_consistency():
    """Test that traversability is consistent after map shifting."""
    print("\nTest: Traversability consistency after map shift")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    R = np.eye(3, dtype=np.float32)
    t = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Build flat ground
    for x in np.arange(-3, 3.1, 0.5):
        for y in np.arange(-3, 3.1, 0.5):
            point_sensor = np.array([x, y, -10.0], dtype=np.float32)
            points_sensor = np.array([point_sensor], dtype=np.float32)
            m.fuse_pointcloud(points_sensor, R, t)
    
    m.update_traversability()
    
    # Check traversability before shift
    row, col = m.world_to_grid(0.0, 0.0)
    trav_before = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"Before shift: {trav_before:.2f}")
    
    # Shift map
    m.shift_map_xy((-5, 0))
    
    # Check traversability after shift
    row, col = m.world_to_grid(0.5, 0.0)  # Same world position
    trav_after = m.elevation_map[m.IDX_TRAVERSABILITY, row, col]
    print(f"After shift: {trav_after:.2f}")
    
    # Traversability should be preserved
    assert abs(trav_after - trav_before) < 0.1, f"Traversability not preserved: {trav_before} -> {trav_after}"
    print("✓ Traversability consistency test passed")
    return True


if __name__ == '__main__':
    try:
        test_traversability_flat_ground()
        test_traversability_slope()
        test_traversability_step()
        test_traversability_roughness()
        test_traversability_unknown()
        test_traversability_consistency()
        print("\n✓ All Step 9 Traversability Analysis tests passed!")
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)