#!/usr/bin/env python3
"""Test Step 6: Visibility Cleanup (Ray Tracing) with proper UAV simulation."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_uav_ray_tracing():
    """Test ray tracing with UAV downward-facing camera."""
    print("Test: UAV downward-facing camera ray tracing")
    
    param = Parameter(resolution=0.2, map_length=30.0, sensor_noise_factor=0.05, 
                      enable_visibility_cleanup=True, cleanup_step=0.1, cleanup_cos_thresh=0.3,
                      max_ray_length=50.0, min_height=-5.0, max_height=20.0,
                      mahalanobis_thresh=10.0)
    param.update()
    m = ElevationMapCPU(param)

    # UAV at (0, 0, 10) with camera pointing DOWN
    # Camera frame: x forward, y left, z up
    # Camera pointing down means: world z maps to camera -y
    # R_cam_to_world: camera -> world
    # World point P_w = R_cw @ P_c + t_w
    # For downward camera at (0,0,10): 
    #   camera x = world x
    #   camera y = world z (but inverted, so -world z)
    #   camera z = world y
    # So: [x_w]   [1 0 0] [x_c]   [0]
    #     [y_w] = [0 0 1] [y_c] + [0]
    #     [z_w]   [0 -1 0] [z_c]   [10]
    R_cam_to_world = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
    t_world = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    
    # Wall at world (10, 5, 0) to (10, 5, 3)
    # Transform to camera frame: P_c = R_wc @ (P_w - t_w)
    # R_wc = R_cw.T
    R_world_to_cam = R_cam_to_world.T  # Transpose for inverse rotation
    
    print("=== Building wall gradually ===")
    for z in np.arange(0, 3.1, 0.5):
        p_world = np.array([10.0, 5.0, z], dtype=np.float32)
        p_cam = R_world_to_cam @ (p_world - t_world)
        print(f"  World ({10}, {5}, {z:.1f}) -> Cam {p_cam}")
        m.fuse_pointcloud(np.array([p_cam], dtype=np.float32), R_cam_to_world, t_world)
    
    # Check wall cells
    wall_row, wall_col = m.world_to_grid(10.0, 5.0)
    elev = m.elevation_map[m.IDX_ELEVATION, wall_row, wall_col]
    valid = m.elevation_map[m.IDX_IS_VALID, wall_row, wall_col]
    print(f"Wall cell after gradual build: elev={elev:.2f}, valid={valid:.2f}")
    assert valid > 0.5, f"Wall should be detected, got valid={valid}"
    
    # Step 2: Hit ground beyond wall at (15, 5, 0)
    far_ground_world = np.array([15.0, 5.0, 0.0], dtype=np.float32)
    far_cam = R_world_to_cam @ (far_ground_world - t_world)
    print(f"\n=== Ground beyond wall at (15, 5, 0) -> Cam {far_cam} ===")
    m.fuse_pointcloud(np.array([far_cam], dtype=np.float32), R_cam_to_world, t_world)
    
    # Check wall cells (should remain)
    elev = m.elevation_map[m.IDX_ELEVATION, wall_row, wall_col]
    valid = m.elevation_map[m.IDX_IS_VALID, wall_row, wall_col]
    print(f"Wall cell after ground hit: elev={elev:.2f}, valid={valid:.2f}")
    assert valid > 0.5, "Wall should remain after ray tracing"
    
    # Check cells in front of wall (should be cleared by ray to far ground)
    print("Cells in front of wall (x=5 to x=9):")
    for x in range(5, 10):
        row, col = m.world_to_grid(float(x), 5.0)
        if m.is_inside(row, col):
            elev = m.elevation_map[m.IDX_ELEVATION, row, col]
            valid = m.elevation_map[m.IDX_IS_VALID, row, col]
            print(f"  x={x}: elev={elev:.2f}, valid={valid:.2f}")
    
    return True


if __name__ == '__main__':
    import sys
    try:
        test_uav_ray_tracing()
        print("\n✓ UAV ray tracing test passed!")
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)