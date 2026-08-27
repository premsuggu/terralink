#!/usr/bin/env python3
"""Test Step 7: Map Shifting (UAV-centric)."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_shift_map_xy():
    """Test shifting map by integer pixels."""
    print("Test: shift_map_xy")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Add data at center (row=100, col=100)
    m.elevation_map[m.IDX_ELEVATION, 100, 100] = 1.0
    m.elevation_map[m.IDX_IS_VALID, 100, 100] = 1.0
    m.elevation_map[m.IDX_VARIANCE, 100, 100] = 0.1
    
    print(f"Before shift: elev[100,100]={m.elevation_map[m.IDX_ELEVATION, 100, 100]}")
    print(f"Center: ({m.center_x}, {m.center_y})")
    
    # Shift map by -1 in X (simulating robot moving +X, map shifts -X)
    # Data at col=100 should move to col=99
    m.shift_map_xy((-1, 0))
    
    print(f"After shift (-1,0): elev[100,99]={m.elevation_map[m.IDX_ELEVATION, 100, 99]}")
    print(f"Center: ({m.center_x}, {m.center_y})")
    
    # Verify shift: data moves to lower column index when shift is negative
    assert abs(m.elevation_map[m.IDX_ELEVATION, 100, 99] - 1.0) < 0.01, "Elevation not shifted correctly"
    print("✓ Shift X test passed")
    
    # Shift map by -1 in Y (simulating robot moving +Y, map shifts -Y)
    # Data at row=100 should move to row=99
    m.shift_map_xy((0, -1))
    print(f"After shift (0,-1): elev[99,99]={m.elevation_map[m.IDX_ELEVATION, 99, 99]}")
    print(f"Center: ({m.center_x}, {m.center_y})")
    
    assert abs(m.elevation_map[m.IDX_ELEVATION, 99, 99] - 1.0) < 0.01, "Elevation not shifted correctly in Y"
    print("✓ Shift Y test passed")
    
    # Test positive shift
    m.shift_map_xy((1, 1))
    assert abs(m.elevation_map[m.IDX_ELEVATION, 100, 100] - 1.0) < 0.01, "Positive shift failed"
    print("✓ Positive shift test passed")
    
    return True


def test_move_to():
    """Test moving map center to new position."""
    print("\nTest: move_to")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Add data at center (row=100, col=100)
    m.elevation_map[m.IDX_ELEVATION, 100, 100] = 1.0
    m.elevation_map[m.IDX_IS_VALID, 100, 100] = 1.0
    m.elevation_map[m.IDX_VARIANCE, 100, 100] = 0.1
    
    print(f"Before move: center=({m.center_x}, {m.center_y})")
    
    # Move to new position (1m forward, 0.5m left)
    new_pos = np.array([1.0, 0.5, 0.0], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    m.move_to(new_pos, R)
    
    print(f"After move to (1, 0.5): center=({m.center_x}, {m.center_y})")
    
    # Check center updated (10 pixels at 0.1m = 1.0m)
    assert abs(m.center_x - 1.0) < 0.01, f"Center X should be ~1.0, got {m.center_x}"
    assert abs(m.center_y - 0.5) < 0.01, f"Center Y should be ~0.5, got {m.center_y}"
    
    # Data should have shifted -10 in X (col 100 -> 90), -5 in Y (row 100 -> 95)
    # move_to calls shift_map_xy(-delta_pixel), so robot moving +X/+Y means map shifts -X/-Y
    assert abs(m.elevation_map[m.IDX_ELEVATION, 95, 90] - 1.0) < 0.01, "Data not shifted correctly in move_to"
    print("✓ Move to test passed")
    
    return True


def test_shift_preserves_data():
    """Test that shifting preserves data in non-padded regions."""
    print("\nTest: shift preserves data")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Create a pattern at center (rows 100-104, cols 100-104)
    for i in range(5):
        for j in range(5):
            m.elevation_map[m.IDX_ELEVATION, 100+i, 100+j] = i * 5 + j
            m.elevation_map[m.IDX_IS_VALID, 100+i, 100+j] = 1.0
            m.elevation_map[m.IDX_VARIANCE, 100+i, 100+j] = 0.1
    
    # Shift by (-2, 1): dx=-2 (left 2), dy=1 (down 1)
    # Data at (row=100, col=100) moves to (row=101, col=98)
    m.shift_map_xy((-2, 1))
    
    # Check pattern preserved (accounting for shift)
    # Original (100+i, 100+j) -> New (101+i, 98+j)
    for i in range(5):
        for j in range(5):
            expected = i * 5 + j
            actual = m.elevation_map[m.IDX_ELEVATION, 101+i, 98+j]
            assert abs(actual - expected) < 0.01, f"Data not preserved at ({i},{j}): expected {expected}, got {actual}"
    
    print("✓ Data preservation test passed")
    return True


def test_edge_padding():
    """Test that new edges are properly padded with zeros and initial variance."""
    print("\nTest: edge padding")
    
    param = Parameter(resolution=0.1, map_length=20.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Fill center region
    m.elevation_map[m.IDX_ELEVATION, 98:102, 98:102] = 1.0
    m.elevation_map[m.IDX_IS_VALID, 98:102, 98:102] = 1.0
    m.elevation_map[m.IDX_VARIANCE, 98:102, 98:102] = 0.1
    
    # Shift by (-50, 0) - simulating robot moving +X by 50 pixels
    # Data moves left by 50, so rightmost 50 cols should be padded
    m.shift_map_xy((-50, 0))
    
    # Check padded edges have initial variance and zero elevation
    # After shift left by 50, rightmost 50 cols (indices 152-201) should be padded
    padded_var = m.elevation_map[m.IDX_VARIANCE, :, -50:]
    assert np.allclose(padded_var, param.initial_variance), "New edges not padded with initial variance"
    
    padded_elev = m.elevation_map[m.IDX_ELEVATION, :, -50:]
    assert np.allclose(padded_elev, 0.0), "New edges not padded with zero elevation"
    
    padded_valid = m.elevation_map[m.IDX_IS_VALID, :, -50:]
    assert np.allclose(padded_valid, 0.0), "New edges not padded with zero validity"
    
    print("✓ Edge padding test passed")
    return True


if __name__ == '__main__':
    try:
        test_shift_map_xy()
        test_move_to()
        test_shift_preserves_data()
        test_edge_padding()
        print("\n✓ All Step 7 Map Shifting tests passed!")
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)