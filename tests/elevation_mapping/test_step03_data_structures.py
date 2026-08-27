#!/usr/bin/env python3
"""Test elevation map core data structures."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


def test_layer_indices():
    print("Test 1: Layer indices")
    m = ElevationMapCPU(Parameter())
    assert m.IDX_ELEVATION == 0
    assert m.IDX_VARIANCE == 1
    assert m.IDX_IS_VALID == 2
    assert m.IDX_TRAVERSABILITY == 3
    assert m.IDX_TIME == 4
    assert m.IDX_UPPER_BOUND == 5
    assert m.IDX_IS_UPPER_BOUND == 6
    print("  PASSED")


def test_layer_names():
    print("\nTest 2: Layer names")
    m = ElevationMapCPU(Parameter())
    expected = [
        "elevation", "variance", "is_valid", 
        "traversability", "time", "upper_bound", "is_upper_bound"
    ]
    assert m.layer_names == expected
    print("  PASSED")


def test_layer_access_helpers():
    print("\nTest 3: Layer access helpers")
    m = ElevationMapCPU(Parameter())
    
    # Test getters
    elev = m.get_elevation()
    var = m.get_variance()
    valid = m.get_validity()
    trav = m.get_traversability()
    
    assert elev.shape == (m.cell_n, m.cell_n)
    assert var.shape == (m.cell_n, m.cell_n)
    assert valid.shape == (m.cell_n, m.cell_n)
    assert trav.shape == (m.cell_n, m.cell_n)
    
    # Test initial values
    assert np.all(var == m.param.initial_variance)
    assert np.all(trav == 1.0)
    assert np.all(valid == 0.0)
    print("  PASSED")


def test_world_to_grid():
    print("\nTest 4: World to grid coordinate transform")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Center of map (0, 0) should be at center cell
    row, col = m.world_to_grid(0.0, 0.0)
    expected_center = m.cell_n // 2
    assert row == expected_center
    assert col == expected_center
    
    # Move 1m in +X direction
    row, col = m.world_to_grid(1.0, 0.0)
    assert col == expected_center + 20  # 1m / 0.05 = 20 cells
    assert row == expected_center
    
    # Move 1m in +Y direction
    row, col = m.world_to_grid(0.0, 1.0)
    assert row == expected_center + 20
    assert col == expected_center
    print("  PASSED")


def test_grid_to_world():
    print("\nTest 5: Grid to world coordinate transform")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    center = m.cell_n // 2
    x, y = m.grid_to_world(center, center)
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    
    # One cell in +X
    x, y = m.grid_to_world(center, center + 1)
    assert abs(x - 0.05) < 1e-6
    assert abs(y) < 1e-6
    print("  PASSED")


def test_round_trip_coordinates():
    print("\nTest 6: Round-trip world -> grid -> world")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    test_points = [(0.0, 0.0), (1.0, 2.0), (-3.0, 1.5), (4.9, -4.9)]
    for x, y in test_points:
        row, col = m.world_to_grid(x, y)
        x2, y2 = m.grid_to_world(row, col)
        assert abs(x - x2) < 0.05  # Within half cell
        assert abs(y - y2) < 0.05
    print("  PASSED")


def test_is_inside():
    print("\nTest 7: Bounds checking")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    center = m.cell_n // 2
    assert m.is_inside(center, center) == True
    assert m.is_inside(0, 0) == True
    assert m.is_inside(m.cell_n - 1, m.cell_n - 1) == True
    assert m.is_inside(-1, 0) == False
    assert m.is_inside(0, -1) == False
    assert m.is_inside(m.cell_n, 0) == False
    assert m.is_inside(0, m.cell_n) == False
    print("  PASSED")


def test_valid_region_slice():
    print("\nTest 8: Valid region slice (excludes border)")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    s_row, s_col = m.get_valid_region_slice()
    assert s_row.start == 1  # border = 1
    assert s_row.stop == m.cell_n - 1
    assert s_col.start == 1
    assert s_col.stop == m.cell_n - 1
    assert s_row.stop - s_row.start == m.true_cell_n
    print(f"  Slice: {s_row}, {s_col} (true_cell_n={m.true_cell_n})")
    print("  PASSED")


def test_internal_to_gridmap():
    print("\nTest 9: Internal to GridMap coordinate conversion", flush=True)
    param = Parameter(resolution=0.1, map_length=2.0)
    param.update()
    m = ElevationMapCPU(param)
    
    # Create simple pattern: known values at specific positions
    arr = np.zeros((m.cell_n, m.cell_n), dtype=np.float32)
    center = m.cell_n // 2
    arr[center, center] = 1.0      # Center (Y=0, X=0)
    arr[center-1, center] = 2.0    # One up in Y (Y=+0.1, X=0)
    arr[center, center+1] = 3.0    # One right in X (Y=0, X=+0.1)
    
    gm = m.internal_to_gridmap(arr)
    
    # In GridMap convention:
    # - Row -> -X (increasing row = decreasing X)
    # - Col -> -Y (increasing col = decreasing Y)
    # Internal (row=Y, col=X) -> GridMap (row=-X, col=-Y)
    # For even grids, center shifts by half cell - verify invertibility instead
    
    # Verify invertibility: GridMap -> Internal
    # Inverse: flip axis 1, flip axis 0, then transpose
    arr2 = np.flip(np.flip(gm, axis=1), axis=0).T
    
    # Debug output
    print(f"  arr[{center-1}:{center+2}, {center-1}:{center+2}]:", flush=True)
    print(arr[center-1:center+2, center-1:center+2], flush=True)
    print(f"  gm[{center-1}:{center+2}, {center-1}:{center+2}]:", flush=True)
    print(gm[center-1:center+2, center-1:center+2], flush=True)
    print(f"  arr2[{center-1}:{center+2}, {center-1}:{center+2}]:", flush=True)
    print(arr2[center-1:center+2, center-1:center+2], flush=True)
    
    close = np.allclose(arr, arr2)
    print(f"  allclose={close}", flush=True)
    if not close:
        print(f"  Max diff: {np.max(np.abs(arr - arr2))}", flush=True)
    assert close, "Transform not invertible!"
    print("  PASSED", flush=True)


def test_get_valid_region():
    print("\nTest 10: Get valid region data")
    m = ElevationMapCPU(Parameter(resolution=0.1, map_length=2.0))
    m.elevation_map[m.IDX_ELEVATION, 1, 1] = 5.0  # Inside valid region
    m.elevation_map[m.IDX_ELEVATION, 0, 0] = 99.0  # In border
    
    s_row, s_col = m.get_valid_region_slice()
    valid_elev = m.elevation_map[m.IDX_ELEVATION, s_row, s_col]
    
    assert valid_elev.shape == (m.true_cell_n, m.true_cell_n)
    assert valid_elev[0, 0] == 5.0  # (1,1) relative to valid region
    # Border value should be excluded
    assert 99.0 not in valid_elev
    print("  PASSED")


def test_get_cell_center_world():
    print("\nTest 11: Get cell center world coordinates")
    param = Parameter(resolution=0.05, map_length=10.0)
    param.update()
    m = ElevationMapCPU(param)
    
    center = m.cell_n // 2
    x, y = m.get_cell_center_world(center, center)
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    
    # Next cell in +X
    x, y = m.get_cell_center_world(center, center + 1)
    assert abs(x - 0.05) < 1e-6
    assert abs(y) < 1e-6
    print("  PASSED")


if __name__ == '__main__':
    tests = [
        test_layer_indices,
        test_layer_names,
        test_layer_access_helpers,
        test_world_to_grid,
        test_grid_to_world,
        test_round_trip_coordinates,
        test_is_inside,
        test_valid_region_slice,
        test_internal_to_gridmap,
        test_get_valid_region,
        test_get_cell_center_world,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  FAILED: {e}")
            failed += 1
    
    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    
    if failed > 0:
        sys.exit(1)