#!/usr/bin/env python3
"""Test parameter validation."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

from terralink_elevation.parameter import Parameter

def test_valid_defaults():
    print("Test 1: Valid defaults")
    p = Parameter()
    print(f"  cell_n={p.cell_n}, true_cell_n={p.true_cell_n}")
    assert p.validate() == []
    print("  PASSED")

def test_invalid_resolution_negative():
    print("\nTest 2: Invalid resolution (negative)")
    try:
        p = Parameter(resolution=-0.05)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_invalid_resolution_too_large():
    print("\nTest 3: Invalid resolution (>1.0)")
    try:
        p = Parameter(resolution=2.0)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_invalid_map_length():
    print("\nTest 4: Invalid map_length (negative)")
    try:
        p = Parameter(map_length=-10)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_min_height_ge_max_height():
    print("\nTest 5: min_height >= max_height")
    try:
        p = Parameter(min_height=5.0, max_height=3.0)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_valid_custom_params():
    print("\nTest 6: Valid custom params")
    p = Parameter(resolution=0.04, map_length=8.0, sensor_noise_factor=0.03)
    print(f"  cell_n={p.cell_n}, true_cell_n={p.true_cell_n}, true_map_length={p.true_map_length}")
    assert p.validate() == []
    print("  PASSED")
    return True

def test_drift_alpha_out_of_range():
    print("\nTest 7: drift_compensation_alpha out of range")
    try:
        p = Parameter(drift_compensation_alpha=1.5)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_cleanup_step_out_of_range():
    print("\nTest 8: cleanup_step out of range")
    try:
        p = Parameter(cleanup_step=1.5)
        print("  ERROR: Should have failed")
        return False
    except ValueError as e:
        print(f"  Correctly rejected: {e}")
        return True

def test_yaml_roundtrip():
    print("\nTest 9: YAML round-trip")
    import tempfile
    import os
    
    p = Parameter(resolution=0.04, map_length=8.0, sensor_noise_factor=0.03)
    
    with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as f:
        temp_path = f.name
    
    try:
        p.save_yaml(temp_path)
        p2 = Parameter.from_yaml(temp_path)
        
        assert p2.resolution == p.resolution
        assert p2.map_length == p.map_length
        assert p2.sensor_noise_factor == p.sensor_noise_factor
        assert p2.cell_n == p.cell_n
        assert p2.true_cell_n == p.true_cell_n
        print("  PASSED")
        return True
    finally:
        os.unlink(temp_path)

def test_from_yaml_nested_structure():
    print("\nTest 10: from_yaml handles nested structure")
    import tempfile
    import os
    import yaml
    
    # Create YAML with nested structure (like ROS params)
    data = {
        'terralink_elevation': {
            'ros__parameters': {
                'resolution': 0.03,
                'map_length': 10.0
            }
        }
    }
    
    with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False, mode='w') as f:
        yaml.dump(data, f)
        temp_path = f.name
    
    try:
        p = Parameter.from_yaml(temp_path)
        assert p.resolution == 0.03
        assert p.map_length == 10.0
        print("  PASSED")
        return True
    finally:
        os.unlink(temp_path)

def test_update_recomputes():
    print("\nTest 11: update() recomputes derived values")
    p = Parameter(resolution=0.05, map_length=10.0)
    old_cell_n = p.cell_n
    
    p.resolution = 0.025
    p.update()
    
    assert p.cell_n != old_cell_n
    assert p.true_cell_n == round(10.0 / 0.025)
    print(f"  Old cell_n: {old_cell_n}, New cell_n: {p.cell_n}")
    print("  PASSED")
    return True

if __name__ == '__main__':
    tests = [
        test_valid_defaults,
        test_invalid_resolution_negative,
        test_invalid_resolution_too_large,
        test_invalid_map_length,
        test_min_height_ge_max_height,
        test_valid_custom_params,
        test_drift_alpha_out_of_range,
        test_cleanup_step_out_of_range,
        test_yaml_roundtrip,
        test_from_yaml_nested_structure,
        test_update_recomputes,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  FAILED with exception: {e}")
            failed += 1
    
    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    
    if failed > 0:
        sys.exit(1)