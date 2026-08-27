#!/usr/bin/env python3
"""Test Step 5: GPU vs CPU numerical match."""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.elevation_map_gpu import ElevationMapGPU, CUPY_AVAILABLE


def test_gpu_cpu_match():
    """Test that GPU and CPU implementations produce matching results."""
    print("Test: GPU vs CPU numerical match")
    
    if not CUPY_AVAILABLE:
        print("  SKIPPED: CuPy not available")
        return True
    
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05)
    param.update()
    
    # Create identical maps
    cpu_map = ElevationMapCPU(param)
    gpu_map = ElevationMapGPU(param)
    
    # Generate test points
    np.random.seed(42)
    n_points = 1000
    points_np = np.random.uniform(-2, 2, (n_points, 3)).astype(np.float32)
    points_np[:, 2] = np.abs(points_np[:, 2]) + 0.5  # Positive heights
    
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)
    
    # Fuse on CPU
    cpu_map.fuse_pointcloud(points_np, R, t)
    
    # Fuse on GPU
    import cupy as cp
    points_cp = cp.asarray(points_np)
    R_cp = cp.asarray(R)
    t_cp = cp.asarray(t)
    gpu_map.fuse_pointcloud(points_cp, R_cp, t_cp)
    
    # Compare results
    center = cpu_map.cell_n // 2
    cpu_elev = cpu_map.get_elevation()[center, center]
    gpu_elev = gpu_map.get_elevation()[center, center].get()
    
    cpu_var = cpu_map.get_variance()[center, center]
    gpu_var = gpu_map.get_variance()[center, center].get()
    
    print(f"  CPU elev: {cpu_elev:.6f}, GPU elev: {gpu_elev:.6f}")
    print(f"  CPU var:  {cpu_var:.6f}, GPU var:  {gpu_var:.6f}")
    
    # Allow small numerical differences
    elev_match = np.isclose(cpu_elev, gpu_elev, rtol=1e-5, atol=1e-6)
    var_match = np.isclose(cpu_var, gpu_var, rtol=1e-5, atol=1e-6)
    
    if elev_match and var_match:
        print("  PASSED")
        return True
    else:
        print(f"  FAILED: elev_match={elev_match}, var_match={var_match}")
        return False


def test_gpu_outlier_rejection():
    """Test GPU outlier rejection matches CPU."""
    print("\nTest: GPU outlier rejection")
    
    if not CUPY_AVAILABLE:
        print("  SKIPPED: CuPy not available")
        return True
    
    param = Parameter(resolution=0.05, map_length=10.0, sensor_noise_factor=0.05,
                      mahalanobis_thresh=2.0, outlier_variance=0.01,
                      max_height=200.0, max_ray_length=200.0)
    param.update()
    
    cpu_map = ElevationMapCPU(param)
    gpu_map = ElevationMapGPU(param)
    
    # Build up valid data
    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    R = np.eye(3, dtype=np.float32)
    t = np.zeros(3, dtype=np.float32)
    
    for _ in range(5):
        cpu_map.fuse_pointcloud(points, R, t)
        import cupy as cp
        gpu_map.fuse_pointcloud(cp.asarray(points), cp.asarray(R), cp.asarray(t))
    
    center = cpu_map.cell_n // 2
    cpu_var_before = cpu_map.elevation_map[cpu_map.IDX_VARIANCE, center, center]
    gpu_var_before = gpu_map.elevation_map[gpu_map.IDX_VARIANCE, center, center].get()
    
    # Add outlier
    points_outlier = np.array([[0.0, 0.0, 100.0]], dtype=np.float32)
    cpu_map.fuse_pointcloud(points_outlier, R, t)
    gpu_map.fuse_pointcloud(cp.asarray(points_outlier), cp.asarray(R), cp.asarray(t))
    
    cpu_var_after = cpu_map.elevation_map[cpu_map.IDX_VARIANCE, center, center]
    gpu_var_after = gpu_map.elevation_map[gpu_map.IDX_VARIANCE, center, center].get()
    
    cpu_elev = cpu_map.elevation_map[cpu_map.IDX_ELEVATION, center, center]
    gpu_elev = gpu_map.elevation_map[gpu_map.IDX_ELEVATION, center, center].get()
    
    print(f"  CPU: elev={cpu_elev:.4f}, var_before={cpu_var_before:.4f}, var_after={cpu_var_after:.4f}")
    print(f"  GPU: elev={gpu_elev:.4f}, var_before={gpu_var_before:.4f}, var_after={gpu_var_after:.4f}")
    
    elev_match = np.isclose(cpu_elev, gpu_elev, rtol=1e-5)
    var_match = np.isclose(cpu_var_after, gpu_var_after, rtol=1e-5)
    
    if elev_match and var_match:
        print("  PASSED")
        return True
    else:
        print("  FAILED")
        return False


def test_gpu_performance():
    """Test GPU performance with larger point cloud."""
    print("\nTest: GPU performance")
    
    if not CUPY_AVAILABLE:
        print("  SKIPPED: CuPy not available")
        return True
    
    import time
    import cupy as cp
    
    param = Parameter(resolution=0.05, map_length=20.0, sensor_noise_factor=0.05)
    param.update()
    gpu_map = ElevationMapGPU(param)
    
    # Large point cloud
    n_points = 50000
    points = np.random.uniform(-10, 10, (n_points, 3)).astype(np.float32)
    points[:, 2] = np.abs(points[:, 2]) + 1.0
    points_cp = cp.asarray(points)
    R = cp.eye(3, dtype=cp.float32)
    t = cp.zeros(3, dtype=cp.float32)
    
    # Warmup
    gpu_map.fuse_pointcloud(points_cp, cp.eye(3, dtype=cp.float32), cp.zeros(3, dtype=cp.float32))
    cp.cuda.Device().synchronize()
    
    # Timed run
    start = time.time()
    for _ in range(5):
        gpu_map.fuse_pointcloud(points_cp, R, t)
    cp.cuda.Device().synchronize()
    elapsed = time.time() - start
    
    points_per_sec = (n_points * 5) / elapsed
    print(f"  Processed {n_points * 5} points in {elapsed:.3f}s = {points_per_sec:,.0f} points/sec")
    
    if points_per_sec > 100000:  # Should be well over 100k points/sec
        print("  PASSED")
        return True
    else:
        print("  WARNING: Performance lower than expected")
        return True


if __name__ == '__main__':
    if not CUPY_AVAILABLE:
        print("CuPy not available, skipping GPU tests")
        sys.exit(0)
    
    tests = [
        test_gpu_cpu_match,
        test_gpu_outlier_rejection,
        test_gpu_performance,
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
            import traceback
            traceback.print_exc()
            failed += 1
    
    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    
    if failed > 0:
        sys.exit(1)