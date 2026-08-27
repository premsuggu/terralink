# Unit Testing with pytest

**Goal**: Write and run unit tests for each implementation step.  
**Time to Read**: ~15 minutes  
**Prerequisites**: [02_python_robotics.md](02_python_robotics.md)

---

## 1. Why Test?

- **Catch bugs early** - Before integration
- **Document expected behavior** - Tests = living documentation
- **Enable refactoring** - Change code, run tests, confidence
- **Required per step** - Our plan: one test file per step

---

## 2. Test Structure

```
tests/elevation_mapping/
├── __init__.py
├── conftest.py              # Shared fixtures
├── test_step01_skeleton.py
├── test_step02_parameters.py
├── test_step03_data_structures.py
├── test_step04_fusion_cpu.py
├── test_step05_fusion_gpu.py
├── test_step06_ray_tracing.py
├── test_step07_map_shifting.py
├── test_step08_drift_comp.py
├── test_step09_traversability.py
└── test_step10_uav_integration.py
```

---

## 3. Basic pytest Syntax

```python
# test_example.py
import numpy as np
import pytest

def test_addition():
    assert 1 + 1 == 2

def test_array_equality():
    a = np.array([1, 2, 3])
    b = np.array([1, 2, 3])
    np.testing.assert_array_equal(a, b)

def test_float_comparison():
    # Use approx for floating point
    assert 0.1 + 0.2 == pytest.approx(0.3, rel=1e-6)

class TestMapOperations:
    def setup_method(self):
        """Run before each test method."""
        self.grid = np.zeros((10, 10))
    
    def test_grid_shape(self):
        assert self.grid.shape == (10, 10)
    
    def test_grid_dtype(self):
        assert self.grid.dtype == np.float64

# Parametrized tests (multiple inputs)
@pytest.mark.parametrize("input,expected", [
    (0, 0),
    (1, 1),
    (2, 4),
    (3, 9),
])
def test_square(input, expected):
    assert input * input == expected
```

---

## 4. Fixtures (Shared Test Data)

```python
# conftest.py
import pytest
import numpy as np
import cupy as cp

@pytest.fixture
def simple_grid():
    """Simple 5x5 grid for testing."""
    return np.zeros((5, 5), dtype=np.float32)

@pytest.fixture
def gaussian_bump_grid():
    """Grid with known Gaussian bump."""
    x = np.arange(-2, 3, dtype=np.float32)
    y = np.arange(-2, 3, dtype=np.float32)
    X, Y = np.meshgrid(x, y)
    Z = np.exp(-(X**2 + Y**2) / 2.0)  # Peak at center = 1.0
    return Z

@pytest.fixture(scope="session")
def cupy_available():
    """Check if CuPy/GPU available."""
    try:
        import cupy as cp
        cp.cuda.Device(0).use()
        return True
    except:
        return False

@pytest.fixture
def sample_pointcloud():
    """Generate synthetic point cloud."""
    # 100 points on flat ground z=0
    xyz = np.random.uniform(-5, 5, (100, 3)).astype(np.float32)
    xyz[:, 2] = 0.0  # Flat ground
    return xyz

@pytest.fixture
def sample_pointcloud_gpu(sample_pointcloud):
    """Same pointcloud on GPU."""
    cp = pytest.importorskip("cupy")
    return cp.asarray(sample_pointcloud)
```

---

## 5. Testing NumPy/CuPy Code

```python
# test_step04_fusion_cpu.py
import numpy as np
import pytest
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.parameter import Parameter

class TestBayesianFusion:
    """Test Bayesian height fusion math."""
    
    @pytest.fixture
    def params(self):
        p = Parameter()
        p.resolution = 0.05
        p.map_length = 10.0
        p.sensor_noise_factor = 0.05
        p.mahalanobis_thresh = 2.0
        p.update()
        return p
    
    @pytest.fixture
    def map_obj(self, params):
        return ElevationMapCPU(params)
    
    def test_fusion_single_point(self, map_obj):
        """Single point at origin should set elevation."""
        # Point at map center (0, 0, 1.0)
        points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
        R = np.eye(3, dtype=np.float32)
        t = np.zeros(3, dtype=np.float32)
        
        map_obj.fuse_pointcloud(points, R, t)
        
        # Check center cell
        center = map_obj.cell_n // 2
        assert map_obj.elevation_map[0, center, center] == pytest.approx(1.0, abs=1e-3)
        assert map_obj.elevation_map[2, center, center] == 1.0  # is_valid
    
    def test_fusion_multiple_points_same_cell(self, map_obj):
        """Multiple points in same cell should average (Bayesian)."""
        # 10 points at same location, z=1.0
        points = np.tile([0.0, 0.0, 1.0], (10, 1)).astype(np.float32)
        R = np.eye(3, dtype=np.float32)
        t = np.zeros(3, dtype=np.float32)
        
        map_obj.fuse_pointcloud(points, R, t)
        
        center = map_obj.cell_n // 2
        # With same measurement, posterior = measurement
        assert map_obj.elevation_map[0, center, center] == pytest.approx(1.0, abs=1e-3)
        # Variance should decrease with more measurements
        assert map_obj.elevation_map[1, center, center] < 0.05
    
    def test_outlier_rejection(self, map_obj):
        """Outlier (z=100) should be rejected, not fused."""
        # First: establish ground at z=0
        points_ground = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        map_obj.fuse_pointcloud(points_ground, np.eye(3), np.zeros(3))
        
        center = map_obj.cell_n // 2
        ground_h = map_obj.elevation_map[0, center, center]
        
        # Now: outlier at z=100 (way above ground)
        points_outlier = np.array([[0.0, 0.0, 100.0]], dtype=np.float32)
        map_obj.fuse_pointcloud(points_outlier, np.eye(3), np.zeros(3))
        
        # Elevation should NOT change (outlier rejected)
        new_h = map_obj.elevation_map[0, center, center]
        assert new_h == pytest.approx(ground_h, abs=1e-3)
        # But variance should increase
        assert map_obj.elevation_map[1, center, center] > 0.01
```

---

## 6. Testing GPU Code (CuPy)

```python
# test_step05_fusion_gpu.py
import cupy as cp
import numpy as np
import pytest
from terralink_elevation.elevation_map import ElevationMapGPU
from terralink_elevation.parameter import Parameter

@pytest.mark.gpu
class TestGPUFusion:
    """Test GPU kernel matches CPU reference."""
    
    @pytest.fixture
    def params(self):
        p = Parameter()
        p.resolution = 0.05
        p.map_length = 10.0
        p.sensor_noise_factor = 0.05
        p.mahalanobis_thresh = 2.0
        p.update()
        return p
    
    def test_gpu_matches_cpu(self, params, sample_pointcloud_gpu):
        """GPU fusion should match CPU fusion numerically."""
        # CPU reference
        cpu_map = ElevationMapCPU(params)
        cpu_map.fuse_pointcloud(sample_pointcloud_cpu, R, t)
        
        # GPU version
        gpu_map = ElevationMapGPU(params)
        gpu_map.fuse_pointcloud(sample_pointcloud_gpu, R_cp, t_cp)
        
        # Compare (allow small numerical differences)
        cp.testing.assert_allclose(
            cpu_map.elevation_map.get(),  # CPU -> GPU for comparison
            gpu_map.elevation_map,
            rtol=1e-5, atol=1e-6
        )
    
    def test_gpu_performance(self, params, sample_pointcloud_gpu):
        """Benchmark GPU fusion speed."""
        import time
        gpu_map = ElevationMapGPU(params)
        
        # Warmup
        gpu_map.fuse_pointcloud(sample_pointcloud_gpu, R_cp, t_cp)
        cp.cuda.Device().synchronize()
        
        # Timed run
        start = time.time()
        for _ in range(10):
            gpu_map.fuse_pointcloud(sample_pointcloud_gpu, R_cp, t_cp)
        cp.cuda.Device().synchronize()
        elapsed = time.time() - start
        
        print(f"10 frames: {elapsed:.3f}s, {elapsed/10*1000:.1f}ms/frame")
        assert elapsed / 10 < 0.01  # < 10ms per frame
```

---

## 7. Running Tests

```bash
# All tests
cd /home/prem/terralink
python -m pytest tests/elevation_mapping/ -v

# Specific step
python -m pytest tests/elevation_mapping/test_step01_skeleton.py -v

# With coverage
python -m pytest tests/elevation_mapping/ --cov=terralink_elevation --cov-report=term-missing

# Only GPU tests (if marked)
python -m pytest tests/elevation_mapping/ -m gpu -v

# Verbose with output
python -m pytest tests/elevation_mapping/test_step04_fusion_cpu.py -v -s

# Stop on first failure
python -m pytest tests/elevation_mapping/ -x
```

---

## 8. Test Markers

```python
# conftest.py
def pytest_configure(config):
    config.addinivalue_line("markers", "gpu: requires CUDA GPU")
    config.addinivalue_line("markers", "slow: takes > 10 seconds")
    config.addinivalue_line("markers", "integration: requires ROS/Gazebo")

# Usage
@pytest.mark.gpu
def test_gpu_kernel():
    ...

@pytest.mark.slow
def test_large_map():
    ...

@pytest.mark.integration
def test_ros_node():
    ...
```

```bash
# Run only GPU tests
pytest -m gpu

# Skip GPU tests
pytest -m "not gpu"

# Skip slow tests
pytest -m "not slow"
```

---

## 9. Testing ROS 2 Nodes (Integration)

```python
# test_step10_uav_integration.py
import pytest
import rclpy
from rclpy.node import Node

@pytest.mark.integration
class TestROS2Node:
    """Requires running ROS 2 environment."""
    
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = Node('test_node')
    
    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_node_creation(self):
        from terralink_elevation.elevation_mapping_node import ElevationMappingNode
        node = ElevationMappingNode()
        assert node.get_name() == 'elevation_mapping_node'
        node.destroy_node()
    
    def test_parameter_loading(self):
        from terralink_elevation.elevation_mapping_node import ElevationMappingNode
        node = ElevationMappingNode()
        # Check parameters loaded
        assert node.get_parameter('resolution').value > 0
        node.destroy_node()
```

---

## 10. CI/CD Integration (GitHub Actions)

```yaml
# .github/workflows/test.yml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble
      
      - name: Install dependencies
        run: |
          sudo apt update
          sudo apt install -y ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin
          pip install cupy-cuda12x==13.6.0 numpy==1.24.2 simple_parsing pytest
      
      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --packages-select terralink_elevation --symlink-install
      
      - name: Run unit tests
        run: |
          source /opt/ros/humble/setup.bash
          source install/local_setup.bash
          python -m pytest tests/elevation_mapping/ -v -m "not gpu and not integration"
```

---

## 11. Best Practices

| Practice | Why |
|----------|-----|
| One test file per step | Matches implementation plan |
| Test CPU before GPU | Isolate algorithm bugs from GPU issues |
| Use fixtures for common data | DRY, consistent test setup |
| Parametrize for edge cases | Test boundaries (0, negative, large) |
| Mock ROS in unit tests | Fast, no ROS dependency |
| Mark integration tests | Run separately, need full environment |
| Assert with `pytest.approx` | Handle floating point correctly |
| Use `cp.testing.assert_allclose` | CuPy array comparison |

---

## Next: Start Step 1 Implementation!