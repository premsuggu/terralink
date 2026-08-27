# TerraLink UAV Elevation Mapping - From-Scratch Implementation Plan

**Focus**: UAV-mounted elevation mapping ONLY (no UGV/Nav2 integration)  
**Reference**: `src/d1/elevation_mapping_gpu_ros2/` (iit-DLSLab fork) - **DO NOT MODIFY**  
**Our Implementation**: New package in `src/terralink_elevation/`  
**Documentation**: `docs/work-logs/elevation_mapping/`  
**Tests**: `tests/elevation_mapping/`

---

## Philosophy & Constraints

| Constraint | Decision |
|------------|----------|
| **No touching reference code** | New package `terralink_elevation` in `src/` |
| **UAV-only focus** | No costmap converter, no Nav2 integration |
| **Modular & testable** | Each step = independent module + unit test |
| **Learning-focused** | Every concept documented in `work-logs/` with code references |
| **GPU-accelerated** | CuPy (like reference) but simpler, cleaner architecture |
| **ROS 2 Humble** | Native rclpy, GridMap messages |
| **Simulation-first** | Gazebo UAV + depth sensor, synthetic test surfaces |

---

## High-Level Architecture (UAV-Only)

```
┌─────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULATION                        │
│  ┌─────────────────┐    ┌──────────────────┐                   │
│  │  UAV Model      │    │  Test World      │                   │
│  │  (my_uav from   │    │  • Flat +        │                   │
│  │   d3/my_bot)    │    │    Gaussian bump │                   │
│  │                 │    │  • Construction  │                   │
│  │  + RGB-D Camera │    │    site terrain  │                   │
│  │  (RealSense     │    │                  │                   │
│  │   D435i model)  │    │                  │                   │
│  └────────┬────────┘    └──────────────────┘                   │
           │                                                      │
           ▼                                                      │
    ┌─────────────────────────────────────────────────────────┐   │
    │              UAV Sensor Output                          │   │
    │  /camera/depth/points (PointCloud2)                     │   │
    │  TF: map → camera_depth_optical_frame                   │   │
    └────────────────────────┬────────────────────────────────┘   │
                             │                                     │
                             ▼                                     │
┌─────────────────────────────────────────────────────────────────┐
│                  ElevationMappingNode (ROS 2)                   │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │ TF Listener  │  │ PointCloud   │  │ ElevationMap (GPU)  │   │
│  │ (sensor→map) │──▶│ Subscriber   │──▶│ • Bayesian Fusion  │   │
│  └──────────────┘  └──────────────┘  │ • Variance Track   │   │
│                                      │ • Ray Tracing      │   │
│                                      │ • Drift Comp       │   │
│                                      │ • Traversability   │   │
│                                      └─────────┬──────────┘   │
└────────────────────────────────────────────────┼────────────────┘
                                                 │ GridMap msg
                                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                        OUTPUT / VISUALIZATION                    │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐  │
│  │ GridMap Publisher│  │ RViz             │  │ Map Saver      │  │
│  │ /elevation_map   │──▶│ (elevation,      │  │ (save/load    │  │
│  │ (elevation,      │   │  variance,       │  │  .bag)        │  │
│  │  variance,       │   │  traversability) │  │              │  │
│  │  traversability) │   │                  │  │              │  │
│  └──────────────────┘  └──────────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Test Surfaces (Two Worlds)

### World 1: Gaussian Bump (Simple Verification)
- **Purpose**: Validate basic elevation mapping works
- **Terrain**: Flat ground (z=0) + single smooth Gaussian bump
- **Expected**: Clean elevation map, correct height at peak, smooth gradients
- **Metrics**: Peak height accuracy < 2cm, smooth gradients, no artifacts

### World 2: Construction Site (Realistic)
- **Purpose**: Stress-test on realistic unstructured terrain
- **Terrain**: 
  - Rough uneven ground (perlin noise, 0-0.15m variation)
  - Ramps (5°, 15°, 25° slopes)
  - Steps (5cm, 15cm, 30cm)
  - Piles of debris (random boxes)
  - Trenches/holes (negative obstacles)
  - Loose gravel patches (high roughness)
- **Expected**: 
  - Ramps < 20° → traversable
  - Steps > 15cm → lethal
  - Rough patches → high cost
  - Holes → detected as negative obstacles

---

## Implementation Steps (10 Steps, Each Verifiable)

### Step 1: Project Skeleton & Build System ✅ **COMPLETE**
**Goal**: Create buildable ROS 2 package with CuPy support

**Deliverables**:
- `src/terralink_elevation/` package structure
- `CMakeLists.txt`, `package.xml` with CuPy, grid_map_msgs deps
- `config/elevation_mapping.yaml` parameter file
- Basic launch file for node only

**Tests**: ✅ All pass
- `colcon build --packages-select terralink_elevation` succeeds
- `ros2 launch terralink_elevation elevation_mapping.launch.py` starts node

**Work-log**: `docs/work-logs/elevation_mapping/step01_project_skeleton.md`

---

### Step 2: Parameter System & Configuration ✅ **COMPLETE**
**Goal**: Clean parameter management (like `parameter.py` but simpler)

**Deliverables**:
- `terralink_elevation/parameter.py` - Dataclass with all params + YAML loading
- Validation & computed properties (cell_n from map_length/resolution)
- ROS 2 parameter overrides work

**Key Parameters**:
```python
# Map geometry
resolution: float = 0.05      # 5cm/cell
map_length: float = 20.0      # 20m x 20m map (larger for UAV coverage)
cell_n: int = None            # Computed: round(map_length/resolution) + 2

# Sensor noise
sensor_noise_factor: float = 0.05
min_valid_distance: float = 0.3
max_height_range: float = 5.0  # UAV flies higher

# Outlier rejection
mahalanobis_thresh: float = 2.0
outlier_variance: float = 0.01

# Drift compensation (UAV pose drifts more than UGV)
enable_drift_compensation: bool = True
max_drift: float = 0.10
position_noise_thresh: float = 0.2  # UAV position less precise

# Visibility cleanup
enable_visibility_cleanup: bool = True
max_ray_length: float = 10.0   # UAV sees further
cleanup_step: float = 0.05
cleanup_cos_thresh: float = 0.3
```

**Tests**:
- Parameter serialization/deserialization round-trip
- Computed properties update correctly when resolution changes

**Work-log**: `docs/work-logs/elevation_mapping/step02_parameter_system.md`

---

### Step 3: Core Data Structures (CPU First, Then GPU) ✅ **COMPLETE**
**Goal**: Multi-layer elevation map on CPU (NumPy) → verify logic → port to GPU (CuPy)

**Deliverables**:
- `terralink_elevation/elevation_map.py` - Core class
- 7-layer map: `[elevation, variance, is_valid, traversability, time, upper_bound, is_upper_bound]`
- Coordinate conventions documented (Row=Y, Col=X)
- GridMap coordinate transform helper

**Tests**: ✅ All 11 tests pass
- Map initialization with correct dimensions
- Coordinate transform: internal (Y,X) ↔ GridMap (flipped)
- Layer access helpers work
- GridMap coordinate conversion (invertible)

**Work-log**: `docs/work-logs/elevation_mapping/step03_core_data_structures.md`

---

### Step 4: Point Cloud → Map Fusion (Bayesian Update) - CPU ✅ **COMPLETE**
**Goal**: Implement the heart - Bayesian height fusion with variance tracking

**Tests**: ✅ All 10 tests pass
- Bayesian fusion single point
- Multiple measurements converge
- Multiple points in same cell
- Outlier rejection (elevation preserved, variance increased)
- Point validation (distance/height)
- Coordinate transform in fusion
- Multiple cells
- Sensor noise model
- Accumulator reset
- Finalize with no points

**Work-log**: `docs/work-logs/elevation_mapping/step04_bayesian_fusion.md`

---

### Step 5: GPU Acceleration with CuPy (Next)
- Synthetic point cloud → known map output
- Bayesian fusion math verified with manual calculation
- Outlier rejection works on synthetic outliers
- Variance decreases with repeated measurements

**Work-log**: `docs/work-logs/elevation_mapping/step04_bayesian_fusion.md`

---

### Step 5: GPU Acceleration with CuPy (In Progress - Known Issue)
**Goal**: Port fusion kernel to CuPy ElementwiseKernel (10-100x speedup)

**Status**: **KNOWN ISSUE** - ElementwiseKernel compilation failing due to argument type mismatch with CuPy's CArray vs raw pointer expectations.

**Deliverables** (Partially Complete):
- `terralink_elevation/kernels/fusion_kernel.py` - CuPy ElementwiseKernel (compilation issues)
- `terralink_elevation/elevation_map_gpu.py` - GPU ElevationMap class (fallback to CPU works)

**Current Issue**: 
- ElementwiseKernel compilation fails with "Wrong number of arguments" and "no suitable conversion function from CArray to const float*"
- The kernel expects raw pointers but receives CuPy CArray objects
- This is a known limitation of ElementwiseKernel with `raw` type parameters

**Workaround**: CPU implementation (ElevationMapCPU) is fully functional and tested. GPU acceleration marked as future improvement.

**Work-log**: `docs/work-logs/elevation_mapping/step05_gpu_acceleration.md`

---

---

### Step 6: Visibility Cleanup (Ray Tracing)
**Goal**: Mark free space along sensor rays (critical for map completeness)

**Algorithm** (from reference `custom_kernels.py:209-269`):
```
For each fused point:
  1. Ray direction: from sensor origin to point
  2. Step along ray at resolution/√2 increments (max max_ray_length)
  3. For each cell along ray (except near endpoint):
     if cell was valid AND not recently updated AND ray penetrates surface:
        Check normal alignment: |ray · normal| < cleanup_cos_thresh
        If aligned (grazing ground): atomicAdd(is_valid, -cleanup_step), atomicAdd(variance, outlier_variance)
        If perpendicular (wall): SKIP (don't clear walls!)
```

**Implementation**:
- Extend fusion kernel OR separate ray tracing kernel
- Normal map computation needed (gradient of elevation)
- Parameters: `ray_step = resolution / sqrt(2)`

**Tests**:
- Synthetic: flat ground → ray cells marked free
- Wall at edge → wall cells NOT cleared (cosine check)
- Variance increases in cleared cells

**Work-log**: `docs/work-logs/elevation_mapping/step06_visibility_cleanup.md`

---

### Step 7: Map Shifting (UAV-Centric Mapping)
**Goal**: Move map to follow UAV (critical for continuous operation over large areas)

**Algorithm** (from reference `elevation_mapping.py:236-258`):
```python
def shift_map_xy(self, delta_pixel):
    # delta_pixel = [dx, dy] in WORLD coords (X forward, Y left)
    # Map array: (layers, height=rows=Y, width=cols=X)
    # cp.roll axis=(1,2) expects [row_shift, col_shift] = [dy, dx]
    # CRITICAL: SWAP [dx, dy] → [dy, dx]!
    shift_value = cp.array([delta_pixel[1], delta_pixel[0]], dtype=cp.int32)
    
    self.elevation_map = cp.roll(self.elevation_map, shift_value, axis=(1, 2))
    
    # Pad new edges
    self.pad_value(self.elevation_map, shift_value, value=0.0)           # elevation
    self.pad_value(self.elevation_map, shift_value, idx=1, value=initial_variance)  # variance
    self.pad_value(self.elevation_map, shift_value, idx=2, value=0.0)    # is_valid
```

**Integration**:
- Called from ROS 2 timer at `update_pose_fps` (10 Hz)
- Uses TF: `map_frame` → `base_frame` (UAV base_link)
- Computes delta from previous center

**Tests**:
- Shift by known pixels → map content moves correctly
- Axis swap verified (X shift moves columns, Y shift moves rows)
- Padding values correct
- No data loss over many shifts

**Work-log**: `docs/work-logs/elevation_mapping/step07_map_shifting.md`

---

### Step 8: Drift Compensation
**Goal**: Correct accumulated height error from UAV odometry/pose drift

**Algorithm** (from reference `elevation_mapping.py:368-379`):
```
1. error_counting_kernel: For each point:
     if valid AND |z - map_h| < map_v * mahalanobis_thresh 
        AND map_v < outlier_variance/2 
        AND traversability > traversability_inlier:
         error += (z - map_h)
         error_cnt += 1
2. mean_error = error / error_cnt
3. If |mean_error| < max_drift AND pose_change > thresholds:
     elevation_map[0] += mean_error * drift_compensation_alpha
```

**Implementation**:
- Separate kernel for error counting (runs on valid traversable points)
- Host computes mean, applies correction
- Triggered by pose update timer

**Tests**:
- Simulated drift → correction applied
- Doesn't trigger on non-flat terrain
- Max drift limit respected

**Work-log**: `docs/work-logs/elevation_mapping/step08_drift_compensation.md`

---

### Step 9: Traversability Estimation (Analytical)
**Goal**: Compute traversability cost from elevation + variance (no ML, runs on GPU)

**Implementation** (analytical, like `integration_guide.md`):
```python
def compute_traversability(elevation, variance, resolution, params):
    # Slope (gradient magnitude)
    grad_x = gradient(elevation, axis=1) / resolution
    grad_y = gradient(elevation, axis=0) / resolution
    slope = sqrt(grad_x² + grad_y²)
    
    # Step height (3x3 max-min)
    step_height = maximum_filter(elevation, 3) - minimum_filter(elevation, 3)
    
    # Roughness = variance
    roughness = variance
    
    # Classify (output 0-1, higher = more traversable)
    lethal = (slope > max_slope) | (step_height > max_step) | (roughness > max_roughness)
    difficult = (slope > max_slope*0.5) | (step_height > max_step*0.5)
    
    trav = ones_like(elevation, dtype=float32)
    trav[lethal] = 0.0
    trav[difficult] = 0.3
    trav[valid & ~lethal & ~difficult] = 1.0
    return trav
```

**Parameters** (tunable for UAV perspective):
```python
max_slope: 0.35      # ~20 deg
max_step: 0.15       # 15cm
max_roughness: 0.05  # 5cm variance
```

**Tests**:
- Ramp (10°) → traversable (~1.0)
- Wall (90°) → lethal (0.0)
- Rough terrain → low traversability (~0.3)
- Flat ground → traversable (1.0)
- Gaussian bump peak → traversable, correct slope

**Work-log**: `docs/work-logs/elevation_mapping/step09_traversability.md`

---

### Step 10: ROS 2 Node + Gazebo UAV Integration
**Goal**: Complete ROS 2 node with UAV simulation, sensor, and test worlds

**Components**:
- `terralink_elevation/elevation_mapping_node.py` - Main node
- PointCloud2 subscriber (QoS SENSOR_DATA)
- TF listener (sensor_frame → map_frame)
- GridMap publisher (configurable layers, rate)
- Pose update timer (10 Hz)
- Variance/time update timers
- Services: save_map, load_map

**Gazebo Integration**:
- Use existing `d3/my_bot/description/uav.sdf` + add RGB-D camera (RealSense D435i model)
- Create `worlds/gaussian_bump.world` - flat + single bump
- Create `worlds/construction_site.world` - realistic rough terrain
- Launch file: `launch/elevation_mapping_sim.launch.py`
  - Spawns UAV at height (e.g., 5m)
  - Starts elevation_mapping_node
  - Optional RViz

**Message Flow**:
```
PointCloud2 callback:
  1. ros2_numpy → CuPy array (zero-copy)
  2. TF lookup: sensor_frame → map_frame (R, t)
  3. elevation_map.fuse_pointcloud(points, R, t)
  4. elevation_map.update_traversability()
  
Pose update timer (10 Hz):
  1. TF lookup: map_frame → base_frame (UAV)
  2. elevation_map.move_to(position, rotation)
  
Publish timer (2 Hz):
  1. elevation_map.to_gridmap_msg()
  2. Publish on /elevation_map
```

**Tests**:
- Node starts without errors
- Receives PointCloud2 from Gazebo, publishes GridMap
- TF transforms work correctly
- Map follows simulated UAV movement
- **Gaussian bump world**: Map shows correct bump height/profile
- **Construction site world**: Map captures ramps, steps, roughness

**Work-log**: `docs/work-logs/elevation_mapping/step10_ros2_uav_integration.md`

---

## Testing Strategy

### Unit Tests (Per Step)
```
tests/elevation_mapping/
├── test_step01_skeleton.py           # Build test
├── test_step02_parameters.py         # Parameter serialization
├── test_step03_data_structures.py    # Map layout, coord transform
├── test_step04_fusion_cpu.py         # Bayesian fusion math
├── test_step05_fusion_gpu.py         # GPU kernel vs CPU
├── test_step06_ray_tracing.py        # Visibility cleanup
├── test_step07_map_shifting.py       # Axis swap, padding
├── test_step08_drift_comp.py         # Drift correction
├── test_step09_traversability.py     # Slope/step/roughness
└── test_step10_uav_integration.py    # ROS node + Gazebo (optional, slow)
```

### Simulation Test Worlds
```
src/terralink_elevation/worlds/
├── gaussian_bump.world       # Simple: flat + Gaussian bump
└── construction_site.world   # Realistic: ramps, steps, holes, debris
```

### Run Commands
```bash
# Unit tests (fast, no ROS/Gazebo)
cd tests/elevation_mapping
python -m pytest test_step01_skeleton.py -v
python -m pytest test_step02_parameters.py -v
# ...

# Integration test: Gaussian bump world
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=gaussian_bump

# Integration test: Construction site world
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=construction_site

# With RViz
ros2 launch terralink_elevation elevation_mapping_sim.launch.py world:=construction_site launch_rviz:=true
```

---

## Documentation Plan (Work-Logs)

Each step gets a detailed work-log explaining:

| Section | Purpose |
|---------|---------|
| **Concept** | Mathematical/algorithmic foundation (with equations) |
| **Reference Code** | Exact line references to `src/d1/...` for comparison |
| **Our Implementation** | Our code with inline comments |
| **Why This Way** | Design decisions, tradeoffs vs reference |
| **Test Cases** | What we verify, expected outputs |
| **Common Pitfalls** | Gotchas we encountered/fixed |

**Files to Create**:
```
docs/work-logs/elevation_mapping/
├── step01_project_skeleton.md
├── step02_parameter_system.md
├── step03_core_data_structures.md
├── step04_bayesian_fusion.md
├── step05_gpu_acceleration.md
├── step06_visibility_cleanup.md
├── step07_map_shifting.md
├── step08_drift_compensation.md
├── step09_traversability.md
└── step10_ros2_uav_integration.md
```

---

## Package Structure (Final)

```
src/terralink_elevation/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── elevation_mapping.yaml      # ROS 2 parameters
├── launch/
│   ├── elevation_mapping.launch.py       # Node only
│   └── elevation_mapping_sim.launch.py   # Gazebo UAV + node
├── worlds/
│   ├── gaussian_bump.world
│   └── construction_site.world
├── terralink_elevation/
│   ├── __init__.py
│   ├── parameter.py              # Parameter dataclass
│   ├── elevation_map.py          # Core ElevationMap class
│   ├── elevation_mapping_node.py # ROS 2 node
│   ├── kernels/
│   │   ├── __init__.py
│   │   ├── fusion_kernel.py      # Main fusion + ray tracing
│   │   ├── drift_kernel.py       # Error counting
│   │   └── utils.py              # Shared device functions
│   └── utils/
│       ├── __init__.py
│       ├── coord_transform.py    # Internal ↔ GridMap coords
│       └── gridmap_utils.py      # GridMap message encoding
├── scripts/
│   └── synthetic_pointcloud.py   # Test data generator (for unit tests)
└── test/
    ├── test_parameter.py
    ├── test_elevation_map.py
    ├── test_kernels.py
    └── test_integration.py
```

---

## Dependencies

### System (One-time)
```bash
# CUDA Toolkit 12.x (for CuPy)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update && sudo apt-get install -y cuda-toolkit-12-0

# ROS 2 packages
sudo apt install -y \
    ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
    ros-humble-nav2-costmap-2d ros-humble-ros2-numpy \
    python3-scipy python3-numpy python3-opencv \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Python (via pip)
```bash
pip install cupy-cuda12x==13.6.0
pip install --upgrade transforms3d
pip install simple_parsing  # For parameter YAML loading
```

---

## Success Criteria (Definition of Done)

| Metric | Target |
|--------|--------|
| Map publish rate | ≥ 10 Hz |
| Valid cell coverage (gaussian bump) | ≥ 95% |
| Valid cell coverage (construction site) | ≥ 85% |
| GPU fusion latency | < 10ms for 100k points |
| Map center drift (UAV) | < 5cm over 60s |
| Gaussian bump peak height error | < 2cm |
| Construction site: ramp (15°) detected as traversable | Yes |
| Construction site: 30cm step detected as lethal | Yes |
| Construction site: hole detected as negative obstacle | Yes |

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| CuPy/CUDA version mismatch | Pin versions: `cupy-cuda12x==13.6.0`, `numpy==1.24.2` |
| Axis swap bug in map shifting | Unit test with known shift pattern |
| TF lookup failures (UAV moves fast) | Robust `safe_lookup_transform` with timeout |
| Coordinate convention mismatch | Automated test: internal → GridMap → RViz visual check |
| Gazebo sensor noise unrealistic | Test with both synthetic + Gazebo data |
| UAV pose drift high | Larger `position_noise_thresh`, test drift compensation |

---

## Next Steps After Approval

1. **Create package skeleton** (Step 1)
2. **Implement parameter system** (Step 2)
3. **Build core data structures** (Step 3)
4. **Implement CPU fusion → verify → GPU** (Steps 4-5)
5. **Add ray tracing, shifting, drift** (Steps 6-8)
6. **Traversability + ROS node** (Step 9)
7. **Gazebo UAV + test worlds** (Step 10)

**Estimated Timeline**: 2-3 weeks (part-time), 1 week (full-time)

---

## Questions for Review

Before starting, please confirm:

1. **Package location**: `src/terralink_elevation/` (new, not touching d1/d2/d3)?
2. **GPU requirement**: CuPy with CUDA 12.x confirmed available?
3. **Test approach**: CPU-first verification, then GPU - acceptable?
4. **Traversability**: Analytical only (no CNN) - confirmed?
5. **UAV model**: Use existing `d3/my_bot/description/uav.sdf` + add depth camera?
6. **Gazebo worlds**: Create new `worlds/` in our package, or use d3 worlds?
7. **UAV height**: Fly at ~5-10m altitude for mapping?

**Please verify this updated plan and confirm any adjustments before I begin implementation.**