# Step 3: Core Data Structures - Work Log

**Date**: 2026-08-22  
**Goal**: Enhance core data structures with layer access helpers, coordinate transforms, and GridMap conversion utilities  
**Reference**: `src/d1/elevation_mapping_gpu_ros2/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py`

---

## What Was Done

### 1. Enhanced `elevation_map.py` with Layer Access Helpers

Added layer index constants and getter/setter methods:

```python
# Layer indices (use these instead of magic numbers)
IDX_ELEVATION = 0
IDX_VARIANCE = 1
IDX_IS_VALID = 2
IDX_TRAVERSABILITY = 3
IDX_TIME = 4
IDX_UPPER_BOUND = 5
IDX_IS_UPPER_BOUND = 6

layer_names = [
    "elevation", "variance", "is_valid", 
    "traversability", "time", "upper_bound", "is_upper_bound"
]

# Helper methods:
get_layer(name)          # Get layer by name (view, not copy)
get_elevation()          # Get elevation layer
get_variance()           # Get variance layer
get_validity()           # Get validity layer
get_traversability()     # Get traversability layer
set_layer(name, data)    # Set layer by name
```

### 2. Coordinate Transform Helpers

Added world ↔ grid coordinate conversion with clear documentation:

```python
def world_to_grid(x, y) -> (row, col):
    """World (X forward, Y left) -> Grid (row=Y, col=X)"""

def grid_to_world(row, col) -> (x, y):
    """Grid indices -> World coordinates"""

def is_inside(row, col) -> bool:
    """Bounds checking"""

def get_cell_center_world(row, col) -> (x, y):
    """Get world coordinates of cell center"""

def get_valid_region_slice() -> (slice, slice):
    """Get slice for valid region (excludes 1-cell border)"""
```

**Key Convention**: Internal array uses `row=Y, col=X` (row-major). World uses `X forward, Y left`.

### 3. GridMap Coordinate Conversion

Added conversion between internal (row=Y, col=X) and GridMap column-major convention:

```python
def internal_to_gridmap(arr):
    """Internal (row=Y, col=X) -> GridMap column-major (Row→-X, Col→-Y)"""
    arr = arr.T
    arr = np.flip(arr, axis=0)  # Flip X axis
    arr = np.flip(arr, axis=1)  # Flip Y axis
    return arr

def to_gridmap_layers() -> dict:
    """Extract all layers in GridMap convention for publishing"""
```

**Important**: For even-sized grids, the center shifts by half a cell during transform. The transform is perfectly invertible but the array center shifts by half a cell for even-sized grids.

### 4. Layer Access via Constants

All map access now uses named constants instead of magic numbers:

```python
# Instead of: map[0, row, col]
# Use: map[IDX_ELEVATION, row, col]

# Instead of: map[1, row, col]  
# Use: map[IDX_VARIANCE, row, col]
```

### 5. Comprehensive Unit Tests

Created `tests/elevation_mapping/test_step03_data_structures.py` with 11 tests:

| Test | Purpose |
|------|---------|
| `test_layer_indices` | Verify layer index constants |
| `test_layer_names` | Verify layer name ordering |
| `test_layer_access_helpers` | Test get_elevation, get_variance, etc. |
| `test_world_to_grid` | World → grid coordinate conversion |
| `test_grid_to_world` | Grid → world coordinate conversion |
| `test_round_trip_coordinates` | World → grid → world accuracy |
| `test_is_inside` | Bounds checking |
| `test_valid_region_slice` | Valid region excludes border |
| `test_internal_to_gridmap` | GridMap transform invertibility |
| `test_get_valid_region` | Valid region excludes border |
| `test_get_cell_center_world` | Cell center world coordinates |

---

## Test Results

```
Test 1: Layer indices                    → PASSED
Test 2: Layer names                      → PASSED
Test 3: Layer access helpers             → PASSED
Test 4: World to grid coordinate transform → PASSED
Test 5: Grid to world coordinate transform → PASSED
Test 6: Round-trip world -> grid -> world → PASSED
Test 7: Bounds checking                  → PASSED
Test 8: Valid region slice               → PASSED
Test 9: Internal to GridMap coordinate conversion → PASSED
Test 10: Get valid region data           → PASSED
Test 11: Get cell center world coordinates → PASSED

Results: 11/11 passed
```

---

## Files Modified/Created

| File | Change |
|------|--------|
| `src/terralink_elevation/terralink_elevation/elevation_map.py` | Enhanced with layer helpers, coordinate transforms, GridMap conversion |
| `tests/elevation_mapping/test_step03_data_structures.py` | New test file with 11 tests |

---

## Key Technical Details

### Coordinate Convention Summary

| System | Row Axis | Col Axis | Origin |
|--------|----------|----------|--------|
| **Internal (NumPy)** | Y (down) | X (right) | Center |
| **World** | Y (left) | X (forward) | Map origin |
| **GridMap msg** | -X | -Y | Map origin |

### GridMap Transform Details

The conversion `internal_to_gridmap` performs:
1. **Transpose**: `(H, W) → (W, H)` — row=X, col=Y
2. **Flip axis 0**: Row 0 becomes last row → X direction reversed
3. **Flip axis 1**: Col 0 becomes last col → Y direction reversed

**Result**: Internal `(row=Y, col=X)` → GridMap `(row=-X, col=-Y)`

### Even Grid Center Shift

For even-sized grids (e.g., 22×22), the center is between 4 cells. The GridMap transform shifts the "center" by half a cell. This is mathematically correct — the transform is perfectly invertible but the array center shifts by half a cell. The test verifies invertibility (`np.allclose(arr, arr2)`) rather than center index preservation.

---

## Time Spent

- Layer helpers & coordinate transforms: ~30 min
- GridMap conversion utilities: ~20 min
- Test creation & debugging: ~30 min
- Documentation: ~15 min
**Total**: ~95 min

---

## Notes for Future Steps

- All coordinate transforms are documented and tested
- Layer access uses constants — easy to maintain
- GridMap conversion ready for Step 10 (ROS 2 publishing)
- Coordinate conventions clearly documented in code comments