"""Core elevation-map data structure: a square grid of cells, each storing a
handful of numbers about the terrain at that (x, y) location.

This module is deliberately "dumb" - it only allocates the grid and lets you
read/write named layers and convert between world and grid coordinates. It
does NOT know about point clouds, sensors, ROS, or Gazebo at all (that's
intentional: AGENTS.md's "CPU-first" rule means we get this part fully
correct and unit-tested in isolation before step 4 teaches it to actually
absorb real sensor measurements).

Why "2.5D" and not full 3D: a real 3D map would need to store information at
every (x, y, z) voxel, which is far more memory and compute than an aerial
mapping task needs. Since we only care about "how high is the ground at this
(x, y)", one height value per (x, y) cell is enough - hence "2.5D" (a 2D grid
carrying a height, rather than a full 3D volume). See docs/work-docs/emap/
00_concepts.md for the from-scratch explanation of this and of why a
*variance* layer is tracked alongside the height.
"""
from __future__ import annotations

import numpy as np

from emap.utils.coord_transform import grid_to_world, in_bounds, world_to_grid

# Maps a human-readable layer name to its position in the stacked array below.
# Using named lookups (map.layer("elevation")) instead of a bare magic number
# (map._data[0]) everywhere else in the codebase means a reader never has to
# remember "0 means elevation" - and if we ever reorder/add layers, only this
# one dict needs to change.
LAYER_INDEX = {
    "elevation": 0,      # terrain height at this cell, in meters
    "variance": 1,       # uncertainty (in meters^2) of that height estimate
    "is_valid": 2,       # 1.0 once at least one measurement has landed here, else 0.0
    "traversability": 3,  # 0 (impassable) .. 1 (easy) - real computation is step 7
}
NUM_LAYERS = len(LAYER_INDEX)


class ElevationMap:
    """A square grid centered on `center`, storing the 4 layers in `LAYER_INDEX`.

    All four layers are stored together as ONE NumPy array of shape
    `(NUM_LAYERS, cell_n, cell_n)` rather than four separate arrays. This
    matches the reference implementation in `src/d1` and is deliberate: later
    steps need to shift the whole map (as the UAV flies) or move it to the
    GPU, and doing that once to a single stacked array is simpler and faster
    than repeating the same operation four times on four separate arrays.
    """

    def __init__(self, resolution: float, length: float, initial_variance: float = 10.0):
        """
        Args:
            resolution: size of one grid cell, in meters (e.g. 0.1 = 10 cm cells).
            length: length of one side of the (square) map, in meters.
            initial_variance: variance assigned to every cell before it has
                ever been observed. Large on purpose - it represents "we have
                no idea what the height is here yet", and step 4's Bayesian
                fusion will trust an actual sensor measurement (which has a
                much smaller, realistic variance) over this placeholder as
                soon as one arrives.
        """
        self.resolution = float(resolution)
        self.length = float(length)
        self.initial_variance = float(initial_variance)

        # round() rather than int()/floor(): int()/floor() would always round
        # DOWN, silently making the map very slightly smaller than requested
        # whenever `length / resolution` isn't a whole number (e.g. 10.0/0.3 =
        # 33.33...). round() picks the nearest whole number of cells instead.
        self.cell_n = int(round(self.length / self.resolution))

        # The world-frame point this map is currently centered on. Step 5
        # ("map shifting") will update this as the UAV moves; for this step
        # it simply stays at the origin.
        self.center_x = 0.0
        self.center_y = 0.0

        self._data = np.zeros((NUM_LAYERS, self.cell_n, self.cell_n), dtype=np.float32)
        self.reset()

    def reset(self) -> None:
        """Restore every layer to its default "nothing observed yet" value.

        Split out from `__init__` (rather than inlined there) so tests - and
        later, a real "clear the map and start over" ROS service - can reset
        an already-constructed map without reallocating its arrays.
        """
        self._data[LAYER_INDEX["elevation"]] = 0.0
        self._data[LAYER_INDEX["variance"]] = self.initial_variance
        self._data[LAYER_INDEX["is_valid"]] = 0.0
        # Optimistic default: an unobserved cell is assumed traversable until
        # a real measurement says otherwise (same convention used in src/d1).
        self._data[LAYER_INDEX["traversability"]] = 1.0

    def layer(self, name: str) -> np.ndarray:
        """Return the live 2D array (a view, not a copy) for one named layer.

        Because this returns a *view*, `map.layer("elevation")[:] = 5` really
        does modify the map - this is intentional (it's how step 4's fusion
        code will efficiently update the map in place), but it does mean
        callers shouldn't expect an independent copy.
        """
        return self._data[LAYER_INDEX[name]]

    @property
    def shape(self) -> tuple[int, int]:
        """(rows, cols) of the grid - both equal to `cell_n` since the map is square."""
        return self.cell_n, self.cell_n

    def world_to_grid(self, x, y):
        """This map's own `resolution`/`center`/`cell_n` applied to
        `coord_transform.world_to_grid` - see that function for the full
        explanation of the conversion and why it's vectorized.
        """
        return world_to_grid(x, y, self.center_x, self.center_y, self.resolution, self.cell_n)

    def grid_to_world(self, row, col):
        """Inverse of `world_to_grid` (see `coord_transform.grid_to_world`)."""
        return grid_to_world(row, col, self.center_x, self.center_y, self.resolution, self.cell_n)

    def in_bounds(self, row, col):
        """True/False (or a boolean array) for whether (row, col) is inside this map."""
        return in_bounds(row, col, self.cell_n)
