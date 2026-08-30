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
        self._reset_region(slice(None), slice(None))

    def _reset_region(self, row_slice, col_slice) -> None:
        """Set the "never observed" defaults on a rectangular sub-region of
        every layer at once - `self._data[:, row_slice, col_slice]`.

        Pulled out as its own method (rather than writing the same 4 lines
        twice) because TWO different situations need to apply exactly the
        same defaults to exactly the same layers: `reset()` above applies
        them to the WHOLE map, and `move_to()` below applies them only to
        the thin strip of cells a shift just exposed. Keeping one definition
        of "what does an unobserved cell look like" means those two code
        paths can never silently drift apart from each other.
        """
        self._data[LAYER_INDEX["elevation"], row_slice, col_slice] = 0.0
        self._data[LAYER_INDEX["variance"], row_slice, col_slice] = self.initial_variance
        self._data[LAYER_INDEX["is_valid"], row_slice, col_slice] = 0.0
        # Optimistic default: an unobserved cell is assumed traversable until
        # a real measurement says otherwise (same convention used in src/d1).
        self._data[LAYER_INDEX["traversability"], row_slice, col_slice] = 1.0

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

    def move_to(self, new_center_x: float, new_center_y: float) -> None:
        """Re-center the map on a new world-frame point (e.g. the UAV's
        current position), carrying over whatever cells are still within
        view and marking the newly-revealed edge as "never observed".

        The map is a FIXED SIZE, centered grid (see `00_concepts.md`), not
        one that grows forever as the UAV flies further from where it
        started - so "the UAV moved" has to mean "shift which patch of the
        world this array currently represents", not "make the array bigger".
        See docs/work-docs/emap/step05_map_shifting.md for the full
        from-scratch explanation and a worked numeric example; the comments
        below cover the two things most likely to go wrong when re-deriving
        this kind of code from scratch.
        """
        delta_x = new_center_x - self.center_x
        delta_y = new_center_y - self.center_y

        # Snap the requested move to a WHOLE number of cells before doing
        # anything else, and only ever move center_x/center_y by that
        # snapped amount (never by the raw, continuous delta_x/delta_y).
        # Without this, the map's center would drift by a fraction of a
        # cell on every single update, and after enough updates that drift
        # would break world_to_grid's exact round-trip guarantee from step 3
        # (every cell's boundaries are only well-defined relative to a fixed
        # lattice - if the center itself isn't aligned to that lattice
        # anymore, "which cell is this point in" quietly stops being exact).
        delta_col = int(round(delta_x / self.resolution))
        delta_row = int(round(delta_y / self.resolution))
        self.center_x += delta_col * self.resolution
        self.center_y += delta_row * self.resolution

        if delta_row == 0 and delta_col == 0:
            return  # didn't move by even one whole cell - nothing to do

        if abs(delta_row) >= self.cell_n or abs(delta_col) >= self.cell_n:
            # The UAV jumped further than the map is wide in one update -
            # NONE of the old data is still within view. This matters
            # because the roll-and-blank-the-edge-strip approach below
            # relies on Python slicing (e.g. `array[:shift]`) to blank the
            # newly-exposed strip, and slicing with a shift bigger than the
            # array's own length does NOT blank "everything" - it silently
            # produces an empty, no-op slice instead. Rather than have that
            # edge case corrupt the map, detect it explicitly and just start
            # over, which is the objectively correct answer anyway (there is
            # nothing left to preserve).
            self.reset()
            return

        # --- Move the array's CONTENT to match the new center ---
        # If the UAV moved +delta_col cells in +x, the ground that used to
        # be delta_col cells ahead of the old center is now directly under
        # the new center - so the data has to slide toward the center by
        # that same amount, i.e. the array content shifts by -delta_col
        # (moving +5 means content shifts by -5). Same reasoning for rows/y.
        #
        # THE PITFALL THIS STEP EXISTS TO CATCH: the array's axes are
        # (layers, rows, cols) with row<->Y and col<->X (00_concepts.md
        # Section 4). `np.roll`'s `axis=(1, 2)` therefore expects its shift
        # amounts in (row_shift, col_shift) = (y_shift, x_shift) order - but
        # we just computed (delta_row, delta_col) which IS already in that
        # order, so passing them through unswapped is correct here. The bug
        # this comment is warning against is accidentally writing
        # `(delta_col, delta_row)` instead (e.g. by habit, if delta_x/y were
        # computed first and used directly) - that would silently swap which
        # physical direction "forward" shifts the map in, and the tests in
        # tests/emap/test_elevation_map.py (pure-x-shift vs. pure-y-shift)
        # exist specifically to catch that exact mistake.
        row_shift, col_shift = -delta_row, -delta_col
        self._data = np.roll(self._data, (row_shift, col_shift), axis=(1, 2))

        # --- Blank whatever `roll` just wrapped in from the opposite edge ---
        # `np.roll` is circular: it doesn't leave the vacated edge empty, it
        # copies in stale data from the far side of the array. That wrapped-
        # in strip is not real information about the world and must be
        # overwritten with "never observed" defaults (_reset_region, shared
        # with `reset()` - see its docstring for why that sharing matters).
        #
        # IMPORTANT: which strip is "new" depends on the sign of row_shift/
        # col_shift (the values actually passed to `roll` above) - NOT on
        # the sign of delta_row/delta_col directly. They're related by a
        # minus sign, so this is an easy detail to get backwards: e.g. the
        # UAV moving in +x (delta_col > 0, "forward") makes col_shift
        # NEGATIVE, and a negative roll shift wraps stale data into the
        # HIGH-index end of the array - so the strip to blank is
        # `array[..., col_shift:]` (the tail), not the head. Verified against
        # np.roll's actual behavior by hand before being written down here -
        # see step05_map_shifting.md for that worked-through example.
        #
        # Two independent strips cover every newly-exposed cell, including
        # the corner where both a row-shift and a col-shift happened at
        # once (resetting that corner twice is harmless - it's the same
        # constant defaults either time):
        if row_shift > 0:
            self._reset_region(slice(0, row_shift), slice(None))
        elif row_shift < 0:
            self._reset_region(slice(row_shift, None), slice(None))
        if col_shift > 0:
            self._reset_region(slice(None), slice(0, col_shift))
        elif col_shift < 0:
            self._reset_region(slice(None), slice(col_shift, None))
