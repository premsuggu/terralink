"""Convert between "world" coordinates (meters, the real/simulated position of a
point) and "grid" coordinates (row, col integer indices into the elevation
map's 2D array).

Every function here is written to work on EITHER a single number OR a whole
NumPy array of numbers at once, with no special-casing needed - this is
because NumPy already applies +, -, *, / etc. element-wise to arrays exactly
the same way it does to plain numbers ("vectorization"). That matters a lot
here: step 4 will need to convert an entire point cloud (tens of thousands of
points) into grid indices every time a sensor reading arrives, and doing that
one point at a time in a Python for-loop would be far too slow. Calling these
same functions once with a whole array is both simpler to write AND orders of
magnitude faster.

Grid convention (matches the reference implementations in src/d1 and
src/terralink_elevation, re-derived and re-tested here rather than copied):
  - The grid is a square of `cell_n x cell_n` cells, each `resolution` meters
    across.
  - The grid is CENTERED on `center = (center_x, center_y)` - i.e. the map
    covers the square area from `center - length/2` to `center + length/2` in
    both X and Y, not a square anchored at some corner. This matters because
    the map is meant to stay centered on the UAV as it flies (a later step
    "shifts" the grid to re-center it, rather than growing it forever).
  - `row` increases with world Y, `col` increases with world X. This is a
    convention, not a law of nature - NumPy 2D arrays are naturally indexed as
    array[row, col], and we're choosing row<->Y, col<->X so that printing the
    array or viewing it as an image looks like looking down at the world from
    above with Y increasing "up the page" and X increasing "right".
"""
from __future__ import annotations

import numpy as np


def world_to_grid(x, y, center_x: float, center_y: float, resolution: float, cell_n: int):
    """Convert world-frame (x, y) position(s) in meters to grid (row, col) indices.

    Args:
        x, y: world coordinates in meters. Either plain floats, or NumPy
            arrays of the same shape (e.g. every point's x and every point's
            y from a point cloud).
        center_x, center_y: world-frame position the grid is currently
            centered on (meters).
        resolution: size of one grid cell, in meters.
        cell_n: number of cells along one side of the (square) grid.

    Returns:
        (row, col): integer NumPy arrays (or plain ints, matching whatever
        type x/y were) - grid indices. These are NOT yet checked against the
        grid's actual bounds; a point far outside the map will produce a
        row/col outside [0, cell_n) - always pass the result through
        `in_bounds()` before using it to index into the map array, or you
        risk a silent wraparound / index-out-of-range bug.
    """
    # Shift so the map's own center becomes the origin, then divide by the
    # cell size to turn "meters from center" into "cells from center", then
    # add cell_n/2 so the map's center lands in the middle of the array
    # instead of at index (0, 0).
    col = np.round((np.asarray(x) - center_x) / resolution + cell_n / 2.0)
    row = np.round((np.asarray(y) - center_y) / resolution + cell_n / 2.0)
    # round() rather than floor()/truncation: this maps each cell's *center*
    # to its own index, so a point exactly on a cell boundary lands in the
    # nearer cell instead of always being pulled toward zero.
    return row.astype(np.int64), col.astype(np.int64)


def grid_to_world(row, col, center_x: float, center_y: float, resolution: float, cell_n: int):
    """Inverse of `world_to_grid`: grid (row, col) indices -> world (x, y) in meters.

    Returns the position of the CENTER of that cell (not a corner) - i.e.
    `grid_to_world(*world_to_grid(x, y, ...), ...)` recovers (x, y) only up to
    the map's resolution (it snaps to the nearest cell center), which is
    exactly what the round-trip test in tests/emap/test_elevation_map.py
    checks for.
    """
    x = (np.asarray(col) - cell_n / 2.0) * resolution + center_x
    y = (np.asarray(row) - cell_n / 2.0) * resolution + center_y
    return x, y


def in_bounds(row, col, cell_n: int):
    """True/False (or a boolean array) for whether each (row, col) index
    actually falls inside a `cell_n x cell_n` grid.

    Always call this before indexing the map array with the output of
    `world_to_grid` - points from a real sensor routinely fall outside the
    map (e.g. terrain far past the map's edge), and NumPy will not stop you
    from indexing out of bounds by accident with a negative or wrapped index.
    """
    row = np.asarray(row)
    col = np.asarray(col)
    return (row >= 0) & (row < cell_n) & (col >= 0) & (col < cell_n)
