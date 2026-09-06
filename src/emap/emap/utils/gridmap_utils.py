"""Encode a plain (rows, cols) NumPy array as the `Float32MultiArray` format
`grid_map_msgs/GridMap` actually expects on the wire.

Why this needs its own file instead of just doing `msg.data = array.tolist()`:
`GridMap` doesn't store each layer as a flat list of numbers in the "obvious"
row-by-row order - it uses a specific column-major layout with particular
`MultiArrayDimension` metadata, because that's the exact convention the
`grid_map_ros` C++ library (and therefore RViz's grid map display plugin)
expects when it decodes the message back into a 2D grid. A `GridMap` message
encoded the "obvious" way would still be technically valid (right message
type, right total number of numbers) but would visualize as a transposed or
scrambled version of the real map - a mistake that's easy to make and easy to
miss until you're staring at a garbled RViz display wondering why the numbers
were right in Python but wrong on screen.

Reference (format only, our own code): `src/d1/elevation_mapping_gpu_ros2/
.../elevation_mapping_cupy/gridmap_utils.py` (`encode_layer_to_multiarray`).
"""
from __future__ import annotations

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


def encode_layer_to_multiarray(array: np.ndarray) -> Float32MultiArray:
    """Encode one (rows, cols) layer for use in `GridMap.data`.

    Args:
        array: a 2D array - one of ElevationMap's layers (e.g. from
            `emap.layer("elevation")`).

    Returns:
        A `Float32MultiArray` whose `layout.dim` and `data` together follow
        `grid_map_ros`'s own on-wire convention: the data is flattened
        column-by-column (NumPy's Fortran/`order="F"`, NOT the default
        row-by-row `order="C"`), and the two `MultiArrayDimension` entries
        describe that layout explicitly (`column_index` first, matching the
        outer/slower-varying axis in the flattened data; `row_index` second,
        the inner/faster-varying axis) so a correct decoder on the other end
        (RViz, or our own future map-loading code) can reconstruct the exact
        same 2D array back out.
    """
    arr = np.asarray(array, dtype=np.float32)
    rows, cols = arr.shape

    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.dim = [
        MultiArrayDimension(label="column_index", size=cols, stride=rows * cols),
        MultiArrayDimension(label="row_index", size=rows, stride=rows),
    ]
    # order="F": walk down each column fully before moving to the next
    # column - the "column-major" layout the dimension metadata above
    # describes. This is the one line that most needs to match the
    # dimension metadata exactly; changing one without the other is what
    # silently produces a transposed-looking map.
    msg.data = arr.flatten(order="F").tolist()
    return msg


def decode_layer_from_multiarray(msg: Float32MultiArray) -> np.ndarray:
    """Inverse of `encode_layer_to_multiarray`: reconstruct the (rows, cols)
    NumPy array from one `GridMap.data` entry.

    Reads `cols`/`rows` straight back out of the two `MultiArrayDimension`
    entries (rather than assuming an order) so this stays correct even if
    the encoder's dimension order is ever revisited - only the `order="F"`
    reshape (the exact inverse of that function's `order="F"` flatten) is
    load-bearing here.
    """
    cols = msg.layout.dim[0].size
    rows = msg.layout.dim[1].size
    return np.array(msg.data, dtype=np.float32).reshape((rows, cols), order="F")


def decode_gridmap(msg) -> dict[str, np.ndarray]:
    """Reconstruct every layer of a received `grid_map_msgs/GridMap` message
    back into `{layer_name: (rows, cols) array}` - the consuming side of the
    exact convention `encode_layer_to_multiarray` writes.

    This is the ONE authoritative decoder for that convention, so that any
    package subscribing to `/elevation_map` (this package's own future code,
    or `nav`'s `planner_node.py`) reads it the same way rather than each
    independently re-deriving the row/col/order convention by hand - a real
    risk otherwise: a scratchpad debugging script earlier in this project's
    history had to reverse-engineer this exact layout once already.
    """
    return {name: decode_layer_from_multiarray(layer) for name, layer in zip(msg.layers, msg.data)}
