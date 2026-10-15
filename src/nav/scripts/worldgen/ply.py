"""PLY export of a world's planner map for the project's own 3D viewer
(`media/3d_maps/viewer.html`, which opens ASCII point-cloud PLY files). The world itself is
shown as real meshes in HTML by `html.py` (`media/worlds/`).

The file:

* `write_elevation_ply` - the planner's 2.5D map: one point per grid cell at the
  cell's elevation, coloured by the PROJECT-WIDE traversability convention of
  `nav.ply_export.traversability_to_rgb` (green easy / yellow difficult / red
  lethal / grey unobserved). The same function and colours are used by
  `scripts/export_elevation_map.py`, so every world and every live map share one
  colour code, and the viewer's legend applies to all of them.

Large grids are block-reduced for display only (max height, worst traversability per
block); the saved `.npz` map is never changed.
"""
from __future__ import annotations

import numpy as np

from nav.ply_export import traversability_to_rgb, write_ply_points


def write_elevation_ply(layers, cx, cy, res, path, max_cells=600) -> int:
    elev, trav, valid = layers["elevation"], layers["traversability"], layers["is_valid"]
    ny, nx = elev.shape
    f = max(1, int(np.ceil(max(ny, nx) / max_cells)))
    if f > 1:
        ny2, nx2 = ny // f, nx // f
        blk = lambda a, op: op(a[:ny2 * f, :nx2 * f].reshape(ny2, f, nx2, f), axis=(1, 3))     # noqa: E731
        elev, trav, valid = blk(elev, np.max), blk(trav, np.min), blk(valid, np.min)
        xs = cx + (np.arange(nx2) * f + (f - 1) / 2 - nx / 2) * res        # centre of each block of f cells
        ys = cy + (np.arange(ny2) * f + (f - 1) / 2 - ny / 2) * res
    else:
        xs = cx + (np.arange(nx) - nx / 2) * res                           # nav snapshot cell-centre convention
        ys = cy + (np.arange(ny) - ny / 2) * res
    X, Y = np.meshgrid(xs, ys)
    pts = np.c_[X.ravel(), Y.ravel(), elev.ravel()]
    write_ply_points(str(path), pts, traversability_to_rgb(trav, valid), [f"point_spacing {res * f:.3f}"])
    return len(pts)
