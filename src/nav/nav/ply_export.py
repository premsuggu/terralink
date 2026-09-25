"""A minimal PLY point-cloud writer, shared by `scripts/export_elevation_map.py`
and `voxel_map_node.py`'s own export-on-request handler (see that node's
`export_voxel_map_request` subscription) - both need to turn a plain (N, 3)
array of world-frame points (optionally colored) into a file any standard 3D
viewer (MeshLab, CloudCompare, Blender, online glTF/PLY viewers, etc.) can
open directly, with zero extra dependencies (no `open3d`/`trimesh` installed
in this environment - confirmed live before writing this).

PLY (Polygon File Format / Stanford Triangle Format) was picked specifically
because it's one of the few 3D formats that supports a plain, colored POINT
CLOUD with no faces/mesh topology required - both the elevation map (a grid
of points, one per cell) and the voxel map (a scattered set of occupied-voxel
centers) are naturally point clouds, not meshes, so forcing either into an
OBJ/STL (mesh-only formats) would need an arbitrary triangulation step this
data doesn't actually have. The ASCII variant (not binary) is used
deliberately - slightly larger files, but trivially human-readable/debuggable
(you can literally look at the header and first few lines to sanity-check
output), and the point counts here (a few thousand to a few tens of
thousands) are nowhere near large enough for ASCII's size overhead to matter.
"""
from __future__ import annotations

import numpy as np

from emap.traversability import DIFFICULT, EASY, LETHAL


def write_ply_points(filepath: str, points_xyz: np.ndarray, colors_rgb: np.ndarray | None = None) -> None:
    """Write `points_xyz` (N, 3) as an ASCII PLY point cloud to `filepath`.

    Args:
        points_xyz: (N, 3) world-frame x, y, z coordinates, meters.
        colors_rgb: optional (N, 3) uint8 (or anything castable to uint8)
            RGB colors, one per point, 0-255 per channel. If omitted, every
            point is written as a flat mid-gray (200, 200, 200) - readable
            without color, just less informative than a real color scheme
            (see `elevation_to_rgb`/`TRAVERSABILITY_COLORS` below for how
            the two callers of this module actually color their points).

    Raises:
        ValueError: if `colors_rgb`'s length doesn't match `points_xyz`'s -
            a silently-mismatched color array would write a corrupted-looking
            cloud (wrong colors on wrong points) rather than failing loudly,
            so this is checked explicitly rather than left to zip() to
            silently truncate the longer array.
    """
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    n_points = points_xyz.shape[0]

    if colors_rgb is None:
        colors_rgb = np.full((n_points, 3), 200, dtype=np.uint8)
    else:
        colors_rgb = np.asarray(colors_rgb)
        if colors_rgb.shape[0] != n_points:
            raise ValueError(
                f"colors_rgb has {colors_rgb.shape[0]} rows but points_xyz has {n_points} - must match"
            )
        colors_rgb = colors_rgb.astype(np.uint8)

    with open(filepath, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n_points}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(points_xyz, colors_rgb):
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")


# Traversability class -> RGB color. green/yellow/red is the same
# "safe/caution/lethal" convention a Nav2 costmap's own RViz color scheme
# already uses, so this looks familiar to anyone who has looked at a costmap
# in RViz before. Gray marks a cell `is_valid == False` (unobserved) -
# distinct from LETHAL, since "haven't looked yet" and "confirmed
# impassable" are a real, different-meaning pair throughout this project
# (see e.g. nav.walkability's own is_valid handling).
_UNKNOWN_RGB = (120, 120, 120)


def traversability_to_rgb(traversability: np.ndarray, is_valid: np.ndarray) -> np.ndarray:
    """Per-cell RGB from emap.traversability's own {LETHAL, DIFFICULT, EASY}
    numeric scores (imported directly from that module, not re-guessed here,
    so this can never silently drift from the real thresholds) plus
    `is_valid` for the "unobserved" gray case. `traversability`/`is_valid`
    must be same-shape arrays (as decode_gridmap already returns them).
    """
    traversability = np.asarray(traversability)
    is_valid = np.asarray(is_valid, dtype=bool)
    n = traversability.size
    flat_trav = traversability.reshape(-1)
    flat_valid = is_valid.reshape(-1)

    colors = np.empty((n, 3), dtype=np.uint8)
    colors[:] = _UNKNOWN_RGB
    # A midpoint check for each class rather than exact float equality -
    # this data has been through GridMap encode/decode (float32 round-trip),
    # so an exact `== LETHAL` could miss on tiny floating-point noise.
    lethal_cut = (LETHAL + DIFFICULT) / 2.0
    easy_cut = (DIFFICULT + EASY) / 2.0
    observed = flat_valid
    colors[observed & (flat_trav <= lethal_cut)] = (220, 40, 40)     # red - LETHAL
    colors[observed & (flat_trav > lethal_cut) & (flat_trav <= easy_cut)] = (230, 200, 40)  # yellow - DIFFICULT
    colors[observed & (flat_trav > easy_cut)] = (60, 200, 60)        # green - EASY
    return colors


def elevation_to_rgb(elevation: np.ndarray, z_min: float | None = None, z_max: float | None = None) -> np.ndarray:
    """Map a height value per point to a blue (low) -> red (high) gradient -
    used by `scripts/export_elevation_map.py` as an alternative color scheme
    to `TRAVERSABILITY_COLORS` (pass `--color height` there), useful for
    visually reading off relative elevation at a glance (e.g. spotting the
    tunnel's roof plateau) rather than only walkability.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    if z_min is None:
        z_min = float(np.min(elevation)) if elevation.size else 0.0
    if z_max is None:
        z_max = float(np.max(elevation)) if elevation.size else 1.0
    span = max(z_max - z_min, 1e-9)  # avoid a divide-by-zero on a perfectly flat map
    t = np.clip((elevation - z_min) / span, 0.0, 1.0)
    r = (t * 255).astype(np.uint8)
    b = ((1.0 - t) * 255).astype(np.uint8)
    g = np.full_like(r, 40)
    return np.stack([r, g, b], axis=1)
