"""Independent feasibility check of a world's start/goal markers.

This is NOT the planner: it only asks "is there any route at all for a robot
of radius r?" on the ground-truth grid, so we can tell a world-design error
(goal accidentally walled in) from a planner failure later. Method: lethal
cells are obstacles (difficult cells stay passable), a Euclidean distance
transform gives each free cell's clearance, and cells with clearance >= r are
flood-filled from the start. A goal is "reachable at r" when its own cell is
free at r and in the start's component.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

LETHAL_BELOW = 0.15
RADII = (0.15, 0.25, 0.35, 0.45)


def to_cell(x, y, n_rows, n_cols, res, cx, cy):
    return int(round((y - cy) / res + n_rows / 2)), int(round((x - cx) / res + n_cols / 2))


def reachability(trav, res, cx, cy, start_xy, goals: dict, radii=RADII):
    """Return {label: {radius: bool}} - goal reachable from start for a robot of that radius."""
    n_rows, n_cols = trav.shape
    free = trav >= LETHAL_BELOW
    clearance = ndimage.distance_transform_edt(free) * res
    out = {k: {} for k in goals}
    sr, sc = to_cell(*start_xy, n_rows, n_cols, res, cx, cy)
    for r in radii:
        ok = clearance >= r
        lab, _ = ndimage.label(ok)
        s_lab = lab[sr, sc]
        for k, (gx, gy) in goals.items():
            gr, gc = to_cell(gx, gy, n_rows, n_cols, res, cx, cy)
            out[k][r] = bool(s_lab != 0 and lab[gr, gc] == s_lab)
    return out


def format_table(result, radii=RADII) -> str:
    head = "goal  " + "  ".join(f"r={r:.2f}" for r in radii)
    rows = [head] + [f"{k:<5} " + "  ".join(f"{'yes' if v[r] else ' - ':^6}" for r in radii) for k, v in result.items()]
    return "\n".join(rows)
