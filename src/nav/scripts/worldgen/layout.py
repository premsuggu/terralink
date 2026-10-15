"""Small placement helpers shared by the world definitions."""
from __future__ import annotations

import math

CONCRETE = (0.62, 0.62, 0.60)
STEEL = (0.45, 0.47, 0.50)
HAZARD = (0.90, 0.70, 0.10)
FENCE = (0.35, 0.40, 0.45)


def wall(scene, name, x1, y1, x2, y2, thick=0.3, h=2.0, rgb=CONCRETE, z0=0.0, kind="structure"):
    """A straight wall from (x1,y1) to (x2,y2) as one box (any orientation)."""
    length = math.hypot(x2 - x1, y2 - y1)
    scene.add_box(name, (x1 + x2) / 2, (y1 + y2) / 2, length, thick, h, z0=z0,
                  yaw=math.atan2(y2 - y1, x2 - x1), rgb=rgb, kind=kind)


def rect_walls(scene, prefix, x0, y0, x1, y1, thick=0.3, h=2.0, rgb=CONCRETE, gaps=()):
    """Four walls of a rectangle. gaps = [(side, centre, width), ...] with side in
    'N','S','E','W' and `centre` the x (N/S) or y (E/W) coordinate of the opening."""
    sides = {"S": (x0, y0, x1, y0, 0), "N": (x0, y1, x1, y1, 0), "W": (x0, y0, x0, y1, 1), "E": (x1, y0, x1, y1, 1)}
    for side, (ax, ay, bx, by, axis) in sides.items():
        cuts = sorted((c - w / 2, c + w / 2) for s, c, w in gaps if s == side)
        lo, hi = (ax, bx) if axis == 0 else (ay, by)
        pos = lo
        pieces = []
        for g0, g1 in cuts:
            if g0 > pos:
                pieces.append((pos, g0))
            pos = g1
        if pos < hi:
            pieces.append((pos, hi))
        for k, (p, q) in enumerate(pieces):
            if axis == 0:
                wall(scene, f"{prefix}_{side}{k}", p, ay, q, ay, thick, h, rgb)
            else:
                wall(scene, f"{prefix}_{side}{k}", ax, p, ax, q, thick, h, rgb)


def place(scene, pack, models, slug, x, y, yaw=0.0, z=0.0, name=None):
    """Place one pack part (exported as model `models[slug]`) at a pose."""
    scene.add_asset(pack.assets[slug], models[slug], x, y, z, yaw, name=name)


def export_props(pack, models_dir, slugs):
    """Export the named pack parts as Gazebo models and return {slug: model_name}."""
    from .assets import export_model
    return {s: export_model(models_dir, pack, pack.assets[s]) for s in slugs}


def seg_wall(scene, prefix, x1, y1, x2, y2, gaps=(), thick=0.25, h=3.1, rgb=CONCRETE):
    """Axis-aligned wall from (x1,y1) to (x2,y2) with door openings.

    gaps = [(centre, width), ...] where `centre` is the x (for a wall along x)
    or y (for a wall along y) coordinate of each opening.
    """
    along_x = abs(y2 - y1) < 1e-9
    lo, hi = (x1, x2) if along_x else (y1, y2)
    lo, hi = min(lo, hi), max(lo, hi)
    pos, pieces = lo, []
    for c, w in sorted(gaps):
        if c - w / 2 > pos:
            pieces.append((pos, c - w / 2))
        pos = c + w / 2
    if pos < hi:
        pieces.append((pos, hi))
    for k, (p, q) in enumerate(pieces):
        if along_x:
            wall(scene, f"{prefix}_{k}", p, y1, q, y1, thick, h, rgb)
        else:
            wall(scene, f"{prefix}_{k}", x1, p, x1, q, thick, h, rgb)
