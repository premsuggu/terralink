"""W3 - "Excavation and slopes" (60 x 60 m terrain).

A real-looking earthworks site, one terrain mesh (visual + collision + map
all from the same triangles). Slopes are chosen against the planner's own
thresholds (max_slope 0.35 = lethal, > 0.175 = difficult):

  * Excavation pit 3 m deep, 24 x 16 m at the rim. East side: a 12 m ramp
    (slope 0.25, difficult but drivable). South side: 6 m wall (0.50, lethal).
    North/west: near-vertical (lethal). The ONLY way to the floor is the ramp.
  * Three spoil mounds: gentle (max slope ~0.16, easy), medium (~0.28,
    difficult), steep (~0.47, lethal ring - its flat top is an unreachable
    island: a goal there must give "no path").
  * Rubble field (NW): random +-9 cm bumps on a 0.5 m lattice: mostly
    difficult, a few lethal cells - "cross it or detour" cost trade-off.
  * Berm along the south edge with ONE saddle: a long 1.3 m ridge, lethal
    flanks, a gentle crossing at x = 6.
  * A few parked trucks/barrels on the flat ground.
"""
from __future__ import annotations

import math

import numpy as np

from ..layout import export_props, place
from ..scene import Scene

NAME = "w3_excavation_slopes"
TITLE = "W3  Excavation & slopes"
CEILING = 3.0
UAV_CEILING = 5.0   # UAV-view variant: just above the live patrol altitude (3.0-4.5 m in tunnel_demo/nav_sim), so decks are seen, jibs are not

MARKERS = [
    ("S", (4.0, -6.0), "s", "#2ca02c"),
    ("A", (-16.5, -6.2), "*", "#d62728"),     # pit floor (via the ramp only)
    ("B", (20.0, 14.0), "*", "#d62728"),      # top of the steep mound (unreachable)
    ("C", (-18.0, 18.0), "*", "#d62728"),     # across the rubble field
    ("D", (-4.0, -28.0), "*", "#d62728"),     # beyond the berm (saddle crossing)
    ("E", (4.0, 18.0), "*", "#d62728"),       # gentle mound top
    ("F", (20.0, -12.0), "*", "#d62728"),     # medium mound top (difficult slope)
]

RES = 0.5          # terrain mesh lattice [m]
PROPS = ["dumper_truck_green", "concrete_truck_red", "small_truck", "barrel", "planks_yellow", "box_stack_brown"]


def _smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def heightfield(X, Y, rng):
    Z = np.zeros_like(X)
    # --- excavation pit: depth profile = 3 * min over sides of clip(inward distance / wall width) ---
    X0, X1, Y0, Y1 = -24.0, 0.0, -16.0, 0.0
    W_E, W_W, W_S, W_N = 12.0, 2.0, 6.0, 1.5          # wall widths -> slopes 0.25 / 1.5 / 0.5 / 2.0
    inside = (X >= X0) & (X <= X1) & (Y >= Y0) & (Y <= Y1)
    d = np.minimum.reduce([np.clip((X1 - X) / W_E, 0, 1), np.clip((X - X0) / W_W, 0, 1),
                           np.clip((Y - Y0) / W_S, 0, 1), np.clip((Y1 - Y) / W_N, 0, 1)])
    Z += np.where(inside, -3.0 * d, 0.0)
    # --- spoil mounds: h * 0.5 * (1 + cos(pi r / R)) ; max slope = pi h / 2R ---
    for (mx, my, R, h) in [(4.0, 18.0, 9.0, 1.0),      # gentle  ~0.17
                           (20.0, -12.0, 9.0, 1.6),    # medium  ~0.28
                           (20.0, 14.0, 8.0, 2.4)]:    # steep   ~0.47
        r = np.hypot(X - mx, Y - my)
        Z += np.where(r < R, h * 0.5 * (1 + np.cos(np.pi * r / R)), 0.0)
    # --- rubble field (NW) ---
    region = (X > -28) & (X < -8) & (Y > 8) & (Y < 26)
    edge = np.minimum.reduce([_smooth((X + 28) / 2), _smooth((-8 - X) / 2), _smooth((Y - 8) / 2), _smooth((26 - Y) / 2)])
    Z += np.where(region, rng.uniform(-0.09, 0.09, X.shape) * edge, 0.0)
    # --- berm with one saddle (south side, ridge at y = -22, flank slope ~0.6) ---
    prof = np.where(np.abs(Y + 22) < 3.5, 1.3 * np.clip(1 - np.abs(Y + 22) / 3.5, 0, 1) * 1.6, 0.0)
    prof = np.clip(prof, 0, 1.3)
    saddle = 1.0 - 0.85 * np.exp(-((X - 6.0) / 3.0) ** 2)      # dips to ~0.15 of full height near x = 6
    Z += prof * saddle * (np.abs(X) < 29)
    return Z


def texture(Z, rng, n=768):
    """RGB texture: grass / soil / gravel / mud by height and slope, with baked hill-shading.

    Computed on a finer lattice than the mesh (bilinear upsample) so slope
    boundaries are smooth, and shaded by a fixed sun direction so the relief
    (pit, ramps, mounds) is readable in a flat top-down render.
    """
    from scipy.ndimage import zoom

    Zf = zoom(Z, n / Z.shape[0], order=1)
    step = 60.0 / n
    gy, gx = np.gradient(Zf, step)
    slope = np.hypot(gx, gy)
    shade = np.clip(0.80 + 0.55 * (-gx * 0.6 + gy * 0.8) / np.maximum(1.0, slope * 2.0) * np.minimum(slope * 3, 1.0), 0.45, 1.15)
    base = np.array([0.52, 0.45, 0.34])                        # packed soil
    grass = np.array([0.38, 0.47, 0.24])
    gravel = np.array([0.58, 0.57, 0.54])
    mud = np.array([0.36, 0.29, 0.21])
    col = np.broadcast_to(base, Zf.shape + (3,)).copy()
    col = np.where(((Zf > 0.15) & (slope < 0.2))[..., None], grass, col)
    col = np.where((slope > 0.2)[..., None], gravel, col)
    col = np.where((Zf < -2.5)[..., None], mud, col)
    col = col * shade[..., None] * (0.94 + 0.12 * rng.random(Zf.shape + (1,)))
    return (np.clip(col, 0, 1) * 255).astype(np.uint8)


def build(pack, models_dir) -> Scene:
    rng = np.random.default_rng(3)
    # ground_z below the deepest terrain (pit floor -3 m), else the plane would cover and block the pit.
    sc = Scene(NAME, bounds=(-30.0, 30.0, -30.0, 30.0), ground_rgb=(0.50, 0.45, 0.34), ground_size=300.0, ground_z=-3.5)
    n = int(round(60.0 / RES)) + 1
    xs = -30.0 + RES * np.arange(n)
    X, Y = np.meshgrid(xs, xs)
    Z = heightfield(X, Y, rng)
    sc.add_terrain(models_dir, NAME + "_terrain", Z, RES, -30.0, -30.0, texture(Z, rng))
    m = export_props(pack, models_dir, PROPS)
    place(sc, pack, m, "dumper_truck_green", 10, -4, yaw=0.5, name="truck_a")
    place(sc, pack, m, "concrete_truck_red", -6, 16, yaw=-0.9, name="truck_b")
    place(sc, pack, m, "small_truck", 26, 0, yaw=1.6, name="truck_c")
    for i in range(5):
        place(sc, pack, m, "barrel", -2 + 1.3 * i, -14 + 0.4 * (i % 2), name=f"barrel_{i}")
    place(sc, pack, m, "planks_yellow", 12, 26, yaw=0.3, name="planks_a")
    place(sc, pack, m, "box_stack_brown", -3, -22, yaw=0.0, name="boxes_a")
    return sc
