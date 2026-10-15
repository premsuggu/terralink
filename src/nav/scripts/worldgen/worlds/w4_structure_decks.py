"""W4 - "Structure with decks, corridors and dead ends" (50 x 50 m).

A partly built concrete structure (32 x 24 m) with a floor-slab deck over the
whole footprint at z = 3.1-3.5 m, plus a scaffold-covered walkway outside.
Anything a deck hides is the occlusion problem of the whole project: seen
from above (UAV) it is one solid block, but a ground robot can drive beneath.

  * Ground-truth map (default): surfaces whose lowest point is >= 3 m are
    "overhead", so the rooms and corridors under the deck are visible.
  * `<world>_uavview.npz`: ceiling = infinity, i.e. what the UAV really sees -
    the deck and walkway roof as solid walls (all corridors hidden).

Floor plan (4 x 3 rooms, doors 1.2 / 1.0 / 0.7 / 0.5 m wide, several routes of different width):
  entrance (west, y=-6)  ->  ...  ->  exit (east, y=+6), plus
  a decoy south door, dead-end rooms, a sealed room (no door), a 0.5 m
  doorway (impassable), a 0.7 m doorway (marginal), a half-blocked door.
Scaffold walkway south of the building: posts every 1.5 m (1.25 m clear).
"""
from __future__ import annotations

import math

from ..layout import CONCRETE, FENCE, HAZARD, STEEL, export_props, place, rect_walls, seg_wall, wall
from ..scene import Scene

NAME = "w4_structure_decks"
TITLE = "W4  Structure with decks, corridors & dead ends"
CEILING = 3.0
UAV_CEILING = 5.0   # UAV-view variant: just above the live patrol altitude (3.0-4.5 m in tunnel_demo/nav_sim), so decks are seen, jibs are not

MARKERS = [
    ("S", (-21.5, -6.0), "s", "#2ca02c"),
    ("A", (13.0, 8.0), "*", "#d62728"),      # far room, only via the true route
    ("B", (-12.0, 8.0), "*", "#d62728"),     # dead-end room
    ("C", (4.0, 8.0), "*", "#d62728"),       # sealed room (infeasible)
    ("D", (0.0, -20.0), "*", "#d62728"),     # under the scaffold walkway
    ("E", (22.0, 6.0), "*", "#d62728"),      # outside the east exit
    ("F", (12.0, -8.0), "*", "#d62728"),     # SE room: via the outside 1.2 m door OR the marginal 0.7 m doorway
]
PROPS = ["small_truck", "barrel", "box_stack_brown", "planks_blue", "box_white", "box_black"]
DECK_Z0, DECK_T = 3.1, 0.4


def build(pack, models_dir) -> Scene:
    sc = Scene(NAME, bounds=(-25.0, 25.0, -25.0, 25.0), ground_rgb=(0.48, 0.47, 0.44), ground_size=300.0)
    m = export_props(pack, models_dir, PROPS)
    rect_walls(sc, "fence", -24.5, -24.5, 24.5, 24.5, thick=0.4, h=2.0, rgb=FENCE)

    # --- exterior shell: west entrance (y=-6), east exit (y=+6), decoy south door (x=10) ---
    seg_wall(sc, "ext_W", -16, -12, -16, 12, gaps=[(-6.0, 2.0)])
    seg_wall(sc, "ext_E", 16, -12, 16, 12, gaps=[(6.0, 2.0)])
    seg_wall(sc, "ext_S", -16, -12, 16, -12, gaps=[(10.0, 1.2)])
    seg_wall(sc, "ext_N", -16, 12, 16, 12)
    # --- interior partitions ---
    seg_wall(sc, "V1", -8, -12, -8, 12, gaps=[(-6.0, 1.2), (8.0, 1.0)])
    seg_wall(sc, "V2", 0, -12, 0, 12, gaps=[(2.0, 1.0)])
    seg_wall(sc, "V3", 8, -12, 8, 12, gaps=[(-2.0, 1.2), (9.0, 0.5)])
    seg_wall(sc, "H1", -16, -4, 16, -4, gaps=[(-12.0, 1.2), (4.0, 1.0), (12.0, 0.7)])
    seg_wall(sc, "H2", -16, 4, 16, 4, gaps=[(-4.0, 1.2), (12.0, 1.2)])
    # --- deck slab over the whole footprint (+ columns inside the big gaps are the walls) ---
    sc.add_box("deck", 0.0, 0.0, 32.6, 24.6, DECK_T, z0=DECK_Z0, rgb=(0.72, 0.72, 0.70), kind="overhead")
    # --- half-blocked door: pallet stack leaves ~0.5 m of the 1.2 m door at H1/x=-12 ---
    place(sc, pack, m, "box_stack_brown", -12.0, -4.0, yaw=math.pi / 2, name="door_block")
    # --- scaffold-covered walkway south of the building: two post lines + roof planks ---
    for i in range(27):
        px = -20.0 + 1.5 * i
        for k, py in enumerate((-18.8, -21.2)):
            sc.add_box(f"post_{i}_{k}", px, py, 0.15, 0.15, 3.1, rgb=HAZARD)
    sc.add_box("walk_roof", 0.0, -20.0, 40.0, 3.4, 0.12, z0=3.1, rgb=(0.55, 0.50, 0.40), kind="overhead")
    # --- clutter outside ---
    place(sc, pack, m, "small_truck", 10.0, -15.0, yaw=math.pi / 2, name="truck_a")
    for i in range(4):
        place(sc, pack, m, "barrel", 19.0, -6.0 + 1.1 * i, name=f"barrel_{i}")
    place(sc, pack, m, "planks_blue", -21.0, 14.0, yaw=0.4, name="planks_a")
    place(sc, pack, m, "box_white", 21.5, 14.0, name="box_a")
    place(sc, pack, m, "box_black", 21.5, 15.2, name="box_b")
    return sc
