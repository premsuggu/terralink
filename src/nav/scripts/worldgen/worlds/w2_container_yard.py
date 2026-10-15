"""W2 - "Container yard and narrow passages" (60 x 60 m).

A fenced material yard built from the pack's containers, trucks and material
piles plus concrete barriers. Every feature is a deliberate planner test:

  * Container field (north): four parallel lanes of DECREASING width
    2.5 / 1.3 / 0.8 / 0.5 m, with 1.6 m cross-gaps between containers.
    Our UGV is ~0.5 m wide, so the last lane is impassable and the 0.8 m one is
    marginal - tests inflation / clearance / "squeeze or detour".
  * Chicane (south-west): staggered barriers leaving 1.2 m gaps.
  * Dead-end alley (south-centre): a 2.5 m lane closed at its far end.
  * Sealed pocket (south-east): a walled 6 x 6 m room with NO opening
    (an infeasible goal: A* must say "no path", not gamble).
  * Near-blocked wall (east): a 0.35 m gap (impassable) next to a 0.9 m gap.
  * Parked trucks, barrel rings and material piles as clutter.
"""
from __future__ import annotations

import math

from ..layout import CONCRETE, FENCE, HAZARD, export_props, place, rect_walls, wall
from ..scene import Scene

NAME = "w2_container_yard"
TITLE = "W2  Container yard & narrow passages"
CEILING = 3.0
UAV_CEILING = 5.0   # UAV-view variant: just above the live patrol altitude (3.0-4.5 m in tunnel_demo/nav_sim), so decks are seen, jibs are not

# (label, (x, y), marker, colour) - drawn on the ground-truth image only.
MARKERS = [
    ("S", (-3.0, -26.0), "s", "#2ca02c"),
    ("A", (24.0, -20.0 + 4.0), "*", "#d62728"),       # open yard
    ("B", (-3.0, -3.0), "*", "#d62728"),              # through the chicane
    ("C", (-12.0, 13.8), "*", "#d62728"),             # inside the 0.8 m lane
    ("G", (-12.0, 10.15), "*", "#d62728"),            # inside the 0.5 m lane (too narrow)
    ("D", (5.0, -24.0), "*", "#d62728"),              # dead-end alley floor
    ("E", (19.0, -21.0), "*", "#d62728"),             # sealed pocket (infeasible)
    ("F", (20.0, -15.0), "*", "#d62728"),             # beyond the near-blocked wall
]

CONTAINERS = ["cargo_container_white_boxes", "cargo_container_blue_boxes"]
PROPS = CONTAINERS + ["dumper_truck_red", "concrete_truck_green", "small_truck", "barrel", "planks_wood_v1",
                      "planks_blue", "planks_yellow", "box_stack_brown", "box_white", "box_black", "dixie",
                      "office_container_single_green", "office_container_single_red"]


def build(pack, models_dir) -> Scene:
    sc = Scene(NAME, bounds=(-30.0, 30.0, -30.0, 30.0), ground_rgb=(0.50, 0.50, 0.46), ground_size=300.0)
    m = export_props(pack, models_dir, PROPS)

    # --- perimeter fence with a 4 m gate in the south wall (x = -14) --------------------
    rect_walls(sc, "fence", -29, -29, 29, 29, thick=0.4, h=2.2, rgb=FENCE, gaps=[("S", -14.0, 4.0)])

    # --- north container field: 4 lanes, widths 2.5 / 1.3 / 0.8 / 0.5 ----------
    lanes = [2.5, 1.3, 0.8, 0.5]
    y = 25.5
    row_y = [y]
    for w in lanes:
        y -= 3.0 + w
        row_y.append(y)
    for r, ry in enumerate(row_y):
        for k, cx in enumerate([-22, -12, -2, 11, 21]):          # two groups, 4.6 m aisle between them
            slug = CONTAINERS[(r + k) % 2]
            place(sc, pack, m, slug, cx, ry, yaw=math.pi / 2, name=f"cf_{r}_{k}")

    # --- south-west chicane: corridor y in [-6, 0], barriers leave 1.2 m gaps ---
    wall(sc, "chi_N", -27, 0.0, -5, 0.0, 0.4, 1.8)
    wall(sc, "chi_S", -27, -6.0, -5, -6.0, 0.4, 1.8)
    for k, bx in enumerate([-24, -19, -14, -9]):
        if k % 2 == 0:
            wall(sc, f"chi_b{k}", bx, 0.0, bx, -4.8, 0.6, 1.0, rgb=HAZARD)      # hangs from the north wall
        else:
            wall(sc, f"chi_b{k}", bx, -6.0, bx, -1.2, 0.6, 1.0, rgb=HAZARD)     # rises from the south wall

    # --- dead-end alley: two container columns, closed at the far (south) end ---
    for k, cy in enumerate([-8.2, -16.6]):
        place(sc, pack, m, CONTAINERS[k % 2], 1.5, cy, yaw=0, name=f"alley_L{k}")
        place(sc, pack, m, CONTAINERS[(k + 1) % 2], 8.5, cy, yaw=0, name=f"alley_R{k}")
    place(sc, pack, m, CONTAINERS[0], 5.0, -26.5, yaw=math.pi / 2, name="alley_end")   # dead end

    # --- sealed pocket (infeasible goal) -------------------------------------
    rect_walls(sc, "pocket", 16, -24, 22, -18, thick=0.4, h=2.2)

    # --- near-blocked wall across the east side: 0.35 m and 0.9 m gaps ---------
    wall(sc, "nb_0", 12, -10, 19.825, -10, 0.4, 2.0)
    wall(sc, "nb_1", 20.175, -10, 25.55, -10, 0.4, 2.0)
    wall(sc, "nb_2", 26.45, -10, 29, -10, 0.4, 2.0)

    # --- clutter: parked trucks, barrel rings, material piles ------------------
    place(sc, pack, m, "dumper_truck_red", -20, -16, yaw=0.45, name="truck_a")
    place(sc, pack, m, "concrete_truck_green", -8, -21, yaw=-0.35, name="truck_b")
    place(sc, pack, m, "small_truck", 24, -4, yaw=1.9, name="truck_c")
    for i in range(6):
        a = 2 * math.pi * i / 6
        place(sc, pack, m, "barrel", 14 + 1.7 * math.cos(a), -3 + 1.7 * math.sin(a), name=f"barrel_{i}")
    for i, (px, py, pyaw, slug) in enumerate([
            (-24, -12, 0.2, "planks_wood_v1"), (-3, -12, 1.2, "planks_blue"), (12, -16, 0.1, "planks_yellow"),
            (26, -25, 0.6, "box_stack_brown"), (-26, -22, 0.0, "box_white"), (-17, -26, 0.8, "box_black"),
            (23, 3, 1.6, "planks_wood_v1"), (-4, 4.5, 0.3, "planks_blue"), (14, -20, 0.0, "dixie"),
            (26, 5, 0.0, "office_container_single_green")]):
        place(sc, pack, m, slug, px, py, yaw=pyaw, name=f"pile_{i}")
    return sc
