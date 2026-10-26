"""W5 - "Door gauntlet" (44 x 16 m): how narrow a door can the UGV REALLY pass?

Why it exists: the Jackal-class UGV is 0.43 m wide and 0.51 m long, the planner thinks in circles and the stress worlds only
contain a handful of doors (W2 lanes, W4 rooms), so "what is the smallest door A* + Nav2 get through" was a guess. The
gauntlet answers it with one measurement per door instead.

Eight cross walls, 4 m apart, each with ONE door; the doors get narrower along the way and alternate between the north and the
south side (+3 m / -3 m), so the robot has to swing across the hall and line up with every door (an S-curve, not a straight
shot - the narrowest doors are the hardest to hit). Door widths (metres, free opening): 1.2, 1.0, 0.9, 0.8, 0.7, 0.6, 0.55, 0.5.
Goals `1`..`8` sit 2 m behind each wall, so every door has its own goal: goal k is feasible only if doors 1..k are passable.

Nothing but walls: no asset pack is needed. The map bounds are SQUARE (44 x 44 m, the hall is 44 x 16 m in the middle): the A* planner assumes square maps.
"""
from __future__ import annotations

from ..layout import CONCRETE, rect_walls, wall
from ..scene import Scene

NAME = "w5_door_gauntlet"
TITLE = "W5  Door gauntlet"
CEILING = 3.0
UAV_CEILING = 5.0

DOORS = [1.2, 1.0, 0.9, 0.8, 0.7, 0.6, 0.55, 0.5]          # free opening of door k (k = 1..8)
WALL_X = [-16.0 + 4.0 * k for k in range(8)]                # x of the cross walls: -16 ... 12
DOOR_Y = [3.0 if k % 2 == 0 else -3.0 for k in range(8)]    # y of the door centre (alternating)
HALF_Y, HALF_X = 8.0, 22.0

MARKERS = [("S", (-20.0, 0.0), "s", "#2ca02c")] + [
    (str(k + 1), (WALL_X[k] + 2.0, 0.0), "*", "#d62728") for k in range(8)]


def build(pack, models_dir) -> Scene:
    sc = Scene(NAME, bounds=(-HALF_X, HALF_X, -HALF_X, HALF_X), ground_rgb=(0.50, 0.50, 0.46), ground_size=300.0)
    rect_walls(sc, "hall", -HALF_X, -HALF_Y, HALF_X, HALF_Y, thick=0.4, h=2.2)
    for k in range(8):
        x, c, w = WALL_X[k], DOOR_Y[k], DOORS[k]
        lo, hi = c - w / 2, c + w / 2
        if lo > -HALF_Y:                                     # the part of the wall on the south side of the door
            wall(sc, f"w{k + 1}_s", x, -HALF_Y, x, lo, 0.3, 2.2, rgb=CONCRETE)
        if hi < HALF_Y:                                      # ... and on the north side
            wall(sc, f"w{k + 1}_n", x, hi, x, HALF_Y, 0.3, 2.2, rgb=CONCRETE)
    return sc
