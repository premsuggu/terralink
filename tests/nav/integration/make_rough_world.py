#!/usr/bin/env python3
"""Generate ugv_rough_test.world: eight lanes of RUBBLE (C-07), each a 6 m field of boxes of random height up to H cm.

Lane i is at y = 3*(i - 3.5); the field covers x = 3 .. 9 (a 6 m run). Boxes are 0.15 x 0.15 m on a 0.35 m grid with a fixed seed;
a box height is uniform in [0, H]. `field(i)` returns the boxes, so ugv_rough.py can compute the map-roughness of the same field.
Usage: python3 make_rough_world.py > ugv_rough_test.world
"""
import random

import make_limits_world as base

HEIGHTS_CM = [2, 4, 6, 8, 10, 12, 16, 20]
GRID, SIZE, X0, LENGTH = 0.35, 0.15, 3.0, 6.0


def field(i):
    """[(x, y_offset, height_m)] of lane i (the same list is used for the world file and for the roughness estimate)."""
    rng = random.Random(1000 + i)
    out = []
    nx, ny = int(LENGTH / GRID), int(1.6 / GRID)
    for a in range(nx):
        for b in range(ny):
            out.append((X0 + (a + 0.5) * GRID, (b + 0.5 - ny / 2) * GRID, rng.uniform(0, HEIGHTS_CM[i] / 100.0)))
    return out


def lanes():
    out = []
    for i in range(len(HEIGHTS_CM)):
        y0 = 3.0 * (i - 3.5)
        for k, (x, dy, h) in enumerate(field(i)):
            if h > 0.005:
                out.append(base.box(f"r{i}_{k}", f"{x:.3f} {y0 + dy:.3f} {h / 2:.4f} 0 0 0", (SIZE, SIZE, h)))
    return "\n    ".join(out)


if __name__ == "__main__":
    base.lanes = lanes                       # world() calls lanes() at call time
    print(base.world().replace('name="ugv_limits"', 'name="ugv_rough"').replace("make_limits_world.py (experiment X-A1", "make_rough_world.py (experiment C-07"))
