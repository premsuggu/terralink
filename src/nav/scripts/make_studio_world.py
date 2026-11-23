#!/usr/bin/env python3
"""Write `<world>_studio.world`: a stress world's RUN world (site + UGV at marker S + overhead camera) PLUS the UAV.

Why a new file: the studio lets a person fly the UAV, so the world needs both vehicles from the start. The existing
`<world>_run.world` has only the UGV (the UAV scan is replaced by a saved map there) and `<world>_scan.world` exists only for worlds that
define an automatic patrol (W4). This script makes the missing combination without rebuilding the whole world (no meshes, no maps):
it takes the finished run world and inserts the project's proven `iris_quad` include (`worldgen.run_world.uav_block`).

    python3 src/nav/scripts/make_studio_world.py w2_container_yard --uav-x -3.0 --uav-y -24.0

By default the UAV starts 2 m north of the UGV so the two do not overlap on the ground.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAV = HERE.parent
sys.path.insert(0, str(HERE))

from worldgen.run_world import uav_block  # noqa: E402  (the same UAV include every scan world uses)


def ugv_xy(world_text: str) -> tuple[float, float]:
    """Spawn position of nav_ugv in a run world (its <include> pose)."""
    i = world_text.index("<name>nav_ugv</name>")
    m = re.search(r"<pose>\s*([-\d.e]+)\s+([-\d.e]+)", world_text[i:])
    if not m:
        raise ValueError("nav_ugv pose not found - is this a *_run.world?")
    return float(m.group(1)), float(m.group(2))


def make_studio_world(world: str, uav_x: float | None = None, uav_y: float | None = None) -> Path:
    src = NAV / "worlds" / f"{world}_run.world"
    if not src.exists():
        raise FileNotFoundError(f"{src} missing - run build_world.py {world} first")
    text = src.read_text()
    if "<name>iris_quad</name>" in text:
        raise ValueError(f"{src.name} already contains a UAV")
    sx, sy = ugv_xy(text)
    x = sx if uav_x is None else uav_x
    y = sy + 2.0 if uav_y is None else uav_y
    # insert the UAV just before the world closes (models may appear anywhere inside <world>)
    end = text.rindex("</world>")
    out = text[:end] + uav_block(x, y) + text[end:]
    dst = NAV / "worlds" / f"{world}_studio.world"
    dst.write_text(out)
    return dst


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="e.g. w2_container_yard (needs worlds/<world>_run.world)")
    ap.add_argument("--uav-x", type=float, default=None)
    ap.add_argument("--uav-y", type=float, default=None)
    a = ap.parse_args()
    print("wrote", make_studio_world(a.world, a.uav_x, a.uav_y))


if __name__ == "__main__":
    main()
