"""Build one stress-test world: models + .world + ground-truth maps + PLY files for the viewer.

    source /opt/ros/humble/setup.bash
    python3 src/nav/scripts/build_world.py w1_house_site

Outputs (all from ONE scene description, so they cannot drift apart):
    src/nav/models/site_*/                     Gazebo models (OBJ + palette texture)
    src/nav/worlds/<world>.world               the Ignition world
    src/nav/maps/<world>_uavview.npz           planner map: what the UAV sees (decks/low roofs solid)  <- the one A* gets
    src/nav/maps/<world>_groundtruth.npz       oracle variant (sees under decks), for diagnostics only
    media/worlds/<world>_world.ply             the world as a coloured point cloud (true surface colours)
    media/3d_maps/<world>_elevation_uavview.ply  the planner map: green easy / yellow difficult / red lethal
Open the HTML in a browser; open the map PLY with media/3d_maps/viewer.html (drag and drop).
"""
from __future__ import annotations

import argparse
import importlib
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAV = HERE.parent
REPO = NAV.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "src" / "emap"))
sys.path.insert(0, str(NAV))

from worldgen.assets import Pack          # noqa: E402
from worldgen.html import write_world_html   # noqa: E402
from worldgen.ply import write_elevation_ply   # noqa: E402

DEFAULT_PACK = REPO / "tests/worlds/gazebo_world_conversion/LowPoly-House-Construction-Site-By-Majadroid"


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="module name in worldgen/worlds/, e.g. w1_house_site")
    ap.add_argument("--pack", default=str(DEFAULT_PACK), help="extracted Majadroid pack folder")
    ap.add_argument("--res", type=float, default=0.1, help="ground-truth map resolution [m]")
    ap.add_argument("--oversample", type=int, default=4)
    args = ap.parse_args(argv)

    mod = importlib.import_module(f"worldgen.worlds.{args.world}")
    models_dir, worlds_dir, maps_dir = NAV / "models", NAV / "worlds", NAV / "maps"
    media = REPO / "media" / "worlds"
    for d in (models_dir, worlds_dir, maps_dir, media):
        d.mkdir(parents=True, exist_ok=True)

    t = time.time()
    pack = Pack(args.pack)
    scene = mod.build(pack, models_dir)
    print(f"[{time.time() - t:5.1f}s] scene built: {len(scene.items)} items")

    (worlds_dir / f"{scene.name}.world").write_text(scene.world_sdf())
    print(f"[{time.time() - t:5.1f}s] wrote worlds/{scene.name}.world")

    layers, cx, cy, _ = scene.ground_truth(args.res, mod.CEILING, args.oversample)
    np.savez_compressed(maps_dir / f"{scene.name}_groundtruth.npz", resolution=np.float64(args.res),
                        center_x=np.float64(cx), center_y=np.float64(cy),
                        **{f"layer_{k}": v for k, v in layers.items()})
    # The planner map: what the UAV would see. The ceiling is the UAV's flight altitude (the live patrol
    # flies at 3.0-4.5 m): anything hovering below it (decks, walkway roofs) is a solid surface to the
    # camera, while crane jibs 40 m up are not. `UAV_CEILING` is per world.
    uav_ceiling = getattr(mod, "UAV_CEILING", 5.0)
    uav_layers, ucx, ucy, R = scene.ground_truth(args.res, uav_ceiling, args.oversample)
    np.savez_compressed(maps_dir / f"{scene.name}_uavview.npz", resolution=np.float64(args.res),
                        center_x=np.float64(ucx), center_y=np.float64(ucy),
                        **{f"layer_{k}": v for k, v in uav_layers.items()})
    trav = uav_layers["traversability"]
    print(f"[{time.time() - t:5.1f}s] planner map {trav.shape}, lethal {100 * (trav < 0.15).mean():.1f}% of cells "
          f"(UAV view, ceiling {uav_ceiling:g} m)")

    markers = getattr(mod, "MARKERS", None)
    if markers:
        from worldgen.feasibility import format_table, reachability
        start = next(m for m in markers if m[0] == "S")[1]
        goals = {m[0]: m[1] for m in markers if m[0] != "S"}
        for name, lay, ccx, ccy in (("UAV-view map (what A* gets)", uav_layers, ucx, ucy), ("oracle map", layers, cx, cy)):
            table = format_table(reachability(lay["traversability"], args.res, ccx, ccy, start, goals))
            print(f"feasibility from S on the {name} (robot radius, metres; 'yes' = a route exists):\n{table}")

    media_maps = REPO / "media" / "3d_maps"
    media_maps.mkdir(parents=True, exist_ok=True)
    n_map = write_elevation_ply(uav_layers, ucx, ucy, args.res, media_maps / f"{scene.name}_elevation_uavview.ply")
    n_world = write_world_html(scene, mod.TITLE, media / f"{scene.name}_world.html")
    print(f"[{time.time() - t:5.1f}s] wrote media/worlds/{scene.name}_world.html ({n_world:,} triangles) and "
          f"media/3d_maps/{scene.name}_elevation_uavview.ply ({n_map:,} points)")


if __name__ == "__main__":
    main()
