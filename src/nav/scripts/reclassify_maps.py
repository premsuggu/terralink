#!/usr/bin/env python3
"""Re-classify the saved world maps (maps/<world>_groundtruth.npz, _uavview.npz) with the v2 terrain rules (step 11).

The maps were built with the old classifier (step 0.15 m, variance as roughness). The stored `elevation` is all that is needed, so this
rewrites `traversability` and adds `confidence` / `hazard` WITHOUT the asset packs the world generator needs. Run it once after
changing the rules; `--check` only prints the feasibility table (goal reachable at the smallest / largest radius) of the new maps.

Usage: python3 reclassify_maps.py [--check] [worlds...]
"""
from __future__ import annotations

import glob
import importlib
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, "..", "..", "emap"), HERE):
    sys.path.insert(0, p)
from emap.terrain_analysis import TerrainParams, analyze_terrain          # noqa: E402
from worldgen.feasibility import RADII, format_table, reachability       # noqa: E402

MAPS = os.path.join(HERE, "..", "maps")


def reclassify(path: str, check_only: bool):
    z = np.load(path)
    layers = {k[len("layer_"):]: z[k] for k in z.files if k.startswith("layer_")}
    res = float(z["resolution"])
    valid = layers["is_valid"] > 0.5
    # a ground-truth map is noiseless: variance 0, every cell observed -> the plain terrain rules, no noise compensation
    r = analyze_terrain(layers["elevation"].astype(np.float64), np.zeros(valid.shape), valid, res, TerrainParams())
    new = dict(layers, traversability=r.tier.astype(np.float32), confidence=r.confidence.astype(np.float32), hazard=r.hazard.astype(np.float32))
    if not check_only:
        np.savez(path, resolution=z["resolution"], center_x=z["center_x"], center_y=z["center_y"], **{f"layer_{k}": v for k, v in new.items()})
    return new, float(z["center_x"]), float(z["center_y"]), res, layers["traversability"]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    check = "--check" in sys.argv
    worlds = args or sorted({os.path.basename(f).rsplit("_", 1)[0] for f in glob.glob(os.path.join(MAPS, "w*_groundtruth.npz"))})
    for w in worlds:
        mod = importlib.import_module(f"worldgen.worlds.{w}")
        start = next(m for m in mod.MARKERS if m[0] == "S")[1]
        goals = {m[0]: m[1] for m in mod.MARKERS if m[0] != "S"}
        for kind in ("groundtruth", "uavview"):
            f = os.path.join(MAPS, f"{w}_{kind}.npz")
            new, cx, cy, res, old_trav = reclassify(f, check)
            changed = (new["traversability"] != old_trav).mean()
            print(f"{w} {kind}: {changed:.2%} of the cells changed class" + ("" if check else " (rewritten)"))
            if kind == "groundtruth":
                print(format_table(reachability(new["traversability"], res, cx, cy, start, goals)))


if __name__ == "__main__":
    main()
