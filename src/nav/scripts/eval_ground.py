#!/usr/bin/env python3
"""Experiment X-C4 (offline): what does a separate GROUND layer from the UGV's own camera add where the UAV sees only a roof?

A 2.5D cell holds ONE height. Under a deck the UAV's cell holds the deck (3.5 m, flat, EASY) while the UGV drives on the floor
beneath it. Two truth views exist per world: the UAV view (ceiling 5 m: decks included) and the floor view (ceiling 3 m: what a
vehicle drives on). Here: UAV scan of the UAV view -> map U; the UGV drives an A* route (planned on the floor view, as an oracle)
and its front camera is fused into a SEPARATE map G over the floor view. On the cells the UGV measured, the class of the UAV map
(what the planner believes today) and the class of the ground layer are both compared with the floor truth.

Usage: python3 eval_ground.py w4_structure_decks [--noise 0.01 --noise-b 0.0] [--goals A B D F]
"""
from __future__ import annotations

import argparse
import importlib
import os
import sys

import numpy as np
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, ".."), os.path.join(HERE, "..", "..", "emap"), HERE):
    sys.path.insert(0, p)
import eval_refine as er                                                    # noqa: E402
from emap.elevation_map import ElevationMap                                 # noqa: E402
from emap.scan_sim import drive_poses, ugv_pass                             # noqa: E402
from emap.terrain_analysis import TerrainParams, analyze_terrain            # noqa: E402
from emap.traversability import EASY, LETHAL                                 # noqa: E402
from nav import astar_planner                                               # noqa: E402
from nav.replay import MapSnapshot, build_masks, run_plan                   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--noise", type=float, default=0.01); ap.add_argument("--noise-b", type=float, default=0.0)
    ap.add_argument("--goals", nargs="*", default=None); ap.add_argument("--cache", default=None)
    a = ap.parse_args()
    mod = importlib.import_module(f"worldgen.worlds.{a.world}")
    start = next(m for m in mod.MARKERS if m[0] == "S")[1]
    goals = [(m[0], m[1]) for m in mod.MARKERS if m[0] != "S" and (a.goals is None or m[0] in a.goals)]
    P = TerrainParams()
    U, deck, res, cx, cy = er.uav_map(a.world, 7.0, a.noise, a.noise_b, a.cache)            # UAV-view truth + the scanned map
    floor = np.load(os.path.join(er.MAPS, f"{a.world}_groundtruth.npz"))["layer_elevation"].astype(np.float64)
    n = floor.shape[0]
    hidden = np.abs(deck - floor) > 0.5
    print(f"{a.world}: {hidden.mean():.1%} of the cells have a roof/deck above the floor the UGV drives on")
    Ft = analyze_terrain(floor, np.zeros_like(floor), np.ones((n, n), bool), res, P)
    tierU = er.classify(U, res, P).tier
    snap = MapSnapshot(layers={"elevation": floor.astype(np.float32), "variance": np.zeros_like(floor, np.float32),
                               "is_valid": np.ones((n, n), np.float32), "traversability": Ft.tier.astype(np.float32)},
                       resolution=res, center_x=cx, center_y=cy)
    masks = build_masks(snap)
    G = ElevationMap(res, n * res, 10.0); G.center_x, G.center_y = cx, cy
    driven = []
    for name, g in goals:
        r = run_plan(snap, masks, start, g, planner="astar", params=astar_planner.PlannerParams())
        if r.valid and len(r.waypoints) > 1:
            route = np.asarray(r.waypoints, float)
            driven.append(name)
            ugv_pass(G, floor, res, cx - n / 2 * res, cy - n / 2 * res, drive_poses(route), noise_a=a.noise, noise_b=a.noise_b)
    print("routes driven (feasible on the floor view):", driven)
    meas = (G.layer("is_valid") > 0.5) & (G.layer("variance") < 0.02)
    meas = ndi.binary_erosion(meas, structure=np.ones((3, 3)))
    tierG = er.classify(G, res, P).tier
    ok = (U.layer("is_valid") > 0.5)
    print(f"{'cells the UGV measured':<42}{'n':>8}{'agree':>9}{'missed-leth':>13}{'false-leth':>12}")
    for lab, m in (("all", meas), ("  under a roof (UAV sees the deck)", meas & hidden), ("  open sky", meas & ~hidden)):
        for who, t, valid in (("UAV map says", tierU, ok), ("ground layer says", tierG, np.ones_like(ok))):
            mm = m & valid
            s = er.score(t, Ft.tier, mm)
            print(f"{lab + ' / ' + who:<42}{s[3]:>8}{s[0]:>9.1%}{s[2]:>13.1%}{s[1]:>12.1%}")
    new = meas & ~ok                                                  # measured by the UGV, never observed by the UAV
    if new.any():
        sn = er.score(tierG, Ft.tier, new)
        print(f"cells only the UGV observed (the UAV map has NO opinion): {sn[3]}, ground layer agrees with the floor truth on {sn[0]:.1%}, "
              f"{int((new & hidden).sum())} of them under a roof")
    # what the planner would do with the UAV map alone: cells under a roof that the floor makes lethal but the deck says walkable
    bad = meas & hidden & (Ft.tier == LETHAL) & (tierU != LETHAL)
    print(f"floor-lethal cells under a roof that the UAV map calls walkable: {int(bad.sum())} "
          f"({int((meas & hidden & (Ft.tier == LETHAL)).sum())} floor-lethal cells measured under roofs)")


if __name__ == "__main__":
    main()
