#!/usr/bin/env python3
"""Experiment X-C3 (offline): does fusing the UGV's own depth camera while it drives the route improve the classification?

UAV scan (scan_sim) -> v2 classification -> A* route to a goal -> the UGV drives the route and its front camera (the Jackal's: 320 x 240,
34 deg down, 0.1-8 m) is fused into the SAME map with the same fuse_points -> v2 classification again. Both are compared with the truth
classification (noiseless height field): everywhere, and in the corridor (cells within --corridor metres of the route).

Usage: python3 eval_refine.py w2_container_yard C [--noise 0.02 --noise-b 0.0005] [--cache DIR]
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
from emap.elevation_map import ElevationMap                                 # noqa: E402
from emap.scan_sim import drive_poses, lawnmower, scan, ugv_pass            # noqa: E402
from emap.terrain_analysis import TerrainParams, analyze_terrain, estimate_noise_sigma   # noqa: E402
from emap.traversability import EASY, LETHAL                                 # noqa: E402
from nav import astar_planner                                               # noqa: E402
from nav.replay import MapSnapshot, build_masks, run_plan                   # noqa: E402

MAPS = os.path.join(HERE, "..", "maps")


def uav_map(world, altitude, a, b, cache):
    d = np.load(os.path.join(MAPS, f"{world}_uavview.npz"))
    res, n = float(d["resolution"]), d["layer_elevation"].shape[0]
    cx, cy = float(d["center_x"]), float(d["center_y"])
    elev = d["layer_elevation"].astype(np.float64)
    f = os.path.join(cache, f"{world}_refine_a{altitude}_{a}_{b}.npz") if cache else None
    em = ElevationMap(res, n * res, 10.0)
    em.center_x, em.center_y = cx, cy
    if f and os.path.exists(f):
        z = np.load(f)
        for k in ("elevation", "variance", "is_valid"):
            em.layer(k)[:] = z[k]
        return em, elev, res, cx, cy
    half = n * res / 2
    poses = lawnmower(cx - half + 1, cx + half - 1, cy - half + 1, cy + half - 1, altitude)
    em = scan(elev, res, cx - n / 2 * res, cy - n / 2 * res, poses, altitude, map_length=n * res, center=(cx, cy), noise_a=a, noise_b=b)
    if f:
        os.makedirs(cache, exist_ok=True)
        np.savez(f, **{k: em.layer(k) for k in ("elevation", "variance", "is_valid")})
    return em, elev, res, cx, cy


def classify(em, res, P):
    """v2 with the scan noise estimated from the map (the node's default behaviour)."""
    e, v, m = em.layer("elevation").astype(np.float64), em.layer("variance").astype(np.float64), em.layer("is_valid") > 0.5
    base = analyze_terrain(e, v, m, res, P)
    from dataclasses import replace
    return analyze_terrain(e, v, m, res, replace(P, noise_sigma=estimate_noise_sigma(base)))


def score(tier, truth, mask):
    t, g = truth[mask], tier[mask]
    n = max(int(mask.sum()), 1)
    easy, leth = t == EASY, t == LETHAL
    return (float((t == g).sum()) / n, float(((g == LETHAL) & easy).sum()) / max(int(easy.sum()), 1),
            float(((g != LETHAL) & leth).sum()) / max(int(leth.sum()), 1), int(mask.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world"); ap.add_argument("goal")
    ap.add_argument("--noise", type=float, default=0.02); ap.add_argument("--noise-b", type=float, default=0.0005)
    ap.add_argument("--altitude", type=float, default=7.0); ap.add_argument("--corridor", type=float, default=1.5)
    ap.add_argument("--cache", default=None)
    a = ap.parse_args()
    mod = importlib.import_module(f"worldgen.worlds.{a.world}")
    start = next(m for m in mod.MARKERS if m[0] == "S")[1]
    goal = next(m for m in mod.MARKERS if m[0] == a.goal)[1]
    P = TerrainParams()
    em, elev, res, cx, cy = uav_map(a.world, a.altitude, a.noise, a.noise_b, a.cache)
    n = elev.shape[0]
    truth = analyze_terrain(elev, np.zeros_like(elev), np.ones((n, n), bool), res, P).tier
    before = classify(em, res, P)
    snap = MapSnapshot(layers={"elevation": em.layer("elevation").copy(), "variance": em.layer("variance").copy(),
                               "is_valid": em.layer("is_valid").copy(), "traversability": before.tier.astype(np.float32)},
                       resolution=res, center_x=cx, center_y=cy)
    r = run_plan(snap, build_masks(snap), start, goal, planner="astar", params=astar_planner.PlannerParams())
    if not (r.valid and len(r.waypoints) > 1):
        print("no route on the UAV map"); return
    route = np.asarray(r.waypoints, float)
    poses = drive_poses(route)
    obs0 = em.layer("is_valid") > 0.5
    ugv_pass(em, elev, res, cx - n / 2 * res, cy - n / 2 * res, poses, noise_a=a.noise, noise_b=a.noise_b)
    after = classify(em, res, P)
    # corridor = cells within --corridor m of the driven route
    rr = np.round((np.array([p[1] for p in poses]) - cy) / res + n / 2).astype(int)
    cc = np.round((np.array([p[0] for p in poses]) - cx) / res + n / 2).astype(int)
    line = np.zeros((n, n), bool); line[np.clip(rr, 0, n - 1), np.clip(cc, 0, n - 1)] = True
    corridor = ndi.distance_transform_edt(~line) * res <= a.corridor
    inter = ndi.binary_erosion(obs0, structure=np.ones((5, 5)))
    seen_by_ugv = (em.layer("is_valid") > 0.5) & (em.layer("variance") < 0.02) & corridor        # cells the UGV camera really measured
    print(f"{a.world} {a.goal}: route {np.hypot(*np.diff(route, axis=0).T).sum():.1f} m, {len(poses)} poses, noise {a.noise}+{a.noise_b}*r^2")
    print(f"{'cells':<28}{'n':>8}{'agree':>9}{'false-leth':>12}{'missed-leth':>13}")
    for name, mask in (("whole map (UAV interior)", inter), ("corridor (UAV interior)", inter & corridor),
                       ("cells the UGV measured", seen_by_ugv & inter)):
        for lab, t in (("UAV only", before.tier), ("UAV + UGV", after.tier)):
            s = score(t, truth, mask)
            print(f"{name + ' ' + lab:<28}{s[3]:>8}{s[0]:>9.1%}{s[1]:>12.1%}{s[2]:>13.1%}")
    # cells hidden from the UAV (unobserved before) that the UGV now covers: classification vs truth
    new = (em.layer("is_valid") > 0.5) & ~obs0
    if new.any():
        s = score(after.tier, truth, new)
        print(f"{'newly observed by the UGV':<28}{s[3]:>8}{s[0]:>9.1%}{s[1]:>12.1%}{s[2]:>13.1%}")
    ch = (before.tier != after.tier) & inter
    fixed = ch & (after.tier == truth) & (before.tier != truth); broke = ch & (before.tier == truth) & (after.tier != truth)
    print(f"tier changes on UAV-observed interior cells: {int(ch.sum())}  fixed {int(fixed.sum())}  broken {int(broke.sum())}")


if __name__ == "__main__":
    main()
