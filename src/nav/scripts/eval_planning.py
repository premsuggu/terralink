#!/usr/bin/env python3
"""Experiment X-C2 (offline): does a better classification change what the A* planner does on a REAL (noisy) scan?

For every goal of a stress world (the markers of worldgen/worlds/<w>.py) A* plans from S on three maps:
  truth    - the terrain_analysis classification of the noiseless height field (what a perfect UAV map would give)
  old      - the old classifier on the simulated scan, max_roughness 0.5 (the override the scans needed)
  new      - terrain_analysis on the same scan
and the two scan-based routes are compared with the truth map: found / not found (vs truth), length, and VIOLATIONS = path
samples (every 5 cm) that fall on a cell the TRUTH map calls lethal (the vehicle would drive into a hazard).
The simulated scans are cached (--cache DIR): a scan of W1 takes two minutes, a plan a second.

Usage: python3 eval_planning.py w3_excavation_slopes [--cache DIR] [--noise 0.01] [--altitude 7]
"""
from __future__ import annotations

import argparse
import importlib
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, ".."), os.path.join(HERE, "..", "..", "emap"), HERE):
    sys.path.insert(0, p)
from emap.scan_sim import lawnmower, scan                                  # noqa: E402
from emap.terrain_analysis import TerrainParams, analyze_terrain, estimate_noise_sigma   # noqa: E402
from emap.traversability import LETHAL, compute_traversability              # noqa: E402
from nav import astar_planner                                              # noqa: E402
from nav.replay import MapSnapshot, build_masks, run_plan                  # noqa: E402

MAPS = os.path.join(HERE, "..", "maps")


def scanned(world, altitude, noise, cache):
    d = np.load(os.path.join(MAPS, f"{world}_uavview.npz"))
    res, n = float(d["resolution"]), d["layer_elevation"].shape[0]
    cx, cy = float(d["center_x"]), float(d["center_y"])
    elev = d["layer_elevation"].astype(np.float64)
    f = os.path.join(cache, f"{world}_scan_a{altitude}_n{noise}.npz") if cache else None
    if f and os.path.exists(f):
        z = np.load(f)
        return res, cx, cy, elev, z["e"], z["v"], z["m"]
    half = n * res / 2
    poses = lawnmower(cx - half + 1, cx + half - 1, cy - half + 1, cy + half - 1, altitude)
    em = scan(elev, res, cx - n / 2 * res, cy - n / 2 * res, poses, altitude, map_length=n * res, center=(cx, cy), noise_a=noise)
    e, v, m = em.layer("elevation").astype(np.float64), em.layer("variance").astype(np.float64), em.layer("is_valid") > 0.5
    if f:
        os.makedirs(cache, exist_ok=True)
        np.savez(f, e=e, v=v, m=m)
    return res, cx, cy, elev, e, v, m


def snapshot(e, v, m, trav, res, cx, cy):
    return MapSnapshot(layers={"elevation": e.astype(np.float32), "variance": v.astype(np.float32),
                               "is_valid": m.astype(np.float32), "traversability": trav.astype(np.float32)},
                       resolution=res, center_x=cx, center_y=cy)


def violations(path, truth_lethal, res, cx, cy):
    """Samples (5 cm apart) of the route that lie on truth-lethal cells."""
    n = truth_lethal.shape[0]
    bad = tot = 0
    for p, q in zip(path[:-1], path[1:]):
        k = max(int(np.hypot(*(np.asarray(q) - np.asarray(p))) / 0.05), 1)
        for t in np.linspace(0, 1, k, endpoint=False):
            x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
            r, c = int(round((y - cy) / res + n / 2)), int(round((x - cx) / res + n / 2))
            tot += 1
            bad += bool(0 <= r < n and 0 <= c < n and truth_lethal[r, c])
    return bad, tot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--cache", default=None)
    ap.add_argument("--noise", type=float, default=0.01)
    ap.add_argument("--altitude", type=float, default=7.0)
    a = ap.parse_args()
    mod = importlib.import_module(f"worldgen.worlds.{a.world}")
    start = next(m for m in mod.MARKERS if m[0] == "S")[1]
    goals = [(m[0], m[1]) for m in mod.MARKERS if m[0] != "S"]
    res, cx, cy, elev, e, v, m = scanned(a.world, a.altitude, a.noise, a.cache)
    P = TerrainParams()
    ones = np.ones_like(m)
    truth_t = analyze_terrain(elev, np.zeros_like(elev), ones, res, P).tier
    maps = {
        "truth": snapshot(elev, np.zeros_like(elev), ones, truth_t, res, cx, cy),
        "old": snapshot(e, v, m, compute_traversability(e, v, m, res, 0.35, 0.15, 0.5), res, cx, cy),
        "new": snapshot(e, v, m, analyze_terrain(e, v, m, res, TerrainParams(noise_sigma=estimate_noise_sigma(analyze_terrain(e, v, m, res, P)))).tier, res, cx, cy),
    }
    masks = {k: build_masks(s) for k, s in maps.items()}
    tl = truth_t == LETHAL
    print(f"{a.world}  start {start}  noise {a.noise}  altitude {a.altitude}")
    print(f"{'goal':<5}{'truth':>14}{'old (rough .5)':>26}{'new':>26}")
    tot = {"old": [0, 0, 0], "new": [0, 0, 0]}             # [disagree with truth on feasibility, violating routes, routes]
    for name, g in goals:
        row = {}
        for k in ("truth", "old", "new"):
            r = run_plan(maps[k], masks[k], start, g, planner="astar", params=astar_planner.PlannerParams())
            if r.valid and len(r.waypoints) > 1:
                pts = np.asarray(r.waypoints, float)
                L = float(np.hypot(*np.diff(pts, axis=0).T).sum())
                bad, tot_s = violations(pts, tl, res, cx, cy)
                row[k] = (True, L, bad, tot_s)
            else:
                row[k] = (False, 0.0, 0, 0)
        def fmt(k):
            ok, L, bad, ts = row[k]
            return "no path" if not ok else (f"{L:6.1f} m  viol {bad}/{ts}" if k != "truth" else f"{L:6.1f} m")
        print(f"{name:<5}{fmt('truth'):>14}{fmt('old'):>26}{fmt('new'):>26}")
        for k in ("old", "new"):
            tot[k][0] += row[k][0] != row["truth"][0]
            tot[k][1] += row[k][2] > 0
            tot[k][2] += row[k][0]
    for k in ("old", "new"):
        print(f"  {k}: feasibility differs from truth on {tot[k][0]}/{len(goals)} goals; {tot[k][1]} of {tot[k][2]} routes touch a truth-lethal cell")


if __name__ == "__main__":
    main()
