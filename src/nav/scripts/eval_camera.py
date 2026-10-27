#!/usr/bin/env python3
"""Experiment X-C5 (offline): where should the UGV's depth camera point? Coverage and quality of a UGV-only ground map per camera pose.

For one route (A* on the truth map) the UGV drives with the camera at several (pitch, height) settings, only that camera is fused (a
fresh map, as for the separate ground layer) and the result is classified (v2 + auto noise) and compared with the truth. Reported per
setting: observed strip width (cells seen / route length), the share of the corridor (+-3 m of the route) that is observed, accuracy and
lethal errors on the observed interior cells, the median variance.

Usage: python3 eval_camera.py w2_container_yard C [--noise 0.002 --noise-b 0.001]
"""
from __future__ import annotations

import argparse
import importlib
import os
import sys
from dataclasses import replace

import numpy as np
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, ".."), os.path.join(HERE, "..", "..", "emap"), HERE):
    sys.path.insert(0, p)
import eval_refine as er                                                    # noqa: E402
from emap.elevation_map import ElevationMap                                 # noqa: E402
from emap.scan_sim import drive_poses, ugv_pass                             # noqa: E402
from emap.terrain_analysis import TerrainParams, analyze_terrain            # noqa: E402
from nav import astar_planner                                               # noqa: E402
from nav.replay import MapSnapshot, build_masks, run_plan                   # noqa: E402

CONFIGS = [(0.60, 0.155), (0.45, 0.155), (0.30, 0.155), (0.15, 0.155), (0.60, 0.40), (0.45, 0.40), (0.30, 0.40), (0.20, 0.60), (0.35, 0.60)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world"); ap.add_argument("goal")
    ap.add_argument("--noise", type=float, default=0.002); ap.add_argument("--noise-b", type=float, default=0.001)
    a = ap.parse_args()
    mod = importlib.import_module(f"worldgen.worlds.{a.world}")
    start = next(m for m in mod.MARKERS if m[0] == "S")[1]
    goal = next(m for m in mod.MARKERS if m[0] == a.goal)[1]
    d = np.load(os.path.join(er.MAPS, f"{a.world}_uavview.npz"))
    res, n = float(d["resolution"]), d["layer_elevation"].shape[0]
    cx, cy = float(d["center_x"]), float(d["center_y"])
    elev = d["layer_elevation"].astype(np.float64)
    P = TerrainParams()
    T = analyze_terrain(elev, np.zeros_like(elev), np.ones((n, n), bool), res, P)
    snap = MapSnapshot(layers={"elevation": elev.astype(np.float32), "variance": np.zeros_like(elev, np.float32),
                               "is_valid": np.ones((n, n), np.float32), "traversability": T.tier.astype(np.float32)},
                       resolution=res, center_x=cx, center_y=cy)
    r = run_plan(snap, build_masks(snap), start, goal, planner="astar", params=astar_planner.PlannerParams())
    route = np.asarray(r.waypoints, float)
    L = float(np.hypot(*np.diff(route, axis=0).T).sum())
    poses = drive_poses(route)
    rr = np.round((np.array([q[1] for q in poses]) - cy) / res + n / 2).astype(int)
    cc = np.round((np.array([q[0] for q in poses]) - cx) / res + n / 2).astype(int)
    line = np.zeros((n, n), bool); line[np.clip(rr, 0, n - 1), np.clip(cc, 0, n - 1)] = True
    corridor = ndi.distance_transform_edt(~line) * res <= 3.0
    print(f"{a.world} {a.goal}: route {L:.1f} m, {len(poses)} poses, noise {a.noise}+{a.noise_b}*r^2")
    print(f"{'pitch':>6}{'height':>8}{'strip m':>9}{'corridor seen':>15}{'agree':>8}{'false-leth':>12}{'missed-leth':>13}{'median var':>12}")
    for pitch, h in CONFIGS:
        G = ElevationMap(res, n * res, 10.0); G.center_x, G.center_y = cx, cy
        ugv_pass(G, elev, res, cx - n / 2 * res, cy - n / 2 * res, poses, noise_a=a.noise, noise_b=a.noise_b, pitch=pitch, cam_height=h)
        ok = G.layer("is_valid") > 0.5
        e, v = G.layer("elevation").astype(np.float64), G.layer("variance").astype(np.float64)
        base = analyze_terrain(e, v, ok, res, P)
        from emap.terrain_analysis import estimate_noise_sigma
        A = analyze_terrain(e, v, ok, res, replace(P, noise_sigma=estimate_noise_sigma(base)))
        inter = ndi.binary_erosion(ok, structure=np.ones((5, 5)))
        s = er.score(A.tier, T.tier, inter)
        print(f"{pitch:>6.2f}{h:>8.2f}{ok.sum() * res * res / L:>9.2f}{(ok & corridor).sum() / max(corridor.sum(), 1):>15.1%}"
              f"{s[0]:>8.1%}{s[1]:>12.1%}{s[2]:>13.1%}{np.median(v[ok]):>12.4f}")


if __name__ == "__main__":
    main()
