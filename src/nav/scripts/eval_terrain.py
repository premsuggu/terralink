#!/usr/bin/env python3
"""Experiment X-C1: how well does a terrain classifier survive a REAL (noisy, partial) UAV scan?

Method: the world's true UAV-view height field (maps/<world>_uavview.npz) is flown with the offline scan simulator
(emap.scan_sim: live camera model + the real fusion), giving the map the node would build. Each classifier is run on
(a) the noiseless truth and (b) the scanned map; cells observed in the scan are compared:
  false-lethal = truth EASY, scan LETHAL      (the planner loses free space)
  missed-lethal = truth LETHAL, scan not LETHAL (the planner drives into a hazard - the dangerous error)
  agreement = the 3-class label is identical.
Classifiers: old(default)  - emap.traversability, max_roughness 0.05 (what the node ran by default)
             old(rough=.5) - the override the stress-world scans needed
             new           - emap.terrain_analysis

Usage: python3 eval_terrain.py w3_excavation_slopes [--altitude 6] [--noise 0.002] [--seed 0]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "emap"))
from emap.scan_sim import lawnmower, scan                                  # noqa: E402
from emap.terrain_analysis import TerrainParams, analyze_terrain           # noqa: E402
from emap.traversability import DIFFICULT, EASY, LETHAL, compute_traversability  # noqa: E402

MAPS = os.path.join(HERE, "..", "maps")
NAMES = {LETHAL: "LETHAL", DIFFICULT: "DIFFICULT", EASY: "EASY"}


def classifiers(res: float):
    return {
        "old(default)": lambda e, v, m: compute_traversability(e, v, m, res, 0.35, 0.15, 0.05),
        "old(rough=.5)": lambda e, v, m: compute_traversability(e, v, m, res, 0.35, 0.15, 0.5),
        "new": lambda e, v, m: analyze_terrain(e, v, m, res, TerrainParams()).tier,
    }


def compare(truth, got, obs):
    t, g = truth[obs], got[obs]
    n = max(int(obs.sum()), 1)
    easy, leth = t == EASY, t == LETHAL
    return {
        "agree": float((t == g).sum()) / n,
        "false_lethal": float(((g == LETHAL) & easy).sum()) / max(int(easy.sum()), 1),
        "missed_lethal": float(((g != LETHAL) & leth).sum()) / max(int(leth.sum()), 1),
        "lethal_frac_truth": float(leth.sum()) / n, "lethal_frac_scan": float((g == LETHAL).sum()) / n,
    }


def run(world: str, altitude: float, noise: float, noise_b: float, seed: int, stride: int, quiet: bool = False):
    d = np.load(os.path.join(MAPS, f"{world}_uavview.npz"))
    res = float(d["resolution"])
    elev = d["layer_elevation"].astype(np.float64)
    n = elev.shape[0]
    cx, cy = float(d["center_x"]), float(d["center_y"])
    x0, y0 = cx - n / 2 * res, cy - n / 2 * res
    half = n * res / 2
    poses = lawnmower(cx - half + 1, cx + half - 1, cy - half + 1, cy + half - 1, altitude)
    t = time.time()
    emap = scan(elev, res, x0, y0, poses, altitude, map_length=n * res, center=(cx, cy), noise_a=noise, noise_b=noise_b,
                seed=seed, stride=stride)
    obs = emap.layer("is_valid") > 0.5
    # "interior" = observed cells whose whole 5x5 neighbourhood is observed: at the rim of the observed area a step to an
    # unobserved cell (the far side of a wall seen only from above) cannot be known, for ANY rule - scored separately.
    from scipy.ndimage import binary_erosion
    interior = binary_erosion(obs, structure=np.ones((5, 5)))
    if not quiet:
        print(f"{world}: {len(poses)} poses, scan {time.time() - t:.0f} s, observed {obs.mean():.1%} of {n}x{n} cells, "
              f"variance median {np.median(emap.layer('variance')[obs]):.3f}")
    truth_valid = np.ones_like(obs)
    rows = {}
    for name, f in classifiers(res).items():
        truth = f(elev, np.zeros_like(elev), truth_valid)
        got = f(emap.layer("elevation").astype(np.float64), emap.layer("variance").astype(np.float64), obs)
        rows[name] = compare(truth, got, interior)
        rows[name]["rim_missed_lethal"] = float(((got != LETHAL) & (truth == LETHAL) & obs & ~interior).sum()) / max(int(((truth == LETHAL) & obs).sum()), 1)
    if not quiet:
        print(f"(interior cells: {interior.mean():.1%} of the map)  rim = observed cells next to unobserved ones")
        print(f"{'classifier':<14}{'agree':>8}{'false-leth':>12}{'missed-leth':>13}{'leth truth':>12}{'leth scan':>11}{'rim-missed':>12}")
        for k, r in rows.items():
            print(f"{k:<14}{r['agree']:>8.1%}{r['false_lethal']:>12.1%}{r['missed_lethal']:>13.1%}"
                  f"{r['lethal_frac_truth']:>12.1%}{r['lethal_frac_scan']:>11.1%}{r['rim_missed_lethal']:>12.1%}")
    return rows, emap, elev, obs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--altitude", type=float, default=6.0)
    ap.add_argument("--noise", type=float, default=0.002, help="depth noise sigma at 0 m (m)")
    ap.add_argument("--noise-b", type=float, default=0.0, help="additional sigma per m^2 of range")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--stride", type=int, default=2)
    a = ap.parse_args()
    run(a.world, a.altitude, a.noise, a.noise_b, a.seed, a.stride)
