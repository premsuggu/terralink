#!/usr/bin/env python3
"""Pictures of the terrain classification: ground truth vs what our rules predict from a UAV scan (writes media/classification/).

For every stress world (W1-W5) and two sensor-noise levels:
  1. TRUE HEIGHT     - the world's true UAV-view height field (maps/<world>_uavview.npz)
  2. GROUND TRUTH    - the class of every cell from the TRUE, complete, noise-free geometry, judged against the Jackal's MEASURED
                       limits (step11 X-A1: a 9 cm step crossed / 10 cm blocked, ramps to 35 deg). It uses the same thresholds as
                       the rules (max step 0.09 m, max slope 0.35 rad), so this comparison tests how well the rules SURVIVE A REAL
                       SCAN (noise, gaps, sparse points) - the thresholds themselves were validated by driving the vehicle.
  3. OLD RULE        - emap.traversability as the node ran it by default before step11 (variance as roughness, limit 0.05)
  4. NEW RULES (v2)  - emap.terrain_analysis with self-calibrated noise compensation (the node's default today)
  5. ERRORS of v2    - where the prediction differs from the ground truth, coloured by how dangerous the mistake is
Panels 3-5 are computed on the map the offline scan simulator builds (emap.scan_sim: the live camera model + the real fusion),
the same method as experiment X-C1 (src/nav/scripts/eval_terrain.py). W4 also gets the REAL scan from the live Gazebo mission.

Usage: python3 render_classification.py [--worlds w1_house_site ...] [--out media/classification]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src", "emap"))
import matplotlib                                                          # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                            # noqa: E402
from matplotlib.colors import ListedColormap                               # noqa: E402
from matplotlib.patches import Patch                                       # noqa: E402

import eval_terrain as et                                                  # noqa: E402
from emap.traversability import DIFFICULT, EASY, LETHAL                    # noqa: E402

WORLDS = ["w1_house_site", "w2_container_yard", "w3_excavation_slopes", "w4_structure_decks", "w5_door_gauntlet"]
# The two noise levels of step11 X-C1 (sigma = a + b * range^2, metres; UAV at 7 m).
NOISE = {"low": (0.01, 0.0), "high": (0.03, 0.001)}

# Class picture: 0 unobserved, 1 EASY, 2 DIFFICULT, 3 LETHAL
CLASS_CMAP = ListedColormap(["#bdbdbd", "#a6d96a", "#fdae61", "#7f0000"])
CLASS_LEGEND = [Patch(color="#a6d96a", label="EASY (drive freely)"), Patch(color="#fdae61", label="DIFFICULT (allowed, costs more)"),
                Patch(color="#7f0000", label="LETHAL (never drive)"), Patch(color="#bdbdbd", label="not observed")]
# Error picture: 0 unobserved, 1 correct, 2 EASY<->DIFFICULT mix-up, 3 false LETHAL (free space lost), 4 MISSED LETHAL (danger)
ERR_CMAP = ListedColormap(["#bdbdbd", "#f7f7f7", "#ffd92f", "#4575b4", "#d73027"])
ERR_LEGEND = [Patch(color="#f7f7f7", label="correct"), Patch(color="#ffd92f", label="EASY/DIFFICULT mix-up (cost only)"),
              Patch(color="#4575b4", label="false LETHAL (free ground lost)"), Patch(color="#d73027", label="MISSED LETHAL (dangerous)"),
              Patch(color="#bdbdbd", label="not observed")]


def class_image(tier, observed):
    img = np.zeros(tier.shape, np.int8)
    img[np.isclose(tier, EASY)] = 1
    img[np.isclose(tier, DIFFICULT)] = 2
    img[np.isclose(tier, LETHAL)] = 3
    img[~observed] = 0
    return img


def error_image(truth, pred, observed):
    t, p = class_image(truth, np.ones_like(observed)), class_image(pred, observed)
    img = np.ones(truth.shape, np.int8)
    img[(t != p) & (t != 3) & (p != 3)] = 2
    img[(p == 3) & (t != 3)] = 3
    img[(t == 3) & (p != 3)] = 4
    img[~observed] = 0
    return img


def scores(truth, pred, observed):
    """agree / false LETHAL / missed LETHAL per cell, on observed cells. `missed_lethal_1cell` forgives a one-cell (0.1 m) offset
    and is counted on interior cells only (see below):
    walls in the truth are often ONE cell thin, so a wall predicted one cell over counts as both a miss and a false LETHAL in the
    strict per-cell score although the planner (which inflates walls by the robot radius) would still avoid it."""
    from scipy.ndimage import binary_dilation
    ti, pi = class_image(truth, np.ones_like(observed)), class_image(pred, observed)
    t, p = ti[observed], pi[observed]
    n = max(t.size, 1)
    from scipy.ndimage import binary_erosion
    # INTERIOR = observed cells whose whole 5 x 5 neighbourhood was observed (as step11 X-C1). At the RIM of the observed area (e.g. the
    # site's boundary wall at the edge of the scan) the far side of a wall was never measured, so no rule can classify it there.
    interior = binary_erosion(observed, structure=np.ones((5, 5), bool))
    near_pred_lethal = binary_dilation(pi == 3, structure=np.ones((3, 3), bool))
    t_leth_int = (ti == 3) & interior
    return {"agree": (t == p).sum() / n,
            "false_lethal": ((p == 3) & (t == 1)).sum() / max((t == 1).sum(), 1),
            "missed_lethal": ((p != 3) & (t == 3)).sum() / max((t == 3).sum(), 1),
            "missed_lethal_1cell": (t_leth_int & ~near_pred_lethal).sum() / max(t_leth_int.sum(), 1)}


def figure(title, extent, elev, truth, panels, out_png):
    """panels: list of (title, tier, observed). The last one also gets an error panel."""
    n = 2 + len(panels) + 1
    fig, axes = plt.subplots(1, n, figsize=(4.2 * n, 5.0), constrained_layout=True)
    im = axes[0].imshow(elev, origin="lower", extent=extent, cmap="terrain")
    fig.colorbar(im, ax=axes[0], shrink=0.7, label="height (m)")
    axes[0].set_title("true height")
    axes[1].imshow(class_image(truth, np.ones(truth.shape, bool)), origin="lower", extent=extent, cmap=CLASS_CMAP, vmin=0, vmax=3,
                   interpolation="nearest")
    axes[1].set_title("GROUND TRUTH class")
    for ax, (name, tier, obs) in zip(axes[2:], panels):
        s = scores(truth, tier, obs)
        ax.imshow(class_image(tier, obs), origin="lower", extent=extent, cmap=CLASS_CMAP, vmin=0, vmax=3, interpolation="nearest")
        ax.set_title(f"{name}\nagree {s['agree']:.1%} | missed lethal {s['missed_lethal']:.1%} ({s['missed_lethal_1cell']:.1%} interior >1 cell)", fontsize=10)
    name, tier, obs = panels[-1]
    axes[-1].imshow(error_image(truth, tier, obs), origin="lower", extent=extent, cmap=ERR_CMAP, vmin=0, vmax=4, interpolation="nearest")
    axes[-1].set_title(f"errors of: {name.splitlines()[0]}")
    for ax in axes:
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    fig.legend(handles=CLASS_LEGEND + ERR_LEGEND, loc="lower center", ncol=5, fontsize=9, bbox_to_anchor=(0.5, -0.12))
    fig.suptitle(title, fontsize=13)
    fig.savefig(out_png, dpi=90, bbox_inches="tight")
    plt.close(fig)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--worlds", nargs="*", default=WORLDS)
    a.add_argument("--out", default=os.path.join(REPO, "media", "classification"))
    a.add_argument("--stride", type=int, default=2)
    args = a.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for w in args.worlds:
        d = np.load(os.path.join(et.MAPS, f"{w}_uavview.npz"))
        res, n = float(d["resolution"]), d["layer_elevation"].shape[0]
        cx, cy = float(d["center_x"]), float(d["center_y"])
        extent = [cx - n / 2 * res, cx + n / 2 * res, cy - n / 2 * res, cy + n / 2 * res]
        clf = et.classifiers(res)
        for level, (na, nb) in NOISE.items():
            _, emap, elev, obs = et.run(w, 7.0, na, nb, 0, args.stride, quiet=True)
            truth = clf["new"](elev, np.zeros_like(elev), np.ones_like(obs))           # complete, noise-free geometry
            e, v = emap.layer("elevation").astype(np.float64), emap.layer("variance").astype(np.float64)
            old, new = clf["old(default)"](e, v, obs), clf["new"](e, v, obs)
            label = f"sigma {na * 100:.0f} cm" + (f" + {nb} r^2" if nb else "")
            figure(f"{w} - simulated UAV scan, {level} noise ({label})", extent, elev, truth,
                   [("OLD rule (before step11)", old, obs), ("NEW rules v2 (today)", new, obs)],
                   os.path.join(args.out, f"{w}_{level}_noise.png"))
            for name, pred in (("old", old), ("new v2", new)):
                rows.append((w, f"simulated, {level} noise", name, scores(truth, pred, obs), float(obs.mean())))
            print(w, level, "done", flush=True)
        if w == "w4_structure_decks":                                              # the REAL scan of the live Gazebo mission
            s = np.load(os.path.join(et.MAPS, f"{w}_uavscan.npz"))
            obs = s["layer_is_valid"] > 0.5
            elev = d["layer_elevation"].astype(np.float64)
            truth = clf["new"](elev, np.zeros_like(elev), np.ones_like(obs))
            se, sv = s["layer_elevation"].astype(np.float64), s["layer_variance"].astype(np.float64)
            old, new = clf["old(default)"](se, sv, obs), clf["new"](se, sv, obs)
            figure(f"{w} - REAL UAV scan from the live Gazebo mission (emap)", extent, elev, truth,
                   [("OLD rule (before step11)", old, obs), ("NEW rules v2 (today)", new, obs)],
                   os.path.join(args.out, f"{w}_real_scan.png"))
            for name, pred in (("old", old), ("new v2", new)):
                rows.append((w, "REAL Gazebo scan", name, scores(truth, pred, obs), float(obs.mean())))
    with open(os.path.join(args.out, "scores.md"), "w") as f:
        f.write("| world | scan | rules | agreement | false LETHAL | MISSED LETHAL (per cell) | MISSED LETHAL (interior, > 1 cell away) | observed |\n"
                "|---|---|---|---|---|---|---|---|\n")
        for w, scan, name, s, o in rows:
            f.write(f"| {w} | {scan} | {name} | {s['agree']:.1%} | {s['false_lethal']:.1%} | {s['missed_lethal']:.1%} | "
                    f"{s['missed_lethal_1cell']:.1%} | {o:.0%} |\n")
    print(open(os.path.join(args.out, "scores.md")).read())


if __name__ == "__main__":
    main()
