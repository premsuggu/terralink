#!/usr/bin/env python3
"""Pictures of how each exploration strategy drives (offline simulator): one panel per strategy, the true walls in black, the
path coloured from start (dark) to end (yellow), start = green square, goal = red star. Writes a PNG.

Same set-up as eval_exploration.py (same simulator, same strategies, same caps). Used for docs/work-docs/nav/step13 and
media/exploration/.

Usage: python3 render_exploration_paths.py --world w4_structure_decks --map none --goal D \
           --policies adopted freespace1 exits_best exits_dfs bug1 bug2 --out media/exploration/w4_noUAV_goalD.png
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_exploration as ev                                               # noqa: E402  (sets up the other paths)
import matplotlib                                                           # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                             # noqa: E402
from explore_sim import ExploreSim                                          # noqa: E402
from nav.replay import load_snapshot                                        # noqa: E402

# what each policy name means, for the panel titles
LABEL = {"adopted": "today's planner", "freespace1": "freespace everywhere", "exits_best": "exit list, best first",
         "exits_dfs": "exit list, newest first (DFS)", "bug1": "Bug1 (circle every wall)", "bug2": "Bug2 (wall to m-line)",
         "ours": "viewpoints (old default)", "explore_lite": "explore_lite", "navfn": "NavFn allow_unknown"}


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--world", default="w4_structure_decks")
    a.add_argument("--map", default="none", help="uavview / uavscan / none")
    a.add_argument("--goal", required=True)
    a.add_argument("--policies", nargs="*", default=["adopted", "freespace1", "exits_best", "exits_dfs", "bug1", "bug2"])
    a.add_argument("--no-ground", action="store_true")
    a.add_argument("--max-travel", type=float, default=600.0)
    a.add_argument("--out", required=True)
    args = a.parse_args()
    start, goals = ev.scenario(args.world)
    truth = load_snapshot(os.path.join(ev.REPO, f"src/nav/maps/{args.world}_groundtruth.npz"))
    base = load_snapshot(os.path.join(ev.REPO, f"src/nav/maps/{args.world}_{'uavview' if args.map == 'none' else args.map}.npz"))
    if args.map == "none":
        base = ev.no_uav_map(base)
    best = ev.shortest_m(base, truth, start, goals[args.goal])
    n, res = truth.shape[0], truth.resolution
    ext = [truth.center_x - n / 2 * res, truth.center_x + n / 2 * res, truth.center_y - n / 2 * res, truth.center_y + n / 2 * res]
    cols = min(3, len(args.policies))
    rows = (len(args.policies) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5.6 * cols, 5.6 * rows), squeeze=False)
    for ax in axes.ravel()[len(args.policies):]:
        ax.axis("off")
    for ax, name in zip(axes.ravel(), args.policies):
        sim = ExploreSim(base, truth, start, goals[args.goal], policy=ev.POLICIES[name](), overhead_on_switch_only=True,
                         ground_layer=not args.no_ground)
        r = sim.run(max_travel_m=args.max_travel, max_plans=1000)
        p = np.array(r.path)
        ax.imshow(truth.layers["traversability"] < 0.15, origin="lower", extent=ext, cmap="Greys")
        ax.scatter(p[:, 0], p[:, 1], c=np.arange(len(p)), s=1.5, cmap="viridis")
        ax.plot(*start, "s", color="#1a9850", ms=9)
        ax.plot(*goals[args.goal], "*", color="#d73027", ms=16)
        verdict = f"reached, {r.travelled_m:.0f} m" if r.reached else f"NOT reached ({r.travelled_m:.0f} m driven)"
        ax.set_title(f"{LABEL.get(name, name)}\n{verdict}", fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
        print(name, verdict, flush=True)
    uav = {"none": "NO UAV map", "uavscan": "real UAV map", "uavview": "ideal UAV map"}[args.map]
    fig.suptitle(f"{args.world}, goal {args.goal}, {uav}{', no camera layer' if args.no_ground else ''} - "
                 f"shortest possible route {best:.0f} m", fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=70)


if __name__ == "__main__":
    main()
