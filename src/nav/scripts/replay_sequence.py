#!/usr/bin/env python3
"""Replay a RECORDED RUN's map snapshots through the planner, in order.

The live monitor saves a snapshot every few seconds plus the robot's position
at each (`<tag>_summary.json`'s `snap_log`). This feeds them through the
planner one by one - route memory and the anomaly hold carried across steps,
exactly as `planner_node` would over time - and reports how the answer
evolved and how many times the route changed. Compare planners offline:

    python3 src/nav/scripts/replay_sequence.py --run maps/astar3 --goal 3 0 --planner astar --hold 30
    python3 src/nav/scripts/replay_sequence.py --run maps/astar3 --goal 3 0 --planner prm
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
for _p in (_ROOT / "nav", _ROOT / "emap"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from nav.replay import load_snapshot, replay_sequence  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="run prefix: reads <run>_summary.json and <run>_snaps/*.npz")
    ap.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--planner", choices=["astar", "prm"], default="astar")
    ap.add_argument("--hold", type=float, default=0.0, help="anomaly hold seconds")
    ap.add_argument("--no-memory", action="store_true", help="astar: disable route memory")
    ap.add_argument("--frontier-min-width", type=float, default=0.0, help="drop frontier strips thinner than this [m] (0.44 = robot width)")
    ap.add_argument("--explore", action="store_true")
    ap.add_argument("--stuck-feedback", action="store_true", help="penalise the route ahead when the robot stops making progress")
    ap.add_argument("--wait-for-mapping", action="store_true", help="hold speculative targets while the map grows")
    ap.add_argument("--fixed-start", nargs=2, type=float, metavar=("X", "Y"),
                    help="pretend the robot stays here (a closed-loop-ish what-if, instead of its recorded path)")
    ap.add_argument("--no-anomaly", action="store_true")
    ap.add_argument("--no-frontier", action="store_true")
    args = ap.parse_args(argv)

    summary = json.loads(Path(f"{args.run}_summary.json").read_text())
    log = summary["snap_log"]  # [index, t, x, y]
    snaps = [load_snapshot(f"{args.run}_snaps/s{i:03d}.npz") for i, *_ in log]
    steps = replay_sequence(
        snaps, [t for _, t, _, _ in log],
        [tuple(args.fixed_start) if args.fixed_start else (x, y) for _, _, x, y in log], tuple(args.goal),
        planner=args.planner, use_frontier=not args.no_frontier, use_anomaly=not args.no_anomaly,
        hold_sec=args.hold, memory=not args.no_memory, explore=args.explore, wait_for_mapping=args.wait_for_mapping, stuck_feedback=args.stuck_feedback,
        frontier_min_width_m=args.frontier_min_width,
    )  # fmt: skip
    for s in steps:
        status = "no path" if not s.valid else f"{s.n_waypoints:2d} wps{' TENTATIVE' if s.tentative else ''}"
        print(f"t={s.t:6.1f}s robot=({s.start_xy[0]:6.2f},{s.start_xy[1]:6.2f})  {status:20s} {'<- ROUTE CHANGED' if s.route_changed else ''} {s.note}")
    valid = [s for s in steps if s.valid]
    print(f"\\n{len(steps)} steps: {len(valid)} with a path, {len(steps) - len(valid)} without; "
          f"route changed {sum(s.route_changed for s in steps)} time(s) (the first plan counts as one)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
