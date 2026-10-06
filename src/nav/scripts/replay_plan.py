#!/usr/bin/env python3
"""Plan on a saved map snapshot - no simulator, no UAV flight, no ROS needed.

    python3 src/nav/scripts/replay_plan.py --snapshot maps/tunnel_test.npz \\
        --start -3 0 --goal 3 0 --anomaly --frontier --explore --png /tmp/plan.png

Prints the waypoints, which segments are uncertain, and route-quality numbers
(length vs straight line, turning, clearance), and optionally draws the route.
Use `--planner prm` to see the old planner's route on the same map.

Snapshots come from `save_map_snapshot.py` (one live run, once).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Make `nav` and `emap` importable straight from a source checkout.
_ROOT = Path(__file__).resolve().parents[2]
for _p in (_ROOT / "nav", _ROOT / "emap"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from nav import astar_planner  # noqa: E402
from nav.replay import build_masks, load_snapshot, path_metrics, render_png, run_plan  # noqa: E402
from nav.resolved_regions import ResolvedRegion  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--snapshot", required=True, help=".npz from save_map_snapshot.py")
    ap.add_argument("--start", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--planner", choices=["astar", "prm"], default="astar")
    ap.add_argument("--frontier", action="store_true", help="allow tentative routes through unobserved cells")
    ap.add_argument("--anomaly", action="store_true", help="allow tentative routes through suspected-anomaly cells")
    ap.add_argument("--frontier-min-width", type=float, default=0.0, help="drop frontier strips thinner than this [m] (0.44 = robot width)")
    ap.add_argument("--explore", action="store_true", help="astar: go to a frontier viewpoint if no route exists")
    ap.add_argument(
        "--resolved", nargs=5, type=float, action="append", metavar=("X0", "Y0", "X1", "Y1", "PASSABLE"),
        help="a voxel-check verdict: box + 1 (passable) / 0 (blocked); repeatable",
    )  # fmt: skip
    ap.add_argument("--standoff", type=float, default=None, help="standoff distance before uncertain stretches [m]")
    ap.add_argument("--png", help="write a picture of the map and route here")
    ap.add_argument("--crop", nargs=4, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="zoom the picture to this window [m] (default: the observed area)")
    args = ap.parse_args(argv)

    snap = load_snapshot(args.snapshot)
    resolved = [ResolvedRegion(x0, y0, x1, y1, passable=bool(p >= 0.5)) for x0, y0, x1, y1, p in (args.resolved or [])]
    masks = build_masks(snap, use_frontier=args.frontier or args.explore, use_anomaly=args.anomaly, resolved=resolved,
                        frontier_min_width_m=args.frontier_min_width)
    params = astar_planner.PlannerParams(**({"standoff_m": args.standoff} if args.standoff is not None else {}))

    start, goal = tuple(args.start), tuple(args.goal)
    result = run_plan(snap, masks, start, goal, planner=args.planner, explore=args.explore, params=params)

    print(f"map: {snap.shape[0]}x{snap.shape[1]} cells @ {snap.resolution} m, centre ({snap.center_x}, {snap.center_y})")
    if not result.valid:
        print("RESULT: no path")
    else:
        kind = "EXPLORATION viewpoint" if getattr(result, "is_exploration", False) else "route to goal"
        print(f"RESULT: {kind}, {len(result.waypoints)} waypoints, "
              f"tentative: frontier={result.has_frontier_segments} anomaly={result.has_anomaly_segments}")
        kinds = getattr(result, "segment_kinds", [])
        for i, (x, y) in enumerate(result.waypoints):
            seg = f"  --{kinds[i]}-->" if i < len(kinds) else ""
            print(f"  [{i}] ({x:6.2f}, {y:6.2f}){seg}")
        m = path_metrics(result.waypoints, snap, masks, kinds)
        print(f"metrics: length {m.length_m:.2f} m (straight {m.straight_line_m:.2f} m, detour x{m.detour_ratio:.2f}), "
              f"turning {m.total_turn_deg:.0f} deg total / {m.sharpest_turn_deg:.0f} deg sharpest, "
              f"min clearance {m.min_clearance_m:.2f} m")
    if args.png:
        render_png(args.png, snap, masks, start, goal, result,
                   title=f"{args.planner}: {Path(args.snapshot).name}", crop=tuple(args.crop) if args.crop else None)
        print(f"wrote {args.png}")
    return 0 if result.valid else 1


if __name__ == "__main__":
    sys.exit(main())
