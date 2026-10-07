#!/usr/bin/env python3
"""Draw images from runs recorded with `trace_run.py` (no ROS or simulator needed).

One run in detail (track coloured by time, numbered plans, pauses, 3D checks):

    python3 src/nav/scripts/render_run_trace.py detail --run runs/tunnel_astar_1 \\
        --out media/figures/path_comparison/tunnel_astar_run1.png --title "Tunnel - A*"

Planners side by side, runs overlaid (the map comes from the first run's map.npz,
or --map):

    python3 src/nav/scripts/render_run_trace.py compare \\
        --group "PRM=runs/tunnel_prm_1,runs/tunnel_prm_2,runs/tunnel_prm_3" \\
        --group "A*=runs/tunnel_astar_1,runs/tunnel_astar_2,runs/tunnel_astar_3" \\
        --out media/figures/path_comparison/tunnel_prm_vs_astar.png --title "Tunnel world"

`--crop XMIN XMAX YMIN YMAX` frames the view (default: the observed part of the map).
"""
from __future__ import annotations

import argparse
import json
import sys

from nav.replay import load_snapshot
from nav.trace_render import load_run, load_trace, render_comparison_png, render_run_png


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("detail")
    d.add_argument("--run", required=True, help="run directory (trace.json + map.npz)")
    d.add_argument("--out", required=True)
    d.add_argument("--title", default="")
    d.add_argument("--map", default=None, help="override the background snapshot")
    d.add_argument("--crop", nargs=4, type=float, default=None)
    c = sub.add_parser("compare")
    c.add_argument("--group", action="append", required=True, help='"Label=dir1,dir2,..." (repeat per planner)')
    c.add_argument("--out", required=True)
    c.add_argument("--title", default="")
    c.add_argument("--map", default=None, help="background snapshot (default: first run's map.npz)")
    c.add_argument("--crop", nargs=4, type=float, default=None)
    c.add_argument("--metrics-json", default=None, help="also write the per-run numbers here")
    args = ap.parse_args(argv)

    if args.cmd == "detail":
        trace, snap = load_run(args.run)
        if args.map:
            snap = load_snapshot(args.map)
        m = render_run_png(args.out, trace, snap, title=args.title, crop=tuple(args.crop) if args.crop else None)
        print(json.dumps(m, indent=1, default=str))
        return

    groups, snap = {}, None
    for spec in args.group:
        label, _, dirs = spec.partition("=")
        traces = []
        for run_dir in dirs.split(","):
            tr, s = load_run(run_dir)
            traces.append(tr)
            snap = snap or s
        groups[label] = traces
    if args.map:
        snap = load_snapshot(args.map)
    m = render_comparison_png(args.out, groups, snap, title=args.title, crop=tuple(args.crop) if args.crop else None)
    if args.metrics_json:
        with open(args.metrics_json, "w") as fh:
            json.dump(m, fh, indent=1, default=str)
    for label, ms in m.items():
        for i, x in enumerate(ms, 1):
            print(label, i, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in x.items()})


if __name__ == "__main__":
    sys.exit(main())
