#!/usr/bin/env python3
"""Experiment EX-03 (offline): which exploration strategy reaches goals under a roof fastest?

Our planner (A* + frontier viewpoints + memory, `nav.global_planner`) is compared with ports of existing strategies
(`tests/nav/explore_policies.py`: freespace assumption = D* Lite / Nav2 `allow_unknown`, and explore_lite's frontier rule)
in the offline simulator `tests/nav/explore_sim.py`: same UAV map, same simulated lidar (the live floor map's own code),
same robot size and speed. Per goal: reached?, metres driven, number of plans, number of target switches (> 2 m apart),
simulated time (driving at 0.5 m/s + ~10 s per NEW target).

What the simulator does NOT model (so a live run is still the final check): Nav2's local planner and its costmap, the
UGV ground layer from the camera, the `plan_blocked` event re-plan, wheel slip.

Usage:
  python3 eval_exploration.py                       # all policies, W4 goals A-F, both maps
  python3 eval_exploration.py --policies ours freespace1 --goals B --maps uavscan
  python3 eval_exploration.py --world w5_door_gauntlet --maps uavview --policies ours island_freespace
"""
from __future__ import annotations

import argparse
import collections
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for p in (os.path.join(REPO, "src", "nav"), os.path.join(REPO, "src", "emap"), os.path.join(REPO, "tests", "nav")):
    sys.path.insert(0, p)
import explore_policies as ep                                               # noqa: E402
import explore_wallfollow as ew                                             # noqa: E402
from nav import astar_planner as ap                                         # noqa: E402
from explore_sim import ExploreSim                                          # noqa: E402
from nav.overhead import GoalIslandPlanner                                  # noqa: E402
from nav.replay import load_snapshot                                        # noqa: E402

sys.path.insert(0, HERE)                                                    # for worldgen.worlds.<world>


def scenario(world):
    """(start, {goal name: xy}) from the world's MARKERS (worldgen/worlds/<world>.py): "S" is the start, the rest are goals."""
    import importlib
    markers = importlib.import_module(f"worldgen.worlds.{world}").MARKERS
    start = next(xy for name, xy, *_ in markers if name == "S")
    return start, {name: xy for name, xy, *_ in markers if name != "S"}


# W4 "structure decks" (the hidden-interior world): kept as module constants for quick interactive use.
START, GOALS = scenario("w4_structure_decks")

def adopted(no_uav_fraction=0.02):
    """planner_node with `island_strategy:=freespace`: GoalIslandPlanner(IslandStrategyPlanner(GlobalPlanner)), GlobalPlanner set up as
    ExploreSim sets up ours (memory, exploration, stuck feedback, standoff 0.8, blocks 3 m)."""
    from nav.astar_planner import PlannerParams
    from nav.freespace import IslandStrategyPlanner
    from nav.global_planner import GlobalPlanner
    gp = GlobalPlanner(PlannerParams(), enable_memory=True, enable_exploration=True, stuck_feedback=True,
                       viewpoint_standoff_m=0.8, viewpoint_block_m=3.0)
    inner = IslandStrategyPlanner(gp, no_uav_fraction=no_uav_fraction)
    outer = GoalIslandPlanner(inner)
    inner.outer = outer
    return outer


# name -> factory returning the `policy` for ExploreSim (None = our planner, which ExploreSim builds itself)
POLICIES = {
    "ours": lambda: None,
    "island_freespace": lambda: ep.island_freespace(),                                  # freespace under a roof only (test port)
    "adopted_underroof": lambda: adopted(no_uav_fraction=0.0),                          # freespace only under roofs (the step12 behaviour)
    "adopted": lambda: adopted(),                                                       # the production nav/freespace.py, as planner_node wires it
    "freespace1": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=1.0)),     # D* Lite freespace assumption
    "freespace12": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=1.2)),
    "freespace15": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=1.5)),
    "freespace2": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=2.0)),
    "navfn": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=5.0)),          # Nav2 NavFn allow_unknown
    "explore_lite": lambda: GoalIslandPlanner(ep.ExploreLitePlanner()),
    # 2026-10-29, wall following / exit list (tests/nav/explore_wallfollow.py, step13 doc)
    "bug1": lambda: GoalIslandPlanner(ew.BugPlanner("bug1")),
    "bug2": lambda: GoalIslandPlanner(ew.BugPlanner("bug2")),
    "exits_best": lambda: GoalIslandPlanner(ew.ExitsPlanner("best")),
    "exits_dfs": lambda: GoalIslandPlanner(ew.ExitsPlanner("dfs")),
}


def no_uav_map(base):
    """The same grid with NOTHING observed: the UGV explores with its own lidar (and camera) only."""
    from nav.replay import MapSnapshot
    layers = {k: np.array(v, copy=True) for k, v in base.layers.items()}
    layers["is_valid"][:] = 0
    layers["traversability"][:] = 1.0
    layers["elevation"][:] = 0.0
    return MapSnapshot(layers=layers, resolution=base.resolution, center_x=base.center_x, center_y=base.center_y)


def shortest_m(base, truth, start, goal):
    """Length of the shortest route on the TRUE, fully known map (same inflation as the planners): the yardstick."""
    from explore_sim import resample
    from nav.replay import MapSnapshot
    layers = {k: resample(truth.layers[k], truth, base, 0.0 if k == "traversability" else 0.0).astype(np.float32)
              for k in ("elevation", "traversability")}
    layers["is_valid"] = np.ones(base.shape, np.float32)
    layers["variance"] = np.full(base.shape, 0.01, np.float32)
    snap = MapSnapshot(layers=layers, resolution=base.resolution, center_x=base.center_x, center_y=base.center_y)
    sim = ExploreSim(snap, truth, start, goal)
    ctx = sim.make_ctx(layers)
    fs = ep.FreespacePlanner()
    passable, _, _, s = fs.grids(ctx, start)
    g = ep._snap(passable, ep._cell(goal, ctx), fs.goal_snap_m / ctx.resolution)
    # uniform cost = the SHORTEST route (the planners' own cost also pays for keeping away from walls)
    found = None if g is None else ap._astar(passable, np.ones(passable.shape), s, g)
    if found is None:
        return float("nan")
    c = np.array(found[0], float)
    return float(np.hypot(*np.diff(c, axis=0).T).sum() * base.resolution)


def switches(targets, min_jump_m=2.0):
    """How often the plan's final point moved by more than `min_jump_m` (a new target, not a re-plan)."""
    return sum(1 for a, b in zip(targets, targets[1:]) if abs(a[1] - b[1]) + abs(a[2] - b[2]) > min_jump_m)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--world", default="w4_structure_decks")
    a.add_argument("--policies", nargs="*", default=list(POLICIES))
    a.add_argument("--goals", default=None, help="goal names, e.g. ABF (default: all of the world's goals)")
    a.add_argument("--maps", nargs="*", default=["uavview", "uavscan"],
                   help="uavview / uavscan / none (= no UAV map at all, the UGV explores alone)")
    a.add_argument("--replan-m", type=float, default=2.0, help="metres driven between re-plans")
    a.add_argument("--max-travel", type=float, default=600.0)
    a.add_argument("--max-plans", type=int, default=1000)
    a.add_argument("--no-ground", action="store_true", help="simulate WITHOUT the UGV camera ground layer (live default is off)")
    args = a.parse_args()
    start, goals = scenario(args.world)
    truth = load_snapshot(os.path.join(REPO, f"src/nav/maps/{args.world}_groundtruth.npz"))
    print("map      policy        goal reached driven_m shortest_m ratio plans switches sim_s  wall_s  last note")
    best = {}
    for m in args.maps:
        path = os.path.join(REPO, f"src/nav/maps/{args.world}_{'uavview' if m == 'none' else m}.npz")
        if not os.path.exists(path):
            print(f"{m}: no map {path} - skipped")
            continue
        base = load_snapshot(path)
        if m == "none":
            base = no_uav_map(base)
        for name in args.policies:
            for g in [g for g in (args.goals or "".join(goals))]:
                if g not in best:
                    best[g] = shortest_m(base, truth, start, goals[g])
                t0 = time.time()
                sim = ExploreSim(base, truth, start, goals[g], replan_every_m=args.replan_m, policy=POLICIES[name](),
                                 overhead_on_switch_only=True, ground_layer=not args.no_ground)
                r = sim.run(max_travel_m=args.max_travel, max_plans=args.max_plans)
                kinds = collections.Counter(k for *_, k in r.targets)
                print(f"{m:8s} {name:13s} {g}    {str(r.reached):5s}  {r.travelled_m:7.1f} {best[g]:9.1f} "
                      f"{r.travelled_m / best[g] if r.reached else float('nan'):5.1f} {r.plans:5d} {switches(r.targets):8d} "
                      f"{r.seconds:6.0f} {time.time() - t0:6.0f}  {(r.notes[-1] if r.notes else '')[-70:]}  {dict(kinds)}", flush=True)


if __name__ == "__main__":
    main()
