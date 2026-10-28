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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for p in (os.path.join(REPO, "src", "nav"), os.path.join(REPO, "src", "emap"), os.path.join(REPO, "tests", "nav")):
    sys.path.insert(0, p)
import explore_policies as ep                                               # noqa: E402
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

def adopted():
    """planner_node with `island_strategy:=freespace`: GoalIslandPlanner(IslandStrategyPlanner(GlobalPlanner)), GlobalPlanner set up as
    ExploreSim sets up ours (memory, exploration, stuck feedback, standoff 0.8, blocks 3 m)."""
    from nav.astar_planner import PlannerParams
    from nav.freespace import IslandStrategyPlanner
    from nav.global_planner import GlobalPlanner
    gp = GlobalPlanner(PlannerParams(), enable_memory=True, enable_exploration=True, stuck_feedback=True,
                       viewpoint_standoff_m=0.8, viewpoint_block_m=3.0)
    inner = IslandStrategyPlanner(gp)
    outer = GoalIslandPlanner(inner)
    inner.outer = outer
    return outer


# name -> factory returning the `policy` for ExploreSim (None = our planner, which ExploreSim builds itself)
POLICIES = {
    "ours": lambda: None,
    "island_freespace": lambda: ep.island_freespace(),                                  # freespace under a roof only (test port)
    "adopted": lambda: adopted(),                                                       # the production nav/freespace.py, as planner_node wires it
    "freespace1": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=1.0)),     # D* Lite freespace assumption
    "freespace2": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=2.0)),
    "navfn": lambda: GoalIslandPlanner(ep.FreespacePlanner(unknown_cost=5.0)),          # Nav2 NavFn allow_unknown
    "explore_lite": lambda: GoalIslandPlanner(ep.ExploreLitePlanner()),
}


def switches(targets, min_jump_m=2.0):
    """How often the plan's final point moved by more than `min_jump_m` (a new target, not a re-plan)."""
    return sum(1 for a, b in zip(targets, targets[1:]) if abs(a[1] - b[1]) + abs(a[2] - b[2]) > min_jump_m)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--world", default="w4_structure_decks")
    a.add_argument("--policies", nargs="*", default=list(POLICIES))
    a.add_argument("--goals", default=None, help="goal names, e.g. ABF (default: all of the world's goals)")
    a.add_argument("--maps", nargs="*", default=["uavview", "uavscan"])
    a.add_argument("--replan-m", type=float, default=2.0, help="metres driven between re-plans")
    a.add_argument("--max-travel", type=float, default=600.0)
    a.add_argument("--max-plans", type=int, default=400)
    a.add_argument("--no-ground", action="store_true", help="simulate WITHOUT the UGV camera ground layer (live default is off)")
    args = a.parse_args()
    start, goals = scenario(args.world)
    truth = load_snapshot(os.path.join(REPO, f"src/nav/maps/{args.world}_groundtruth.npz"))
    print("map      policy        goal reached driven_m plans switches sim_s  wall_s  last note")
    for m in args.maps:
        path = os.path.join(REPO, f"src/nav/maps/{args.world}_{m}.npz")
        if not os.path.exists(path):
            print(f"{m}: no map {path} - skipped")
            continue
        base = load_snapshot(path)
        for name in args.policies:
            for g in (args.goals or "".join(goals)):
                t0 = time.time()
                sim = ExploreSim(base, truth, start, goals[g], replan_every_m=args.replan_m, policy=POLICIES[name](),
                                 overhead_on_switch_only=True, ground_layer=not args.no_ground)
                r = sim.run(max_travel_m=args.max_travel, max_plans=args.max_plans)
                kinds = collections.Counter(k for *_, k in r.targets)
                print(f"{m:8s} {name:13s} {g}    {str(r.reached):5s}  {r.travelled_m:7.1f} {r.plans:5d} {switches(r.targets):8d} "
                      f"{r.seconds:6.0f} {time.time() - t0:6.0f}  {(r.notes[-1] if r.notes else '')[-70:]}  {dict(kinds)}", flush=True)


if __name__ == "__main__":
    main()
