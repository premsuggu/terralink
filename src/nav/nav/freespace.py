"""Freespace planning: drive to the goal as if the space nobody has observed were open (used under roofs).

WHY (docs/work-docs/nav/step12_exploration.md)
-----------------------------------------------
When the goal is inside a building the UAV saw only the roof of (`nav.overhead`), the interior is "unknown" until the UGV's own
sensors see it. Our frontier-viewpoint exploration (`nav.exploration`) answers "which edge of the unknown is best to LOOK at?"
and walks the facade in short legs, switching target as scores change. Offline (W4, real UAV scan) it did not reach the
dead-end room B at all and needed 16-76 target switches elsewhere.

THE IDEA (the "freespace assumption" of D* Lite, Koenig & Likhachev 2002; also Nav2's `allow_unknown`)
---------------------------------------------------------------------------------------------------
Ask a simpler question: "what is the shortest way to the goal if everything I have not seen is open?" and drive it. As the
lidar reveals walls the next plan bends around them. The TARGET never changes (it is the goal), so there is nothing to dither
between. If even this optimistic question has no answer - every wall around the goal is known - the goal is unreachable on the
current map, and the planner says so (the sealed room was given up after 113 m offline instead of wandering ~500 m).
Offline result on W4's real scan: every feasible goal reached, 0 target switches, room B in 99 m.

    cost of a cell = (1 + keep-away-from-walls + DIFFICULT terrain)          as in our A* (`build_cost_grid`)
                     x unknown_cost        if nobody has observed it         (1 = pure freespace; Nav2 NavFn is ~5)
                     x stuck penalty       near where the robot got stuck    (`nav.stuck_feedback`)

Unknown cost 1 stays the default. EX-08 (2026-11-11) tried 1.2 and 1.5: 1.5 cut W4-D from 48.6 to 27.5 m but made the planner give up much later on unreachable goals (W1-B 75 -> 242 m, +240 m in total over 11 unreachable goals) with the same 19/30 goals reached - net negative, discarded. Much higher values (2-5) toured the building before entering (B: 183-204 m).
Known obstacles are inflated exactly like A*'s first pass; unknown cells are NOT obstacles (they might be open).

WHERE IT IS USED
----------------
Only under a roof (`island_strategy:=freespace` in planner_node): `IslandStrategyPlanner` routes a request to `FreespacePlanner`
when `GoalIslandPlanner` has hidden an island (goal or robot under a roof) and to the normal `GlobalPlanner` otherwise. In the
open a UAV gap can be a pit a 2D lidar cannot see, so "unknown = open" is not assumed there.

Pure NumPy, no ROS.
"""
from __future__ import annotations

import math
import time

import numpy as np

from nav import astar_planner as ap
from nav.stuck_feedback import route_ahead_points


class FreespacePlanner:
    """Plans to the goal through unobserved space. Same `plan(ctx, start, goal, now)` / `last_note` interface as GlobalPlanner.

    stuck: an optional `nav.stuck_feedback.StuckTracker` (shared with the GlobalPlanner, so being stuck is tracked across both).
        Freespace has no blacklist; when Nav2 cannot pass a gap the map calls open (a 0.5-0.6 m door), the tracker penalises the
        stretch ahead of the robot and the next route avoids it.
    """

    def __init__(self, params: ap.PlannerParams | None = None, *, unknown_cost: float = 1.0, stuck=None):
        self.params = params if params is not None else ap.PlannerParams()
        self.unknown_cost = unknown_cost
        self.stuck = stuck
        self._last: ap.AStarResult | None = None
        self.last_note = ""

    def _route_in_progress(self, xy) -> bool:
        """Is the robot supposed to be driving right now? (input to the stuck tracker, as in GlobalPlanner)"""
        r = self._last
        return r is not None and r.valid and r.goal_xy is not None and math.dist(xy, r.goal_xy) > 0.8

    def plan(self, ctx, start_xy, goal_xy, now: float | None = None) -> ap.AStarResult:
        now = time.monotonic() if now is None else now
        notes = []
        multiplier = None
        if self.stuck is not None:
            target = self._last.waypoints[-1] if self._last is not None and self._last.valid and self._last.waypoints else None
            if self.stuck.note_request(now, start_xy, self._route_in_progress(start_xy), target_xy=target):
                failed = list(self._last.waypoints) if self._last is not None and self._last.valid else []
                self.stuck.add_zone(route_ahead_points(failed, start_xy) if failed else [start_xy], now)
                notes.append("stuck: penalised the stretch ahead of the robot")
            multiplier = self.stuck.multiplier_grid(ctx.walkable.shape, ctx.resolution, ctx.center_x, ctx.center_y)

        unknown = np.asarray(ctx.unobserved, dtype=bool)
        walkable = np.asarray(ctx.walkable, dtype=bool)
        # The scene gives us A*'s own start handling: the robot's footprint is free and its "self-pollution" blob (the UAV
        # scan saw the parked UGV as an obstacle) is cleared - same rules as every other plan.
        scene = ap._make_scene(walkable, ctx.resolution, ctx.center_x, ctx.center_y, start_xy, ctx.traversability,
                               None, False, None, False, ctx.force_free, self.params, {}, None)
        if not scene.start_in_grid:
            self.last_note = "freespace: the robot is off the map"
            self._last = ap.AStarResult(valid=False)
            return self._last
        # FULL robot radius + margin (0.27 m), not A*'s rim-reduced value: A* subtracts the 0.1 m lethal rim the UAV map draws
        # round every wall, but walls found by the UGV's lidar under a roof have no such rim (and 0.27 m is what was evaluated).
        p = self.params
        inflation = p.robot_radius_m + p.inflation_margin_m
        known_blocked = ~scene.base_walkable & ~unknown                    # unknown is NOT an obstacle
        clear = ap.clearance_m(known_blocked, ctx.resolution)
        passable = (clear >= inflation - 1e-6) | scene.force_free
        cost = ap.build_cost_grid(clear, inflation, ctx.traversability, None, unknown, p.clearance_weight, p.clearance_range_m,
                                  p.difficult_weight, 1.0, self.unknown_cost)
        if multiplier is not None:
            cost = cost * multiplier

        gr, gc = ap.world_to_grid(goal_xy[0], goal_xy[1], ctx.center_x, ctx.center_y, ctx.resolution, scene.n)
        goal_cell = None
        if scene.in_grid((int(gr), int(gc))):
            goal_cell = ap._nearest_passable(passable, (int(gr), int(gc)), p.goal_snap_m / ctx.resolution)
        found = ap._astar(passable, cost, scene.start, goal_cell) if goal_cell is not None else None
        if found is None:
            notes.append("freespace: no route even through unobserved space - the goal is walled off on the current map")
            self.last_note = "; ".join(notes)
            self._last = ap.AStarResult(valid=False)
            return self._last
        cells, total = found
        wp_cells, kinds = ap._postprocess(cells, passable, passable, None, unknown, cost, ctx.resolution, 0.0, p.merge_gap_m,
                                          p.shorten_lookahead, p.shorten_cost_tolerance)
        through_unknown = any(unknown[c] for c in cells)
        notes.append("freespace: route " + ("through unobserved space" if through_unknown else "over observed ground"))
        self.last_note = "; ".join(notes)
        # Through unknown space = TENTATIVE (has_frontier_segments): the follower keeps re-planning as the lidar fills it in.
        self._last = ap.AStarResult(waypoints=[scene.xy_of(c) for c in wp_cells], valid=True, has_frontier_segments=through_unknown,
                                    has_anomaly_segments=False, segment_kinds=kinds, cost=total, inflation_m=inflation,
                                    goal_xy=scene.xy_of(cells[-1]), mode="freespace")
        return self._last


class IslandStrategyPlanner:
    """Inner planner of a `GoalIslandPlanner`: freespace planning while an island is hidden (goal or robot under a roof),
    the normal `GlobalPlanner` everywhere else.

    `outer` is the GoalIslandPlanner; it sets `last_zone` before calling the inner planner, which is how this class knows.
    `last_note` mirrors whichever planner answered (planner_node logs it).

    no_uav_fraction (EX-07, step13): when the UAV observed less than this fraction of the grid (0.02 = 2 %) there is no UAV map to
        speak of - the UGV explores alone - and freespace planning is used EVERYWHERE, not only under roofs. Offline (10 goals
        in W2/W4/W5, no UAV map) freespace reached 10/10, the viewpoint planner 7/10. With a real UAV map the viewpoint planner
        stays better, so the switch is on the map, not global. 0 disables it.
    """

    def __init__(self, global_planner, freespace: FreespacePlanner | None = None, *, no_uav_fraction: float = 0.02):
        self.no_uav_fraction = no_uav_fraction
        self.global_planner = global_planner
        self.freespace = freespace if freespace is not None else FreespacePlanner(global_planner.params, stuck=global_planner.stuck)
        self.outer = None
        self.last_note = ""

    def plan(self, ctx, start_xy, goal_xy, now=None):
        under_roof = self.outer is not None and self.outer.last_zone is not None
        no_uav = (self.outer is not None and self.no_uav_fraction > 0
                  and getattr(self.outer, "last_uav_fraction", 1.0) < self.no_uav_fraction)
        use = self.freespace if (under_roof or no_uav) else self.global_planner
        result = use.plan(ctx, start_xy, goal_xy, now=now)
        self.last_note = use.last_note
        return result
