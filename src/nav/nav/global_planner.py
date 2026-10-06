"""The whole global-planning decision in one place.

`planner_node` (live) and `nav.replay` (offline) both need the same sequence:

    1. plan a route with A*                       (nav.astar_planner)
    2. keep the route we were already driving
       unless the new one is clearly better       (nav.plan_memory)
    3. if there is no route at all, or only a gamble through unobserved space,
       go and LOOK at something instead           (nav.exploration)

Keeping that sequence in ONE class means what the offline replay shows is, by
construction, what the node does - there is no second copy to drift out of
date. This module is pure Python/NumPy; the node only wires parameters and
ROS messages around it.

THE INVESTIGATE-BEFORE-GAMBLING RULE (step 3)
---------------------------------------------
A route that only exists by assuming unobserved cells are free (a "frontier
gamble") is a guess about nothing. A suspected passage - cells the anomaly
detector flagged as the mouth of a tunnel or culvert - is a guess backed by a
specific signature. Seen live: the tunnel's flagged cells were visible from
about 30 s, too incomplete to route through yet, while the planner sent the
UGV off on a frontier gamble around the room instead of simply going to look at
the tunnel (it wandered for ~45 s). So when exploration is on and the best
route is only a frontier gamble, a viewpoint on a suspected passage takes
priority over it. If the best route already goes THROUGH the suspected passage
(an anomaly route) nothing is overridden: the follower will stop at the
standoff point and have the 3D voxel map verify it.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from nav import astar_planner
from nav.exploration import ExplorationMemory, plan_exploration
from nav.plan_memory import PlanMemory
from nav.stuck_feedback import StuckTracker, route_ahead_points


@dataclass
class MapContext:
    """The map as the planner sees it (derived from one `/elevation_map`)."""

    walkable: np.ndarray
    resolution: float
    center_x: float
    center_y: float
    traversability: np.ndarray | None = None
    frontier: np.ndarray | None = None
    anomaly: np.ndarray | None = None
    unobserved: np.ndarray | None = None
    force_free: np.ndarray | None = None
    allow_frontier: bool = False
    allow_anomaly: bool = False
    # True while the map is still being built (see nav.map_growth). Only has
    # an effect on a planner created with `wait_for_mapping=True`.
    mapping_active: bool = False


class GlobalPlanner:
    """Stateful: remembers the route being followed and what exploration has
    already visited, for ONE goal at a time (a new goal resets both)."""

    def __init__(
        self,
        params: astar_planner.PlannerParams | None = None,
        *,
        enable_memory: bool = True,
        switch_margin: float = 0.15,
        enable_exploration: bool = False,
        exploration_gain_weight: float = 1.0,
        wait_for_mapping: bool = False,
        stuck_feedback: bool = False,
        stuck_window_sec: float = 18.0,
    ):
        self.params = params if params is not None else astar_planner.PlannerParams()
        self.enable_memory = enable_memory
        self.enable_exploration = enable_exploration
        self.exploration_gain_weight = exploration_gain_weight
        self.wait_for_mapping = wait_for_mapping
        # Off by default; see nav.stuck_feedback for why a deterministic
        # planner with route memory needs it.
        self.stuck = StuckTracker(window_sec=stuck_window_sec) if stuck_feedback else None
        self._last_result: astar_planner.AStarResult | None = None
        self.memory = PlanMemory(switch_margin=switch_margin)
        self.exploration = ExplorationMemory()
        self._goal_xy: tuple[float, float] | None = None
        self.last_note = ""

    def _astar_kwargs(self, ctx: MapContext, cost_multiplier=None) -> dict:
        return dict(
            cost_multiplier=cost_multiplier,
            traversability=ctx.traversability,
            frontier_mask=ctx.frontier,
            allow_frontier=ctx.allow_frontier,
            anomaly_mask=ctx.anomaly,
            allow_anomaly=ctx.allow_anomaly,
            force_free_mask=ctx.force_free,
            params=self.params,
        )

    def _route_in_progress(self, xy) -> bool:
        """Is the robot supposed to be driving somewhere right now? True if
        the last answer was a valid route - unless it was a viewpoint the robot
        has already reached (then it is legitimately sitting there)."""
        r = self._last_result
        if r is None or not r.valid:
            return False
        if r.is_exploration and r.goal_xy is not None and math.dist(xy, r.goal_xy) <= 0.8:
            return False
        return True

    def plan(self, ctx: MapContext, start_xy, goal_xy, now: float | None = None) -> astar_planner.AStarResult:
        now = time.monotonic() if now is None else now
        if self._goal_xy is None or math.dist(self._goal_xy, goal_xy) > 0.1:
            self.exploration.clear()  # a new goal resets what was already visited
            if self.stuck is not None:
                self.stuck.clear()
        self._goal_xy = goal_xy
        notes: list[str] = []

        multiplier = None
        if self.stuck is not None:
            if self.stuck.note_request(now, start_xy, self._route_in_progress(start_xy)):
                last = self._last_result
                failed = list(last.waypoints) if last is not None and last.valid else []
                self.stuck.add_zone(route_ahead_points(failed, start_xy) if failed else [start_xy], now)
                self.memory.clear()
                notes.append("stuck: dropped the stored route and penalised the stretch ahead of the robot")
            multiplier = self.stuck.multiplier_grid(ctx.walkable.shape, ctx.resolution, ctx.center_x, ctx.center_y)
        kw = self._astar_kwargs(ctx, multiplier)

        result = astar_planner.plan(
            ctx.walkable, ctx.resolution, ctx.center_x, ctx.center_y, start_xy, goal_xy, **kw
        )

        # While the map is still being built, do not commit to SPECULATION: a
        # route that exists only by assuming unobserved space is free is
        # information the next few seconds of scanning will probably replace.
        # Specific signals (confirmed routes, anomaly routes, anomaly
        # viewpoints) are still acted on. Nothing is committed to memory.
        waiting = self.wait_for_mapping and ctx.mapping_active
        if waiting and result.valid and result.has_frontier_segments and not result.has_anomaly_segments:
            result = astar_planner.AStarResult(valid=False)
            notes.append("waiting: only a frontier gamble exists and the map is still growing")

        if self.enable_memory:

            def evaluate(waypoints, mode, inflation_m):
                return astar_planner.evaluate_path(
                    waypoints, ctx.walkable, ctx.resolution, ctx.center_x, ctx.center_y,
                    start_xy, mode, inflation_m, **kw,
                )  # fmt: skip

            decision = self.memory.decide(result, start_xy, goal_xy, evaluate)
            result = decision.result
            notes.append(f"memory: {decision.reason}")

        if self.enable_exploration:
            gamble = result.valid and result.has_frontier_segments and not result.has_anomaly_segments
            if not result.valid or gamble:
                explored = plan_exploration(
                    ctx.walkable, ctx.frontier, ctx.resolution, ctx.center_x, ctx.center_y,
                    start_xy, goal_xy, self.exploration,
                    unobserved_mask=ctx.unobserved,
                    anomaly_mask=ctx.anomaly if ctx.allow_anomaly else None,
                    traversability=ctx.traversability, force_free_mask=ctx.force_free,
                    params=self.params, gain_weight=self.exploration_gain_weight,
                    frontier_viewpoints=not waiting, cost_multiplier=multiplier,
                )  # fmt: skip
                if explored.valid and (not result.valid or explored.exploration_kind == "anomaly"):
                    result = explored
                    notes.append(f"exploring: {explored.exploration_kind} viewpoint")
        self.last_note = "; ".join(notes)
        self._last_result = result
        return result
