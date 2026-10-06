"""Route memory: keep driving the route we already chose unless there is a
real reason to change it.

THE PROBLEM THIS SOLVES
-----------------------
`/elevation_map` keeps changing cell-by-cell from sensor fusion even where
nothing new was observed (see the long comment in `planner_node.py`). A
planner that re-plans from scratch on every request will therefore keep
returning slightly different routes, and whenever two routes are almost equal
in cost it can flip between them - the UGV starts toward one, then the other,
then back. That is exactly the "useless, random-looking motion" this module
exists to stop.

THE RULE
--------
Given the route we are currently following (`stored`) and a freshly planned one
(`new`), judged against the CURRENT map from the robot's CURRENT position:

    stored route no longer valid                  -> switch to new
    stored is tentative, new one is confirmed     -> switch (a safe route appeared)
    new is clearly cheaper (by `switch_margin`)   -> switch
    otherwise                                      -> KEEP the stored route

"Clearly cheaper" is a relative margin (default 15 %), so tiny cost wobbles
from map noise never flip the route, while a genuinely better one still wins.

WHY THE OLD WAYPOINT LIST IS RETURNED UNCHANGED WHEN KEPT
---------------------------------------------------------
`waypoint_follower` decides whether a plan changed with
`_waypoints_effectively_equal` - same number of waypoints, each within 5 cm.
If we returned a copy trimmed to start at the robot's current position, the
first waypoint would differ on every call and the follower would treat each
reply as a NEW plan, re-publish a goal, and preempt Nav2 mid-drive. Returning
the original list makes a kept route a true no-op for the follower.

Pure Python, no ROS and no NumPy - the planner node wires it up.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable

from nav.astar_planner import AStarResult

# evaluate(waypoints, mode, inflation_m) -> (still_valid, cost, per-segment kinds)
Evaluator = Callable[[list[tuple[float, float]], str, float], tuple[bool, float, list[str]]]


@dataclass
class MemoryDecision:
    """What to hand back to the caller, and why (the reason is for logs)."""

    result: AStarResult
    kept: bool
    reason: str


def trim_path(
    waypoints: list[tuple[float, float]], xy: tuple[float, float]
) -> list[tuple[float, float]]:
    """The part of `waypoints` still ahead of the robot at `xy`.

    Finds the point on the polyline closest to `xy` and returns
    [that point] + every waypoint after the segment it lies on. This is for
    JUDGING the old route (validity and remaining cost) - not what is sent to
    the follower (see the module docstring).
    """
    if len(waypoints) < 2:
        return list(waypoints)
    best_seg, best_point, best_d = 0, waypoints[0], math.inf
    for i in range(len(waypoints) - 1):
        (x0, y0), (x1, y1) = waypoints[i], waypoints[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg_len2 = dx * dx + dy * dy
        t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((xy[0] - x0) * dx + (xy[1] - y0) * dy) / seg_len2))
        px, py = x0 + t * dx, y0 + t * dy
        d = math.hypot(xy[0] - px, xy[1] - py)
        # `<` (not `<=`): on a tie keep the EARLIER segment, so a robot exactly
        # at a shared corner is not considered to have already passed it.
        if d < best_d - 1e-12:
            best_seg, best_point, best_d = i, (px, py), d
    ahead = waypoints[best_seg + 1 :]
    if ahead and math.hypot(ahead[0][0] - best_point[0], ahead[0][1] - best_point[1]) < 1e-9:
        return [best_point] + ahead[1:] if len(ahead) > 1 else [best_point, ahead[0]]
    return [best_point] + ahead


def _is_tentative(kinds: list[str]) -> bool:
    return any(k != "free" for k in kinds)


class PlanMemory:
    """Remembers the route currently being followed for ONE goal."""

    def __init__(self, switch_margin: float = 0.15, goal_tolerance_m: float = 0.1):
        self.switch_margin = switch_margin
        self.goal_tolerance_m = goal_tolerance_m
        self._stored: AStarResult | None = None
        self._goal_xy: tuple[float, float] | None = None

    def clear(self) -> None:
        self._stored = None
        self._goal_xy = None

    @property
    def stored(self) -> AStarResult | None:
        return self._stored

    def _adopt(self, new: AStarResult, goal_xy, reason: str) -> MemoryDecision:
        if new.valid:
            self._stored, self._goal_xy = new, goal_xy
        else:
            self.clear()
        return MemoryDecision(new, kept=False, reason=reason)

    def decide(
        self,
        new: AStarResult,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        evaluate: Evaluator,
    ) -> MemoryDecision:
        """Pick between the stored route and `new` (see module docstring)."""
        stored = self._stored
        if (
            stored is None
            or self._goal_xy is None
            or math.dist(self._goal_xy, goal_xy) > self.goal_tolerance_m
        ):
            return self._adopt(new, goal_xy, "first plan for this goal")

        # Judge both routes on the same grids: those of the new plan's own
        # mode/inflation if it found one, else the stored plan's.
        mode, infl = (new.mode, new.inflation_m) if new.valid else (stored.mode, stored.inflation_m)
        ahead = trim_path(stored.waypoints, start_xy)
        old_ok, old_cost, old_kinds = evaluate(ahead, mode, infl)

        if not new.valid:
            if old_ok:
                return self._keep(stored, old_cost, old_kinds, "no new route found; stored one still valid")
            return self._adopt(new, goal_xy, "stored route invalid and no new route")

        if not old_ok:
            return self._adopt(new, goal_xy, "stored route no longer valid")

        new_ok, new_cost, new_kinds = evaluate(new.waypoints, mode, infl)
        if not new_ok:  # should not happen (the planner checked it) - trust the planner's own numbers
            new_cost, new_kinds = new.cost, new.segment_kinds

        if _is_tentative(old_kinds) and not _is_tentative(new_kinds):
            return self._adopt(new, goal_xy, "a confirmed route replaced a tentative one")
        if new_cost < old_cost * (1.0 - self.switch_margin):
            return self._adopt(new, goal_xy, f"new route is clearly cheaper ({new_cost:.1f} vs {old_cost:.1f})")
        return self._keep(stored, old_cost, old_kinds, f"kept stored route ({old_cost:.1f} vs new {new_cost:.1f})")

    def _keep(self, stored: AStarResult, ahead_cost: float, ahead_kinds: list[str], reason: str) -> MemoryDecision:
        # Same waypoints (see module docstring), but flags describe what is
        # still AHEAD: once the robot has passed the tentative stretch the plan
        # is no longer tentative.
        kept = replace(
            stored,
            cost=ahead_cost,
            has_anomaly_segments="anomaly" in ahead_kinds,
            has_frontier_segments="frontier" in ahead_kinds,
        )
        self._stored = kept
        return MemoryDecision(kept, kept=True, reason=reason)
