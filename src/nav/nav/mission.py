"""Mission layer (S-03): a goal list / patrol / return-to-base instead of one `goal_x, goal_y`. Pure Python, no ROS.

A `Mission` hands out ONE goal at a time (`current_goal()`); the caller (mission_node) publishes it to the follower and calls
`update(robot_xy, now)` with the robot's position. States:

    DRIVING   heading for goals[i]
    DWELL     arrived at goals[i], doing its task for `dwell_s` seconds (the payload task of a real mission would go here)
    RETURNING driving to the base (after the last goal in mode "once", after `abort_to_base`, or on a RETURN from the safety layer)
    DONE      arrived at the base (mode once) / finished all laps (patrol with loops)
    ABORTED   `abort()` was called (the robot stays where it is)

Modes: "once" = visit the goals in order, then go home; "patrol" = visit them in order `loops` times (0 = forever), then go home.
A goal the planner cannot reach is reported with `goal_failed()`: it is skipped (and remembered in `skipped`), the mission goes on.
"""
from __future__ import annotations

import math

DRIVING, DWELL, RETURNING, DONE, ABORTED = "DRIVING", "DWELL", "RETURNING", "DONE", "ABORTED"


class Mission:
    def __init__(self, goals, base_xy, mode: str = "once", loops: int = 1, reach_radius_m: float = 0.8, dwell_s: float = 0.0):
        if mode not in ("once", "patrol"):
            raise ValueError(f"mode must be 'once' or 'patrol', got {mode!r}")
        self.goals = [tuple(map(float, g)) for g in goals]
        self.base_xy = tuple(map(float, base_xy))
        self.mode, self.loops = mode, loops
        self.reach_radius_m, self.dwell_s = reach_radius_m, dwell_s
        self.index = 0
        self.lap = 0
        self.skipped: list[tuple[float, float]] = []
        self.visited: list[tuple[float, float]] = []
        self.state = DRIVING if self.goals else RETURNING
        self._dwell_until: float | None = None

    # ---- what to drive to --------------------------------------------------------------------------------------------------
    def current_goal(self):
        """(x, y) the robot should be driving to now (None when DONE / ABORTED)."""
        if self.state in (DRIVING, DWELL):
            return self.goals[self.index]
        if self.state == RETURNING:
            return self.base_xy
        return None

    # ---- events ------------------------------------------------------------------------------------------------------------
    def update(self, robot_xy, now: float) -> bool:
        """Advance the state machine. Returns True when the goal to drive to CHANGED (publish it again)."""
        before = self.current_goal()
        if self.state == DRIVING and math.dist(robot_xy, self.goals[self.index]) <= self.reach_radius_m:
            self.visited.append(self.goals[self.index])
            if self.dwell_s > 0:
                self.state, self._dwell_until = DWELL, now + self.dwell_s
            else:
                self._next_goal()
        elif self.state == DWELL and now >= self._dwell_until:
            self._next_goal()
        elif self.state == RETURNING and math.dist(robot_xy, self.base_xy) <= self.reach_radius_m:
            self.state = DONE
        return self.current_goal() != before

    def _next_goal(self) -> None:
        self.index += 1
        if self.index >= len(self.goals):
            self.index, self.lap = 0, self.lap + 1
            if self.mode == "once" or (self.loops and self.lap >= self.loops):
                self.state = RETURNING
                return
        self.state = DRIVING

    def goal_failed(self) -> None:
        """The planner says the current goal is unreachable: skip it and continue with the next one."""
        if self.state in (DRIVING, DWELL):
            self.skipped.append(self.goals[self.index])
            self._next_goal()
        elif self.state == RETURNING:
            self.state = ABORTED                                  # even the way home is blocked: nothing left to try

    def return_to_base(self) -> None:
        """Safety layer / operator: abandon the remaining goals and go home."""
        if self.state in (DRIVING, DWELL):
            self.state = RETURNING

    def abort(self) -> None:
        self.state = ABORTED
