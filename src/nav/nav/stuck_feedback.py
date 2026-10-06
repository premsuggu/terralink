"""Feedback for a deterministic planner: "the robot is not getting anywhere".

WHY A DETERMINISTIC PLANNER NEEDS THIS
--------------------------------------
`prm_planner` was random: when the robot got stuck and the follower asked for
a fresh plan, it might get a different route by luck. A deterministic planner
with route memory does the opposite - asked again from the same spot with the
same map, it gives the SAME route back ("kept stored route"), so the follower's
stuck-recovery replans change nothing. Seen live: ~200 s stuck against the
same obstacle with the planner calmly returning the same plan every 20 s.

The planner cannot see Nav2, but it can see the one thing that matters: it
keeps being asked for a route from (almost) the same place. If it has handed
out a route and the robot has not moved for `window_sec`, treat the robot as
stuck here:

  * the planner drops its memory of the route, and
  * the area around the robot gets a temporary extra cost, so a route that
    avoids it wins if one exists.

The penalty expires after `zone_sec` so a one-off hiccup does not permanently
close a corridor. If there is NO alternative (a single corridor), the penalty
changes nothing - recovery from that is the local controller's job, not the
global planner's.

Pure Python + NumPy; time is passed in so tests need no real clock.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from nav.plan_memory import trim_path


@dataclass
class _Zone:
    points: list[tuple[float, float]]  # the stretch of route that failed
    expires_at: float


def route_ahead_points(
    waypoints: list[tuple[float, float]],
    xy: tuple[float, float],
    length_m: float = 1.5,
    step_m: float = 0.2,
) -> list[tuple[float, float]]:
    """Points spaced `step_m` apart along the next `length_m` of `waypoints`
    from the robot's position - the stretch of the route it failed to drive.
    A penalty around the robot ALONE would be a symmetric blob that favours no
    alternative; the obstacle is AHEAD, so that is where the penalty goes."""
    path = trim_path(waypoints, xy)
    out = [path[0]] if path else [xy]
    travelled, next_mark = 0.0, step_m
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        while next_mark - travelled <= seg and next_mark <= length_m:
            t = (next_mark - travelled) / seg if seg > 0 else 0.0
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            next_mark += step_m
        travelled += seg
        if travelled >= length_m:
            break
    return out


class StuckTracker:
    def __init__(
        self,
        window_sec: float = 18.0,
        move_radius_m: float = 0.25,
        zone_radius_m: float = 0.5,
        zone_sec: float = 60.0,
        zone_multiplier: float = 6.0,
    ):
        self.window_sec = window_sec
        self.move_radius_m = move_radius_m
        self.zone_radius_m = zone_radius_m
        self.zone_sec = zone_sec
        self.zone_multiplier = zone_multiplier
        self._dwell: tuple[float, tuple[float, float]] | None = None
        self.zones: list[_Zone] = []

    def clear(self) -> None:
        self._dwell = None
        self.zones.clear()

    def note_request(self, now: float, xy: tuple[float, float], route_in_progress: bool) -> bool:
        """Call on every planning request. `route_in_progress`: the PREVIOUS
        answer was a route the robot should be driving (not "no path", and not
        a viewpoint it has already reached). Returns True the moment the robot
        is judged stuck (once per stuck episode)."""
        self.zones = [z for z in self.zones if z.expires_at > now]
        if self._dwell is None or math.dist(xy, self._dwell[1]) > self.move_radius_m:
            self._dwell = (now, xy)  # moved (or first request): restart the clock here
            return False
        if route_in_progress and now - self._dwell[0] >= self.window_sec:
            self._dwell = (now, xy)  # a fresh window for the next episode
            return True
        return False

    def add_zone(self, points: list[tuple[float, float]], now: float) -> None:
        """Penalise the cells near `points` until `zone_sec` from now."""
        self.zones.append(_Zone(points=list(points), expires_at=now + self.zone_sec))

    def multiplier_grid(
        self, shape: tuple[int, int], resolution: float, center_x: float, center_y: float
    ) -> np.ndarray | None:
        """Cost multiplier grid for the active zones, or None if there are none."""
        if not self.zones:
            return None
        n = shape[0]
        rows, cols = np.indices(shape)
        x = (cols - n / 2.0) * resolution + center_x
        y = (rows - n / 2.0) * resolution + center_y
        grid = np.ones(shape, dtype=np.float64)
        for z in self.zones:
            for px, py in z.points:
                inside = np.hypot(x - px, y - py) <= self.zone_radius_m
                grid[inside] = self.zone_multiplier
        return grid
