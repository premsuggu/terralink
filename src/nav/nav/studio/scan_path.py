"""Auto-scan: a lawnmower sweep the studio server flies by itself so the UAV maps the site without anybody holding a key. No ROS here.

Two small pieces, both pure functions (so the tests need no simulator):
  * `lawnmower(bounds, spacing)`  - the list of corner points of a back-and-forth sweep ("mowing the lawn") that covers a rectangle;
  * `autoscan_step(...)`          - one control step: given where the UAV is and which corner it is heading for, the velocity to command.

Why a lawnmower: the UAV's downward camera sees a strip of ground (about 1.2 x the flight height wide at 60 deg field of view). Flying parallel
lanes a little closer together than that strip leaves no gaps. Why slow and low: a fast or high scan leaves a speckled map the planner
cannot route through (step 16); the defaults here are the settings that gave a clean map in the careful live run.
"""
from __future__ import annotations

import math

Bounds = tuple[float, float, float, float]            # (xmin, ymin, xmax, ymax)
Pose = tuple[float, float, float, float]              # (x, y, z, yaw)


def lawnmower(bounds: Bounds, spacing: float = 5.0) -> list[tuple[float, float]]:
    """Corner points of a sweep over `bounds`: lanes run along y, are `spacing` apart in x, and alternate direction so the UAV never flies empty.

    Example: bounds (0,0,10,10) with spacing 5 -> lanes at x = 0, 5, 10:
        (0,0) -> (0,10) -> (5,10) -> (5,0) -> (10,0) -> (10,10)
    The last lane is always exactly at xmax so the far edge is covered even when the width is not a multiple of `spacing`.
    """
    x0, y0, x1, y1 = bounds
    if x1 <= x0 or y1 <= y0 or spacing <= 0:
        raise ValueError("lawnmower needs a non-empty rectangle and a positive spacing")
    n_lanes = max(2, math.ceil((x1 - x0) / spacing) + 1)
    pts: list[tuple[float, float]] = []
    for i in range(n_lanes):
        x = x0 + (x1 - x0) * i / (n_lanes - 1)         # evenly spread, so lanes are never farther apart than `spacing`
        ys = (y0, y1) if i % 2 == 0 else (y1, y0)
        pts += [(x, ys[0]), (x, ys[1])]
    return pts


def autoscan_step(pose: Pose, target: tuple[float, float], alt: float, cruise: float, max_z: float,
                  arrive: float = 0.8) -> tuple[bool, tuple[float, float, float, float]]:
    """One control step of the sweep. Returns (reached this corner?, body-frame velocity (vx, vy, vz, wz) in m/s).

    Horizontal: head straight for `target` at `cruise` m/s, slowing down near it (so it does not overshoot). Vertical: hold `alt`. Yaw: hold.
    The simulator's /cmd_vel is body-frame, so the world-frame direction is rotated by -yaw.
    """
    x, y, z, yaw = pose
    dx, dy = target[0] - x, target[1] - y
    dist = math.hypot(dx, dy)
    vz = max(-max_z, min(max_z, 0.8 * (alt - z)))
    if z < alt - 0.8:
        return False, (0.0, 0.0, vz, 0.0)              # still climbing: no sideways motion until near the sweep height (containers are tall)
    if dist <= arrive:
        return True, (0.0, 0.0, vz, 0.0)
    speed = min(cruise, max(0.3, 0.8 * dist))
    wx, wy = dx / dist * speed, dy / dist * speed
    c, s = math.cos(yaw), math.sin(yaw)
    return False, (c * wx + s * wy, -s * wx + c * wy, vz, 0.0)
