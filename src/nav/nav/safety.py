"""Safety layer (S-01) and communication-loss policy (S-02): a pure-Python monitor, no ROS.

WHAT IT DECIDES
---------------
`SafetyMonitor.update(...)` looks at what the robot knows RIGHT NOW and returns a `Verdict`:

    OK      drive on
    STOP    stand still (an obstacle too close, a dangerous tilt, outside the fence); clears by itself once the condition has
            been gone for `clear_after_s` (so a person stepping away lets the robot continue)
    RETURN  stop what you are doing and go home (battery budget, UAV link lost for too long)

Checks, in priority order (a higher one hides the lower ones):
  1. geofence      - the robot is outside the allowed rectangle                      -> STOP   (a fence is a hard limit)
  2. tilt          - |roll| or |pitch| above `max_tilt_deg` (about to tip / a cliff)  -> STOP
  3. obstacle      - the lidar sees something closer than `min_range_m` in the way  -> STOP   (collision guard)
  4. battery       - energy budget left < what the way home needs (x margin)         -> RETURN (battery-aware return)
  5. link          - no UAV map for `link_timeout_s`                                 -> flag only, then RETURN after
                     `link_return_s` (S-02: until then the UGV simply FINISHES ON THE LAST MAP - the planner keeps its last map
                     and every plan is made on it, so nothing has to change; this class only tracks how stale it is)

The battery is a simple energy model (a budget in metres of driving, because the simulator has no battery): remaining = budget -
distance driven. Return when remaining < (straight-line distance to base) x `return_margin` + `reserve_m`. On a real robot replace
`distance_driven` by the battery state of charge (same rule).

Pure Python / NumPy so it is unit-tested without ROS (tests/nav/test_safety.py); `safety_node.py` is the thin ROS wrapper.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

OK, STOP, RETURN = "OK", "STOP", "RETURN"


@dataclass
class SafetyParams:
    fence: tuple[float, float, float, float] | None = None   # (xmin, ymin, xmax, ymax) in the map frame; None = no fence
    max_tilt_deg: float = 35.0          # the vehicle climbs 35 deg ramps (X-A1) but tips on side slopes earlier: stop above this
    min_range_m: float = 0.30           # lidar range (in the direction of travel) below which the robot stops
    clear_after_s: float = 1.5          # the stop condition must be gone this long before driving on (no flicker)
    battery_budget_m: float | None = None   # driving distance available (None = battery check off)
    return_margin: float = 1.3          # need this many times the straight-line distance home
    reserve_m: float = 5.0              # plus this much spare
    link_timeout_s: float = 5.0         # no UAV map message for this long = link lost (flag)
    link_return_s: float = 120.0        # link lost for this long = go home (None = never)


@dataclass
class Verdict:
    action: str = OK
    reasons: list[str] = field(default_factory=list)
    link_lost: bool = False
    battery_left_m: float | None = None


class SafetyMonitor:
    def __init__(self, params: SafetyParams | None = None, base_xy=(0.0, 0.0)):
        self.p = params if params is not None else SafetyParams()
        self.base_xy = base_xy
        self._driven = 0.0
        self._last_xy = None
        self._stop_clear_since: float | None = None     # when the last stop condition disappeared
        self._stopping = False
        self._last_map_t: float | None = None
        self._link_lost_since: float | None = None

    # ---- inputs that arrive on their own schedule -------------------------------------------------------------------------
    def map_received(self, now: float) -> None:
        """Call whenever a UAV map message arrives."""
        self._last_map_t = now
        self._link_lost_since = None

    # ---- the decision ------------------------------------------------------------------------------------------------------
    def update(self, now: float, xy, roll_deg: float = 0.0, pitch_deg: float = 0.0, front_range_m: float | None = None) -> Verdict:
        p = self.p
        if self._last_xy is not None:
            self._driven += math.dist(xy, self._last_xy)
        self._last_xy = xy
        v = Verdict()

        # --- hard stops: fence, tilt, obstacle ---
        stop = []
        if p.fence is not None:
            x0, y0, x1, y1 = p.fence
            if not (x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1):
                stop.append("outside the geofence")
        if max(abs(roll_deg), abs(pitch_deg)) > p.max_tilt_deg:
            stop.append(f"tilt {max(abs(roll_deg), abs(pitch_deg)):.0f} deg > {p.max_tilt_deg:.0f}")
        if front_range_m is not None and front_range_m < p.min_range_m:
            stop.append(f"obstacle at {front_range_m:.2f} m < {p.min_range_m:.2f}")
        if stop:
            self._stopping, self._stop_clear_since = True, None
        elif self._stopping:                                   # the condition is gone: wait `clear_after_s` before releasing
            if self._stop_clear_since is None:
                self._stop_clear_since = now
            if now - self._stop_clear_since >= p.clear_after_s:
                self._stopping = False
            else:
                stop.append("waiting for the way to stay clear")
        if self._stopping or stop:
            v.action, v.reasons = STOP, stop or ["waiting for the way to stay clear"]

        # --- battery-aware return ---
        if p.battery_budget_m is not None:
            left = p.battery_budget_m - self._driven
            v.battery_left_m = left
            need = math.dist(xy, self.base_xy) * p.return_margin + p.reserve_m
            if left < need and v.action != STOP:
                v.action, v.reasons = RETURN, [f"battery budget {left:.0f} m < {need:.0f} m needed to get home"]

        # --- UAV link (S-02) ---
        if self._last_map_t is not None and now - self._last_map_t > p.link_timeout_s:
            v.link_lost = True
            if self._link_lost_since is None:
                self._link_lost_since = self._last_map_t + p.link_timeout_s
            if p.link_return_s is not None and now - self._link_lost_since > p.link_return_s and v.action == OK:
                v.action, v.reasons = RETURN, [f"UAV link lost for {now - self._link_lost_since:.0f} s"]
        return v
