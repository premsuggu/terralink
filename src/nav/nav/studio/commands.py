"""What the browser may ask the studio to do - parsed, validated and LIMITED on the server.

The page is untrusted input: a bug in the JavaScript, a stuck gamepad axis or a hand-written message must never be able to command more than
the configured limits. So the client sends *normalised* values (stick positions in [-1, 1]) and this module turns them into physical
velocities with the limits applied here, not in the browser.

Messages (JSON text frames, client -> server):
    {"t": "uav_vel", "vx": f, "vy": f, "vz": f, "wz": f, "scale": f}   sticks in [-1, 1]; scale in [0, 1] = "slow mode" factor
    {"t": "estop"}            stop the UAV (zero velocity) and ignore uav_vel until released
    {"t": "estop_release"}
    {"t": "goal", "x": f, "y": f}   UGV goal in the map frame: the planner plans and the UGV drives (mode "auto")
    {"t": "cancel_goal"}            forget the goal; the UGV stops
    {"t": "mode", "mode": "auto"|"manual"}   who drives the UGV: the planner (auto) or the person (manual)
    {"t": "ugv_vel", "vx": f, "wz": f, "scale": f}   sticks in [-1, 1] for the UGV (manual mode only; same dead-man as the UAV)
    {"t": "takeoff", "alt": f}      climb to `alt` metres by itself (default 6); moving the climb stick or an e-stop cancels it
    {"t": "ping"}

UAV velocity frame: `/cmd_vel` is in the UAV's BODY frame (measured on the simulator, 2026-11-23: after yawing 86 degrees, +vx moved the UAV
north and +vy moved it to its left): x forward, y left, z up, +wz = counter-clockwise seen from above.

Dead-man: if no `uav_vel` message arrives for `deadman_sec` the output is zero. A browser tab that is closed, frozen or loses the websocket
therefore stops the UAV by itself.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field


class CommandError(ValueError):
    """The message is malformed or asks for something not allowed (the text is safe to show to the user)."""


@dataclass(frozen=True)
class UavLimits:
    """Largest velocities the studio will ever send to the UAV. Defaults are cautious: the patrol autopilot cruises at 2.5 m/s."""

    max_xy: float = 3.0      # m/s horizontal (forward and sideways, each)
    max_z: float = 1.5       # m/s vertical
    max_yaw: float = 1.0     # rad/s


@dataclass(frozen=True)
class UgvLimits:
    """Largest velocities the studio will send to the UGV by hand. The planner-driven UGV is limited by Nav2's own parameters instead."""

    max_lin: float = 1.0     # m/s forward/backward (the Jackal-class model tips if pushed much harder, see step10)
    max_ang: float = 1.5     # rad/s


@dataclass(frozen=True)
class Command:
    kind: str
    data: dict = field(default_factory=dict)


# (lowest, highest) the server accepts for each run-time setting. The UGV caps equal its defaults: the Jackal-class model tips if pushed harder (step10).
HARD_CAPS = {
    "max_xy": (0.5, 5.0), "max_z": (0.3, 2.5), "max_yaw": (0.3, 2.0),     # UAV speeds: horizontal m/s, vertical m/s, turn rad/s
    "ugv_max_lin": (0.2, 1.0), "ugv_max_ang": (0.3, 1.5),                  # manual UGV speeds
    "ceiling": (3.0, 14.0),                                                # UAV flight ceiling [m]
}


def _num(msg: dict, key: str, lo: float, hi: float, default: float | None = None) -> float:
    """A finite number from the message, clamped to [lo, hi]. NaN/inf/non-numbers are rejected (not silently turned into 0)."""
    if key not in msg:
        if default is None:
            raise CommandError(f"missing field '{key}'")
        return default
    value = msg[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CommandError(f"field '{key}' must be a finite number")
    return max(lo, min(hi, float(value)))


def parse_command(text: str) -> Command:
    """Parse one client text frame. Raises CommandError with a readable reason when it is not acceptable."""
    try:
        msg = json.loads(text)
    except (TypeError, ValueError):
        raise CommandError("not valid JSON") from None
    if not isinstance(msg, dict) or not isinstance(msg.get("t"), str):
        raise CommandError("a command is a JSON object with a string field 't'")
    kind = msg["t"]
    if kind == "uav_vel":
        return Command(kind, {
            "vx": _num(msg, "vx", -1.0, 1.0, 0.0), "vy": _num(msg, "vy", -1.0, 1.0, 0.0),
            "vz": _num(msg, "vz", -1.0, 1.0, 0.0), "wz": _num(msg, "wz", -1.0, 1.0, 0.0),
            "scale": _num(msg, "scale", 0.0, 1.0, 1.0),
        })
    if kind == "goal":
        # generous sanity bounds (no world here is anywhere near 1 km); the planner decides whether the point is reachable
        return Command(kind, {"x": _num(msg, "x", -1000.0, 1000.0), "y": _num(msg, "y", -1000.0, 1000.0)})
    if kind == "ugv_vel":
        return Command(kind, {"vx": _num(msg, "vx", -1.0, 1.0, 0.0), "wz": _num(msg, "wz", -1.0, 1.0, 0.0),
                              "scale": _num(msg, "scale", 0.0, 1.0, 1.0)})
    if kind == "mode":
        if msg.get("mode") not in ("auto", "manual"):
            raise CommandError("mode must be 'auto' or 'manual'")
        return Command(kind, {"mode": msg["mode"]})
    if kind == "takeoff":
        # 1..13 m: above the flight floor and below the ceiling the server enforces anyway
        return Command(kind, {"alt": _num(msg, "alt", 1.0, 13.0, 6.0)})
    if kind == "settings":
        # P5: change limits at run time. Every value is CLAMPED to a hard cap (HARD_CAPS) so a page can never ask for something unsafe;
        # a message may carry any subset of the keys, but at least one.
        got = {k: _num(msg, k, lo, hi) for k, (lo, hi) in HARD_CAPS.items() if k in msg}
        if not got:
            raise CommandError("settings needs at least one of: " + ", ".join(HARD_CAPS))
        return Command(kind, got)
    if kind == "autoscan":
        # fly the whole-site sweep by itself at this height (3..10 m; the server also keeps it under the ceiling)
        return Command(kind, {"alt": _num(msg, "alt", 3.0, 10.0, 5.0)})
    if kind in ("estop", "estop_release", "ping", "cancel_goal", "autoscan_cancel"):
        return Command(kind)
    raise CommandError(f"unknown command '{kind}'")


def uav_twist(cmd: Command, limits: UavLimits) -> tuple[float, float, float, float]:
    """(vx, vy, vz, wz) in m/s and rad/s for a `uav_vel` command, with the limits applied."""
    d, s = cmd.data, cmd.data["scale"]
    return (d["vx"] * s * limits.max_xy, d["vy"] * s * limits.max_xy, d["vz"] * s * limits.max_z, d["wz"] * s * limits.max_yaw)


def ugv_twist(cmd: Command, limits: UgvLimits) -> tuple[float, float]:
    """(linear m/s, angular rad/s) for a `ugv_vel` command, with the limits applied."""
    d, s = cmd.data, cmd.data["scale"]
    return (d["vx"] * s * limits.max_lin, d["wz"] * s * limits.max_ang)


class UgvCommandState:
    """Latest manual UGV command with a dead-man timeout (same idea as `UavCommandState`; the UGV has no separate e-stop - leaving manual
    mode, or the UAV e-stop button, zeroes it via `clear`)."""

    def __init__(self, limits: UgvLimits | None = None, deadman_sec: float = 0.5):
        self.limits = limits or UgvLimits()
        self.deadman_sec = deadman_sec
        self._twist = (0.0, 0.0)
        self._stamp: float | None = None

    def handle(self, cmd: Command, now: float) -> None:
        if cmd.kind == "ugv_vel":
            self._twist = ugv_twist(cmd, self.limits)
            self._stamp = now

    def clear(self) -> None:
        self._twist = (0.0, 0.0)
        self._stamp = None

    def current(self, now: float) -> tuple[float, float]:
        if self._stamp is None or (now - self._stamp) > self.deadman_sec:
            return (0.0, 0.0)
        return self._twist


class UavCommandState:
    """Holds the latest UAV command and answers "what velocity should be sent right now?" - with dead-man and e-stop.

    Time is passed in (seconds, any monotonic clock) so tests need no real clock. Not thread-safe by itself; the ROS node guards it with a lock.
    """

    def __init__(self, limits: UavLimits | None = None, deadman_sec: float = 0.5):
        self.limits = limits or UavLimits()
        self.deadman_sec = deadman_sec
        self.estop = False
        self._twist = (0.0, 0.0, 0.0, 0.0)
        self._stamp: float | None = None

    def handle(self, cmd: Command, now: float) -> None:
        if cmd.kind == "uav_vel":
            if self.estop:
                return              # ignored, and NOT remembered: it must not come back to life when the e-stop is released
            self._twist = uav_twist(cmd, self.limits)
            self._stamp = now
        elif cmd.kind == "estop":
            self.estop = True
            self._twist = (0.0, 0.0, 0.0, 0.0)
            self._stamp = None
        elif cmd.kind == "estop_release":
            # released = start again from zero: the person has to send a fresh command to move
            self.estop = False
            self._twist = (0.0, 0.0, 0.0, 0.0)
            self._stamp = None

    def current(self, now: float) -> tuple[float, float, float, float]:
        """The velocity to publish now: zero when e-stopped, never commanded, or the last command is older than the dead-man time."""
        if self.estop or self._stamp is None or (now - self._stamp) > self.deadman_sec:
            return (0.0, 0.0, 0.0, 0.0)
        return self._twist


def takeoff_climb(z: float, target: float, pilot_vz: float, max_z: float) -> tuple[float | None, float]:
    """One step of the automatic take-off: returns (target still being climbed to or None when finished/cancelled, vertical velocity to use).

    The pilot always wins: any climb/descend input (`pilot_vz` != 0) ends the take-off and is passed through unchanged. It also ends 0.2 m below the
    target. Otherwise the climb is proportional (gentle near the target), never above the vertical limit `max_z`, and never below 0.3 m/s so it
    reliably finishes instead of crawling.
    """
    if pilot_vz != 0.0 or z >= target - 0.2:
        return None, pilot_vz
    return target, min(max_z, max(0.3, 0.8 * (target - z)))


@dataclass(frozen=True)
class UavBounds:
    """Where the UAV may fly. Outside these limits the studio removes only the part of the command that would go further out, so the
    pilot can always fly back in. (The browser is untrusted, so this lives on the server like the speed limits.)"""

    min_z: float = 0.6                                   # no descending at or below this height (climbing is always allowed)
    max_z: float = 14.0                                  # ceiling [m]
    fence: tuple[float, float, float, float] | None = None   # (xmin, ymin, xmax, ymax) in the map frame; None = no horizontal limit


def apply_uav_bounds(twist: tuple[float, float, float, float], pose: tuple[float, float, float, float] | None,
                     bounds: UavBounds) -> tuple[float, float, float, float]:
    """Return `twist` (body-frame vx, vy, vz, wz) with the outward-going parts removed when the UAV `pose` (x, y, z, yaw) is at a limit."""
    if pose is None:
        return twist
    vx, vy, vz, wz = twist
    x, y, z, yaw = pose
    if z >= bounds.max_z and vz > 0.0:
        vz = 0.0
    if z <= bounds.min_z and vz < 0.0:
        vz = 0.0                                          # at/below the floor height: no further descent (climbing is always allowed)
    if bounds.fence is not None:
        # body -> world velocity (x forward, y left; yaw counter-clockwise), clamp, and back
        c, s_ = math.cos(yaw), math.sin(yaw)
        wx, wy = c * vx - s_ * vy, s_ * vx + c * vy
        x0, y0, x1, y1 = bounds.fence
        if (x <= x0 and wx < 0.0) or (x >= x1 and wx > 0.0):
            wx = 0.0
        if (y <= y0 and wy < 0.0) or (y >= y1 and wy > 0.0):
            wy = 0.0
        vx, vy = c * wx + s_ * wy, -s_ * wx + c * wy
    return (vx, vy, vz, wz)
