"""Pure logic for the UAV autopilot demo mode: fly to a fixed sequence of
hover waypoints, hold briefly at each, and repeat forever - so the whole
UAV-maps/UGV-navigates pipeline can be watched running with nobody sending
any `/cmd_vel` commands by hand.

No ROS here (matches every other algorithm module in this project -
`emap/fusion.py`, `nav/prm_planner.py`, etc.) - `uav_autopilot_node.py` is
the thin ROS wrapper that calls this.

Why a small PATROL, not just a single static hover (which is closer to what
was literally asked for - "start it in the air... keep it there"): a
downward camera held perfectly still can only ever see what its fixed cone
of view exposes from ONE vantage point, and any wall tall enough to matter
(room_maze.world's are 2.5m) casts a real, unavoidable blind spot behind
itself no matter how high that one vantage point is raised - a strictly
geometric limit, not a mapping bug. `emap`'s persistent GLOBAL map (step 8)
was built exactly to accumulate observations from *different* vantage
points over time without forgetting earlier ones, so a handful of hover
points spread across the room, each held long enough to actually build
confident measurements before moving on, gets meaningfully more of the
room's floor into the map than one fixed hover ever could - while still
requiring zero manual control, exactly like a plain hover would. This is
still an HONEST partial-coverage compromise, not a claim of perfect
coverage - small pockets directly behind walls from every waypoint's
vantage point can still end up unobserved; a real full-coverage sweep would
need many more, closer-together waypoints.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class AutopilotCommand:
    """A WORLD-frame velocity command - `compute_hold_command` below works
    entirely in world coordinates (current/target positions are both world-
    frame). This is NOT yet what `/cmd_vel` needs: `MulticopterVelocityControl`
    (this project's UAV control plugin) takes a BODY-frame command, and the
    two only coincide when the vehicle's yaw is exactly 0.

    REAL BUG FOUND LIVE: every manual `/cmd_vel` test up to this point
    (README's "Controlling the UAV" examples included) happened to work
    without ever converting frames, simply because the vehicle was always
    level (yaw=0) when those commands were sent - it either started that
    way at spawn or was already hovering upright from a previous command.
    The autonomous demo mode broke that assumption for the first time: with
    nothing commanding it during this launch's startup stagger, the UAV
    free-falls and can land/bounce with a real, non-zero yaw (confirmed
    live: 120 degrees off level after one such fall) - and a WORLD-frame
    command applied as if it were BODY-frame then sends the vehicle off at
    that same wrong angle, converging to nothing. `world_to_body_xy` below
    is the fix: called every control tick with the vehicle's CURRENT yaw
    (not just once at startup), so the commanded direction stays correct
    even if the vehicle is transiently rotated when control resumes."""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


def compute_hold_command(
    current_xyz: tuple[float, float, float],
    target_xyz: tuple[float, float, float],
    xy_gain: float,
    z_gain: float,
    max_xy_speed: float,
    max_z_speed: float,
) -> AutopilotCommand:
    """Simple proportional ("P") controller: velocity command proportional
    to the remaining position error, clamped to a max speed per axis.

    This is the whole trick behind "fly to a point and hold" without a full
    trajectory planner: far from the target, the error (and so the
    commanded speed) is large; as the UAV gets close, the error shrinks and
    the commanded speed shrinks with it, naturally slowing to a stop right
    at the target rather than overshooting past it and having to correct -
    the same "damped, not instant" idea already used in this project's
    drift-compensation gain (`emap/drift.py`), applied here to position
    instead of a Z-bias estimate.
    """
    dx = target_xyz[0] - current_xyz[0]
    dy = target_xyz[1] - current_xyz[1]
    dz = target_xyz[2] - current_xyz[2]

    vx = xy_gain * dx
    vy = xy_gain * dy
    vz = z_gain * dz

    xy_speed = math.hypot(vx, vy)
    if xy_speed > max_xy_speed and xy_speed > 0:
        scale = max_xy_speed / xy_speed
        vx *= scale
        vy *= scale
    vz = max(-max_z_speed, min(max_z_speed, vz))

    return AutopilotCommand(vx=vx, vy=vy, vz=vz)


def world_to_body_xy(vx_world: float, vy_world: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate a WORLD-frame XY velocity into the vehicle's BODY frame, given
    its current yaw (radians, standard convention: 0 = facing +X, positive
    = counterclockwise from above) - the missing piece `compute_hold_command`
    itself doesn't do (see `AutopilotCommand`'s docstring for why this needs
    to exist and be called EVERY tick with the live yaw, not computed once).

    Z is untouched by this (not returned) - a real rotation about the yaw
    (Z) axis only ever mixes X and Y, never Z, and `MulticopterVelocityControl`
    controls vertical speed independent of heading regardless.
    """
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    # Standard "rotate a world vector into a frame that's rotated by +yaw
    # from world" transform - the inverse of how you'd rotate a BODY-frame
    # vector INTO world (which would use -yaw, or equivalently swap the signs
    # below).
    vx_body = vx_world * cos_yaw + vy_world * sin_yaw
    vy_body = -vx_world * sin_yaw + vy_world * cos_yaw
    return vx_body, vy_body


@dataclass
class WaypointSequencer:
    """Tracks which waypoint the UAV is currently flying to/holding at, and
    when to advance to the next one - the dwell-and-advance state machine,
    kept separate from `compute_hold_command` (a pure per-tick calculation)
    so each half can be tested independently.
    """
    waypoints: list[tuple[float, float, float]]
    arrival_radius_m: float
    dwell_time_sec: float
    loop: bool = True

    _index: int = field(default=0, init=False)
    _dwell_elapsed_sec: float = field(default=0.0, init=False)
    _arrived: bool = field(default=False, init=False)
    _finished: bool = field(default=False, init=False)

    @property
    def current_target(self) -> tuple[float, float, float] | None:
        if self._finished:
            return None
        return self.waypoints[self._index]

    @property
    def finished(self) -> bool:
        """True once a non-looping sequence has held its final waypoint for
        the full dwell time - never true when `loop=True` (an autonomous
        demo is meant to run indefinitely until the user stops it)."""
        return self._finished

    def update(self, current_xyz: tuple[float, float, float], dt_sec: float) -> None:
        """Call once per control tick with the UAV's current position and
        the elapsed time since the last call - advances the internal
        dwell/waypoint-index state, never touches velocity directly (that's
        `compute_hold_command`'s job, called separately by the ROS node
        against whatever `current_target` is after this).
        """
        if self._finished:
            return

        target = self.waypoints[self._index]
        distance = math.dist(current_xyz, target)

        if distance <= self.arrival_radius_m:
            self._arrived = True
            self._dwell_elapsed_sec += dt_sec
        else:
            # Drifted back out of arrival radius before the dwell finished
            # (e.g. a gust-like disturbance) - restart the dwell clock
            # rather than counting time spent still approaching as if it
            # were time spent actually holding position.
            self._arrived = False
            self._dwell_elapsed_sec = 0.0

        if self._arrived and self._dwell_elapsed_sec >= self.dwell_time_sec:
            self._advance()

    def _advance(self) -> None:
        self._arrived = False
        self._dwell_elapsed_sec = 0.0
        if self._index + 1 < len(self.waypoints):
            self._index += 1
        elif self.loop:
            self._index = 0
        else:
            self._finished = True
