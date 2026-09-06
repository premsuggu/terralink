"""ROS wrapper around `nav.autopilot` - the UAV "demo mode" node: fly a
fixed patrol of hover waypoints over `worlds/room_maze.world`, forever, with
no `/cmd_vel` input from a person. Opt-in (see `launch/nav_sim.launch.py`'s
`autonomous_uav` argument) - manual control (the exact commands documented
in the top-level README) still works exactly as before when this node isn't
running.

This is deliberately NOT a real path planner or exploration algorithm - it's
a simple proportional "fly to a point and hold" controller
(`autopilot.compute_hold_command`) stepping through a short, fixed list of
hover points (`autopilot.WaypointSequencer`) - see `autopilot.py`'s module
docstring for why a patrol was built instead of the single static hover
that was literally asked for (wall-occlusion geometry, and `emap`'s
persistent global map already being built to accumulate multiple vantage
points).
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from nav.autopilot import WaypointSequencer, compute_hold_command, world_to_body_xy


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Standard quaternion -> yaw (rotation about Z) extraction. Only yaw is
    needed here (see `AutopilotCommand`'s docstring in autopilot.py for why
    it's the load-bearing one) - roll/pitch are ignored, which is exactly
    right at a normal hover (this airframe's attitude controller keeps them
    near zero) and an acceptable simplification during the brief transient
    right after an uncommanded fall, before `MulticopterVelocityControl`
    has leveled it back out.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

# Five hover points spread across room_maze.world's actual wall layout (see
# nav/worlds/room_maze.world's own docstring for the wall model's pose and
# the docs/work-docs/nav/step01_nav_pipeline.md verification section for the
# live-measured room extent this was derived from: roughly x in
# [-5.8, 4.2], y in [-7.9, 2.0], center near (-0.8, -3.0)) - room center
# plus one point per quadrant, all at the same altitude.
DEFAULT_WAYPOINTS = [
    (-0.8, -3.0, 4.5),   # room center
    (1.7, -0.5, 4.5),    # NE quadrant
    (-3.3, -0.5, 4.5),   # NW quadrant
    (1.7, -5.4, 4.5),    # SE quadrant
    (-3.3, -5.4, 4.5),   # SW quadrant
]


class UavAutopilotNode(Node):
    def __init__(self):
        super().__init__("uav_autopilot_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("waypoints_flat", [c for wp in DEFAULT_WAYPOINTS for c in wp])
        self.declare_parameter("arrival_radius_m", 0.5)
        self.declare_parameter("dwell_time_sec", 15.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("xy_gain", 0.4)
        self.declare_parameter("z_gain", 0.4)
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_z_speed", 0.8)
        self.declare_parameter("control_rate_hz", 5.0)

        flat = list(self.get_parameter("waypoints_flat").value)
        if len(flat) % 3 != 0:
            raise ValueError("waypoints_flat must be a flat list of x,y,z triples")
        waypoints = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self._sequencer = WaypointSequencer(
            waypoints=waypoints,
            arrival_radius_m=float(self.get_parameter("arrival_radius_m").value),
            dwell_time_sec=float(self.get_parameter("dwell_time_sec").value),
            loop=bool(self.get_parameter("loop").value),
        )
        self._xy_gain = float(self.get_parameter("xy_gain").value)
        self._z_gain = float(self.get_parameter("z_gain").value)
        self._max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self._max_z_speed = float(self.get_parameter("max_z_speed").value)

        self._current_xyz: tuple[float, float, float] | None = None
        self._current_yaw: float = 0.0
        self._last_tick = time.monotonic()

        odom_topic = self.get_parameter("odom_topic").value
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._timer = self.create_timer(1.0 / control_rate_hz, self._control_tick)

        self.get_logger().info(
            f"uav_autopilot_node: patrolling {len(waypoints)} waypoints "
            f"(dwell={self._sequencer.dwell_time_sec}s, loop={self._sequencer.loop}) - "
            "no manual /cmd_vel needed."
        )

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._current_xyz = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self._current_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _control_tick(self) -> None:
        if self._current_xyz is None:
            return  # haven't heard from the UAV's odometry yet

        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        self._sequencer.update(self._current_xyz, dt)
        target = self._sequencer.current_target
        if target is None:
            return  # a non-looping sequence finished - hold last command (none published)

        cmd = compute_hold_command(
            self._current_xyz, target, self._xy_gain, self._z_gain, self._max_xy_speed, self._max_z_speed
        )
        # cmd.vx/vy are WORLD-frame (compute_hold_command works entirely in
        # world coordinates) - MulticopterVelocityControl needs BODY-frame,
        # so this conversion is NOT optional (see AutopilotCommand's
        # docstring in autopilot.py for the real bug found live from
        # skipping it: a UAV that picked up yaw during an uncommanded fall
        # flew off in the wrong direction entirely).
        body_vx, body_vy = world_to_body_xy(cmd.vx, cmd.vy, self._current_yaw)
        twist = Twist()
        twist.linear.x = body_vx
        twist.linear.y = body_vy
        twist.linear.z = cmd.vz
        self._cmd_pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavAutopilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
