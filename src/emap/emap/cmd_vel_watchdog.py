"""Safety fix for a real bug found this session: Ignition's
`MulticopterVelocityControl` plugin (worlds/*.world) has no built-in command
timeout - it holds the LAST `/cmd_vel` message forever, applying it every
physics step until a NEWER message arrives. A single `ros2 topic pub --once
/cmd_vel ...` (or any short-lived publisher) therefore makes the UAV climb
(or move) FOREVER, not just for the moment the command was intended for.

Live impact this had on elevation mapping: a user testing this project sent
one upward pulse and, because nothing ever told the drone to stop, it kept
climbing far past the depth camera's own 20m sensing range, well outside any
altitude this project was ever meant to operate at - which is what actually
exposed the far-clip clamp-artifact bug fixed in `fusion.py`
(`max_valid_range`). That fusion-side fix makes bad sensor readings harmless
even if it happens again, but the ROOT cause - a command that never expires -
is a real flight-safety gap on its own, independent of mapping, and is fixed
here instead of just documented as "remember to always send a stop command".

This is the standard robotics answer to that class of problem: a small
watchdog that expects to keep seeing FRESH `/cmd_vel` messages, and publishes
an explicit zero-velocity Twist the moment they stop arriving for longer than
`timeout_sec` - exactly what "always send a stop after a pulse" means, done
automatically so no operator has to remember it.
"""
from __future__ import annotations

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelWatchdog(Node):
    """Publishes a zero Twist to /cmd_vel if no new command has arrived
    within `timeout_sec` - see module docstring for why this exists.
    """

    def __init__(self):
        super().__init__("cmd_vel_watchdog")

        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("check_rate_hz", 5.0)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        check_rate_hz = float(self.get_parameter("check_rate_hz").value)

        # None until the first real command arrives - deliberately inert at
        # startup (no forced stop before the UAV has ever been commanded at
        # all; the plugin already starts at zero velocity on its own).
        self._last_cmd_time = None
        # Only publish the stop ONCE per stale period, not on every timer
        # tick after the timeout - re-sending zero repeatedly would just
        # spam the topic without changing anything, since the plugin already
        # holds the last (zero) command until something new arrives.
        self._stop_already_sent = False

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._timer = self.create_timer(1.0 / check_rate_hz, self._check_timeout)

        self.get_logger().info(f"cmd_vel_watchdog: timeout_sec={self._timeout_sec}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        # This node both PUBLISHES to and SUBSCRIBES from /cmd_vel (the stop
        # has to go out on the same topic the bridge/plugin already listens
        # to), so its own zero-Twist stop message loops back into this exact
        # callback. Treating that as "fresh operator input" would reset the
        # timer, which fires again one timeout later, forever - a real bug
        # caught live (the watchdog kept re-warning every few seconds with
        # nothing actually commanding the UAV). Only a message with some
        # actual nonzero command counts as evidence the operator is still
        # actively driving; an all-zero Twist (ours or a legitimate operator
        # stop) doesn't need to re-arm anything, since the vehicle is
        # already at rest either way.
        is_real_command = any(
            getattr(msg.linear, axis) != 0.0 or getattr(msg.angular, axis) != 0.0
            for axis in ("x", "y", "z")
        )
        if not is_real_command:
            return
        self._last_cmd_time = self.get_clock().now()
        self._stop_already_sent = False

    def _check_timeout(self) -> None:
        if self._last_cmd_time is None or self._stop_already_sent:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed >= self._timeout_sec:
            self._pub.publish(Twist())  # all-zero Twist
            self._stop_already_sent = True
            self.get_logger().warn(
                f"no /cmd_vel for {elapsed:.1f}s (timeout={self._timeout_sec}s) - "
                f"publishing zero velocity to stop the UAV",
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
