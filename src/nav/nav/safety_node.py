"""ROS wrapper of nav/safety.py (S-01/S-02) + the cmd_vel gate.

Subscribes: odometry (position + tilt), the lidar scan (closest return in a cone in FRONT), the UAV map (only to know the link is alive).
Publishes:  /safety/verdict (String "STOP: reason; reason"), /safety/stop (Bool), and on a RETURN verdict one "return" on /mission/command.
The GATE: Nav2's velocity output is remapped (launch arg enable_safety) to `gate_in`; this node republishes it on `gate_out` (the topic the
Gazebo bridge forwards to the wheels) - unchanged while the verdict is OK/RETURN, a ZERO twist while it is STOP. So a stop cannot be
out-voted by the controller (a second publisher on the same topic would be).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from nav.safety import RETURN, STOP, SafetyMonitor, SafetyParams


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")
        d = self.declare_parameter
        d("odom_topic", "/ugv/odom_ground_truth"); d("scan_topic", "/scan"); d("map_topic", "/elevation_map")
        d("gate_in", "/nav2_cmd_vel_raw"); d("gate_out", "/nav2_cmd_vel")
        d("fence", ""); d("max_tilt_deg", 35.0); d("min_range_m", 0.30); d("front_cone_deg", 25.0)
        d("battery_budget_m", 0.0); d("base_x", 0.0); d("base_y", 0.0); d("link_timeout_s", 5.0); d("link_return_s", 120.0)
        g = lambda n: self.get_parameter(n).value
        fence = [float(v) for v in str(g("fence")).split(",") if v.strip()]       # "xmin,ymin,xmax,ymax" or empty
        params = SafetyParams(fence=tuple(fence) if len(fence) == 4 else None, max_tilt_deg=g("max_tilt_deg"), min_range_m=g("min_range_m"),
                              battery_budget_m=g("battery_budget_m") or None, link_timeout_s=g("link_timeout_s"),
                              link_return_s=g("link_return_s") or None)
        self.monitor = SafetyMonitor(params, base_xy=(g("base_x"), g("base_y")))
        self.cone = math.radians(g("front_cone_deg"))
        self.front = None
        self.verdict_action = "OK"
        self._returned = False
        self.create_subscription(Odometry, g("odom_topic"), self._odom, 10)
        self.create_subscription(LaserScan, g("scan_topic"), self._scan, qos_profile_sensor_data)
        self.create_subscription(GridMap, g("map_topic"), lambda _m: self.monitor.map_received(self._now()), 10)
        self.create_subscription(Twist, g("gate_in"), self._gate, 10)
        self.gate_pub = self.create_publisher(Twist, g("gate_out"), 10)
        self.verdict_pub = self.create_publisher(String, "/safety/verdict", 10)
        self.stop_pub = self.create_publisher(Bool, "/safety/stop", 10)
        self.cmd_pub = self.create_publisher(String, "/mission/command", 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan(self, m: LaserScan):
        """Closest valid range within +-cone of straight ahead (the lidar's 0 rad = forward)."""
        best = None
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if abs(a) <= self.cone and m.range_min < r < m.range_max and (best is None or r < best):
                best = r
        self.front = best

    def _odom(self, m: Odometry):
        q, p = m.pose.pose.orientation, m.pose.pose.position
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        v = self.monitor.update(self._now(), (p.x, p.y), math.degrees(roll), math.degrees(pitch), self.front)
        if v.action != self.verdict_action:
            self.get_logger().warn(f"safety: {v.action} {'; '.join(v.reasons)}")
        self.verdict_action = v.action
        self.verdict_pub.publish(String(data=f"{v.action}: {'; '.join(v.reasons)}"))
        self.stop_pub.publish(Bool(data=v.action == STOP))
        if v.action == RETURN and not self._returned:
            self._returned = True
            self.cmd_pub.publish(String(data="return"))

    def _gate(self, m: Twist):
        self.gate_pub.publish(Twist() if self.verdict_action == STOP else m)


def main(args=None):
    rclpy.init(args=args)
    n = SafetyNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
