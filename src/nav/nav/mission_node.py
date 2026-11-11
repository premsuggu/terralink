"""ROS wrapper of nav/mission.py (S-03): publishes the CURRENT goal on /mission/goal (waypoint_follower with goal_topic:=/mission/goal
follows it), listens to /mission/command ("return" | "abort") and to /mission/goal_failed (Bool, the follower says the planner found no
route), and publishes /mission/state ("DRIVING 2/3 lap 0")."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from nav.mission import DONE, Mission


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        d = self.declare_parameter
        d("goals", ""); d("base_x", 0.0); d("base_y", 0.0); d("mode", "once"); d("loops", 1); d("dwell_s", 0.0)
        d("reach_radius_m", 0.8); d("odom_topic", "/ugv/odom_ground_truth"); d("plan_frame", "iris_quad/odom")
        g = lambda n: self.get_parameter(n).value
        flat = [float(v) for v in str(g("goals")).split(",") if v.strip()]         # "x1,y1,x2,y2,..."
        pairs = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.mission = Mission(pairs, (g("base_x"), g("base_y")), g("mode"), int(g("loops")), g("reach_radius_m"), g("dwell_s"))
        self.frame = g("plan_frame")
        self.goal_pub = self.create_publisher(PoseStamped, "/mission/goal", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(Odometry, g("odom_topic"), self._odom, 10)
        self.create_subscription(String, "/mission/command", self._command, 10)
        self.create_subscription(Bool, "/mission/goal_failed", lambda m: m.data and self._failed(), 10)
        self.create_timer(1.0, self._republish)        # a late-starting follower still gets the goal
        self._publish()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self):
        goal = self.mission.current_goal()
        if goal is None:
            return
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.orientation.w = goal[0], goal[1], 1.0
        self.goal_pub.publish(m)
        self.get_logger().info(f"mission: {self.mission.state} -> goal ({goal[0]:.1f}, {goal[1]:.1f})")

    def _odom(self, m: Odometry):
        p = m.pose.pose.position
        if self.mission.update((p.x, p.y), self._now()):
            self._publish()

    def _command(self, m: String):
        if m.data == "return":
            self.mission.return_to_base()
        elif m.data == "abort":
            self.mission.abort()
        self._publish()

    def _failed(self):
        self.mission.goal_failed()
        self._publish()

    def _republish(self):
        self.state_pub.publish(String(data=f"{self.mission.state} {self.mission.index + 1}/{len(self.mission.goals)} lap {self.mission.lap}"))
        if self.mission.state != DONE:
            self._publish()


def main(args=None):
    rclpy.init(args=args)
    n = MissionNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
