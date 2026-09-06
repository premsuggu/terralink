"""nav's ROS node: requests a path from `planner_node`'s `get_plan` service
and walks the UGV through it - a Python port of
`src/d3/my_bot/src/waypoints_client.cpp`'s exact logic (same algorithm,
reused as instructed, only the service interface changed from d3's custom
`GetWaypoints.srv` to the standard `nav_msgs/srv/GetPlan`):

  - request a plan once, from the UGV's current position to a fixed
    configured goal;
  - subscribe to the UGV's own odometry;
  - advance through the returned waypoints one at a time, publishing each as
    a `PoseStamped` to `/goal_pose` (Nav2's `bt_navigator` has its own
    built-in subscription to that topic and starts a `NavigateToPose` action
    for whatever pose arrives on it - the same mechanism d3 relies on, so
    this node never talks to Nav2 directly);
  - only publish the NEXT waypoint once the UGV has actually gotten within
    `waypoint_reached_radius_m` of the previous one (mirrors d3's hardcoded
    0.5m distance check in `odom_callback`).
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped


class WaypointFollower(Node):
    def __init__(self):
        # Named "prm_waypoint_follower", not "waypoint_follower" - nav2's own
        # bringup already runs an unrelated node literally named
        # "waypoint_follower" (nav2_waypoint_follower's FollowWaypoints
        # action server, part of the standard Nav2 stack this package
        # includes) - reusing that name here would collide in the ROS graph.
        super().__init__("prm_waypoint_follower")

        self.declare_parameter("odom_topic", "/ugv/odom")
        # REAL BUG FOUND LIVE: (4.0, 0.0) - this node's original default -
        # sits only ~0.1m from room_maze.world's real east boundary wall
        # (Wall_2, at world x=4.10 - see worlds/room_maze.world's own
        # docstring for the wall layout math), so it reads LETHAL and could
        # NEVER be reached, no matter how long the UAV scanned. It was
        # picked as a generic "far corner" example before the room world
        # even existed, and never actually checked against the real
        # geometry once it did - the same class of mistake as the
        # original nav_ugv spawn point (see step03 docs). (1.7, -0.5) is
        # confirmed live to be walkable with real clearance, AND sits
        # inside the UAV's default patrol's own NE-quadrant waypoint (see
        # uav_autopilot_node.py's DEFAULT_WAYPOINTS), so it gets scanned
        # early rather than needing a long wait. Any goal_x/goal_y you pass
        # instead should be checked against a live-flown `/elevation_map`
        # first - a point close to any wall will fail exactly the same way.
        self.declare_parameter("goal_x", 1.7)
        self.declare_parameter("goal_y", -0.5)
        self.declare_parameter("waypoint_reached_radius_m", 0.5)
        self.declare_parameter("plan_frame", "iris_quad/odom")
        self.declare_parameter("retry_interval_sec", 2.0)

        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._reached_radius = float(self.get_parameter("waypoint_reached_radius_m").value)
        self._plan_frame = self.get_parameter("plan_frame").value
        # REAL BUG FOUND LIVE: without this, a failed request (e.g. no map
        # coverage yet, or the UAV just hasn't scanned that area) got retried
        # on the VERY NEXT odom message - and nav_ugv's odometry publishes at
        # ~30Hz (see worlds/room_maze.world's DiffDrive <odom_publish_frequency>),
        # so a genuinely unreachable goal produced dozens of get_plan calls
        # and matching log lines per second, drowning out everything else
        # and making it look like the whole node was stuck in a crash loop
        # rather than just waiting for the map to catch up.
        self._retry_interval_sec = float(self.get_parameter("retry_interval_sec").value)

        self._current_xy: tuple[float, float] | None = None
        self._waypoints: list[tuple[float, float]] | None = None  # None until a plan request succeeds
        self._next_index = 0

        odom_topic = self.get_parameter("odom_topic").value
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._plan_client = self.create_client(GetPlan, "get_plan")

        # Same "give the map/PRM service some time to have real data before
        # the first request" idea as d3's 10-second startup timer
        # (waypoints_client.cpp), just event-driven here (fires once odom
        # arrives) rather than a fixed wall-clock delay.
        self._requested = False
        self._last_request_time: float | None = None

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if not self._requested:
            now = time.monotonic()
            if self._last_request_time is not None and now - self._last_request_time < self._retry_interval_sec:
                return  # still cooling down since the last attempt - see retry_interval_sec above
            self._last_request_time = now
            self._requested = True
            self._request_plan()
            return

        if self._waypoints is None or self._next_index >= len(self._waypoints):
            return

        target = self._waypoints[self._next_index]
        if self._next_index == 0:
            self._publish_goal(target)
            self._next_index += 1
            return

        prev = self._waypoints[self._next_index - 1]
        distance = math.hypot(prev[0] - self._current_xy[0], prev[1] - self._current_xy[1])
        if distance < self._reached_radius:
            self.get_logger().info(f"Reached waypoint {self._next_index - 1}, publishing next.")
            self._publish_goal(target)
            self._next_index += 1

    def _publish_goal(self, xy: tuple[float, float]) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self._plan_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.orientation.w = 1.0
        self._goal_pub.publish(pose)

    def _request_plan(self) -> None:
        if not self._plan_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("get_plan service not up yet - will retry on the next odom message.")
            self._requested = False
            return

        request = GetPlan.Request()
        request.start.header.frame_id = self._plan_frame
        request.start.pose.position.x = self._current_xy[0]
        request.start.pose.position.y = self._current_xy[1]
        request.goal.header.frame_id = self._plan_frame
        request.goal.pose.position.x = self._goal_x
        request.goal.pose.position.y = self._goal_y

        future = self._plan_client.call_async(request)
        future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future) -> None:
        response = future.result()
        if not response.plan.poses:
            self.get_logger().warn("get_plan returned no path - retrying on the next odom message.")
            self._requested = False  # allow _odom_callback to try again
            return

        self._waypoints = [(p.pose.position.x, p.pose.position.y) for p in response.plan.poses]
        self._next_index = 0
        self.get_logger().info(f"Received a {len(self._waypoints)}-waypoint path - beginning navigation.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
