"""nav's ROS node: turns the live `/elevation_map` into a walkable mask
(`nav.walkability`) and answers path-planning requests over it
(`nav.prm_planner`) via the standard `nav_msgs/srv/GetPlan` service - the
direct replacement for `src/d3/my_bot/src/waypoints_server.cpp`
(`waypoints_service`, a custom `tutorial_interfaces/srv/GetWaypoints`).
`GetPlan` was used instead of vendoring that custom interface: it already
has exactly the shape this needs (`start`/`goal` PoseStamped in, a
`nav_msgs/Path` out), so no new `.srv` file or interfaces package was
needed.

Unlike d3's server (which continuously rebuilds its PRM roadmap frame-by-
frame from a live camera image as soon as the node starts), this node does
nothing until a plan is actually requested: it just keeps the latest decoded
walkable mask around (updated every time `/elevation_map` publishes), and
builds a fresh PRM roadmap on demand inside the service callback. That's a
deliberate difference, not an oversight - `/elevation_map` already updates
on its own schedule regardless of whether anyone's asking for a path, so
there's no equivalent here of "keep building the roadmap so it's ready
later"; the map itself is the thing that's kept ready.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

from emap.utils.gridmap_utils import decode_gridmap
from nav.walkability import compute_walkable_mask
from nav.prm_planner import plan as prm_plan


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.declare_parameter("map_topic", "/elevation_map")
        self.declare_parameter("num_samples", 400)
        self.declare_parameter("connect_radius_m", 3.0)
        self.declare_parameter("footprint_radius_m", 0.3)

        self._num_samples = int(self.get_parameter("num_samples").value)
        self._connect_radius_m = float(self.get_parameter("connect_radius_m").value)
        self._footprint_radius_m = float(self.get_parameter("footprint_radius_m").value)

        # Cached from the most recent /elevation_map message - see module
        # docstring for why this is updated passively rather than driven by
        # plan requests.
        self._walkable_mask = None
        self._resolution = None
        self._center_x = None
        self._center_y = None
        self._map_frame = None

        map_topic = self.get_parameter("map_topic").value
        self._map_sub = self.create_subscription(GridMap, map_topic, self._map_callback, 10)
        self._plan_srv = self.create_service(GetPlan, "get_plan", self._get_plan_callback)

        self.get_logger().info(f"planner_node: waiting for a map on {map_topic}...")

    def _map_callback(self, msg: GridMap) -> None:
        layers = decode_gridmap(msg)
        self._walkable_mask = compute_walkable_mask(layers["traversability"], layers["is_valid"])
        self._resolution = msg.info.resolution
        self._center_x = msg.info.pose.position.x
        self._center_y = msg.info.pose.position.y
        self._map_frame = msg.header.frame_id

    def _get_plan_callback(self, request: GetPlan.Request, response: GetPlan.Response) -> GetPlan.Response:
        if self._walkable_mask is None:
            self.get_logger().warn("get_plan requested before any /elevation_map message arrived - rejecting.")
            return response  # empty response.plan.poses - GetPlan has no separate "valid" field

        start_xy = (request.start.pose.position.x, request.start.pose.position.y)
        goal_xy = (request.goal.pose.position.x, request.goal.pose.position.y)

        result = prm_plan(
            self._walkable_mask,
            self._resolution,
            self._center_x,
            self._center_y,
            start_xy,
            goal_xy,
            num_samples=self._num_samples,
            connect_radius_m=self._connect_radius_m,
            footprint_radius_m=self._footprint_radius_m,
        )

        response.plan = Path()
        response.plan.header.frame_id = self._map_frame
        response.plan.header.stamp = self.get_clock().now().to_msg()
        if result.valid:
            for x, y in result.waypoints:
                pose = PoseStamped()
                pose.header.frame_id = self._map_frame
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                response.plan.poses.append(pose)
            self.get_logger().info(f"get_plan: found a {len(result.waypoints)}-waypoint path.")
        else:
            self.get_logger().warn("get_plan: no path found between the requested start and goal.")

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
