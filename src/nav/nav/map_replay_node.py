"""Publish a SAVED map as `/elevation_map` - the live stack, minus the scan.

Planning work does not need the UAV to fly around for ~95 s every run: the
finished map is the same each time. This node replaces `emap`'s
`elevation_mapping_node` with a replay of a snapshot taken once
(`scripts/save_map_snapshot.py`), so the REAL downstream stack - planner,
waypoint follower, voxel map, Nav2, the UGV physics - runs exactly as normal
but starts at the point where planning begins.

What this deliberately does NOT reproduce: the map never changes (no UAV
updates, no UGV camera filling in a tunnel interior). That is the point for
testing the planner and follower, and a limit for anything about live mapping.

Parameters:
  snapshot_path  (str)   .npz written by `save_map_snapshot.py`
  rate_hz        (float) republish rate (emap publishes at roughly 0.5-2 Hz)
  topic          (str)   default /elevation_map
  frame_id       (str)   default iris_quad/odom (the frame emap publishes in)
"""
from __future__ import annotations

import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from emap.utils.gridmap_utils import encode_layer_to_multiarray
from nav.replay import load_snapshot


def snapshot_to_gridmap(snap, frame_id: str) -> GridMap:
    """A `grid_map_msgs/GridMap` carrying every layer of `snap`, in the same
    wire layout `emap` publishes (so `decode_gridmap` reads it back as is)."""
    msg = GridMap()
    msg.header.frame_id = frame_id
    n_rows, n_cols = snap.shape
    msg.info.resolution = float(snap.resolution)
    msg.info.length_x = float(n_cols * snap.resolution)
    msg.info.length_y = float(n_rows * snap.resolution)
    msg.info.pose.position.x = float(snap.center_x)
    msg.info.pose.position.y = float(snap.center_y)
    msg.info.pose.orientation.w = 1.0
    msg.layers = list(snap.layers)
    msg.basic_layers = ["elevation"] if "elevation" in snap.layers else []
    msg.data = [encode_layer_to_multiarray(snap.layers[name].astype("float32")) for name in msg.layers]
    return msg


class MapReplayNode(Node):
    def __init__(self):
        super().__init__("map_replay_node")
        self.declare_parameter("snapshot_path", "")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("topic", "/elevation_map")
        self.declare_parameter("frame_id", "iris_quad/odom")
        path = str(self.get_parameter("snapshot_path").value)
        if not path:
            raise ValueError("map_replay_node: parameter snapshot_path is required")
        self._snap = load_snapshot(path)
        self._frame = str(self.get_parameter("frame_id").value)
        self._pub = self.create_publisher(GridMap, str(self.get_parameter("topic").value), 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish)
        self.get_logger().info(
            f"map_replay_node: replaying {path} ({self._snap.shape[0]}x{self._snap.shape[1]} cells @ "
            f"{self._snap.resolution} m) as {self.get_parameter('topic').value}"
        )

    def _publish(self) -> None:
        msg = snapshot_to_gridmap(self._snap, self._frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
