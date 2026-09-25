#!/usr/bin/env python3
"""Thin trigger for `voxel_map_node.py`'s own `export_voxel_map_request`
subscription (see that node's `__init__` for why the export has to happen
INSIDE that process - its octree is never published in bulk over ROS).

This script just publishes the target file path once and waits briefly for
the node's own log line confirming the write - it does not do any exporting
itself. Usage (after a `tunnel_demo.launch.py`/`nav_sim.launch.py` run is
already live, so the octree actually has real geometry in it):

    python3 src/nav/scripts/export_voxel_map.py --out /tmp/voxel_map.ply
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="Output .ply file path (written by voxel_map_node itself)")
    parser.add_argument(
        "--wait", type=float, default=3.0, help="Seconds to wait after publishing, for the write to complete"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("export_voxel_map_trigger")
    pub = node.create_publisher(String, "export_voxel_map_request", 10)
    # REAL BUG FOUND LIVE: a fixed time.sleep(1.0) here was NOT enough - a
    # first live test published, exited, and `ros2 topic info` immediately
    # afterward showed "Publisher count: 0" with the message never having
    # reached voxel_map_node (no export/no confirmation log line) - the
    # DDS discovery handshake between this fresh publisher and the
    # already-running node's subscription hadn't completed in that 1s
    # window under this environment's real load. Fixed by actually waiting
    # for `get_subscription_count() > 0` (rclpy exposes this directly)
    # instead of guessing a sleep duration - only publish once we KNOW a
    # matched subscriber exists, so the message can't be silently dropped
    # by a discovery race.
    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            "export_voxel_map_trigger: no subscriber ever appeared on export_voxel_map_request - "
            "is voxel_map_node actually running (a tunnel_demo.launch.py/nav_sim.launch.py session live)?"
        )
        node.destroy_node()
        rclpy.try_shutdown()
        return
    pub.publish(String(data=args.out))
    node.get_logger().info(f"export_voxel_map_trigger: requested export to {args.out} - check voxel_map_node's own log for confirmation.")
    time.sleep(args.wait)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
