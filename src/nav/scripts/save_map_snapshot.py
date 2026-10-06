#!/usr/bin/env python3
"""Save ONE `/elevation_map` message as a snapshot file for offline replay.

Run this once against a live simulation after the UAV has finished mapping:

    python3 src/nav/scripts/save_map_snapshot.py --out maps/tunnel_test.npz

then plan on it as many times as you like WITHOUT relaunching the simulator:

    python3 src/nav/scripts/replay_plan.py --snapshot maps/tunnel_test.npz \\
        --start -3 0 --goal 3 0 --anomaly --frontier --png /tmp/plan.png

See `nav/replay.py` for why this exists. Standalone script (not an installed
executable), same convention as the other scripts in this folder.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from emap.utils.gridmap_utils import decode_gridmap
from nav.replay import save_snapshot


class _OneShot(Node):
    def __init__(self, topic: str, out_path: str, timeout_sec: float):
        super().__init__("map_snapshot_saver")
        self._out_path = out_path
        self.done = False
        self.success = False
        self.create_subscription(GridMap, topic, self._on_map, 10)
        self.create_timer(timeout_sec, self._on_timeout)
        self.get_logger().info(f"waiting for one message on {topic}...")

    def _on_timeout(self) -> None:
        if not self.done:
            self.get_logger().error("no map received in time - is a launch file running and mapping?")
            self.done = True

    def _on_map(self, msg: GridMap) -> None:
        layers = decode_gridmap(msg)
        save_snapshot(self._out_path, layers, msg.info.resolution, msg.info.pose.position.x, msg.info.pose.position.y)
        self.get_logger().info(f"saved {self._out_path} (layers: {', '.join(layers)})")
        self.success = True
        self.done = True


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/elevation_map")
    parser.add_argument("--out", required=True, help="output .npz path")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = _OneShot(args.topic, args.out, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not node.success:
        sys.exit(1)


if __name__ == "__main__":
    main()
