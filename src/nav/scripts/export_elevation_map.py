#!/usr/bin/env python3
"""User-requested capability (2026-09-24): "I want to check out the 3D maps
and elevation maps generated for the room... generate the exact files in a
format which can be directly used to have a look at these maps in 3D view".

Standalone script (not a packaged ROS node/executable - same convention as
`emap/scripts/synthetic_pointcloud_tf_publisher.py`): subscribes to a live
`/elevation_map` (or `/elevation_mapping_node/local_map` - see --topic) GridMap
topic, waits for exactly one message, decodes it with `emap`'s own
`decode_gridmap` (the single authoritative decoder every other consumer in
this project already uses - see that function's own docstring for why a
second hand-derived decoder would be a real risk), and writes it out as a
colored PLY point cloud any standard 3D viewer can open directly (MeshLab,
CloudCompare, Blender's own PLY importer, or any online PLY/glTF viewer).

One point is written per grid cell where `is_valid` is True (skips cells the
UAV/UGV never actually observed - writing a fake flat plane for unscanned
territory would misrepresent the map, not visualize it), at that cell's real
(x, y, elevation) - so the tunnel's roof correctly appears floating above its
own floor, not merged into one number the way `d1`'s (and step05's) original
2.5D-only representation would have.

Usage (after `colcon build --packages-select nav` and sourcing the
workspace, with a `tunnel_demo.launch.py`/`nav_sim.launch.py` run already
live so /elevation_map has real data on it):

    python3 src/nav/scripts/export_elevation_map.py --out /tmp/elevation_map.ply
    python3 src/nav/scripts/export_elevation_map.py --out /tmp/elevation_map.ply --color height
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap

from emap.utils.coord_transform import grid_to_world
from emap.utils.gridmap_utils import decode_gridmap
from nav.ply_export import elevation_to_rgb, traversability_to_rgb, write_ply_points


class _OneShotGridMapExporter(Node):
    def __init__(self, topic: str, out_path: str, color_mode: str, timeout_sec: float):
        super().__init__("elevation_map_exporter")
        self._out_path = out_path
        self._color_mode = color_mode
        self._done = False
        self._success = False
        self._sub = self.create_subscription(GridMap, topic, self._on_map, 10)
        self.get_logger().info(f"elevation_map_exporter: waiting for one message on {topic}...")
        self._timeout_timer = self.create_timer(timeout_sec, self._on_timeout)

    def _on_timeout(self) -> None:
        if not self._done:
            self.get_logger().error(
                "elevation_map_exporter: no message received within the timeout - "
                "is a launch file actually running and publishing to this topic?"
            )
            self._done = True

    def _on_map(self, msg: GridMap) -> None:
        layers = decode_gridmap(msg)
        elevation = layers["elevation"]
        is_valid = layers["is_valid"].astype(bool)
        resolution = msg.info.resolution
        center_x = msg.info.pose.position.x
        center_y = msg.info.pose.position.y
        cell_n = elevation.shape[0]

        rows, cols = np.nonzero(is_valid)
        if rows.size == 0:
            self.get_logger().error(
                "elevation_map_exporter: the map has no valid (observed) cells yet - "
                "fly the UAV over some real terrain first."
            )
            self._done = True
            return

        xs, ys = grid_to_world(rows, cols, center_x, center_y, resolution, cell_n)
        zs = elevation[rows, cols]
        points = np.stack([xs, ys, zs], axis=1)

        if self._color_mode == "height":
            colors = elevation_to_rgb(zs)
        else:
            traversability = layers["traversability"]
            colors = traversability_to_rgb(traversability[rows, cols], is_valid[rows, cols])

        write_ply_points(self._out_path, points, colors_rgb=colors)
        self.get_logger().info(
            f"elevation_map_exporter: wrote {points.shape[0]} points to {self._out_path} "
            f"(color={self._color_mode})"
        )
        self._success = True
        self._done = True


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/elevation_map", help="GridMap topic to read (default: /elevation_map)")
    parser.add_argument("--out", required=True, help="Output .ply file path")
    parser.add_argument(
        "--color",
        choices=["traversability", "height"],
        default="traversability",
        help="Color scheme: traversability (green/yellow/red/gray, default) or height (blue-to-red gradient)",
    )
    parser.add_argument("--timeout", type=float, default=15.0, help="Seconds to wait for a message (default: 15)")
    args = parser.parse_args(argv)

    rclpy.init()
    node = _OneShotGridMapExporter(args.topic, args.out, args.color, args.timeout)
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not node._success:
        sys.exit(1)


if __name__ == "__main__":
    main()
