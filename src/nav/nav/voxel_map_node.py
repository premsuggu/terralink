"""step06 Phase 1: the thin ROS wrapper around `nav.voxel_map.BoundedVoxelMap`
- subscribes to both robots' point clouds (the exact same two topics
`elevation_mapping_node`'s `secondary_points_topic` already wires up for
step05: the UAV's primary `/camera/points` and the UGV's `/ugv/camera/points`),
fuses them into one shared, bounded 3D occupancy map, and periodically evicts
anything far from the UGV's current position so the map's memory/compute cost
stays flat no matter how long or far the mission runs.

Originally ONLY the Phase 1 feasibility spike (prove the representation and
the bounding mechanism work live). step06 Phase 4 adds the first real
consumer of the map: a verification request/response exchange over plain
`std_msgs` topics (`verify_region_request` in, `voxel_verification` out) -
not a custom `.srv`, matching the "avoid vendoring a new interface for one
small exchange" choice already made for `plan_has_frontier`/`plan_has_anomaly`
in `planner_node.py`/`waypoint_follower.py`. See `nav.voxel_map.
check_region_headroom`'s own docstring for the actual three-way decision
logic (passable/blocked/unknown) this node just wires up to real sensor data.

KNOWN SIMPLIFICATION: requests are answered one at a time with no
request/response correlation ID - the latest `voxel_verification` message is
always the answer to the latest `verify_region_request` received. Safe as
long as callers (currently only `waypoint_follower.py`, which enforces this
with its own request cooldown) never have more than one exchange in flight -
see that module's own docstring for the same tradeoff stated from its side.

Diagnostic logging: this node periodically logs occupancy at four fixed,
known-significant coordinates in `tunnel_test.world`'s real geometry (see
that world file's own docstring for the tunnel's placement) - not because a
real deployment would ever want fixed-coordinate debug logging baked into a
node, but because this is exactly how every other `emap`/`nav` step in this
project has verified a live claim (read back real values, not just "it
launched") - see e.g. step07's "peak elevation measured at 1.4978m against a
true 1.5m" or step05's live tunnel-centerline scan. Once Phase 1 is confirmed
working this log block can be trimmed to a lower log level or removed
entirely for later phases - it stays for this spike because it *is* the
verification.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String

import numpy as np
import tf2_ros
from rclpy.time import Time

from emap.utils.tf_utils import transform_points, translation_of
from nav.ply_export import elevation_to_rgb, write_ply_points
from nav.voxel_map import BoundedVoxelMap, check_region_headroom


class VoxelMapNode(Node):
    def __init__(self):
        super().__init__("voxel_map_node")

        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("max_radius_m", 12.0)
        self.declare_parameter("map_frame", "iris_quad/odom")
        self.declare_parameter("primary_points_topic", "/camera/points")
        self.declare_parameter("secondary_points_topic", "/ugv/camera/points")
        self.declare_parameter("ugv_odom_topic", "/ugv/odom")
        # 2.0s, not every callback: eviction is O(num_leaf_nodes) (it has to
        # walk every leaf to test it against the current radius), and the
        # UGV moving a few cm between point-cloud frames doesn't meaningfully
        # change which voxels are in/out of a 12m radius - same "don't do
        # expensive bookkeeping on every single high-rate sensor frame" idea
        # already used elsewhere in this project (e.g. waypoint_follower's
        # retry_interval_sec).
        self.declare_parameter("evict_interval_sec", 2.0)
        self.declare_parameter("diagnostic_log_interval_sec", 5.0)

        # step06 Phase 4 - see this module's docstring. required_clearance_m
        # = robot_height_m + headroom_margin_m: nav_ugv's chassis is a
        # 0.3x0.3x0.15m box on 0.05m-radius wheels (see
        # models/nav_ugv/model.sdf) - roughly 0.2-0.25m tall overall, so
        # 0.25m + 0.15m margin comfortably covers the real robot while
        # staying well under tunnel_test.world's own real ~0.7m tunnel
        # headroom (see step06 Phase 1 results) - a real, non-trivial check,
        # not one that trivially always passes regardless of resolution.
        self.declare_parameter("robot_height_m", 0.25)
        self.declare_parameter("headroom_margin_m", 0.15)
        # The vertical range any column scan searches for a free run -
        # generous enough to span from below any real floor micro-variation
        # up past any single-story ceiling/overhang this project's worlds
        # use, without scanning the whole map's full possible height for
        # every request.
        self.declare_parameter("z_scan_min_m", -0.3)
        self.declare_parameter("z_scan_max_m", 1.5)
        # Same parameter, same rationale, as elevation_mapping_node's own
        # point_cloud_stride - see its declare_parameter comment for the
        # full story. This node inserts every point of the SAME two 10Hz,
        # 320x240 depth cameras into its octree (octree insertion has its
        # own real per-point cost, on top of whatever elevation_mapping_node
        # already pays) - a second, independent contributor to the CPU
        # contention that made Gazebo's own GUI render client tip this
        # sandbox's physics simulation into a real stall (confirmed live).
        # Off (1, no change) by default.
        self.declare_parameter("point_cloud_stride", 1)
        self._point_cloud_stride = max(1, int(self.get_parameter("point_cloud_stride").value))

        self._required_clearance_m = (
            float(self.get_parameter("robot_height_m").value) + float(self.get_parameter("headroom_margin_m").value)
        )
        self._z_scan_min_m = float(self.get_parameter("z_scan_min_m").value)
        self._z_scan_max_m = float(self.get_parameter("z_scan_max_m").value)

        self._map_frame = self.get_parameter("map_frame").value
        self._voxel_map = BoundedVoxelMap(
            resolution_m=float(self.get_parameter("resolution_m").value),
            max_radius_m=float(self.get_parameter("max_radius_m").value),
        )
        self._ugv_xyz: np.ndarray | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        primary_topic = self.get_parameter("primary_points_topic").value
        secondary_topic = self.get_parameter("secondary_points_topic").value
        self.create_subscription(PointCloud2, primary_topic, self._points_callback, qos_profile_sensor_data)
        if secondary_topic:
            self.create_subscription(PointCloud2, secondary_topic, self._points_callback, qos_profile_sensor_data)

        self.create_subscription(Odometry, self.get_parameter("ugv_odom_topic").value, self._odom_callback, 10)

        # step06 Phase 4's verification exchange - see module docstring for
        # why this is plain std_msgs topics rather than a custom service.
        self.create_subscription(Float64MultiArray, "verify_region_request", self._verify_request_callback, 10)
        self._verification_pub = self.create_publisher(String, "voxel_verification", 10)

        # User-requested capability (2026-09-24): "I want to check out the
        # 3D maps... generate the exact files in a format which can be
        # directly used to have a look at these maps in 3D view". This
        # node's own octree is the ONLY place the full 3D voxel structure
        # exists (it's never published in bulk over ROS - see the module
        # docstring's KNOWN SIMPLIFICATION note on why even the headroom
        # exchange above only ever sends small summaries, not raw voxels),
        # so exporting has to happen from inside this process, not an
        # external subscriber. A plain std_msgs/String request (the target
        # file path) rather than a service, matching this node's own
        # existing verify_region_request/voxel_verification convention -
        # `ros2 topic pub --once` is enough to trigger it, no client code
        # needed. See scripts/export_voxel_map.py for the (thin) trigger
        # helper this is meant to be used with.
        self.create_subscription(String, "export_voxel_map_request", self._export_request_callback, 10)

        self.create_timer(float(self.get_parameter("evict_interval_sec").value), self._evict_tick)
        self.create_timer(float(self.get_parameter("diagnostic_log_interval_sec").value), self._diagnostic_tick)

        self.get_logger().info(
            f"voxel_map_node: resolution={self._voxel_map.resolution_m}m, "
            f"max_radius={self._voxel_map.max_radius_m}m, map_frame='{self._map_frame}', "
            f"primary='{primary_topic}', secondary='{secondary_topic or '(disabled)'}'"
        )

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        """Same TF-lookup-with-fallback pattern as
        `elevation_mapping_node._lookup_transform` (reused, not re-derived -
        see that method's own docstring for why the fallback to the latest
        available transform matters)."""
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, stamp)
        except tf2_ros.TransformException:
            try:
                return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(f"TF lookup {source_frame} -> {target_frame} failed: {exc}", throttle_duration_sec=5.0)
                return None

    def _points_callback(self, msg: PointCloud2) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        sensor_tf = self._lookup_transform(self._map_frame, msg.header.frame_id, stamp)
        if sensor_tf is None:
            return

        points_sensor_frame = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points_sensor_frame.size == 0:
            return
        if self._point_cloud_stride > 1:
            points_sensor_frame = points_sensor_frame[:: self._point_cloud_stride]
        finite = np.all(np.isfinite(points_sensor_frame), axis=1)
        if not np.any(finite):
            return
        points_sensor_frame = points_sensor_frame[finite]

        points_map_frame = transform_points(points_sensor_frame, sensor_tf)
        sensor_origin = translation_of(sensor_tf)
        self._voxel_map.insert_points(points_map_frame, sensor_origin)

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._ugv_xyz = np.array([p.x, p.y, p.z])

    def _evict_tick(self) -> None:
        if self._ugv_xyz is None:
            return  # no odom yet - nothing to recenter on, leave the map alone
        evicted = self._voxel_map.evict_outside_radius(self._ugv_xyz)
        if evicted:
            self.get_logger().info(
                f"voxel_map_node: evicted {evicted} voxel(s) outside {self._voxel_map.max_radius_m}m "
                f"of UGV position {self._ugv_xyz.round(2).tolist()} - "
                f"{self._voxel_map.num_leaf_nodes} leaf nodes remain, "
                f"{self._voxel_map.memory_usage_bytes} bytes."
            )

    def _verify_request_callback(self, msg: Float64MultiArray) -> None:
        """Answers one step06 Phase 4 verification request: `msg.data` is
        `[x_min, y_min, x_max, y_max]` (built by `waypoint_follower.py`'s
        `_segment_bounding_box`, padded to cover the UGV's own footprint and
        real tracking error, not just the bare centerline it's driving).
        Sampled along the box's own diagonal - since the caller's actual use
        case is always "a segment about to be driven" (a line, not an
        arbitrary 2D area), 5 points along that line give real coverage of
        the region that matters without the cost of a dense 2D grid of
        column checks this use case doesn't need.
        """
        if len(msg.data) != 4:
            self.get_logger().warn(f"verify_region_request: expected 4 floats [x_min,y_min,x_max,y_max], got {list(msg.data)} - ignoring.")
            return
        x_min, y_min, x_max, y_max = msg.data
        sample_points = list(zip(np.linspace(x_min, x_max, 5), np.linspace(y_min, y_max, 5)))
        # check_region_headroom/check_column_headroom's query_fn contract is
        # query_fn(x, y, z) - three separate scalars, chosen so their unit
        # tests can use a plain synthetic function with no array plumbing
        # (see tests/nav/test_voxel_map.py). BoundedVoxelMap.query takes one
        # (x, y, z) point instead (matching the OctoMap binding it wraps) -
        # REAL BUG FOUND LIVE: passing self._voxel_map.query directly here
        # crashed this node outright (TypeError, killing the whole
        # accumulated map) the first time a verification request actually
        # arrived - this thin lambda is the adapter between the two
        # conventions, not a workaround for a design flaw in either one.
        verdict = check_region_headroom(
            lambda x, y, z: self._voxel_map.query([x, y, z]),
            sample_points,
            z_min=self._z_scan_min_m,
            z_max=self._z_scan_max_m,
            step_m=self._voxel_map.resolution_m,
            required_clearance_m=self._required_clearance_m,
        )
        self._verification_pub.publish(String(data=verdict))
        self.get_logger().info(
            f"voxel_map_node: verified region ({x_min:.2f},{y_min:.2f})->({x_max:.2f},{y_max:.2f}) = {verdict}"
        )

    def _export_request_callback(self, msg: String) -> None:
        """Writes the CURRENT occupied-voxel set to a PLY point cloud at the
        path in `msg.data`, synchronously, right here - see this
        subscription's own comment in `__init__` for why an in-process write
        is the only way to get this node's octree data out at all. Colored
        by height (blue=low, red=high) via `ply_export.elevation_to_rgb` -
        a fixed color scheme regardless of what's actually in the map (unlike
        the elevation-map export's traversability coloring, this map has no
        walkability concept of its own, just occupied/free/unknown).
        """
        centers = self._voxel_map.occupied_leaf_centers()
        if centers.shape[0] == 0:
            self.get_logger().warn(
                f"export_voxel_map_request: no occupied voxels yet - not writing {msg.data} "
                "(fly the UAV over some real geometry first)."
            )
            return
        colors = elevation_to_rgb(centers[:, 2])
        write_ply_points(msg.data, centers, colors_rgb=colors)
        self.get_logger().info(f"voxel_map_node: exported {centers.shape[0]} occupied voxels to {msg.data}")

    def _diagnostic_tick(self) -> None:
        """Phase 1's actual live-verification evidence - see module
        docstring. Coordinates match tunnel_test.world's real geometry: the
        tunnel's centerline sits at world (x=0, y=0), its 7-segment
        box-approximated roof peaks at pose z=0.7 (see that world file's
        <model name="tunnel"> block), and (2.0, 0.0) is real open room floor
        well clear of the tunnel footprint.
        """
        floor = self._voxel_map.query([0.0, 0.0, 0.0])
        headroom = self._voxel_map.query([0.0, 0.0, 0.35])
        roof = self._voxel_map.query([0.0, 0.0, 0.7])
        outside_tunnel_same_z = self._voxel_map.query([2.0, 0.0, 0.7])
        self.get_logger().info(
            f"voxel_map_node DIAGNOSTIC: tunnel floor(0,0,0)={floor}, "
            f"tunnel headroom(0,0,0.35)={headroom}, tunnel roof(0,0,0.7)={roof}, "
            f"outside-tunnel-same-height(2,0,0.7)={outside_tunnel_same_z}, "
            f"leaf_nodes={self._voxel_map.num_leaf_nodes}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoxelMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
