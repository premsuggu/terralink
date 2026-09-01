"""emap step 6: the live ROS 2 node that ties steps 2-5 together into an
actually-running elevation map.

Everything this node does is a thin wrapper around pieces already built and
verified in isolation - it does not introduce any new mapping concept:
  - the point cloud + TF lookup pattern is exactly what step 2's
    verification script proved correct (points landing at the true ground
    height after transforming);
  - `ElevationMap.move_to` is step 5, unit-tested against exactly this kind
    of "the robot moved, re-center the grid" call;
  - `fuse_points` is step 4, unit-tested against hand-computed numbers;
  - the `GridMap` message encoding is `utils/gridmap_utils.py`.
See docs/work-docs/emap/step06_ros_node_integration.md for the full
from-scratch walkthrough of how these pieces are wired together here.

This node keeps TWO separate `ElevationMap` instances, not one - see
docs/work-docs/emap/step08_persistent_global_map.md for the full story, but
in short: a small map that re-centers on the UAV (`move_to` every callback,
forgetting whatever falls outside it) is the right shape of memory for fast
local reactions, but the WRONG shape for the actual end goal - autonomous UGV
navigation needs a planner to be able to route around terrain it saw minutes
ago and can no longer currently see. So `self._local_map` (unchanged since
step 6) exists alongside `self._global_map`, a second, larger map that is
simply never re-centered - `move_to` is never called on it - so it just keeps
accumulating everything ever seen, forever, at a fixed position. Both are fed
by the exact same sensor data every callback; neither `ElevationMap`,
`fuse_points`, nor `compute_traversability` needed to change at all to
support this - they already work correctly whether or not `move_to` is ever
called on a given instance.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from grid_map_msgs.msg import GridMap

import tf2_ros

from emap.elevation_map import ElevationMap, LAYER_INDEX
from emap.fusion import fuse_points
from emap.traversability import compute_traversability
from emap.utils.gridmap_utils import encode_layer_to_multiarray
from emap.utils.tf_utils import transform_points, translation_of


class ElevationMappingNode(Node):
    """Subscribes to the depth camera's point cloud, fuses it into an
    `ElevationMap`, and periodically publishes that map as a `GridMap`.
    """

    def __init__(self):
        super().__init__("elevation_mapping_node")

        # --- parameters (defaults here match config/elevation_mapping.yaml;
        # declaring defaults too means the node also works if launched
        # without that file, e.g. while testing from the command line) ---
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("length", 20.0)
        self.declare_parameter("global_map_length", 40.0)
        self.declare_parameter("initial_variance", 10.0)
        self.declare_parameter("sensor_noise_factor", 0.01)
        self.declare_parameter("mahalanobis_thresh", 2.0)
        self.declare_parameter("outlier_variance", 1.0)
        self.declare_parameter("min_valid_distance", 0.2)
        self.declare_parameter("map_frame", "iris_quad/odom")
        self.declare_parameter("base_frame", "iris_quad/base_link")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("max_slope", 0.35)
        self.declare_parameter("max_step", 0.15)
        self.declare_parameter("max_roughness", 0.05)

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._sensor_noise_factor = float(self.get_parameter("sensor_noise_factor").value)
        self._mahalanobis_thresh = float(self.get_parameter("mahalanobis_thresh").value)
        self._outlier_variance = float(self.get_parameter("outlier_variance").value)
        self._min_valid_distance = float(self.get_parameter("min_valid_distance").value)
        self._max_slope = float(self.get_parameter("max_slope").value)
        self._max_step = float(self.get_parameter("max_step").value)
        self._max_roughness = float(self.get_parameter("max_roughness").value)

        resolution = self.get_parameter("resolution").value
        initial_variance = self.get_parameter("initial_variance").value

        # The rolling map: re-centers on the UAV every callback (move_to,
        # below) - a small, fast window around wherever the drone currently
        # is. Kept for whatever future local-reaction/local-costmap use
        # it's suited for, even though nothing consumes it yet.
        self._local_map = ElevationMap(
            resolution=resolution,
            length=self.get_parameter("length").value,
            initial_variance=initial_variance,
        )
        # The persistent map: constructed the same way, but move_to is never
        # called on it anywhere in this file - it stays centered at (0, 0)
        # forever and simply keeps accumulating. This is the one published
        # on /elevation_map (below) - the one that matters for navigation.
        self._global_map = ElevationMap(
            resolution=resolution,
            length=self.get_parameter("global_map_length").value,
            initial_variance=initial_variance,
        )

        # tf2_ros.Buffer holds a rolling history of recent transforms;
        # TransformListener is what actually subscribes to /tf and /tf_static
        # (step 2's bridge topics) and feeds them into the buffer. This is
        # the exact same pattern step 2's verification script used.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # BEST_EFFORT/VOLATILE (qos_profile_sensor_data) is the standard
        # choice for high-rate sensor topics like a camera's point cloud -
        # a subscriber can always receive from a more-reliable publisher, so
        # this is compatible regardless of what the bridge itself uses, and
        # it means a single dropped point cloud message is never worth
        # blocking on a retransmission for.
        self._points_sub = self.create_subscription(
            PointCloud2, "/camera/points", self._pointcloud_callback, qos_profile_sensor_data
        )

        # /elevation_map is the GLOBAL (persistent) map - the one anything
        # downstream (RViz, later a planner) should actually look at.
        # /elevation_map_local is the rolling window, kept available but not
        # the default/primary topic.
        self._global_map_pub = self.create_publisher(GridMap, "/elevation_map", 10)
        self._local_map_pub = self.create_publisher(GridMap, "/elevation_map_local", 10)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._publish_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_maps)

        self.get_logger().info(
            f"elevation_mapping_node: local {self._local_map.cell_n}x{self._local_map.cell_n} "
            f"+ global {self._global_map.cell_n}x{self._global_map.cell_n} cells @ "
            f"{self._local_map.resolution} m/cell, map_frame='{self._map_frame}'"
        )

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        """`tf2_ros` lookup with a fallback to the latest available transform
        if the message's own timestamp isn't in the buffer yet.

        The TF buffer is always a little behind live sim/wall time - by the
        time a point cloud callback runs, TF for that *exact* timestamp may
        not have arrived, even though a transform from a few milliseconds
        earlier (close enough for our purposes) has. Refusing to process a
        perfectly good point cloud just because of that would throw away
        real data for no benefit - this mirrors `safe_lookup_transform` in
        the `src/d1` reference (`elevation_mapping_node.py`).
        """
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, stamp)
        except tf2_ros.TransformException:
            try:
                return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(
                    f"TF lookup {source_frame} -> {target_frame} failed: {exc}",
                    throttle_duration_sec=5.0,
                )
                return None

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        stamp = Time.from_msg(msg.header.stamp)

        # Where the CAMERA was when it took this cloud - needed both to
        # transform the points themselves (step 2) and, separately, as the
        # sensor's own position for fuse_points' distance-based noise model
        # (step 4) - the same lookup answers both, no extra work needed.
        camera_tf = self._lookup_transform(self._map_frame, msg.header.frame_id, stamp)
        if camera_tf is None:
            return

        # Where the DRONE ITSELF was - used only to re-center the map
        # (step 5). Deliberately a separate lookup from the camera's own
        # transform above: the map should follow the drone's body, not
        # wobble around following the camera's fixed 8cm mounting offset
        # from it.
        base_tf = self._lookup_transform(self._map_frame, self._base_frame, stamp)
        if base_tf is None:
            return

        points_sensor_frame = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if points_sensor_frame.size == 0:
            return

        # `skip_nans=True` above only drops NaN - a depth camera reports
        # +inf (not NaN) for "no return at all" pixels (e.g. nothing within
        # range, or - as found while testing this node - literally every
        # pixel when the camera is closer to the ground than its own near
        # clip plane). An inf point isn't caught by that filter, and
        # multiplying it through the rotation matrix in transform_points can
        # produce NaN (0 * inf = NaN for any zero entry in the rotation
        # matrix, which this camera's fixed downward mount always has) -
        # so filter both NaN and inf here, defensively, regardless of how
        # they got introduced upstream.
        finite = np.all(np.isfinite(points_sensor_frame), axis=1)
        if not np.any(finite):
            return
        points_sensor_frame = points_sensor_frame[finite]

        points_map_frame = transform_points(points_sensor_frame, camera_tf)
        base_position = translation_of(base_tf)
        sensor_origin = translation_of(camera_tf)

        # The local map re-centers on the drone's body (step 5) before being
        # fused into - the whole point of this map is to always be "centered
        # on me right now".
        self._local_map.move_to(base_position[0], base_position[1])
        self._fuse_and_update_traversability(self._local_map, points_map_frame, sensor_origin)

        # The global map NEVER moves - no move_to call here, ever. Points
        # that land outside its fixed extent are simply dropped by
        # fuse_points' existing in_bounds check (step 4) - nothing new to
        # handle, it already behaves correctly for a map that doesn't follow
        # the sensor.
        self._fuse_and_update_traversability(self._global_map, points_map_frame, sensor_origin)

    def _fuse_and_update_traversability(self, emap: ElevationMap, points_map_frame, sensor_origin) -> None:
        """Run one point cloud through `fuse_points` (step 4) then
        `compute_traversability` (step 7) against a single map - pulled out
        as its own method so the local and global maps in the callback above
        get IDENTICAL treatment from the same sensor data, rather than two
        copies of this logic risking drifting apart from each other.
        """
        fuse_points(
            emap,
            points_map_frame,
            sensor_origin,
            sensor_noise_factor=self._sensor_noise_factor,
            mahalanobis_thresh=self._mahalanobis_thresh,
            outlier_variance=self._outlier_variance,
            min_valid_distance=self._min_valid_distance,
        )

        # recompute traversability from this map's just-updated
        # elevation/variance. Done here (once per point cloud, right after
        # fusion) rather than on its own timer - traversability only ever
        # needs to reflect the map's current elevation/variance, so there's
        # no reason to recompute it on any different schedule than "whenever
        # that data just changed".
        is_valid = emap.layer("is_valid") > 0.5
        traversability = compute_traversability(
            emap.layer("elevation"),
            emap.layer("variance"),
            is_valid,
            emap.resolution,
            self._max_slope,
            self._max_step,
            self._max_roughness,
        )
        # Only ever overwrite cells that have actually been observed -
        # compute_traversability itself already returns EASY for every
        # is_valid=False cell (see its own docstring), but writing that
        # back into every cell here would still be redundant/harmless; the
        # explicit mask keeps this line self-documenting about the rule
        # every other layer already follows.
        emap.layer("traversability")[is_valid] = traversability[is_valid]

    def _publish_maps(self) -> None:
        """Publish both maps as `GridMap` messages."""
        self._global_map_pub.publish(self._build_gridmap_msg(self._global_map))
        self._local_map_pub.publish(self._build_gridmap_msg(self._local_map))

    def _build_gridmap_msg(self, emap: ElevationMap) -> GridMap:
        """Build one `GridMap` message from a map's current state - see
        `utils/gridmap_utils.py` for why the layer encoding below isn't
        simply `layer.flatten().tolist()`. Shared by both maps (Section
        above) so they can never accidentally be encoded differently.
        """
        gm = GridMap()
        gm.header.frame_id = self._map_frame
        gm.header.stamp = self.get_clock().now().to_msg()
        gm.info.resolution = emap.resolution
        # Unlike src/d1, ElevationMap has no reserved border cells (every
        # cell is used - see step 3), so length_x/length_y need no
        # adjustment beyond the map's own configured length.
        gm.info.length_x = emap.length
        gm.info.length_y = emap.length
        gm.info.pose.position.x = emap.center_x
        gm.info.pose.position.y = emap.center_y
        # Left neutral (z=0, identity orientation) rather than embedding the
        # drone's actual altitude/attitude here - some GridMap viewers apply
        # info.pose's z/orientation directly to the whole map, which would
        # make a perfectly flat map appear tilted or floating for no reason;
        # the map's real height DATA already lives in the elevation layer.
        gm.info.pose.position.z = 0.0
        gm.info.pose.orientation.w = 1.0

        # LAYER_INDEX (step 3) is a dict, and Python dicts preserve
        # insertion order - so this list is always exactly the 4 layer names
        # in the same fixed order they were defined in, and adding a layer
        # there automatically means it gets published here too.
        gm.layers = list(LAYER_INDEX.keys())
        gm.basic_layers = ["elevation"]
        gm.data = [encode_layer_to_multiarray(emap.layer(name)) for name in gm.layers]
        gm.outer_start_index = 0
        gm.inner_start_index = 0
        return gm


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
