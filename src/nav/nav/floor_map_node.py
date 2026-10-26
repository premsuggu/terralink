"""The UGV's OWN map of the ground it has seen: a lidar occupancy map in the UAV map's grid.

WHY A SEPARATE MAP, AND WHY THE LIDAR
-------------------------------------
Under a roof or deck the UAV measured the roof (3.5 m) and the UGV sees the floor (0 m) in the SAME cells; one height
per cell cannot hold both, so the UGV's observations get their own map (see `nav/overhead.py`, which decides where
the UAV map must not be trusted and takes those cells from here).

The UGV's depth camera cannot do this job: it is tilted down 0.6 rad at 0.19 m height, so it sees only a 0.1-0.9 m
strip of floor ahead and no wall farther than ~1 m (found when a live run, planned with a 4 m camera assumption in the
simulator, never got inside the building). The 360 degree lidar (360 rays, 0.3-12 m) is the right sensor: every ray says
"free up to here" and, if it stopped short of its maximum range, "occupied here".

HOW
---
Log-odds occupancy per cell: each free cell along a ray lowers it, the end cell raises it; a cell is OBSERVED once any ray
touched it, LETHAL when net-occupied, EASY otherwise. Free cells take the UGV's own base height as elevation (the lidar
sees walls, not height: the floor is assumed level around the robot - true for the rooms and corridors this is for).
Output: `/ugv_floor_map`, a `grid_map_msgs/GridMap` with the same layers and grid as `/elevation_map` (parameters
`resolution`, `global_map_length`, centre (0, 0)); `nav.overhead.compose_with_floor` overlays them cell for cell.

The ray-casting core (`integrate_scan`) is plain NumPy and unit-tested in tests/nav/test_floor_map.py.
"""
from __future__ import annotations

import math

import numpy as np

L_FREE, L_HIT, L_MIN, L_MAX = -0.4, 1.2, -4.0, 4.0        # a hit outweighs the neighbouring rays that merely graze the cell
LETHAL_LOGODDS = 0.5                    # one net hit is enough; a wall seen from several angles stays lethal


def integrate_scan(logodds: np.ndarray, observed: np.ndarray, resolution: float, center_x: float, center_y: float,
                   sensor_xy, yaw: float, ranges, angle_min: float, angle_inc: float, range_min: float, range_max: float,
                   step_frac: float = 0.5) -> None:
    """Add one lidar scan to `logodds` / `observed` (both (rows, cols), modified in place).

    Ray i has bearing `yaw + angle_min + i * angle_inc`. A reading >= `range_max` (or inf/NaN) means "nothing out to
    range_max": the cells are free and nothing is marked occupied. A reading below `range_min` (-inf in Gazebo: the lidar is blind that close) is ignored entirely: no free cells, no hit.
    """
    n_r, n_c = logodds.shape
    ranges = np.asarray(ranges, float)
    bearing = yaw + angle_min + np.arange(len(ranges)) * angle_inc
    too_close = ranges < range_min                      # Gazebo reports -inf for a return nearer than the minimum range: blind, say nothing
    hit = np.isfinite(ranges) & (ranges < range_max - 1e-3) & (ranges >= range_min)
    reach = np.where(hit, ranges, range_max)
    reach = np.where(np.isfinite(reach), reach, range_max)
    dx, dy = np.cos(bearing), np.sin(bearing)
    step = resolution * step_frac
    seen = np.zeros_like(observed)
    for s in np.arange(range_min, range_max + step, step):
        alive = (s < reach - resolution * 0.5) & ~too_close   # stop short of the end cell: that one is the hit (or just range limit)
        if not alive.any():
            break
        r = np.round((sensor_xy[1] + s * dy[alive] - center_y) / resolution + n_r / 2).astype(int)
        c = np.round((sensor_xy[0] + s * dx[alive] - center_x) / resolution + n_c / 2).astype(int)
        ok = (r >= 0) & (r < n_r) & (c >= 0) & (c < n_c)
        seen[r[ok], c[ok]] = True
    # every ray's free cells count ONCE per scan (rays are denser than cells near the robot)
    logodds[seen] = np.clip(logodds[seen] + L_FREE, L_MIN, L_MAX)
    observed |= seen
    idx = np.nonzero(hit)[0]
    r = np.round((sensor_xy[1] + reach[idx] * dy[idx] - center_y) / resolution + n_r / 2).astype(int)
    c = np.round((sensor_xy[0] + reach[idx] * dx[idx] - center_x) / resolution + n_c / 2).astype(int)
    ok = (r >= 0) & (r < n_r) & (c >= 0) & (c < n_c)
    ends = np.zeros_like(observed)
    ends[r[ok], c[ok]] = True
    logodds[ends] = np.clip(logodds[ends] + L_HIT - L_FREE * 0, L_MIN, L_MAX)       # a hit cell may also have been "seen free" this scan
    observed |= ends


def to_layers(logodds: np.ndarray, observed: np.ndarray, floor_z: float) -> dict[str, np.ndarray]:
    lethal = observed & (logodds > LETHAL_LOGODDS)
    trav = np.ones(logodds.shape, np.float32)
    trav[lethal] = 0.0
    elev = np.full(logodds.shape, floor_z, np.float32)
    var = np.full(logodds.shape, 0.01, np.float32)
    return {"elevation": elev, "variance": var, "is_valid": observed.astype(np.float32), "traversability": trav}


def _make_node_class():
    """The ROS wrapper, built lazily so the module (and `integrate_scan`) imports without ROS in unit tests."""
    import rclpy
    import tf2_ros
    from grid_map_msgs.msg import GridMap
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan

    from emap.elevation_map import LAYER_INDEX
    from emap.utils.gridmap_utils import encode_layer_to_multiarray

    class FloorMapNode(Node):
        def __init__(self):
            super().__init__("floor_map_node")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("map_frame", "iris_quad/odom")
            self.declare_parameter("resolution", 0.1)
            self.declare_parameter("global_map_length", 40.0)       # must equal the UAV map's grid
            self.declare_parameter("publish_rate_hz", 2.0)
            self.declare_parameter("max_scan_rate_hz", 5.0)         # integrating 360 rays x 12 m is ~20 ms of NumPy
            self.declare_parameter("base_frame", "nav_ugv/base_link")
            p = lambda n: self.get_parameter(n).value               # noqa: E731
            self._frame = str(p("map_frame"))
            self._base = str(p("base_frame"))
            self._res = float(p("resolution"))
            n = int(round(float(p("global_map_length")) / self._res))
            self._logodds = np.zeros((n, n), np.float32)
            self._observed = np.zeros((n, n), bool)
            self._floor_z = 0.0
            self._min_dt = 1.0 / float(p("max_scan_rate_hz"))
            self._last = -1e9
            self._tf = tf2_ros.Buffer()
            tf2_ros.TransformListener(self._tf, self)
            self.create_subscription(LaserScan, str(p("scan_topic")), self._on_scan, qos_profile_sensor_data)
            self._pub = self.create_publisher(GridMap, "/ugv_floor_map", 10)
            self.create_timer(1.0 / float(p("publish_rate_hz")), self._publish)
            self._length = n * self._res
            self.get_logger().info(f"floor_map_node: lidar occupancy {n}x{n} @ {self._res} m from '{p('scan_topic')}' -> /ugv_floor_map")

        def _on_scan(self, msg: LaserScan) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last < self._min_dt:
                return
            try:
                tf = self._tf.lookup_transform(self._frame, msg.header.frame_id, Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(f"TF {msg.header.frame_id} -> {self._frame}: {exc}", throttle_duration_sec=5.0)
                return
            self._last = now
            q, t = tf.transform.rotation, tf.transform.translation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self._floor_z = float(t.z) - 0.095                      # the lidar sits 0.095 m above the base/floor
            integrate_scan(self._logodds, self._observed, self._res, 0.0, 0.0, (t.x, t.y), yaw, msg.ranges,
                           msg.angle_min, msg.angle_increment, max(msg.range_min, 0.3), msg.range_max)

        def _publish(self) -> None:
            layers = to_layers(self._logodds, self._observed, self._floor_z)
            gm = GridMap()
            gm.header.frame_id = self._frame
            gm.header.stamp = self.get_clock().now().to_msg()
            gm.info.resolution = self._res
            gm.info.length_x = gm.info.length_y = self._length
            gm.info.pose.orientation.w = 1.0
            gm.layers = list(LAYER_INDEX.keys())
            gm.basic_layers = ["elevation"]
            gm.data = [encode_layer_to_multiarray(layers[nm]) for nm in gm.layers]
            self._pub.publish(gm)

    return rclpy, FloorMapNode


def main(args=None) -> None:
    rclpy, FloorMapNode = _make_node_class()
    rclpy.init(args=args)
    node = FloorMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
