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

import numpy as np
import rclpy
from rclpy.node import Node

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray

from emap.utils.gridmap_utils import decode_gridmap, encode_layer_to_multiarray
from nav.anomaly import compute_anomaly_mask
from nav.resolved_regions import ResolvedRegion, ResolvedRegionStore
from nav.walkability import compute_frontier_mask, compute_walkable_mask
from nav.prm_planner import plan as prm_plan

# get_plan is re-run from scratch every time it's asked (see module
# docstring), including by waypoint_follower.py's own background
# `enable_frontier_replan` polling every few seconds while a plan is still
# tentative. `nav.prm_planner.plan`'s default `rng=None` creates a fresh,
# OS-entropy-seeded generator on every call - so two get_plan requests made
# seconds apart against a walkable_mask that hasn't meaningfully changed
# still returned a genuinely different random roadmap every time.
#
# NOTE ON A DEAD END (GUI-mode debugging, 2026-09-24, corrected after
# verifying against live logs - see waypoint_follower.py's
# _skip_reached_leading_waypoints for the actual root cause and fix of the
# "UGV moves here and there and just stops" bug): unseeded sampling was
# suspected as the reason two replans a few seconds apart could hand the
# follower very different waypoint counts/targets, and it was tempting to
# blame the resulting goal churn for the stall. That theory doesn't survive
# the live log, though: `/elevation_map`'s traversability layer keeps
# changing cell-by-cell from ongoing sensor fusion even where nothing new
# was physically observed, and `plan()`'s footprint-clearing around the
# (moving) start pose also shifts `walkable_mask` call to call - so even
# with this fixed seed, consecutive get_plan calls kept returning different
# plans in a live GUI run, and the UGV still didn't move. Seeding is kept
# anyway (a real, harmless reproducibility improvement, and it helps
# `_waypoints_effectively_equal` in waypoint_follower.py catch the cases
# where the map genuinely has settled) - but it is NOT what fixes the
# stall, and should not be described as such.
_PLAN_RNG_SEED = 20260924


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.declare_parameter("map_topic", "/elevation_map")
        self.declare_parameter("num_samples", 400)
        self.declare_parameter("connect_radius_m", 3.0)
        self.declare_parameter("footprint_radius_m", 0.3)
        # OFF by default - existing launches/tests get byte-for-byte the same
        # planner_node behavior as before this parameter existed. See
        # nav.prm_planner's module docstring and
        # docs/work-docs/nav/step05_frontier_tunnel_navigation.md: when on,
        # a region the UAV's overhead camera can never observe (e.g. a
        # tunnel/culvert interior) is no longer automatically treated as
        # impassable - a path that can only be completed by crossing such a
        # "frontier" cell is returned as a TENTATIVE plan instead of failing.
        self.declare_parameter("enable_frontier_mode", False)
        # OFF by default, same discipline as enable_frontier_mode above. See
        # nav.anomaly's module docstring and
        # docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md. Unlike
        # frontier mode (which reacts to UNOBSERVED cells), this flags cells
        # that ARE confidently observed but whose reading looks like a
        # step-discontinuity data artifact (e.g. a tunnel roof) rather than a
        # genuine obstacle. Phase 2 introduced this purely for
        # inspection/verification (compute + publish `/anomaly_map`, no
        # planning change); Phase 3 (this code) extends its meaning to also
        # gate actual planning use - turning it on now additionally lets
        # `plan()` return a TENTATIVE route through anomalous cells when
        # that's the only way to reach a goal, exactly as previewed in
        # Phase 2's own docstring ("no planner behavior changes from turning
        # it on yet (that's Phase 3)"). One flag, not two, because "publish
        # this signal" and "trust this signal enough to plan through it" are
        # never meaningfully wanted independently in this project - the same
        # reasoning `enable_frontier_mode` already applies to frontier cells.
        self.declare_parameter("enable_anomaly_mode", False)

        self._num_samples = int(self.get_parameter("num_samples").value)
        self._connect_radius_m = float(self.get_parameter("connect_radius_m").value)
        self._footprint_radius_m = float(self.get_parameter("footprint_radius_m").value)
        self._enable_frontier_mode = bool(self.get_parameter("enable_frontier_mode").value)
        self._enable_anomaly_mode = bool(self.get_parameter("enable_anomaly_mode").value)

        # Cached from the most recent /elevation_map message - see module
        # docstring for why this is updated passively rather than driven by
        # plan requests. _frontier_mask/_anomaly_mask stay None whenever
        # their mode is off, which is also exactly what prm_plan's own
        # defaults expect.
        self._walkable_mask = None
        self._frontier_mask = None
        self._anomaly_mask = None
        self._resolution = None
        self._center_x = None
        self._center_y = None
        self._map_frame = None

        # step06 Phase 5 - see nav.resolved_regions's own module docstring
        # for why this has to persist here (not be re-derived from
        # /elevation_map each message) and _map_callback below for where
        # it's actually applied.
        self._resolved_regions = ResolvedRegionStore()

        map_topic = self.get_parameter("map_topic").value
        self._map_sub = self.create_subscription(GridMap, map_topic, self._map_callback, 10)
        self._resolved_region_sub = self.create_subscription(
            Float64MultiArray, "resolved_region_report", self._resolved_region_callback, 10
        )
        self._plan_srv = self.create_service(GetPlan, "get_plan", self._get_plan_callback)

        # Published only when enable_anomaly_mode is on - a single-layer
        # GridMap (reusing the exact wire format /elevation_map already uses,
        # via the same encode_layer_to_multiarray helper) rather than a new
        # custom message type, since one boolean layer is all this needs and
        # GridMap already has RViz support for free.
        self._anomaly_pub = self.create_publisher(GridMap, "anomaly_map", 10)

        # A side channel, deliberately NOT added to GetPlan's own response
        # (that's a standard nav_msgs interface with no room for a custom
        # field, and vendoring a custom .srv just for one boolean would be
        # more machinery than this needs). Published right after every
        # get_plan response - waypoint_follower.py uses this to decide
        # whether the path it's about to follow is a confirmed route or a
        # tentative guess through unobserved territory, and therefore
        # whether it's worth periodically re-planning as the map improves.
        self._frontier_pub = self.create_publisher(Bool, "plan_has_frontier", 10)
        # Same idea, kept as its own separate topic rather than folded into
        # plan_has_frontier - see PlanResult.has_anomaly_segments's
        # docstring for why a downstream consumer (step06 Phase 4/5) needs
        # to distinguish WHICH resolution mechanism a tentative segment
        # needs (more UAV scanning vs. a 3D voxel-map check), not just
        # whether the plan is tentative at all.
        self._anomaly_plan_pub = self.create_publisher(Bool, "plan_has_anomaly", 10)

        self.get_logger().info(f"planner_node: waiting for a map on {map_topic}...")

    def _map_callback(self, msg: GridMap) -> None:
        layers = decode_gridmap(msg)
        self._walkable_mask = compute_walkable_mask(layers["traversability"], layers["is_valid"])
        # Only computed when actually needed - compute_frontier_mask is cheap
        # (a handful of NumPy array ops), but there's no reason to pay for it
        # on every map update in the default (non-frontier) configuration.
        if self._enable_frontier_mode:
            self._frontier_mask = compute_frontier_mask(self._walkable_mask, layers["is_valid"])
        self._resolution = msg.info.resolution
        self._center_x = msg.info.pose.position.x
        self._center_y = msg.info.pose.position.y
        self._map_frame = msg.header.frame_id

        if self._enable_anomaly_mode:
            # Cached on self, not just a local variable, so _get_plan_callback
            # can pass it into prm_plan below - the same pattern
            # self._frontier_mask already uses.
            # step06 anomaly-detection-v2 (second fix): a live-found bug in
            # the first redesign (two real rooms across an ordinary thin
            # wall got flagged as if they were a tunnel) meant the
            # elevation layer had to come back - component topology ALONE
            # can't tell a real wall apart from a real occluded passage; the
            # island-vs-room elevation check is what does that, see
            # nav.anomaly's own module docstring for the full story.
            raw_anomaly_mask = compute_anomaly_mask(
                layers["elevation"], self._walkable_mask, layers["is_valid"], self._resolution
            )
            # Published RAW (pre-overlay), deliberately - /anomaly_map is a
            # debug/visualization signal ("what does the detector currently
            # think"), and a resolved patch genuinely disappearing from it in
            # RViz is useful information for a human watching, not something
            # worth hiding. The OVERLAID version below (with step06 Phase 5's
            # already-settled verdicts applied) is what actually gets used
            # for planning, kept as a separate step so the two purposes don't
            # have to share one meaning.
            self._publish_anomaly_map(msg, raw_anomaly_mask)

            # step06 Phase 5: overlay any already-settled step06 Phase 4
            # verdicts on top of this message's freshly computed masks
            # before they're used for planning - see
            # ResolvedRegionStore.apply's own docstring for why this has to
            # happen every message rather than once (compute_anomaly_mask
            # has no memory of its own and will keep re-flagging the same
            # cells forever otherwise). Cheap no-op (returns the inputs
            # unchanged) until at least one resolution has actually been
            # reported.
            self._walkable_mask, self._anomaly_mask = self._resolved_regions.apply(
                self._walkable_mask, raw_anomaly_mask, self._resolution, self._center_x, self._center_y
            )

    def _resolved_region_callback(self, msg: Float64MultiArray) -> None:
        """step06 Phase 5: `waypoint_follower.py` has settled a previously-
        tentative anomaly segment with a real 3D voxel-map check (step06
        Phase 4) - remember it (`ResolvedRegionStore`) so the next
        `_map_callback` treats this specific patch as confirmed rather than
        re-flagging it as merely suspected forever. Payload: `[x_min, y_min,
        x_max, y_max, verdict]`, `verdict` 1.0=passable/0.0=blocked (see
        `waypoint_follower.py`'s `_report_resolution_and_maybe_replan`, the
        only publisher of this topic).
        """
        if len(msg.data) != 5:
            self.get_logger().warn(
                f"resolved_region_report: expected 5 floats [x_min,y_min,x_max,y_max,verdict], "
                f"got {list(msg.data)} - ignoring."
            )
            return
        x_min, y_min, x_max, y_max, verdict = msg.data
        region = ResolvedRegion(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max, passable=(verdict >= 0.5))
        self._resolved_regions.add(region)
        self.get_logger().info(
            f"planner_node: region ({x_min:.2f},{y_min:.2f})->({x_max:.2f},{y_max:.2f}) resolved as "
            f"{'passable' if region.passable else 'BLOCKED'} - will be applied to future plans."
        )

    def _publish_anomaly_map(self, source_msg: GridMap, anomaly_mask) -> None:
        """Republish the just-computed anomaly mask as its own single-layer
        GridMap, reusing `source_msg`'s own info/header (same frame,
        resolution, extent, timestamp as the `/elevation_map` message it was
        derived from) so the two overlay correctly in RViz without any
        separate bookkeeping.
        """
        gm = GridMap()
        gm.header = source_msg.header
        gm.info = source_msg.info
        gm.layers = ["anomaly"]
        gm.basic_layers = []
        gm.data = [encode_layer_to_multiarray(anomaly_mask.astype("float32"))]
        gm.outer_start_index = source_msg.outer_start_index
        gm.inner_start_index = source_msg.inner_start_index
        self._anomaly_pub.publish(gm)

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
            # See _PLAN_RNG_SEED's own comment above - fixed seed, fresh
            # Generator per call, so an unchanged map yields an unchanged
            # plan instead of a new random one on every request.
            rng=np.random.default_rng(_PLAN_RNG_SEED),
            frontier_mask=self._frontier_mask,
            allow_frontier=self._enable_frontier_mode,
            anomaly_mask=self._anomaly_mask,
            allow_anomaly=self._enable_anomaly_mode,
        )
        self._frontier_pub.publish(Bool(data=result.has_frontier_segments))
        self._anomaly_plan_pub.publish(Bool(data=result.has_anomaly_segments))

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
            # Both flags can be True together (a path can need unmeasured
            # frontier cells in one place and a suspected data-artifact
            # anomaly cell in another) - report both rather than collapsing
            # to a single generic "tentative" note, since a caller reacts to
            # each differently (see step06 Phase 4/5).
            tentative_bits = []
            if result.has_frontier_segments:
                tentative_bits.append("unconfirmed frontier cells")
            if result.has_anomaly_segments:
                tentative_bits.append("suspected-anomaly cells")
            tentative_note = f" (TENTATIVE - crosses {' and '.join(tentative_bits)})" if tentative_bits else ""
            self.get_logger().info(f"get_plan: found a {len(result.waypoints)}-waypoint path.{tentative_note}")
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
