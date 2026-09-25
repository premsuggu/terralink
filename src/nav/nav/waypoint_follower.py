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

A later, OFF-BY-DEFAULT addition (`enable_frontier_replan`): when the plan
being followed was only a tentative guess through unobserved "frontier"
territory (see `nav.prm_planner`/`nav.walkability` and
docs/work-docs/nav/step05_frontier_tunnel_navigation.md - e.g. a tunnel
whose interior the UAV can never see from overhead), this node periodically
asks `planner_node` for a fresh plan rather than trusting that one guess for
the whole rest of the mission. `planner_node` publishes whether its last
plan was tentative on `plan_has_frontier` (a small side channel, since
`nav_msgs/srv/GetPlan`'s response has no room for a custom field) - this
node only bothers re-requesting while that's true.

Despite the name, this same periodic loop ALSO covers step06's later
"anomaly"-tentative plans (`plan_has_anomaly`), not just frontier ones - see
`_plan_is_tentative`'s own docstring for a real bug found live in the step06
Phase 6 acceptance run where this was NOT yet true and the UGV's mission
silently stalled with no recovery once the frontier flag alone had resolved.

A step06 Phase 4 addition, also OFF-BY-DEFAULT (`enable_voxel_verification`):
a plan flagged `plan_has_anomaly` (see `nav.prm_planner`/`nav.anomaly` and
docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md) is still just a
hopeful GUESS that a suspected data-fusion artifact (e.g. a tunnel roof
wrongly read as a solid LETHAL step) is actually crossable - nothing has
confirmed real 3D headroom there yet. Before publishing a goal for a
waypoint that's part of an anomaly-tentative plan, this node asks
`voxel_map_node` (step06 Phase 1's bounded 3D map) to check the real
vertical clearance across that specific segment, over `verify_region_request`
/`voxel_verification` (plain `std_msgs` types, not a custom `.srv` - same
"avoid vendoring a new interface for one small exchange" choice already made
for `plan_has_frontier`/`plan_has_anomaly`). Only a CONFIRMED-BLOCKED verdict
changes behavior here (refuse to advance onto that waypoint, and say so
clearly) - "passable" and "not yet resolved" are both treated as "keep going,
carefully", the same way the default (non-anomaly) path always has, on the
principle that Nav2's own local costmap/DWB controller is the real safety
net for anything this node doesn't yet have a confirmed answer about (see
`check_region_headroom`'s own docstring in `nav.voxel_map` for the full
three-way reasoning). Deliberately does NOT attempt to trigger a fresh
global replan around a confirmed-blocked segment - that's step06 Phase 5's
job; this phase only has to make sure the UGV doesn't drive somewhere the
3D map has actually ruled out.

KNOWN SIMPLIFICATION, scoped deliberately for this phase rather than left
unstated: `voxel_map_node` answers verification requests one at a time with
no request/response correlation ID (see its own module docstring) - this
node always interprets the latest verdict on `voxel_verification` as an
answer to the latest region it asked about. This is safe as long as only one
verification exchange is ever in flight at a time, which the cooldown below
(`verification_request_interval_sec`) enforces; a genuinely concurrent,
multi-request version would need real request IDs, which is more machinery
than a single-UGV prototype needs yet.

A step06 Phase 5 addition, closing the loop Phase 4 deliberately left open
("does NOT attempt to trigger a fresh global replan around a confirmed-
blocked segment - that's step06 Phase 5's job"): whenever a voxel
verification verdict is genuinely SETTLED (`passable` or `blocked` - never
`unknown`, which isn't a resolution at all, see
`nav.voxel_map.check_region_headroom`'s three-way docstring), this node
reports it to `planner_node` over `resolved_region_report`
(`std_msgs/Float64MultiArray`, `[x_min, y_min, x_max, y_max, verdict]`,
`verdict` 1.0=passable/0.0=blocked). `planner_node` remembers it
(`nav.resolved_regions.ResolvedRegionStore`) and re-applies it to every
future mask computation, so the same patch is never re-treated as merely
"suspected" once ground truth has actually settled it. A BLOCKED verdict
additionally (OFF BY DEFAULT, `enable_anomaly_replan`) triggers an immediate
fresh `get_plan` request - reusing `enable_frontier_replan`'s own
`_last_replan_time`/`frontier_replan_interval_sec` cooldown rather than a
second parallel timer (only one kind of replan is ever meaningfully "in
flight" from this node's perspective) - since the whole point of a
confirmed-blocked verdict is that the plan currently being followed is now
KNOWN to be unusable; there's no reason to wait for a periodic retry to get
around to it. A PASSABLE verdict does not itself trigger a replan - the plan
already being followed is exactly what the check just confirmed as safe, so
there's nothing to change; only the report to `planner_node` matters, so a
LATER plan (from a different position, or later in the mission) can reuse
this settled answer instead of re-treating the same patch as suspected.

REAL BUG FOUND LIVE (fixed here), found from an actual user run where the
UGV drove partway through `tunnel_test.world`'s tunnel and then stopped
forever, never resuming: this node used to take `self._current_xy` straight
from the `/ugv/odom` message's own position field. That topic is
`nav_ugv`'s DiffDrive-plugin odometry - DEAD-RECKONING integrated from ZERO
AT SPAWN (see `config/bridge.yaml`'s own comment on this exact topic),
**not** a position in `plan_frame` (`iris_quad/odom`, the frame every
waypoint/goal here is actually expressed in - confirmed live: for
`nav_ugv`'s `tunnel_test.world` spawn at world `(-3.0, 0.0)`, `/ugv/odom`
read `x=3.205` at the SAME instant `tf2_echo iris_quad/odom
nav_ugv/base_link` read `x=0.205` - an exact, constant +3.0 offset, matching
the spawn pose precisely). Using the raw topic value as if it were already
in `plan_frame` corrupted two things at once: (1) `_request_plan`'s `start`
pose, which is what `prm_planner` anchors the FIRST roadmap node to - so
every plan's own waypoints[0] silently inherited this same offset, which
(when re-published as an absolute `/goal_pose` Nav2 correctly executes in
the REAL world frame) still drove genuine progress, by coincidence, as long
as the offset happened to point roughly toward the goal; and (2) far more
seriously, the waypoint-REACHED distance check in `_odom_callback` below,
which compares this same corrupted value against `self._waypoints[i]`
(genuinely correct `plan_frame` coordinates, since they come straight from
`planner_node`'s PRM output against the true-world elevation map) - a
constant ~3m phantom gap that can NEVER close, so `distance < reached_radius`
simply never becomes true again once a plan's waypoints stop happening to
numerically resemble whatever `/ugv/odom` read when that plan was
requested. This is exactly what happened live: the FIRST plan's early
waypoints advanced correctly (near-coincidentally - request-time raw odom
was still close to spawn-relative zero, which happened to nearly cancel
against the offset for that specific plan), but the SECOND plan (from
step06 Phase 5's resolved-region replan, requested once the UGV had already
moved well into the tunnel) permanently stalled - Nav2 itself successfully
drove to and reached that plan's first real waypoint (confirmed live via
`controller_server: Reached the goal!`, and the robot's own TF-composed
pose settling exactly there), but `_odom_callback`'s own advancement check
kept comparing that success against the wrong, offset `self._current_xy`
and never recognized it, leaving the UGV correctly parked at a real
waypoint with no way to ever be told to move on to the next one.

Fixed by sourcing `self._current_xy` from an actual TF lookup
(`plan_frame -> robot_base_frame`, the same composition `tf2_echo` and
Nav2's own localization already use) instead of the raw odometry message -
see `_lookup_current_xy` below. The `/ugv/odom` subscription is kept only
as this node's "tick" (a periodic trigger to re-check state), not as a
position source anymore.
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray, String


def _should_replan(now: float, last_replan_time: float | None, interval_sec: float) -> bool:
    """Pure cooldown check, pulled out of the node so it's testable without
    ROS - same "extract the plain logic" discipline `nav.autopilot`/
    `nav.prm_planner` already follow. True the first time (no replan has
    happened yet), then True again only once `interval_sec` has passed -
    the SAME pattern `_retry_interval_sec` already uses for the initial
    get_plan retry loop below, reused here for periodic re-planning instead.
    """
    return last_replan_time is None or (now - last_replan_time) >= interval_sec


def _made_progress(
    current_xy: tuple[float, float],
    reference_xy: tuple[float, float] | None,
    progress_radius_m: float,
) -> bool:
    """Pure predicate, pulled out for the same reason `_should_replan` is:
    unit-testable without ROS. Answers a different question than the
    waypoint-reached check elsewhere in this module ("did we arrive
    somewhere specific") - this asks "have we moved AT ALL since the last
    time we looked", for `enable_stuck_recovery`'s progress watchdog (see
    its own `declare_parameter` comment for the live stall this addresses).
    `reference_xy is None` means no reference has been recorded yet (e.g.
    right after a new plan is adopted) - treated as "progress", the same
    "nothing to compare against yet, don't false-trigger" reasoning
    `_should_replan` applies to `last_replan_time is None`.
    """
    if reference_xy is None:
        return True
    return math.hypot(current_xy[0] - reference_xy[0], current_xy[1] - reference_xy[1]) >= progress_radius_m


def _plan_is_tentative(has_frontier: bool, has_anomaly: bool) -> bool:
    """Pure predicate, pulled out for the same reason `_should_replan` is:
    unit-testable without ROS, and a single source of truth for "is the plan
    currently being followed still just a guess, for ANY reason".

    step06 Phase 6 BUG FOUND LIVE (fixed here): the periodic replan trigger
    in `_odom_callback` originally checked ONLY `_current_plan_has_frontier`,
    a leftover from when `enable_frontier_replan` was written in step05,
    before anomaly-tentative plans existed at all (that came in step06
    Phase 3). The two flags are independent and can resolve at different
    times - live evidence from the Phase 6 acceptance run: a plan crossing
    the tunnel started out `(TENTATIVE - crosses unconfirmed frontier cells
    and suspected-anomaly cells)`, then a few seconds later a fresh plan
    resolved to `(TENTATIVE - crosses suspected-anomaly cells)` only - i.e.
    frontier had already resolved (`has_frontier=False`) while anomaly was
    STILL open (`has_anomaly=True`). With the old `and has_frontier` check,
    the periodic "keep asking for something better" safety net silently
    switched itself off at exactly that moment, even though the plan was
    still only a guess. In that same live run, the UGV's own progress then
    stalled on a downstream waypoint (a Nav2/NavFn hiccup near a
    PRM-sampled point close to real wall geometry - see step06's Phase 6
    results for the full story) with NO periodic replan ever firing again to
    route around it, because the (now-irrelevant) frontier flag alone was
    being watched. A CONFIRMED-BLOCKED voxel verdict already triggers an
    IMMEDIATE replan regardless of this flag (`enable_anomaly_replan`, step06
    Phase 5) - but that only fires for an explicit "blocked" answer, not for
    "the plan is just not making progress for some other reason". Widening
    this check to `has_frontier OR has_anomaly` restores the periodic safety
    net for the whole time EITHER kind of tentativeness is still open, not
    just one of the two - the same "the plan is a guess, keep checking for
    better" idea `enable_frontier_replan` was always meant to express.
    """
    return has_frontier or has_anomaly


def _waypoints_effectively_equal(
    old: list[tuple[float, float]] | None,
    new: list[tuple[float, float]],
    tol_m: float = 0.05,
) -> bool:
    """Pure predicate, pulled out for the same reason `_should_replan` is:
    unit-testable without ROS.

    A background frontier replan can legitimately return the exact plan
    already being followed (more often now that `planner_node.py` seeds its
    PRM sampling deterministically - see its own `_PLAN_RNG_SEED` comment).
    When that happens there's no reason to reset `_next_index`/republish and
    needlessly preempt Nav2's in-flight `NavigateToPose` goal for a route
    that hasn't changed. NOTE: in a live GUI run, `/elevation_map` kept
    changing cell-by-cell from ongoing sensor fusion even without new
    geometry being observed, so consecutive replans usually did NOT come
    back equal - this predicate helps in the case where the map genuinely
    has settled, but it is NOT what fixes the "UGV moves here and there and
    just stops" bug; see `_skip_reached_leading_waypoints`'s own docstring
    for the actual root cause and fix, found by checking this predicate's
    effect live and seeing the stall persist anyway. `tol_m` is
    float/quantization slop, not a "close enough route" tolerance - a plan
    that changed because the map genuinely changed will differ by far more
    than this.
    """
    if old is None or len(old) != len(new):
        return False
    return all(math.hypot(ox - nx, oy - ny) <= tol_m for (ox, oy), (nx, ny) in zip(old, new))


def _skip_reached_leading_waypoints(
    waypoints: list[tuple[float, float]],
    current_xy: tuple[float, float],
    reached_radius: float,
) -> int:
    """Pure helper, pulled out for the same reason `_should_replan` is:
    unit-testable without ROS. Scans forward from index 0 and returns the
    index of the first waypoint NOT already within `reached_radius` of
    `current_xy` (can return `len(waypoints)` if the whole plan is already
    "reached" - `_odom_callback` already treats that as nothing left to do).

    step06 Phase 6 ACTUAL root cause of "the UGV moves here and there and
    just stops" (found live after two other theories - CPU/physics stall,
    then unseeded PRM randomness - didn't survive checking the log): every
    `nav.prm_planner.plan()` result's waypoint 0 is always ~the robot's own
    current position (`_request_plan` sets `request.start` to wherever the
    robot currently is), so the OLD code always published it as a real goal
    first. Live GUI-mode log, one full cycle:

        317.317  controller_server: Received a goal (waypoint 0)
        317.324  controller_server: Reached the goal!        (trivial - ~0m away)
        317.330  waypoint_follower: Reached waypoint 0, publishing next
        317.336  bt_navigator: Received goal preemption request
        317.337  bt_navigator: Begin navigating ... to (-0.80, -0.90)  (the REAL target)
        317.356  bt_navigator: Goal succeeded                 (!!  ~19ms later)

    No `controller_server: Received a goal` ever appears for the real
    target between 317.337 and 317.356 - the second `/goal_pose` (the real
    waypoint), published within milliseconds of the first, raced the first
    goal's own success being reported up through bt_navigator's behavior
    tree and lost: bt_navigator's action-server bookkeeping settled on
    "succeeded" for what was actually the just-preempted trivial goal,
    never properly started executing toward the real one, and controller_server
    then had nothing to do - no active goal, so no cmd_vel - until the next
    replan (`frontier_replan_interval_sec`, ~7s later) restarted the same
    cycle. This is what looked exactly like a physics/CPU stall (TF frozen
    for 150+ seconds despite `ign gazebo gui` running at 400%+ CPU and Nav2
    logging as if everything were healthy) - the sim was fine; two
    `/goal_pose` messages published milliseconds apart was the bug. Fix:
    never publish the trivial "already there" waypoint at all - skip
    straight past any leading waypoints already within `reached_radius` and
    publish only the first REAL, not-yet-reached target, so exactly one
    `/goal_pose` goes out per plan adoption instead of two in quick
    succession.
    """
    index = 0
    while index < len(waypoints) and math.hypot(
        waypoints[index][0] - current_xy[0], waypoints[index][1] - current_xy[1]
    ) < reached_radius:
        index += 1
    return index


def _new_resolution_to_report(
    verdict: str,
    region: tuple[float, float, float, float] | None,
    last_reported: tuple[tuple[float, float, float, float], str] | None,
) -> bool:
    """Pure decision logic for step06 Phase 5's report-back gate, pulled out
    for the same reason `_should_replan`/`_segment_bounding_box` are:
    unit-testable without ROS. True only when `verdict` is a genuinely
    SETTLED answer (`"passable"`/`"blocked"`, never `"unknown"` - see
    `nav.voxel_map.check_region_headroom`'s three-way docstring) for a
    region this node actually asked about (`region is not None`), and it
    isn't simply a repeat of the last thing already reported for that exact
    region - repeating an unchanged report would still be harmless
    downstream (`ResolvedRegionStore.add`/`apply` are both idempotent), but
    there's no reason to spam the topic (or planner_node's log) with it.
    """
    if verdict not in ("passable", "blocked"):
        return False
    if region is None:
        return False
    return (region, verdict) != last_reported


def _segment_bounding_box(
    from_xy: tuple[float, float], to_xy: tuple[float, float], margin_m: float
) -> tuple[float, float, float, float]:
    """Pure geometry, pulled out for the same reason `_should_replan` is:
    unit-testable without ROS. Returns (x_min, y_min, x_max, y_max) - the
    request payload `voxel_map_node` expects on `verify_region_request` -
    covering the straight line from `from_xy` to `to_xy` (the segment the
    UGV is about to drive) padded by `margin_m` on every side.

    The padding matters for two real reasons, not just caution for its own
    sake: the UGV won't track the line with zero error (Nav2's local
    controller has real tracking tolerance), and the check needs to cover
    the UGV's own footprint width, not just its centerline path - a
    zero-margin box could report "passable" for a path whose edge clips an
    obstruction the centerline itself would have missed.
    """
    x0, y0 = from_xy
    x1, y1 = to_xy
    return (
        min(x0, x1) - margin_m,
        min(y0, y1) - margin_m,
        max(x0, x1) + margin_m,
        max(y0, y1) + margin_m,
    )


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
        # The frame this node's own position is looked up in, via TF, rather
        # than trusted from a raw odometry topic - see this module's
        # docstring for the real bug this fixes (nav_ugv/base_link's
        # dead-reckoned /ugv/odom silently disagreeing with plan_frame by
        # exactly the spawn offset).
        self.declare_parameter("robot_base_frame", "nav_ugv/base_link")
        self.declare_parameter("retry_interval_sec", 2.0)

        # OFF by default - existing launches get identical behavior. See
        # docs/work-docs/nav/step05_frontier_tunnel_navigation.md: a plan
        # that had to cross a "frontier" cell (unobserved territory the
        # UAV's overhead camera structurally cannot see into, e.g. a
        # tunnel's interior - see nav.prm_planner/nav.walkability) is only
        # ever a tentative GUESS, not a confirmed-safe route. Rather than
        # trust that one guess for the rest of the mission, this node keeps
        # asking for a fresh plan every `frontier_replan_interval_sec` while
        # `plan_has_frontier` (published by planner_node.py) says the
        # current plan is still tentative - once the UGV's own onboard
        # sensor has actually driven far enough to resolve those cells with
        # real data (fused into the shared /elevation_map), a fresh
        # get_plan naturally confirms or improves the route, same idea as
        # D* Lite's incremental replanning without adopting that whole
        # algorithm - a cheap "patch it once better data exists", not a
        # from-scratch re-plan philosophy change.
        self.declare_parameter("enable_frontier_replan", False)
        self.declare_parameter("frontier_replan_interval_sec", 5.0)
        # REAL BUG FOUND LIVE (GUI-mode run, 2026-09-24, user report: "it
        # just goes to the edge of the tunnel and moves uselessly a lot" -
        # "we cannot have it take 10 mins... this is seriously frustrating
        # how long it takes"): the periodic check above used to swap in
        # whatever fresh plan came back UNCONDITIONALLY, every single
        # interval, with no regard for whether the CURRENT plan was
        # actually working. Since two get_plan calls against a
        # still-evolving map often return genuinely different PRM samples
        # (see planner_node.py's own _PLAN_RNG_SEED comment - determinism
        # alone doesn't fix this, the map itself keeps drifting), this
        # meant Nav2's in-flight NavigateToPose got preempted onto a
        # different target roughly every `frontier_replan_interval_sec`
        # regardless of how well the current one was going - live evidence:
        # the UGV bounced between meaningfully different positions in room
        # A for 100+ seconds (confirmed via ground truth, not odometry)
        # before ever committing to one approach into the tunnel, even
        # though it was making genuine incremental progress most of that
        # time. Fixed: the periodic replan now only actually interrupts
        # anything if the UGV has moved LESS than
        # `frontier_replan_min_progress_m` since the LAST time this check
        # ran (see `_odom_callback` for exactly how) - a plan that's
        # visibly working (the robot is covering real ground toward it) is
        # left alone; only a plan that's stalled or barely moving anything
        # gets reconsidered. This is the concrete version of "go straight,
        # analyse the tunnel as you approach it, only ask for a new route
        # once the current one stops actually helping" rather than
        # interrupting on a blind timer.
        self.declare_parameter("frontier_replan_min_progress_m", 0.3)

        # OFF by default - see this module's docstring for the full step06
        # Phase 4 story. verification_region_margin_m pads the straight-line
        # segment being checked (see _segment_bounding_box); 0.5m is
        # comfortably wider than nav_ugv's own 0.3m chassis (see
        # models/nav_ugv/model.sdf) plus real driving/tracking slop.
        # verification_request_interval_sec mirrors frontier_replan's own
        # cooldown idea - don't re-request on every single odom message.
        self.declare_parameter("enable_voxel_verification", False)
        self.declare_parameter("verification_region_margin_m", 0.5)
        self.declare_parameter("verification_request_interval_sec", 2.0)

        # OFF by default - see this module's docstring for the full step06
        # Phase 5 story. Meaningless (and never checked) unless
        # enable_voxel_verification is also on, since a BLOCKED verdict can
        # only ever arrive through that mechanism.
        self.declare_parameter("enable_anomaly_replan", False)

        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._reached_radius = float(self.get_parameter("waypoint_reached_radius_m").value)
        self._plan_frame = self.get_parameter("plan_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        # REAL BUG FOUND LIVE: without this, a failed request (e.g. no map
        # coverage yet, or the UAV just hasn't scanned that area) got retried
        # on the VERY NEXT odom message - and nav_ugv's odometry publishes at
        # ~30Hz (see worlds/room_maze.world's DiffDrive <odom_publish_frequency>),
        # so a genuinely unreachable goal produced dozens of get_plan calls
        # and matching log lines per second, drowning out everything else
        # and making it look like the whole node was stuck in a crash loop
        # rather than just waiting for the map to catch up.
        self._retry_interval_sec = float(self.get_parameter("retry_interval_sec").value)
        self._enable_frontier_replan = bool(self.get_parameter("enable_frontier_replan").value)
        self._frontier_replan_interval_sec = float(self.get_parameter("frontier_replan_interval_sec").value)
        self._frontier_replan_min_progress_m = float(self.get_parameter("frontier_replan_min_progress_m").value)
        self._last_replan_time: float | None = None
        # See frontier_replan_min_progress_m's own declare_parameter
        # comment - where the UGV was the LAST time the periodic replan
        # check ran, so this check (unlike enable_stuck_recovery's own
        # progress tracking below) measures progress specifically between
        # consecutive checks, not since a plan was first adopted.
        self._last_replan_check_xy: tuple[float, float] | None = None
        # Updated by planner_node.py's plan_has_frontier side channel (see
        # its own module docstring for why this isn't a GetPlan response
        # field) - starts False so a node launched with frontier replan
        # enabled, but before any plan has arrived at all, doesn't try to
        # "replan" something that was never following anything tentative.
        self._current_plan_has_frontier = False

        self._enable_voxel_verification = bool(self.get_parameter("enable_voxel_verification").value)
        self._verification_region_margin_m = float(self.get_parameter("verification_region_margin_m").value)
        self._verification_request_interval_sec = float(self.get_parameter("verification_request_interval_sec").value)
        # Same "starts False/neutral so nothing tentative is assumed before
        # any real signal has arrived" reasoning as _current_plan_has_frontier.
        self._current_plan_has_anomaly = False
        # "unknown" is the correct starting value, not "passable" or
        # "blocked" - before any verification response has ever arrived,
        # this node genuinely doesn't know, and "unknown" already means
        # "proceed carefully, let Nav2's local costmap be the real safety
        # net" (see check_region_headroom's docstring) - exactly the right
        # default behavior for "haven't asked yet" too.
        self._latest_voxel_verdict = "unknown"
        self._last_verification_request_time: float | None = None
        # step06 Phase 5 bookkeeping - see _new_resolution_to_report and
        # _report_resolution_and_maybe_replan below. _last_verification_region
        # is the exact bbox the most recent verify_region_request asked
        # about, so a verdict arriving later on voxel_verification can be
        # attributed to a specific region (voxel_map_node's own "no
        # correlation ID" simplification means this is the only place that
        # attribution can happen - see this module's docstring).
        self._last_verification_region: tuple[float, float, float, float] | None = None
        self._last_reported_resolution: tuple[tuple[float, float, float, float], str] | None = None
        self._enable_anomaly_replan = bool(self.get_parameter("enable_anomaly_replan").value)

        # OFF by default - existing launches get identical behavior, same
        # discipline as every other opt-in feature in this module.
        #
        # REAL BUG FOUND LIVE (GUI-mode run, 2026-09-24): none of this
        # module's existing replan mechanisms fire when Nav2 itself simply
        # GIVES UP on the current goal. Live evidence: the UGV approached
        # the tunnel, DWB logged "No valid trajectories out of 419! -
        # BaseObstacle/Trajectory Hits Obstacle" repeatedly, then
        # "Controller patience exceeded", then two "Failed to make
        # progress" events, then `bt_navigator: Goal failed` - and the UGV
        # simply sat there forever afterward. `enable_frontier_replan`'s
        # periodic timer only re-requests a plan while the CURRENT plan is
        # still frontier/anomaly-tentative; a plan that Nav2 flatly failed
        # to execute (for any reason - a bad PRM waypoint too close to a
        # wall, a local-costmap deadlock, anything) has no recovery path at
        # all otherwise. This is the general "stuck detection" capability
        # AGENTS.md's own Direction 3 fix-list already calls out as needed
        # (d3's `waypoints_client.cpp` has the exact same gap) - a plain
        # progress watchdog: if the robot hasn't moved at least
        # `stuck_progress_radius_m` in `stuck_timeout_sec`, request a fresh
        # plan regardless of WHY it stalled, rather than trying to
        # enumerate every possible stall cause individually.
        self.declare_parameter("enable_stuck_recovery", False)
        self.declare_parameter("stuck_timeout_sec", 20.0)
        self.declare_parameter("stuck_progress_radius_m", 0.15)
        self._enable_stuck_recovery = bool(self.get_parameter("enable_stuck_recovery").value)
        self._stuck_timeout_sec = float(self.get_parameter("stuck_timeout_sec").value)
        self._stuck_progress_radius_m = float(self.get_parameter("stuck_progress_radius_m").value)
        # Both reset fresh every time a plan is adopted (_on_plan_response)
        # - see that method for why: a brand new plan deserves its own
        # full timeout window, not one inherited from whatever segment was
        # being driven before.
        self._last_progress_xy: tuple[float, float] | None = None
        self._last_progress_time: float | None = None

        self._current_xy: tuple[float, float] | None = None
        self._waypoints: list[tuple[float, float]] | None = None  # None until a plan request succeeds
        self._next_index = 0

        # See this module's docstring for why this exists at all: the raw
        # /ugv/odom topic is dead-reckoned from spawn, not a plan_frame
        # position, and using it directly as one silently corrupted both the
        # get_plan request's start pose and the waypoint-reached distance
        # check. This buffer is the actual source of "where is the robot,
        # in plan_frame" from here on - the same TF composition Nav2's own
        # localization and `tf2_echo` already rely on.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        odom_topic = self.get_parameter("odom_topic").value
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._plan_client = self.create_client(GetPlan, "get_plan")
        self._frontier_sub = self.create_subscription(Bool, "plan_has_frontier", self._frontier_callback, 10)
        self._anomaly_plan_sub = self.create_subscription(Bool, "plan_has_anomaly", self._anomaly_callback, 10)
        self._voxel_verdict_sub = self.create_subscription(String, "voxel_verification", self._voxel_verdict_callback, 10)
        self._verify_request_pub = self.create_publisher(Float64MultiArray, "verify_region_request", 10)
        # step06 Phase 5 - see this module's docstring and
        # nav.resolved_regions.ResolvedRegionStore for the receiving side.
        self._resolved_region_pub = self.create_publisher(Float64MultiArray, "resolved_region_report", 10)

        # Same "give the map/PRM service some time to have real data before
        # the first request" idea as d3's 10-second startup timer
        # (waypoints_client.cpp), just event-driven here (fires once odom
        # arrives) rather than a fixed wall-clock delay.
        self._requested = False
        self._last_request_time: float | None = None

    def _frontier_callback(self, msg: Bool) -> None:
        self._current_plan_has_frontier = msg.data

    def _anomaly_callback(self, msg: Bool) -> None:
        self._current_plan_has_anomaly = msg.data

    def _voxel_verdict_callback(self, msg: String) -> None:
        self._latest_voxel_verdict = msg.data
        # step06 Phase 5 - see _new_resolution_to_report's own docstring for
        # exactly what counts as "new" here (a genuinely settled verdict for
        # this specific region, not a repeat of what was already reported).
        if _new_resolution_to_report(
            self._latest_voxel_verdict, self._last_verification_region, self._last_reported_resolution
        ):
            self._report_resolution_and_maybe_replan()

    def _report_resolution_and_maybe_replan(self) -> None:
        """step06 Phase 5's actual report-back + conditional-replan action -
        see this module's docstring for the full reasoning. Only ever
        called once `_new_resolution_to_report` has already confirmed this
        is a genuinely new, settled verdict, so no further gating is needed
        here beyond updating the "last reported" bookkeeping and acting.
        """
        region = self._last_verification_region
        verdict = self._latest_voxel_verdict
        self._last_reported_resolution = (region, verdict)

        x_min, y_min, x_max, y_max = region
        passable = verdict == "passable"
        self._resolved_region_pub.publish(
            Float64MultiArray(data=[x_min, y_min, x_max, y_max, 1.0 if passable else 0.0])
        )
        self.get_logger().info(
            f"reporting resolved region ({x_min:.2f},{y_min:.2f})->({x_max:.2f},{y_max:.2f}) "
            f"= {'passable' if passable else 'BLOCKED'} to planner_node."
        )

        if not passable and self._enable_anomaly_replan:
            now = time.monotonic()
            # Reuses enable_frontier_replan's own cooldown fields on purpose
            # (see this module's docstring) - only one kind of replan is
            # ever meaningfully "in flight" from this node's perspective, so
            # a second, independent timer here would just be two clocks
            # racing to trigger the same underlying action.
            if _should_replan(now, self._last_replan_time, self._frontier_replan_interval_sec):
                self._last_replan_time = now
                self.get_logger().info("confirmed-blocked segment - requesting a fresh plan now.")
                self._request_plan()

    def _maybe_request_voxel_verification(self, from_xy: tuple[float, float], to_xy: tuple[float, float]) -> None:
        """Rate-limited request for step06 Phase 4's ground-truth check over
        the segment about to be driven - only fires at all while
        `enable_voxel_verification` is on AND the current plan is anomaly-
        tentative (an all-confirmed-walkable plan has nothing here to check;
        this must stay a no-op for it, same additive discipline as every
        other opt-in feature in this module).
        """
        if not (self._enable_voxel_verification and self._current_plan_has_anomaly):
            return
        now = time.monotonic()
        if not _should_replan(now, self._last_verification_request_time, self._verification_request_interval_sec):
            return  # still cooling down since the last request - see verification_request_interval_sec
        self._last_verification_request_time = now
        region = _segment_bounding_box(from_xy, to_xy, self._verification_region_margin_m)
        # Recorded BEFORE publishing so that even a same-callback-tick
        # response (unlikely over real ROS pub/sub, but harmless to guard
        # against) already has a region to be attributed to - see
        # _voxel_verdict_callback/step06 Phase 5's docstring.
        self._last_verification_region = region
        self._verify_request_pub.publish(Float64MultiArray(data=list(region)))

    def _publish_goal_unless_voxel_confirmed_blocked(
        self, from_xy: tuple[float, float], target: tuple[float, float]
    ) -> bool:
        """The one behavior change step06 Phase 4 makes to this node's
        otherwise-unchanged waypoint-advance logic: kick off (or continue
        cooling down on) a verification request for this segment, then
        refuse to publish it ONLY if the most recent verdict this node has
        actually received says "blocked" - see this module's docstring for
        why "unknown"/"passable" both mean "proceed, Nav2's local costmap is
        the real safety net here" and why a confirmed-blocked segment isn't
        also given a replan here (that's Phase 5). Returns True if the goal
        was published (the existing behavior, unconditionally, whenever
        voxel verification isn't even active for this segment), False if it
        was withheld.
        """
        self._maybe_request_voxel_verification(from_xy, target)
        if (
            self._enable_voxel_verification
            and self._current_plan_has_anomaly
            and self._latest_voxel_verdict == "blocked"
        ):
            self.get_logger().warn(
                f"Refusing to advance toward {target} - voxel map confirms insufficient clearance "
                f"across this segment. Holding position (step06 Phase 5 will add real replanning here)."
            )
            return False
        self._publish_goal(target)
        return True

    def _lookup_current_xy(self) -> tuple[float, float] | None:
        """The robot's real position in `plan_frame`, via TF - see this
        module's docstring for why this replaced reading `/ugv/odom`'s
        position field directly. Returns None (rather than raising) when the
        transform isn't available yet - e.g. right at startup, before this
        node's own TF listener has received enough of the tree, or before
        the static transform tying nav_ugv/odom into plan_frame has
        published its first message - exactly the same "not ready yet, try
        again next tick" situation `self._current_xy is None` already meant
        before this change, so callers don't need to distinguish the two.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._plan_frame, self._robot_base_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().info(
                f"TF lookup {self._plan_frame} -> {self._robot_base_frame} not ready yet ({exc}) - "
                "will retry on the next odom tick.",
                throttle_duration_sec=5.0,
            )
            return None
        translation = transform.transform.translation
        return (translation.x, translation.y)

    def _odom_callback(self, msg: Odometry) -> None:
        # `msg` itself is no longer used for position (see this module's
        # docstring) - the odometry message is kept only as a convenient,
        # already-existing periodic trigger ("something happened, re-check
        # state") at the same ~30Hz cadence this node's timing already
        # assumed, rather than adding a separate timer for the same purpose.
        current_xy = self._lookup_current_xy()
        if current_xy is None:
            return
        self._current_xy = current_xy

        if not self._requested:
            now = time.monotonic()
            if self._last_request_time is not None and now - self._last_request_time < self._retry_interval_sec:
                return  # still cooling down since the last attempt - see retry_interval_sec above
            self._last_request_time = now
            self._requested = True
            self._request_plan()
            return

        # Investigate-and-replan loop (see enable_frontier_replan's
        # declare_parameter comment above): keep asking for a better plan
        # while the one currently being followed is only tentative. This
        # deliberately does NOT pause/return here - the UGV keeps advancing
        # through its current best plan (below) while a fresh one is being
        # computed in the background; _on_plan_response swaps it in only if
        # and when a reply actually arrives, same "never stop moving while
        # waiting on a slower global answer" principle Nav2's own
        # local-costmap/global-planner split already relies on.
        if self._enable_frontier_replan and _plan_is_tentative(
            self._current_plan_has_frontier, self._current_plan_has_anomaly
        ):
            now = time.monotonic()
            if _should_replan(now, self._last_replan_time, self._frontier_replan_interval_sec):
                self._last_replan_time = now
                # See frontier_replan_min_progress_m's own declare_parameter
                # comment for the "useless motion" this fixes: only
                # actually ask for (and adopt) a different plan if the UGV
                # hasn't covered meaningful ground since the last time this
                # fired - a plan that's visibly working is left alone
                # rather than preempted just because the interval elapsed.
                if not _made_progress(
                    self._current_xy, self._last_replan_check_xy, self._frontier_replan_min_progress_m
                ):
                    self._request_plan()
                self._last_replan_check_xy = self._current_xy

        if self._waypoints is None or self._next_index >= len(self._waypoints):
            return

        # See enable_stuck_recovery's own declare_parameter comment for the
        # live stall this catches: unlike the frontier-replan loop above
        # (which only re-requests while the CURRENT plan is tentative),
        # this fires regardless of why progress stopped - a bad PRM
        # waypoint too close to a wall, a local-costmap deadlock, or
        # anything else that leaves Nav2 sitting still with a "confirmed"
        # plan it simply can't execute.
        if self._enable_stuck_recovery:
            if _made_progress(self._current_xy, self._last_progress_xy, self._stuck_progress_radius_m):
                self._last_progress_xy = self._current_xy
                self._last_progress_time = time.monotonic()
            else:
                now = time.monotonic()
                if _should_replan(now, self._last_progress_time, self._stuck_timeout_sec):
                    self.get_logger().warn(
                        f"No progress in over {self._stuck_timeout_sec}s (stuck near "
                        f"{self._current_xy}) - requesting a fresh plan (stuck recovery)."
                    )
                    self._last_progress_time = now
                    self._request_plan()

        target = self._waypoints[self._next_index]
        if self._next_index == 0:
            if self._publish_goal_unless_voxel_confirmed_blocked(self._current_xy, target):
                self._next_index += 1
            return

        prev = self._waypoints[self._next_index - 1]
        distance = math.hypot(prev[0] - self._current_xy[0], prev[1] - self._current_xy[1])
        if distance < self._reached_radius:
            if self._publish_goal_unless_voxel_confirmed_blocked(prev, target):
                self.get_logger().info(f"Reached waypoint {self._next_index - 1}, publishing next.")
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
        # REAL BUG FOUND LIVE (GUI-mode run, 2026-09-24, user report: robot
        # sat at spawn for 170+ seconds, no plan ever requested again after
        # the very first attempt): `future.result()` raises if the response
        # never actually arrives - seen live as planner_server logging
        # `RuntimeWarning: failed to send response (timeout): client will
        # not receive response` for the FIRST get_plan call, made while
        # /elevation_map/the rest of the stack was still under heavy
        # startup CPU load. Without this try/except, that exception
        # propagated straight out of this done-callback and `self._requested`
        # was never reset back to False - `_odom_callback`'s "if not
        # self._requested" retry path never fires again, so losing even
        # this one very first request this way stalled the UGV at spawn
        # permanently, forever. Treated exactly like "no plan found yet"
        # below: retry if no plan has ever succeeded, otherwise just log
        # and let the next periodic replan try again.
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"get_plan request failed ({exc!r}) - retrying on the next odom message.")
            if self._waypoints is None:
                self._requested = False
            return
        if not response.plan.poses:
            if self._waypoints is None:
                # No plan has EVER succeeded yet - this is the initial
                # request failing (e.g. the map doesn't cover the goal yet).
                # Keep the original retry-on-next-odom-message behavior.
                self.get_logger().warn("get_plan returned no path - retrying on the next odom message.")
                self._requested = False  # allow _odom_callback to try again
            else:
                # A background frontier-replan attempt failed to find
                # anything (e.g. the tunnel still hasn't been resolved with
                # real data yet) - this must NOT discard the plan already
                # being followed, or a periodic refresh that briefly finds
                # nothing better would strand the UGV mid-route for no
                # reason. Just keep going with what's already working;
                # _should_replan will try again after the next interval.
                self.get_logger().info(
                    "frontier replan found no path yet - continuing on the existing plan."
                )
            return

        new_waypoints = [(p.pose.position.x, p.pose.position.y) for p in response.plan.poses]

        # See _waypoints_effectively_equal's own docstring: when a
        # background replan happens to return the exact plan already being
        # followed, don't reset progress/republish and needlessly preempt
        # Nav2's in-flight NavigateToPose goal for a route that hasn't
        # changed. In practice this fires less often than hoped (the map
        # keeps drifting cell-by-cell - see that docstring), so it's a
        # bonus, not the fix for the stall below.
        if self._waypoints is not None and _waypoints_effectively_equal(self._waypoints, new_waypoints):
            self._last_replan_time = time.monotonic()
            return

        self._waypoints = new_waypoints
        # See _skip_reached_leading_waypoints's own docstring for the real
        # bug this fixes: waypoint 0 is always ~the robot's own current
        # position, and publishing it as a real goal - immediately followed
        # by a second /goal_pose for the actual next target - raced
        # bt_navigator's own goal-success reporting and left Nav2 idle with
        # no active goal for seconds at a time. Skipping straight to the
        # first not-yet-reached waypoint means only one /goal_pose is ever
        # published per plan adoption.
        self._next_index = _skip_reached_leading_waypoints(
            self._waypoints, self._current_xy, self._reached_radius
        )
        self.get_logger().info(f"Received a {len(self._waypoints)}-waypoint path - beginning navigation.")

        # See enable_stuck_recovery's own declare_parameter comment - a
        # freshly-adopted plan gets its own full stuck-timeout window
        # rather than inheriting whatever progress state applied to the
        # previous plan/segment.
        self._last_progress_xy = self._current_xy
        self._last_progress_time = time.monotonic()

        # step06 Phase 6 BUG FOUND LIVE (fixed here): _last_replan_time was
        # previously only ever set from INSIDE the periodic-replan/blocked-
        # replan code paths, never from a plan's actual arrival. That left a
        # real gap: `_should_replan` treats `last_replan_time=None` as "never
        # replanned, go ahead" - so the very FIRST time a plan became
        # tentative (has_frontier/has_anomaly flipping True), the periodic
        # check's OWN cooldown hadn't started yet and fired again almost
        # immediately (next odom tick, ~33ms later) rather than waiting the
        # full frontier_replan_interval_sec. Live evidence from the Phase 6
        # acceptance run: three successful plans arrived within about 6.5
        # real seconds of each other (a 5-second cooldown should have spread
        # them out far more), and the resulting rapid-fire /goal_pose
        # republishing repeatedly preempted Nav2's in-flight NavigateToPose
        # action - under this environment's real, heavy CPU contention
        # (`emap`'s elevation_mapping_node + this node's own voxel_map_node +
        # Gazebo together routinely exceed the machine's core count - see
        # step06 Phase 6 results for the measured load), each preemption is
        # expensive enough that the UGV ended up driving well past its
        # intended stop instead of settling at it. Stamping the cooldown
        # here, on every successful plan arrival regardless of which code
        # path requested it, means "don't ask for ANOTHER plan within
        # frontier_replan_interval_sec of having just received one" - the
        # plain, correct meaning of a request cooldown - closing the gap
        # that let the very first tentative-plan transition double-fire.
        self._last_replan_time = time.monotonic()


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
