"""Unit tests for nav.waypoint_follower's pure, ROS-free logic: the
frontier-replan cooldown check, (step06 Phase 4) the verification-region
bounding-box helper, and (step06 Phase 5) the report-back gate that decides
whether a voxel verdict is a genuinely new resolution worth telling
planner_node about. Everything else in that module is a thin ROS node
wrapper (subscriptions/service calls/publishers) and is only verified live
(see docs/work-docs/nav/step05_frontier_tunnel_navigation.md and
step06_hybrid_3d_voxel_navigation.md) - same "pull the plain logic out and
unit test just that" discipline every other nav/emap module already follows
for its own timing/cooldown checks (e.g. this mirrors the retry_interval_sec
idea already used elsewhere in this same file).
"""
from nav.waypoint_follower import (
    _made_progress,
    _new_resolution_to_report,
    _plan_is_tentative,
    _segment_bounding_box,
    _should_replan,
    _skip_reached_leading_waypoints,
    _waypoints_effectively_equal,
)


class TestPlanIsTentative:
    # step06 Phase 6 regression test - see _plan_is_tentative's own docstring
    # for the real live stall this pins down: the periodic replan trigger
    # used to watch only the frontier flag, so a plan that had resolved
    # frontier-wise but was STILL anomaly-tentative silently stopped ever
    # being refreshed.
    def test_neither_flag_set_is_not_tentative(self):
        assert _plan_is_tentative(has_frontier=False, has_anomaly=False) is False

    def test_frontier_only_is_tentative(self):
        assert _plan_is_tentative(has_frontier=True, has_anomaly=False) is True

    def test_anomaly_only_is_tentative(self):
        # The exact case that stalled live: frontier resolved, anomaly did not.
        assert _plan_is_tentative(has_frontier=False, has_anomaly=True) is True

    def test_both_flags_set_is_tentative(self):
        assert _plan_is_tentative(has_frontier=True, has_anomaly=True) is True


class TestShouldReplan:
    def test_true_the_first_time_before_any_replan_has_happened(self):
        assert _should_replan(now=100.0, last_replan_time=None, interval_sec=5.0) is True

    def test_false_before_the_interval_has_elapsed(self):
        assert _should_replan(now=102.0, last_replan_time=100.0, interval_sec=5.0) is False

    def test_true_once_the_interval_has_fully_elapsed(self):
        assert _should_replan(now=105.0, last_replan_time=100.0, interval_sec=5.0) is True

    def test_true_well_past_the_interval(self):
        assert _should_replan(now=999.0, last_replan_time=100.0, interval_sec=5.0) is True


class TestWaypointsEffectivelyEqual:
    # step06 Phase 6 regression test (bug #2, found live in the same GUI-mode
    # run as TestPlanIsTentative above) - see _waypoints_effectively_equal's
    # own docstring: an unseeded PRM roadmap meant every background replan
    # handed the follower a brand new random target, and resetting progress
    # to reach it every time meant the UGV never actually got anywhere.
    def test_none_old_is_never_equal(self):
        # The very first plan ever received - must always be adopted.
        assert _waypoints_effectively_equal(None, [(1.0, 2.0)]) is False

    def test_different_lengths_are_not_equal(self):
        assert _waypoints_effectively_equal([(1.0, 2.0)], [(1.0, 2.0), (3.0, 4.0)]) is False

    def test_identical_waypoints_are_equal(self):
        old = [(-3.0, 0.0), (-0.8, -0.9), (3.0, 0.0)]
        new = [(-3.0, 0.0), (-0.8, -0.9), (3.0, 0.0)]
        assert _waypoints_effectively_equal(old, new) is True

    def test_tiny_float_noise_within_tolerance_is_equal(self):
        old = [(-3.0, 0.0), (-0.8, -0.9)]
        new = [(-3.0001, 0.0002), (-0.7998, -0.9003)]
        assert _waypoints_effectively_equal(old, new, tol_m=0.05) is True

    def test_a_genuinely_different_random_target_is_not_equal(self):
        # The exact live failure: waypoint 0 (current position) unchanged,
        # but the real next waypoint is a totally different PRM sample.
        old = [(-3.0, 0.0), (-0.8, -0.9)]
        new = [(-3.0, 0.0), (-1.0, 0.9)]
        assert _waypoints_effectively_equal(old, new) is False


class TestMadeProgress:
    # step06 Phase 6 regression test - the stuck-recovery watchdog's own
    # progress check. See _made_progress's own docstring and
    # enable_stuck_recovery's declare_parameter comment for the live
    # DWB-gives-up-and-never-recovers stall this feeds into.
    def test_no_reference_yet_counts_as_progress(self):
        assert _made_progress((1.0, 2.0), None, progress_radius_m=0.15) is True

    def test_movement_past_radius_is_progress(self):
        assert _made_progress((1.0, 2.0), (0.8, 2.0), progress_radius_m=0.15) is True

    def test_movement_within_radius_is_not_progress(self):
        assert _made_progress((1.0, 2.0), (1.05, 2.0), progress_radius_m=0.15) is False

    def test_no_movement_at_all_is_not_progress(self):
        assert _made_progress((1.0, 2.0), (1.0, 2.0), progress_radius_m=0.15) is False


class TestSkipReachedLeadingWaypoints:
    # step06 Phase 6 regression test - the ACTUAL root cause of "the UGV
    # moves here and there and just stops": see _skip_reached_leading_
    # waypoints's own docstring for the live bt_navigator race this avoids
    # by never publishing waypoint 0 (always ~the robot's own current
    # position) as a real goal at all.
    def test_waypoint_zero_already_at_current_position_is_skipped(self):
        waypoints = [(-3.0, 0.0), (-0.8, -0.9), (3.0, 0.0)]
        assert _skip_reached_leading_waypoints(waypoints, (-3.0, 0.0), reached_radius=0.3) == 1

    def test_no_leading_waypoint_within_radius_returns_zero(self):
        waypoints = [(5.0, 5.0), (6.0, 6.0)]
        assert _skip_reached_leading_waypoints(waypoints, (0.0, 0.0), reached_radius=0.3) == 0

    def test_multiple_leading_waypoints_within_radius_are_all_skipped(self):
        waypoints = [(0.0, 0.0), (0.05, 0.0), (5.0, 5.0)]
        assert _skip_reached_leading_waypoints(waypoints, (0.0, 0.0), reached_radius=0.3) == 2

    def test_entire_plan_already_reached_returns_len(self):
        waypoints = [(0.0, 0.0), (0.1, 0.0)]
        assert _skip_reached_leading_waypoints(waypoints, (0.0, 0.0), reached_radius=0.3) == 2

    def test_empty_plan_returns_zero(self):
        assert _skip_reached_leading_waypoints([], (0.0, 0.0), reached_radius=0.3) == 0


class TestSegmentBoundingBox:
    def test_box_covers_both_endpoints_with_margin(self):
        # Segment crossing tunnel_test.world's tunnel mouths (see step06
        # Phase 2/3 results) - a real shape, not an arbitrary pair of points.
        box = _segment_bounding_box(from_xy=(-0.5, 0.0), to_xy=(0.8, 0.0), margin_m=0.3)
        x_min, y_min, x_max, y_max = box
        assert x_min == -0.8  # -0.5 - 0.3
        assert x_max == 1.1   # 0.8 + 0.3
        assert y_min == -0.3  # both endpoints at y=0.0, so just -margin/+margin
        assert y_max == 0.3

    def test_endpoint_order_does_not_matter(self):
        # The UGV could be driving either direction along a segment -
        # min/max must come out the same regardless of which point is
        # "from" and which is "to".
        forward = _segment_bounding_box(from_xy=(0.0, 0.0), to_xy=(2.0, 1.0), margin_m=0.1)
        backward = _segment_bounding_box(from_xy=(2.0, 1.0), to_xy=(0.0, 0.0), margin_m=0.1)
        assert forward == backward

    def test_zero_margin_gives_a_tight_box_around_the_two_points(self):
        box = _segment_bounding_box(from_xy=(1.0, 1.0), to_xy=(1.0, 1.0), margin_m=0.0)
        assert box == (1.0, 1.0, 1.0, 1.0)


class TestNewResolutionToReport:
    _REGION = (-0.8, -0.3, 1.1, 0.3)  # a real tunnel-crossing bbox (see TestSegmentBoundingBox above)

    def test_unknown_verdict_is_never_reportable(self):
        # "unknown" isn't a resolution at all - see
        # nav.voxel_map.check_region_headroom's own three-way docstring.
        assert _new_resolution_to_report("unknown", self._REGION, None) is False

    def test_no_region_yet_is_not_reportable(self):
        # A verdict can't be attributed to anything before this node has
        # ever actually sent a verify_region_request.
        assert _new_resolution_to_report("passable", None, None) is False

    def test_first_settled_verdict_for_a_region_is_reportable(self):
        assert _new_resolution_to_report("passable", self._REGION, None) is True

    def test_repeating_the_same_region_and_verdict_is_not_reportable(self):
        assert _new_resolution_to_report("blocked", self._REGION, (self._REGION, "blocked")) is False

    def test_a_different_verdict_for_the_same_region_is_reportable(self):
        # e.g. a stale "blocked" from before more sensor data arrived, now
        # correctly resolving to "passable" - a real change worth reporting.
        assert _new_resolution_to_report("passable", self._REGION, (self._REGION, "blocked")) is True

    def test_a_different_region_is_reportable_even_with_the_same_verdict(self):
        other_region = (2.0, -0.3, 3.9, 0.3)
        assert _new_resolution_to_report("passable", other_region, (self._REGION, "passable")) is True
