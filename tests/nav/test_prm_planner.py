"""Unit tests for nav.prm_planner: synthetic boolean grids, no ROS/Gazebo -
same "prove the algorithm alone before wiring it into a live node" discipline
every `emap` algorithm module (fusion.py, drift.py) already follows.
"""
import numpy as np
import pytest

from nav.prm_planner import PlanResult, _line_is_walkable, plan

RESOLUTION = 0.5
CENTER_X = 0.0
CENTER_Y = 0.0


def make_mask(n=20, blocked=None):
    mask = np.ones((n, n), dtype=bool)
    if blocked is not None:
        mask[blocked] = False
    return mask


class TestLineIsWalkable:
    def test_straight_line_through_open_space_is_clear(self):
        mask = make_mask(10)
        assert _line_is_walkable(mask, 0, 0, 9, 0) is True

    def test_line_crossing_a_single_blocked_cell_is_not_clear(self):
        mask = make_mask(10)
        mask[5, 0] = False
        assert _line_is_walkable(mask, 0, 0, 9, 0) is False


class TestPlanOnOpenGrid:
    def test_finds_a_valid_path_between_two_open_points(self):
        mask = make_mask(30)
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-5.0, -5.0), goal_xy=(5.0, 5.0),
            num_samples=150, connect_radius_m=3.0,
            rng=np.random.default_rng(42),
        )
        assert result.valid
        assert len(result.waypoints) >= 2
        # First/last waypoints should land near the requested start/goal
        # (within one cell's worth of rounding - see world_to_grid/grid_to_world).
        assert result.waypoints[0] == pytest.approx((-5.0, -5.0), abs=RESOLUTION)
        assert result.waypoints[-1] == pytest.approx((5.0, 5.0), abs=RESOLUTION)

    def test_every_waypoint_pair_has_line_of_sight_through_walkable_space(self):
        # A real correctness guarantee, not just "a path exists": every
        # consecutive pair of returned waypoints must be an edge the planner
        # actually verified with _line_is_walkable - re-check it here on
        # the returned waypoints themselves as an end-to-end sanity check.
        from emap.utils.coord_transform import world_to_grid

        mask = make_mask(30)
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-6.0, 0.0), goal_xy=(6.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(1),
        )
        assert result.valid
        cell_n = mask.shape[0]
        for (x0, y0), (x1, y1) in zip(result.waypoints, result.waypoints[1:]):
            r0, c0 = world_to_grid(x0, y0, CENTER_X, CENTER_Y, RESOLUTION, cell_n)
            r1, c1 = world_to_grid(x1, y1, CENTER_X, CENTER_Y, RESOLUTION, cell_n)
            assert _line_is_walkable(mask, int(r0), int(c0), int(r1), int(c1))


class TestPlanRejectsBadQueries:
    def test_start_in_non_walkable_cell_is_exempted_via_footprint_clearing(self):
        # REAL BUG FOUND LIVE: a parked nav_ugv gets fused into its own
        # elevation map by the UAV's depth camera (it's a physical object,
        # not just empty terrain), which can mark the UGV's own current
        # cell LETHAL - permanently locking it out of planning any route
        # away from wherever it's sitting, since the old behavior rejected
        # any query whose START cell wasn't walkable. Now the start's own
        # cell is exempted (a small footprint radius forced walkable,
        # mirroring standard costmap footprint-clearing) - this exact
        # scenario (only the single start cell blocked, everything else
        # open) must now succeed.
        mask = make_mask(20)
        mask[10, 10] = False
        from emap.utils.coord_transform import grid_to_world
        start_x, start_y = grid_to_world(10, 10, CENTER_X, CENTER_Y, RESOLUTION, 20)
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(float(start_x), float(start_y)), goal_xy=(3.0, 3.0),
            num_samples=100, rng=np.random.default_rng(0),
        )
        assert result.valid is True

    def test_goal_in_non_walkable_cell_is_still_rejected(self):
        # The footprint-clearing exemption is deliberately NOT applied to
        # the goal - see prm_planner.py's module docstring for why ("trust
        # that I'm not sitting inside a wall right now" only makes sense
        # for a live position report, not for something you're being asked
        # to drive INTO).
        mask = make_mask(20)
        mask[10, 10] = False
        from emap.utils.coord_transform import grid_to_world
        goal_x, goal_y = grid_to_world(10, 10, CENTER_X, CENTER_Y, RESOLUTION, 20)
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-5.0, -5.0), goal_xy=(float(goal_x), float(goal_y)),
            num_samples=100, rng=np.random.default_rng(0),
        )
        assert result == PlanResult(waypoints=[], valid=False)

    def test_footprint_clearing_does_not_bridge_a_real_wall(self):
        # The exemption is LOCAL (a small radius around start) - it must
        # not punch a hole all the way through a genuinely thick wall just
        # because the start happens to sit deep inside it. The wall here
        # (6 columns) is deliberately much thicker than the clearing radius
        # (1 cell each side), so the cleared disk around start can't reach
        # either open side.
        n = 30
        mask = make_mask(n)
        mask[:, 10:16] = False  # a thick wall, columns 10-15
        from emap.utils.coord_transform import grid_to_world
        start_x, start_y = grid_to_world(15, 12, CENTER_X, CENTER_Y, RESOLUTION, n)  # deep inside the wall
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(float(start_x), float(start_y)), goal_xy=(6.0, 0.0),
            num_samples=200, connect_radius_m=3.0, footprint_radius_m=0.5,
            rng=np.random.default_rng(5),
        )
        assert result.valid is False

    def test_no_walkable_cells_at_all_is_rejected(self):
        mask = np.zeros((10, 10), dtype=bool)
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(0.0, 0.0), goal_xy=(1.0, 1.0),
            num_samples=50, rng=np.random.default_rng(0),
        )
        assert result.valid is False


class TestPlanOnFullyBlockedCorridor:
    def test_two_rooms_separated_by_a_solid_wall_have_no_path(self):
        # A full-height wall down the middle column with no gap anywhere -
        # start and goal are each individually walkable, but genuinely
        # unreachable from each other. Confirms the planner reports
        # valid=False rather than fabricating a path through the wall.
        n = 20
        mask = make_mask(n)
        mask[:, 10] = False  # solid wall, every row
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(7),
        )
        assert result.valid is False

    def test_a_narrow_gap_in_the_wall_is_found(self):
        # Same wall, but with one gap - the whole point of a PRM is to find
        # the way through it (with enough samples and a big enough radius to
        # reliably land a sample in the gap).
        n = 20
        mask = make_mask(n)
        mask[:, 10] = False
        mask[9, 10] = True  # the gap
        mask[10, 10] = True
        result = plan(
            mask, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
        )
        assert result.valid is True


class TestFrontierAwarePlanning:
    """See docs/work-docs/nav/step05_frontier_tunnel_navigation.md: a
    tunnel/culvert's interior stays `is_valid=False` FOREVER under the
    existing pipeline (the UAV's overhead camera structurally cannot see
    inside it), even though it may be the only route to the goal. These
    tests use the exact same "solid wall down the middle" setup as
    TestPlanOnFullyBlockedCorridor above, except the one gap that exists is
    never marked walkable - only `frontier_mask` (simulating "unobserved,
    but bordered by known walkable space, and worth investigating") knows
    about it - proving `allow_frontier` is what actually unlocks it, and
    that a plain `plan()` call still correctly reports no path, exactly as
    before this feature existed.
    """

    def _wall_with_unobserved_gap(self, n=20):
        walkable = make_mask(n)
        walkable[:, 10] = False  # solid wall, every row - same as the sibling test above
        frontier = np.zeros((n, n), dtype=bool)
        frontier[9, 10] = True  # the "tunnel interior" - unobserved, not walkable
        frontier[10, 10] = True
        return walkable, frontier

    def test_default_call_still_reports_no_path_through_an_unobserved_gap(self):
        # A caller that never opts in to allow_frontier must see IDENTICAL
        # behavior to before this feature existed - an unobserved gap is
        # just as impassable as a solid wall by default.
        walkable, frontier = self._wall_with_unobserved_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(7),
        )
        assert result.valid is False
        assert result.has_frontier_segments is False

    def test_allow_frontier_without_a_mask_is_a_no_op(self):
        # Passing allow_frontier=True but no frontier_mask must not crash,
        # and must not change the outcome - there's nothing to allow through.
        walkable, _frontier = self._wall_with_unobserved_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(7),
            allow_frontier=True,
        )
        assert result.valid is False

    def test_allow_frontier_with_a_mask_finds_a_tentative_path_through_the_gap(self):
        walkable, frontier = self._wall_with_unobserved_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            frontier_mask=frontier,
            allow_frontier=True,
        )
        assert result.valid is True
        assert result.has_frontier_segments is True

    def test_a_path_that_never_needed_a_frontier_cell_is_not_flagged_tentative(self):
        # Same scenario as TestPlanOnFullyBlockedCorridor's narrow-gap test,
        # but with allow_frontier=True and a (here, irrelevant) frontier_mask
        # supplied - the gap is ALREADY genuinely walkable, so the plan found
        # doesn't actually need to rely on the frontier cell at all, and must
        # not be mislabeled as tentative just because the option was enabled.
        n = 20
        walkable = make_mask(n)
        walkable[:, 10] = False
        walkable[9, 10] = True  # a REAL, confirmed gap this time
        walkable[10, 10] = True
        frontier = np.zeros((n, n), dtype=bool)  # nowhere else is frontier
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            frontier_mask=frontier,
            allow_frontier=True,
        )
        assert result.valid is True
        assert result.has_frontier_segments is False


class TestAnomalyAwarePlanning:
    """See docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md (Phase 3):
    unlike a frontier cell (merely unmeasured), an anomaly cell is one the
    map actively scored LETHAL but suspects is a step-discontinuity data
    artifact (e.g. a tunnel roof) rather than a genuine obstacle - see
    nav/anomaly.py. Same "solid wall down the middle" setup as the frontier
    tests above, except the gap is only known via `anomaly_mask`, not
    `walkable_mask` - proving `allow_anomaly` is what unlocks it, that a
    plain call still reports no path exactly as before, AND (the part that
    makes anomaly handling different from frontier handling) that a genuinely
    confirmed-safe alternate route is preferred over a shorter anomaly route
    when both exist.
    """

    def _wall_with_anomaly_gap(self, n=20):
        walkable = make_mask(n)
        walkable[:, 10] = False  # solid wall, every row
        anomaly = np.zeros((n, n), dtype=bool)
        anomaly[9, 10] = True  # the "tunnel roof" - scored LETHAL, suspected artifact
        anomaly[10, 10] = True
        return walkable, anomaly

    def test_default_call_still_reports_no_path_through_an_anomaly_gap(self):
        # A caller that never opts in to allow_anomaly must see IDENTICAL
        # behavior to before this feature existed - a suspected-artifact gap
        # is just as impassable as a solid wall by default.
        walkable, _anomaly = self._wall_with_anomaly_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(7),
        )
        assert result.valid is False
        assert result.has_anomaly_segments is False

    def test_allow_anomaly_without_a_mask_is_a_no_op(self):
        # Passing allow_anomaly=True but no anomaly_mask must not crash, and
        # must not change the outcome - there's nothing to allow through.
        walkable, _anomaly = self._wall_with_anomaly_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=200, connect_radius_m=3.0,
            rng=np.random.default_rng(7),
            allow_anomaly=True,
        )
        assert result.valid is False

    def test_allow_anomaly_with_a_mask_finds_a_tentative_path_when_it_is_the_only_route(self):
        walkable, anomaly = self._wall_with_anomaly_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            anomaly_mask=anomaly,
            allow_anomaly=True,
        )
        assert result.valid is True
        assert result.has_anomaly_segments is True
        assert result.has_frontier_segments is False  # this path never touched frontier cells

    def test_a_path_that_never_needed_an_anomaly_cell_is_not_flagged_tentative(self):
        # The gap is ALREADY genuinely walkable here, so the plan found
        # doesn't actually rely on the (here, irrelevant) anomaly_mask at
        # all, and must not be mislabeled as tentative just because the
        # option was enabled.
        n = 20
        walkable = make_mask(n)
        walkable[:, 10] = False
        walkable[9, 10] = True  # a REAL, confirmed gap
        walkable[10, 10] = True
        anomaly = np.zeros((n, n), dtype=bool)  # nowhere is anomaly
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            anomaly_mask=anomaly,
            allow_anomaly=True,
        )
        assert result.valid is True
        assert result.has_anomaly_segments is False

    def test_confirmed_route_is_preferred_over_a_closer_anomaly_route(self):
        # The behavior that makes anomaly handling genuinely different from
        # frontier handling: two gaps exist in the same wall. An
        # ANOMALY-only gap sits right on the direct line between start and
        # goal (rows 9-10, the "easy", short route). A REAL,
        # confirmed-walkable gap sits far off to one side (rows 0-1),
        # reachable only via a real, measurably longer detour. Even though
        # the anomaly route is objectively shorter, the planner must still
        # choose the longer-but-confirmed route rather than gamble on a cell
        # the map explicitly scored LETHAL - see _ANOMALY_EDGE_PENALTY.
        n = 20
        walkable = make_mask(n)
        walkable[:, 10] = False
        walkable[0, 10] = True  # confirmed gap, far from the direct line
        walkable[1, 10] = True
        anomaly = np.zeros((n, n), dtype=bool)
        anomaly[9, 10] = True  # anomaly gap, right on the direct line
        anomaly[10, 10] = True
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            anomaly_mask=anomaly,
            allow_anomaly=True,
        )
        assert result.valid is True
        assert result.has_anomaly_segments is False  # took the longer, confirmed route instead

    def test_anomaly_route_still_used_when_it_is_the_only_option_despite_the_penalty(self):
        # The penalty makes anomaly routes UNattractive, not unusable - when
        # there is genuinely no alternative (same setup as
        # test_allow_anomaly_with_a_mask_finds_a_tentative_path_when_it_is_the_only_route,
        # restated here to make the "penalty is finite, not infinite"
        # property explicit as its own named guarantee), the path must still
        # be found rather than reported as valid=False.
        walkable, anomaly = self._wall_with_anomaly_gap()
        result = plan(
            walkable, RESOLUTION, CENTER_X, CENTER_Y,
            start_xy=(-4.0, 0.0), goal_xy=(4.0, 0.0),
            num_samples=400, connect_radius_m=4.0,
            rng=np.random.default_rng(3),
            anomaly_mask=anomaly,
            allow_anomaly=True,
        )
        assert result.valid is True
