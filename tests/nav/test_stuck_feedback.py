"""Unit tests for nav.stuck_feedback and its use inside GlobalPlanner - making
a deterministic planner react when the robot is not getting anywhere."""
import math

import numpy as np
import pytest

from nav import astar_planner as ap
from nav.global_planner import GlobalPlanner, MapContext
from nav.stuck_feedback import StuckTracker, route_ahead_points

RES = 0.1
N = 60


def _xy(row, col):
    return ((col - N / 2.0) * RES, (row - N / 2.0) * RES)


class TestStuckTracker:
    def test_a_robot_that_keeps_moving_is_never_stuck(self):
        tr = StuckTracker(window_sec=10.0)
        for k in range(10):
            assert not tr.note_request(now=k * 5.0, xy=(k * 1.0, 0.0), route_in_progress=True)

    def test_sitting_still_with_a_route_for_a_window_is_stuck(self):
        tr = StuckTracker(window_sec=10.0)
        assert not tr.note_request(0.0, (1.0, 1.0), True)
        assert not tr.note_request(5.0, (1.02, 1.0), True)
        assert tr.note_request(11.0, (1.05, 1.0), True)

    def test_waiting_with_no_route_is_not_stuck(self):
        tr = StuckTracker(window_sec=10.0)
        tr.note_request(0.0, (1.0, 1.0), False)
        assert not tr.note_request(30.0, (1.0, 1.0), False)

    def test_it_only_fires_once_per_episode(self):
        tr = StuckTracker(window_sec=10.0)
        tr.note_request(0.0, (0.0, 0.0), True)
        assert tr.note_request(11.0, (0.0, 0.0), True)
        assert not tr.note_request(12.0, (0.0, 0.0), True)  # fresh window just started
        assert tr.note_request(23.0, (0.0, 0.0), True)  # next episode, a full window later

    def test_moving_away_restarts_the_clock(self):
        tr = StuckTracker(window_sec=10.0)
        tr.note_request(0.0, (0.0, 0.0), True)
        tr.note_request(8.0, (2.0, 0.0), True)  # moved
        assert not tr.note_request(12.0, (2.0, 0.0), True)  # only 4 s at the new spot

    def test_zones_expire(self):
        tr = StuckTracker(zone_sec=60.0)
        tr.add_zone([(1.0, 1.0)], now=0.0)
        assert tr.multiplier_grid((20, 20), RES, 0.0, 0.0) is not None
        tr.note_request(61.0, (5.0, 5.0), False)  # any call prunes expired zones
        assert tr.multiplier_grid((20, 20), RES, 0.0, 0.0) is None

    def test_multiplier_grid_covers_the_zone_only(self):
        tr = StuckTracker(zone_radius_m=0.3, zone_multiplier=6.0)
        tr.add_zone([(0.0, 0.0)], now=0.0)
        g = tr.multiplier_grid((20, 20), RES, 0.0, 0.0)
        assert g[10, 10] == 6.0  # at the zone centre
        assert g[0, 0] == 1.0  # far away
        assert g.min() == 1.0

    def test_no_zones_gives_no_grid(self):
        assert StuckTracker().multiplier_grid((10, 10), RES, 0.0, 0.0) is None


class TestRouteAheadPoints:
    def test_points_run_along_the_next_stretch_of_the_route(self):
        pts = route_ahead_points([(0.0, 0.0), (10.0, 0.0)], (2.0, 0.0), length_m=1.0, step_m=0.25)
        assert pts[0] == pytest.approx((2.0, 0.0))
        assert pts[-1][0] == pytest.approx(3.0, abs=0.01)
        assert all(abs(y) < 1e-9 for _, y in pts)

    def test_it_follows_a_corner(self):
        pts = route_ahead_points([(0.0, 0.0), (1.0, 0.0), (1.0, 5.0)], (0.5, 0.0), length_m=1.5, step_m=0.5)
        assert any(abs(x - 1.0) < 1e-9 and y > 0.4 for x, y in pts)

    def test_does_not_run_past_the_end_of_the_route(self):
        pts = route_ahead_points([(0.0, 0.0), (1.0, 0.0)], (0.5, 0.0), length_m=5.0)
        assert max(x for x, _ in pts) <= 1.0 + 1e-9


def _two_gap_world():
    """A wall with two gaps, the TOP one clearly shorter for a robot at the
    west side's vertical middle-top."""
    w = np.ones((N, N), dtype=bool)
    w[:, 28:32] = False
    w[8:14, 28:32] = True  # top gap (close to the robot)
    w[46:52, 28:32] = True  # bottom gap (far)
    return w


def _ctx(w):
    return MapContext(walkable=w, resolution=RES, center_x=0.0, center_y=0.0)


def _planner(**kw):
    return GlobalPlanner(ap.PlannerParams(mask_rim_m=0.0), stuck_feedback=True, stuck_window_sec=15.0, **kw)


def _uses_top_gap(res):
    """Which gap does the route cross? The wall is at x ~ 0; the top gap is at
    y ~ -1.9, the bottom one at y ~ +1.9 (the start/goal rows sit at y=-1.2)."""
    return not any(p[1] > 1.0 for p in res.waypoints)


class TestPlannerReactsToBeingStuck:
    def test_without_feedback_the_same_route_comes_back_forever(self):
        gp = GlobalPlanner(ap.PlannerParams(mask_rim_m=0.0))  # feedback off
        w = _two_gap_world()
        start, goal = _xy(18, 5), _xy(18, 55)
        first = gp.plan(_ctx(w), start, goal, now=0.0)
        later = gp.plan(_ctx(w), start, goal, now=60.0)
        assert later.waypoints == first.waypoints and _uses_top_gap(later)

    def test_after_being_stuck_the_planner_switches_to_the_other_gap(self):
        gp = _planner()
        w = _two_gap_world()
        start, goal = _xy(18, 20), _xy(18, 55)  # standing right in front of the top gap
        first = gp.plan(_ctx(w), start, goal, now=0.0)
        assert _uses_top_gap(first)
        gp.plan(_ctx(w), start, goal, now=8.0)  # not stuck yet
        stuck = gp.plan(_ctx(w), start, goal, now=20.0)  # >15 s without moving
        assert "stuck" in gp.last_note
        assert not _uses_top_gap(stuck), "should have routed away from the stretch that failed"

    def test_the_penalty_expires_and_the_short_route_returns(self):
        gp = _planner()
        gp.stuck.zone_sec = 30.0
        w = _two_gap_world()
        start, goal = _xy(18, 20), _xy(18, 55)
        gp.plan(_ctx(w), start, goal, now=0.0)
        gp.plan(_ctx(w), start, goal, now=20.0)  # stuck -> penalised until t=50
        later = gp.plan(_ctx(w), _xy(18, 10), goal, now=200.0)  # long after, robot elsewhere
        assert _uses_top_gap(later)

    def test_a_moving_robot_never_triggers_it(self):
        gp = _planner()
        w = _two_gap_world()
        goal = _xy(18, 55)
        for k, col in enumerate(range(5, 20, 3)):
            gp.plan(_ctx(w), _xy(18, col), goal, now=k * 6.0)
            assert "stuck" not in gp.last_note

    def test_a_viewpoint_the_robot_has_reached_is_not_stuck(self):
        # exploration viewpoints are places to sit and look
        from nav.walkability import compute_frontier_mask

        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:30] = True
        w[26:36, 30:32] = True
        valid[:, 32:] = False
        frontier = compute_frontier_mask(w, valid)
        ctx = MapContext(
            walkable=w, resolution=RES, center_x=0.0, center_y=0.0,
            frontier=frontier, unobserved=~valid,
        )  # fmt: skip
        gp = GlobalPlanner(
            ap.PlannerParams(mask_rim_m=0.0), enable_exploration=True, stuck_feedback=True, stuck_window_sec=15.0
        )
        first = gp.plan(ctx, _xy(30, 5), _xy(30, 55), now=0.0)
        assert first.is_exploration
        at = first.goal_xy
        gp.plan(ctx, at, _xy(30, 55), now=1.0)
        gp.plan(ctx, at, _xy(30, 55), now=40.0)
        assert "stuck" not in gp.last_note
