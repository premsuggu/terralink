"""Unit tests for nav.global_planner - the plan -> memory -> explore decision.

This is the code BOTH planner_node and the offline replay run, so these tests
pin down the behaviour of the whole decision, including the
investigate-before-gambling rule."""
import math

import numpy as np
import pytest

from nav import astar_planner as ap
from nav.global_planner import GlobalPlanner, MapContext
from nav.walkability import compute_frontier_mask

RES = 0.1
N = 60


def _xy(row, col):
    return ((col - N / 2.0) * RES, (row - N / 2.0) * RES)


def _planner(**kw):
    kw.setdefault("params", ap.PlannerParams(mask_rim_m=0.0))
    return GlobalPlanner(**kw)


def _ctx(walkable, **kw):
    return MapContext(walkable=walkable, resolution=RES, center_x=0.0, center_y=0.0, **kw)


def _two_rooms():
    w = np.zeros((N, N), dtype=bool)
    w[5:55, 0:20] = True
    w[5:55, 40:60] = True
    return w


def _gamble_world():
    """Two rooms joined ONLY by a 4-wide corridor lined with unobserved space
    (a frontier gamble), plus a suspected-passage cluster (anomaly cells) on
    the west room's wall near the bottom - too small to route through."""
    w = _two_rooms()
    valid = np.ones((N, N), dtype=bool)
    w[14:18, 20:40] = True  # the gamble corridor
    valid[:14, 20:40] = False
    valid[18:, 20:40] = False
    anomaly = np.zeros((N, N), dtype=bool)
    anomaly[44:49, 20:22] = True  # flagged cells in the wall band, going nowhere yet
    frontier = compute_frontier_mask(w, valid)
    return w, valid, frontier, anomaly


class TestPlainRouting:
    def test_confirmed_route_is_returned_as_is(self):
        res = _planner().plan(_ctx(np.ones((N, N), dtype=bool)), _xy(10, 10), _xy(45, 40))
        assert res.valid and len(res.waypoints) == 2 and not res.is_exploration

    def test_no_route_and_exploration_off_gives_no_path(self):
        w = _two_rooms()
        assert not _planner().plan(_ctx(w), _xy(30, 5), _xy(30, 55)).valid

    def test_a_new_goal_resets_visited_exploration_spots(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid)
        gp = _planner(enable_exploration=True)
        gp.plan(ctx, _xy(30, 5), _xy(30, 55))
        gp.exploration.visited.append((1.0, 1.0))
        gp.plan(ctx, _xy(30, 5), _xy(30, 55))
        assert gp.exploration.visited == [(1.0, 1.0)]  # same goal: remembered
        gp.plan(ctx, _xy(30, 5), _xy(10, 55))  # different goal
        assert gp.exploration.visited == []


class TestMemoryInsideThePlanner:
    def test_repeated_calls_keep_the_same_waypoints(self):
        w = np.ones((N, N), dtype=bool)
        w[:, 28:32] = False
        w[8:14, 28:32] = True
        w[46:52, 28:32] = True
        gp = _planner()
        first = gp.plan(_ctx(w), _xy(30, 5), _xy(30, 55)).waypoints
        again = gp.plan(_ctx(w), _xy(26, 9), _xy(30, 55)).waypoints  # robot moved a bit
        assert again == first
        assert "kept stored route" in gp.last_note

    def test_memory_can_be_disabled(self):
        gp = _planner(enable_memory=False)
        gp.plan(_ctx(np.ones((N, N), dtype=bool)), _xy(10, 10), _xy(45, 40))
        assert "memory" not in gp.last_note


class TestInvestigateBeforeGambling:
    def test_without_exploration_the_frontier_gamble_is_taken(self):
        w, valid, frontier, anomaly = _gamble_world()
        ctx = _ctx(w, frontier=frontier, anomaly=anomaly, unobserved=~valid, allow_frontier=True, allow_anomaly=True)
        res = _planner().plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and res.has_frontier_segments and not res.is_exploration

    def test_with_exploration_a_suspected_passage_is_investigated_first(self):
        w, valid, frontier, anomaly = _gamble_world()
        ctx = _ctx(w, frontier=frontier, anomaly=anomaly, unobserved=~valid, allow_frontier=True, allow_anomaly=True)
        res = _planner(enable_exploration=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and res.is_exploration and res.exploration_kind == "anomaly"
        # the viewpoint is near the flagged cluster (wall band at rows 44-48, cols 20-21)
        cluster_xy = _xy(46, 20)
        assert math.dist(res.goal_xy, cluster_xy) <= 1.3
        assert res.has_frontier_segments  # tentative: keep replanning

    def test_without_an_anomaly_cluster_the_gamble_is_kept(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, allow_frontier=True)
        res = _planner(enable_exploration=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and not res.is_exploration  # nothing better to look at

    def test_a_route_that_goes_through_the_anomaly_is_not_overridden(self):
        # a COMPLETE tunnel: anomaly route exists, so no detour to "investigate"
        w = np.zeros((N, N), dtype=bool)
        a = np.zeros((N, N), dtype=bool)
        w[5:55, 0:20] = True
        w[5:55, 40:60] = True
        w[26:34, 22:38] = True
        a[26:34, 20:22] = True
        a[26:34, 38:40] = True
        ctx = _ctx(w, anomaly=a, allow_anomaly=True, unobserved=np.zeros((N, N), dtype=bool))
        res = _planner(enable_exploration=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and res.has_anomaly_segments and not res.is_exploration

    def test_once_visited_the_anomaly_viewpoint_is_not_chosen_again(self):
        w, valid, frontier, anomaly = _gamble_world()
        ctx = _ctx(w, frontier=frontier, anomaly=anomaly, unobserved=~valid, allow_frontier=True, allow_anomaly=True)
        gp = _planner(enable_exploration=True, enable_memory=False)
        first = gp.plan(ctx, _xy(30, 5), _xy(30, 55))
        assert first.exploration_kind == "anomaly"
        # the robot arrives and the cluster is still unresolved -> falls back
        second = gp.plan(ctx, first.goal_xy, _xy(30, 55))
        assert not (second.is_exploration and second.exploration_kind == "anomaly")


class TestAnomalyViewpointSelection:
    def test_the_nearest_reachable_spot_within_view_range_is_chosen(self):
        from nav.exploration import find_anomaly_viewpoints

        w, valid, frontier, anomaly = _gamble_world()
        found = find_anomaly_viewpoints(w, anomaly, RES, 0.0, 0.0, _xy(30, 5), mask_rim_m=0.0)
        vps, *_ = found
        assert len(vps) == 1
        d = math.dist(vps[0].xy, _xy(46, 20))
        assert d <= 1.2 + 0.15

    def test_no_anomaly_cells_means_no_viewpoints(self):
        from nav.exploration import find_anomaly_viewpoints

        w = _two_rooms()
        found = find_anomaly_viewpoints(w, np.zeros((N, N), dtype=bool), RES, 0.0, 0.0, _xy(30, 5), mask_rim_m=0.0)
        assert found[0] == []

    def test_an_unreachable_cluster_is_ignored(self):
        from nav.exploration import find_anomaly_viewpoints

        w = _two_rooms()
        a = np.zeros((N, N), dtype=bool)
        a[28:32, 44:46] = True  # flagged cells inside the EAST room (the robot is in the west room)
        found = find_anomaly_viewpoints(w, a, RES, 0.0, 0.0, _xy(30, 5), mask_rim_m=0.0)
        assert found[0] == []


class TestWaitForMapping:
    """Do not act on speculation while the UAV is still adding to the map."""

    def _gamble(self, mapping_active):
        w, valid, frontier, anomaly = _gamble_world()
        return _ctx(
            w, frontier=frontier, anomaly=anomaly, unobserved=~valid,
            allow_frontier=True, allow_anomaly=True, mapping_active=mapping_active,
        )  # fmt: skip

    def test_a_frontier_gamble_is_held_back_while_the_map_is_growing(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, allow_frontier=True, mapping_active=True)
        gp = _planner(wait_for_mapping=True)
        res = gp.plan(ctx, _xy(30, 5), _xy(30, 55))
        assert not res.valid
        assert "waiting" in gp.last_note

    def test_the_same_gamble_is_taken_once_mapping_has_finished(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, allow_frontier=True, mapping_active=False)
        res = _planner(wait_for_mapping=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and res.has_frontier_segments

    def test_the_rule_is_off_unless_asked_for(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, allow_frontier=True, mapping_active=True)
        assert _planner().plan(ctx, _xy(30, 5), _xy(30, 55)).valid

    def test_a_confirmed_route_is_never_held_back(self):
        ctx = _ctx(np.ones((N, N), dtype=bool), mapping_active=True)
        assert _planner(wait_for_mapping=True).plan(ctx, _xy(10, 10), _xy(45, 40)).valid

    def test_an_anomaly_viewpoint_is_still_visited_while_mapping(self):
        res = _planner(enable_exploration=True, wait_for_mapping=True).plan(
            self._gamble(mapping_active=True), _xy(30, 5), _xy(30, 55)
        )
        assert res.valid and res.is_exploration and res.exploration_kind == "anomaly"

    def test_frontier_viewpoints_are_not_visited_while_mapping(self):
        w, valid, frontier, _ = _gamble_world()
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, mapping_active=True)  # no anomaly cluster
        res = _planner(enable_exploration=True, wait_for_mapping=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert not res.valid

    def test_frontier_viewpoints_are_visited_after_mapping_finishes(self):
        w = _two_rooms()
        valid = np.ones((N, N), dtype=bool)
        valid[:, 30:] = False
        w[:, 30:] = False
        w[5:55, 0:30] = True
        frontier = compute_frontier_mask(w, valid)
        ctx = _ctx(w, frontier=frontier, unobserved=~valid, mapping_active=False)
        res = _planner(enable_exploration=True, wait_for_mapping=True).plan(ctx, _xy(30, 5), _xy(30, 55))
        assert res.valid and res.is_exploration and res.exploration_kind == "frontier"
