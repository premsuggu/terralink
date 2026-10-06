"""Unit tests for nav.exploration - choosing where the UGV should go to look
when no route to the goal exists yet. Pure NumPy, no ROS.

Maps are n x n, centred on world (0, 0); `_xy` converts cell -> world."""
import math

import numpy as np
import pytest

from nav import astar_planner as ap
from nav.exploration import ExplorationMemory, Viewpoint, find_viewpoints, plan_exploration
from nav.walkability import compute_frontier_mask

RES = 0.1
N = 60


def _xy(row, col, n=N):
    return ((col - n / 2.0) * RES, (row - n / 2.0) * RES)


def _two_opening_world(top=True, bottom=True):
    """A room (cols 0-29) walled on its east side (cols 30-31) with up to two
    openings, each leading to unobserved space (cols >= 32)."""
    walkable = np.zeros((N, N), dtype=bool)
    valid = np.ones((N, N), dtype=bool)
    walkable[5:55, 0:30] = True
    if top:
        walkable[8:13, 30:32] = True
    if bottom:
        walkable[48:53, 30:32] = True
    valid[:, 32:] = False  # nothing observed east of the wall
    frontier = compute_frontier_mask(walkable, valid)
    return walkable, valid, frontier


def _unobserved(valid):
    return ~valid


def _vp(cell, score, xy=None):
    return Viewpoint(cell=cell, xy=xy or (float(cell[1]), float(cell[0])), score=score, reach_cost=score, cluster_size=1)


def _find(walkable, frontier, start, goal, valid=None, **kw):
    out = find_viewpoints(
        walkable, frontier, RES, 0.0, 0.0, start, goal, mask_rim_m=0.0,
        unobserved_mask=None if valid is None else ~valid, **kw,
    )
    assert out is not None
    return out[0]


class TestFindViewpoints:
    def test_each_opening_is_one_cluster_not_many_candidates(self):
        w, valid, f = _two_opening_world()
        vps = _find(w, f, _xy(30, 5), _xy(10, 55), valid=valid)
        assert len(vps) == 2

    def test_no_frontier_means_no_viewpoints(self):
        w, valid, _ = _two_opening_world()
        none = np.zeros_like(w)
        assert _find(w, none, _xy(30, 5), _xy(10, 55), valid=valid) == []

    def test_viewpoints_are_confirmed_reachable_ground(self):
        w, valid, f = _two_opening_world()
        for vp in _find(w, f, _xy(30, 5), _xy(10, 55), valid=valid):
            assert w[vp.cell]

    def test_a_frontier_cut_off_by_a_wall_is_ignored(self):
        w, valid, _ = _two_opening_world()
        # an isolated pocket of walkable ground bordering unobserved space,
        # not connected to where the robot is
        w2 = w.copy()
        w2[5:10, 40:45] = True
        valid2 = valid.copy()
        valid2[5:10, 40:45] = True
        valid2[4, 40:45] = False
        f = compute_frontier_mask(w2, valid2)
        vps = _find(w2, f, _xy(30, 5), _xy(10, 55), valid=valid2)
        assert all(vp.cell[1] < 32 for vp in vps)

    def test_spots_right_next_to_the_robot_are_not_offered(self):
        w, valid, f = _two_opening_world()
        # robot standing right at the top opening
        start = _xy(10, 29)
        for vp in _find(w, f, start, _xy(10, 55), valid=valid, min_target_dist_m=0.5):
            assert math.dist(vp.xy, start) >= 0.5 - 1e-9

    def test_robot_off_the_map_returns_none(self):
        w, valid, f = _two_opening_world()
        assert find_viewpoints(w, f, RES, 0.0, 0.0, (100.0, 100.0), (0.0, 0.0)) is None


class TestScoring:
    def test_the_opening_toward_the_goal_wins(self):
        w, valid, f = _two_opening_world()
        mem = ExplorationMemory()
        res = plan_exploration(w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), mem, mask_rim_m=0.0, unobserved_mask=~valid)
        assert res.valid
        # goal is in the upper half (small row -> small y): chosen target is the top opening
        assert res.goal_xy[1] < 0

        res2 = plan_exploration(
            w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(50, 55), ExplorationMemory(), mask_rim_m=0.0,
            unobserved_mask=~valid,
        )
        assert res2.goal_xy[1] > 0

    def test_a_much_bigger_opening_is_preferred_when_otherwise_equal(self):
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:30] = True
        w[8:15, 30:32] = True  # small (7-cell) opening, upper
        w[40:56, 30:32] = True  # large opening, lower
        valid[:, 32:] = False
        f = compute_frontier_mask(w, valid)
        vps = _find(w, f, _xy(30, 5), _xy(30, 55), valid=valid, gain_weight=5.0)
        by_size = sorted(vps, key=lambda v: v.cluster_size)
        assert by_size[-1].score < by_size[0].score + 5.0  # gain pulls the big one's score down
        assert by_size[-1].cluster_size > by_size[0].cluster_size


class TestPlanExploration:
    def test_result_is_flagged_as_an_exploration_plan(self):
        w, valid, f = _two_opening_world()
        res = plan_exploration(w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), ExplorationMemory(), mask_rim_m=0.0, unobserved_mask=~valid)
        assert res.valid and res.is_exploration
        assert res.has_frontier_segments  # tentative: keep replanning

    def test_path_ends_at_the_chosen_viewpoint_and_stays_on_free_ground(self):
        w, valid, f = _two_opening_world()
        res = plan_exploration(w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), ExplorationMemory(), mask_rim_m=0.0, unobserved_mask=~valid)
        assert res.waypoints[-1] == pytest.approx(res.goal_xy)
        for x, y in res.waypoints:
            r = int(round(y / RES + N / 2))
            c = int(round(x / RES + N / 2))
            assert w[r, c]

    def test_nothing_to_explore_is_an_invalid_plan(self):
        w, valid, _ = _two_opening_world()
        none = np.zeros_like(w)
        res = plan_exploration(w, none, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), ExplorationMemory(), mask_rim_m=0.0, unobserved_mask=~valid)
        assert not res.valid

    def test_same_input_gives_the_same_plan(self):
        w, valid, f = _two_opening_world()
        a = plan_exploration(w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), ExplorationMemory(), mask_rim_m=0.0, unobserved_mask=~valid)
        b = plan_exploration(w, f, RES, 0.0, 0.0, _xy(30, 5), _xy(10, 55), ExplorationMemory(), mask_rim_m=0.0, unobserved_mask=~valid)
        assert a.waypoints == b.waypoints


class TestExplorationMemory:
    def test_arriving_at_the_target_blacklists_it(self):
        mem = ExplorationMemory(arrive_radius_m=0.6)
        mem.choose([_vp((0, 0), 1.0, (1.0, 1.0)), _vp((0, 9), 2.0, (9.0, 1.0))])
        assert mem.target_xy == (1.0, 1.0)
        mem.note_robot_at((1.2, 1.0))
        assert mem.target_xy is None
        assert mem.is_blacklisted((1.0, 1.0))

    def test_blacklisted_spots_are_not_chosen_again(self):
        mem = ExplorationMemory()
        mem.visited.append((1.0, 1.0))
        pick = mem.choose([_vp((0, 0), 0.5, (1.1, 1.0)), _vp((0, 9), 5.0, (9.0, 1.0))])
        assert pick.xy == (9.0, 1.0)

    def test_everything_blacklisted_gives_none(self):
        mem = ExplorationMemory()
        mem.visited.append((1.0, 1.0))
        assert mem.choose([_vp((0, 0), 0.5, (1.1, 1.0))]) is None

    def test_current_target_is_kept_against_a_slightly_better_rival(self):
        mem = ExplorationMemory(switch_margin=0.2)
        mem.choose([_vp((0, 0), 10.0, (1.0, 1.0)), _vp((0, 9), 12.0, (9.0, 1.0))])
        assert mem.target_xy == (1.0, 1.0)
        # scores wobble: the rival is now 10% better - under the 20% margin
        pick = mem.choose([_vp((0, 0), 11.0, (1.0, 1.0)), _vp((0, 9), 9.9, (9.0, 1.0))])
        assert pick.xy == (1.0, 1.0)

    def test_a_clearly_better_rival_does_replace_the_target(self):
        mem = ExplorationMemory(switch_margin=0.2)
        mem.choose([_vp((0, 0), 10.0, (1.0, 1.0)), _vp((0, 9), 12.0, (9.0, 1.0))])
        pick = mem.choose([_vp((0, 0), 10.0, (1.0, 1.0)), _vp((0, 9), 5.0, (9.0, 1.0))])
        assert pick.xy == (9.0, 1.0)

    def test_visiting_each_opening_in_turn_then_running_out(self):
        w, valid, f = _two_opening_world()
        mem = ExplorationMemory()
        pos = _xy(30, 5)
        goal = _xy(10, 55)
        seen = []
        for _ in range(4):
            res = plan_exploration(w, f, RES, 0.0, 0.0, pos, goal, mem, mask_rim_m=0.0, unobserved_mask=~valid)
            if not res.valid:
                break
            seen.append(res.goal_xy)
            pos = res.goal_xy  # "drive" there
        # both openings get visited, then there is nothing left to explore
        assert len(seen) == 2
        assert seen[0][1] < 0 < seen[1][1]
