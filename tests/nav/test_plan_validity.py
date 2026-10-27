"""nav.plan_validity: does the plan being driven cross cells the current map KNOWS are blocked?"""
import numpy as np

from nav.plan_validity import blocked_samples, plan_is_blocked

RES = 0.1


def _map(n=60):
    return np.ones((n, n), bool), np.ones((n, n), bool)          # walkable, is_valid


def _xy(row, col, n=60):
    return ((col - n / 2) * RES, (row - n / 2) * RES)


def test_clear_plan_is_not_blocked():
    walk, valid = _map()
    path = [_xy(30, 5), _xy(30, 50)]
    assert blocked_samples(path, walk, valid, RES, 0.0, 0.0) == 0 and not plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)


def test_a_wall_found_later_blocks_the_plan_only_where_it_is_observed_and_blocked():
    walk, valid = _map()
    path = [_xy(30, 5), _xy(30, 50)]
    walk[25:36, 30:33] = False                                    # the UGV's camera found a wall across the route (observed, lethal)
    assert plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)
    valid[25:36, 30:33] = False                                   # ... but if those cells were merely UNOBSERVED a tentative plan is fine
    assert not plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)


def test_the_robots_own_start_footprint_is_ignored_and_stray_cells_need_min_samples():
    walk, valid = _map()
    path = [_xy(30, 10), _xy(30, 50)]
    walk[29:32, 9:13] = False                                     # blocked right at the start (the robot's own shadow): first 0.4 m skipped
    assert not plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)
    walk[30, 30] = False                                          # one stray lethal cell on the route: below the 3-sample threshold
    assert not plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)
    walk[30, 31:33] = False
    assert plan_is_blocked(path, walk, valid, RES, 0.0, 0.0)


def test_degenerate_inputs():
    walk, valid = _map()
    assert blocked_samples([], walk, valid, RES, 0.0, 0.0) == 0
    assert blocked_samples([_xy(30, 30)], walk, valid, RES, 0.0, 0.0) == 0
    assert blocked_samples([(100.0, 100.0), (101.0, 100.0)], walk, valid, RES, 0.0, 0.0) == 0     # off the map
