"""Tests for nav/mission.py (S-03 mission layer)."""
import pytest

from nav.mission import ABORTED, DONE, DRIVING, DWELL, RETURNING, Mission

GOALS = [(10, 0), (10, 10), (0, 10)]


def drive_to(m, xy, t=0.0):
    return m.update(xy, t)


def test_once_visits_the_goals_in_order_then_goes_home_then_done():
    m = Mission(GOALS, base_xy=(0, 0), mode="once")
    assert m.current_goal() == (10.0, 0.0)
    assert drive_to(m, (9.5, 0)) and m.current_goal() == (10.0, 10.0)       # goal changed -> publish again
    assert not drive_to(m, (9.5, 2))                                          # still driving: no change
    drive_to(m, (10, 9.5)); drive_to(m, (0.5, 10))
    assert m.state == RETURNING and m.current_goal() == (0.0, 0.0)
    drive_to(m, (0.3, 0.2))
    assert m.state == DONE and m.current_goal() is None and len(m.visited) == 3


def test_patrol_loops_the_requested_number_of_times():
    m = Mission(GOALS, (0, 0), mode="patrol", loops=2)
    for lap in range(2):
        for g in GOALS:
            drive_to(m, g)
    assert m.state == RETURNING and m.lap == 2 and len(m.visited) == 6


def test_patrol_forever_never_finishes():
    m = Mission(GOALS, (0, 0), mode="patrol", loops=0)
    for _ in range(5):
        for g in GOALS:
            drive_to(m, g)
    assert m.state == DRIVING and m.lap == 5


def test_dwell_waits_before_the_next_goal():
    m = Mission(GOALS, (0, 0), dwell_s=3.0)
    m.update((10, 0), 100.0)
    assert m.state == DWELL and m.current_goal() == (10.0, 0.0)
    assert not m.update((10, 0), 101.0) and m.state == DWELL
    assert m.update((10, 0), 103.5) and m.state == DRIVING and m.current_goal() == (10.0, 10.0)


def test_unreachable_goal_is_skipped_and_remembered():
    m = Mission(GOALS, (0, 0))
    m.goal_failed()
    assert m.skipped == [(10.0, 0.0)] and m.current_goal() == (10.0, 10.0)


def test_return_to_base_abandons_the_rest():
    m = Mission(GOALS, (1, 1))
    m.return_to_base()
    assert m.state == RETURNING and m.current_goal() == (1.0, 1.0)


def test_blocked_way_home_aborts_and_abort_stops_everything():
    m = Mission(GOALS, (0, 0)); m.return_to_base(); m.goal_failed()
    assert m.state == ABORTED and m.current_goal() is None
    m2 = Mission(GOALS, (0, 0)); m2.abort()
    assert m2.state == ABORTED


def test_empty_goal_list_just_returns_home_and_bad_mode_is_rejected():
    assert Mission([], (0, 0)).state == RETURNING
    with pytest.raises(ValueError):
        Mission(GOALS, (0, 0), mode="wander")
