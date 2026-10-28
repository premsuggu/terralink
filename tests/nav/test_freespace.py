"""Tests for nav/freespace.py: planning to a goal as if unobserved space were open (used under roofs, step12)."""
import numpy as np

from nav.freespace import FreespacePlanner, IslandStrategyPlanner
from nav.global_planner import MapContext
from nav.stuck_feedback import StuckTracker

RES = 0.1
N = 120                    # 12 m x 12 m, centred on (0, 0)


def xy(row, col):
    return ((col - N / 2) * RES, (row - N / 2) * RES)


def _ctx(walkable, unobserved):
    return MapContext(walkable=walkable, resolution=RES, center_x=0.0, center_y=0.0,
                      traversability=np.where(walkable, 1.0, 0.0).astype(np.float32), unobserved=unobserved)


def _room_world():
    """Known free ground on the left (cols < 60), unobserved on the right; a known wall at col 60 with a 1.2 m door at rows 54-66."""
    unobserved = np.zeros((N, N), bool)
    unobserved[:, 61:] = True
    walkable = ~unobserved
    walkable[:, 60] = False
    walkable[54:66, 60] = True
    return walkable, unobserved


def test_plans_through_unobserved_space_and_flags_the_route_tentative():
    walkable, unobserved = _room_world()
    p = FreespacePlanner()
    r = p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(60, 100), now=0.0)
    assert r.valid and r.has_frontier_segments and not r.is_exploration
    assert "frontier" in r.segment_kinds                       # the unobserved stretch is marked, like any tentative stretch
    assert "through unobserved" in p.last_note
    assert abs(r.waypoints[-1][0] - xy(60, 100)[0]) < 0.2


def test_uses_the_door_not_the_known_wall():
    walkable, unobserved = _room_world()
    r = FreespacePlanner().plan(_ctx(walkable, unobserved), xy(10, 20), xy(10, 100), now=0.0)
    crossing = [(a, b) for a, b in zip(r.waypoints, r.waypoints[1:]) if a[0] < 0.0 <= b[0]]
    assert r.valid and crossing
    (ax, ay), (bx, by) = crossing[0]
    y0 = ay + (by - ay) * (0.0 - ax) / (bx - ax)              # where the route crosses the wall line x = 0
    assert -0.6 <= y0 <= 0.6


def test_gives_up_when_known_walls_enclose_the_goal():
    """A sealed room: once every wall is known there is no route even through unobserved space."""
    walkable, unobserved = _room_world()
    walkable[54:66, 60] = False
    p = FreespacePlanner()
    assert not p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(60, 100), now=0.0).valid
    assert "walled off" in p.last_note


def test_a_route_over_observed_ground_is_not_tentative():
    walkable = np.ones((N, N), bool)
    r = FreespacePlanner().plan(_ctx(walkable, np.zeros((N, N), bool)), xy(60, 20), xy(60, 100), now=0.0)
    assert r.valid and not r.has_frontier_segments


def test_the_robots_own_footprint_is_free():
    """The UAV scan sees the parked UGV as an obstacle: a small blocked blob under the robot must not box it in."""
    walkable = np.ones((N, N), bool)
    walkable[57:64, 17:24] = False                              # 0.7 x 0.7 m blob centred on the start
    r = FreespacePlanner().plan(_ctx(walkable, np.zeros((N, N), bool)), xy(60, 20), xy(60, 100), now=0.0)
    assert r.valid


def test_stuck_penalty_moves_the_route_to_the_other_door():
    """Two doors; the robot sits in front of the near one (as if Nav2 refused it). After the stuck window the stretch ahead is
    penalised and the next route uses the far door."""
    walkable, unobserved = _room_world()
    walkable[10:22, 60] = True                                  # a second door far south (rows 10-22)
    stuck = StuckTracker(window_sec=18.0)
    p = FreespacePlanner(stuck=stuck)
    ctx = _ctx(walkable, unobserved)
    start, goal = xy(60, 50), xy(60, 100)
    first = p.plan(ctx, start, goal, now=0.0)
    for t in (5.0, 10.0, 15.0, 20.0):                           # the robot does not move
        last = p.plan(ctx, start, goal, now=t)
    assert "stuck" in p.last_note or any(abs(w[1] - first.waypoints[1][1]) > 1.0 for w in last.waypoints)
    crossing = [(a, b) for a, b in zip(last.waypoints, last.waypoints[1:]) if a[0] < 0.0 <= b[0]]
    (ax, ay), (bx, by) = crossing[0]
    assert ay + (by - ay) * (0.0 - ax) / (bx - ax) < -3.0       # crosses at the far (south) door


class _Rec:
    def __init__(self, name):
        self.name, self.last_note, self.params, self.stuck = name, name, None, None

    def plan(self, ctx, start, goal, now=None):
        return self.name


class _Outer:
    last_zone = None


def test_island_strategy_uses_freespace_only_under_a_roof():
    gp, fs = _Rec("global"), _Rec("freespace")
    inner = IslandStrategyPlanner(gp, freespace=fs)
    inner.outer = _Outer()
    assert inner.plan(None, (0, 0), (1, 1)) == "global" and inner.last_note == "global"
    inner.outer.last_zone = np.ones((2, 2), bool)               # GoalIslandPlanner has hidden an island
    assert inner.plan(None, (0, 0), (1, 1)) == "freespace" and inner.last_note == "freespace"
