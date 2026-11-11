"""Tests for tests/nav/explore_wallfollow.py: Bug1 / Bug2 wall following and the exit-list explorer (eval_exploration.py)."""
import math

import numpy as np
import pytest

import explore_wallfollow as ew
from nav.global_planner import MapContext

RES = 0.1
N = 120                    # 12 m x 12 m, centred on (0, 0)


def xy(row, col):
    return ((col - N / 2) * RES, (row - N / 2) * RES)


def _ctx(walkable, unobserved=None):
    unobserved = np.zeros(walkable.shape, bool) if unobserved is None else unobserved
    return MapContext(walkable=walkable, resolution=RES, center_x=0.0, center_y=0.0,
                      traversability=np.where(walkable, 1.0, 0.0).astype(np.float32), unobserved=unobserved)


def _block_world():
    """Open, fully known ground with a 4 m x 2 m block between start (west) and goal (east)."""
    walkable = np.ones((N, N), bool)
    walkable[40:80, 50:70] = False
    return walkable


def _drive(planner, walkable, start, goal, unobserved=None, max_plans=200):
    """Follow every plan exactly (a perfect robot) until the goal or `max_plans`; returns (reached, path length)."""
    pos, length = start, 0.0
    for _ in range(max_plans):
        if math.dist(pos, goal) < 0.3:
            return True, length
        r = planner.plan(_ctx(walkable, unobserved), pos, goal)
        if not r.valid:
            return False, length
        for w in r.waypoints[1:]:
            length += math.dist(pos, w)
            pos = w
    return math.dist(pos, goal) < 0.3, length


def test_trace_keeps_the_wall_on_the_left():
    """Moore tracing from a cell just west of the block (the block to the east). With the wall on the robot's LEFT the
    robot faces south, so its next cell is directly south of the start, and the new cell still has a blocked neighbour."""
    p = ew.BugPlanner()
    p.passable = np.ones((N, N), bool)
    p.passable[40:80, 50:70] = False
    start = (60, 49)
    step = p._trace(start, ew.N8.index((0, 1)))          # the blocked neighbour is east
    assert step is not None
    (r, c), bi = step
    assert r < start[0] and c == 49                       # moved south along the west face
    assert not p._free((r + ew.N8[bi][0], c + ew.N8[bi][1]))


@pytest.mark.parametrize("variant", ["bug1", "bug2"])
def test_bug_gets_round_a_block(variant):
    walkable = _block_world()
    reached, length = _drive(ew.BugPlanner(variant), walkable, xy(60, 20), xy(60, 100))
    assert reached
    assert length < (60 if variant == "bug1" else 25)     # bug1 circles the whole block first (~14 m round it)


def test_bug1_circles_the_whole_block_bug2_does_not():
    walkable = _block_world()
    _, l1 = _drive(ew.BugPlanner("bug1"), walkable, xy(60, 20), xy(60, 100))
    _, l2 = _drive(ew.BugPlanner("bug2"), walkable, xy(60, 20), xy(60, 100))
    assert l1 > l2 + 5.0


def test_bug_reports_a_goal_inside_a_sealed_box():
    walkable = np.ones((N, N), bool)
    walkable[50:71, 80:101] = False
    walkable[53:68, 83:98] = True                                          # a closed ring, the goal inside it
    for variant in ("bug1", "bug2"):
        reached, _ = _drive(ew.BugPlanner(variant), walkable, xy(60, 20), xy(60, 90))
        assert not reached


def _two_door_world():
    """Known west half; a wall at col 60 with TWO doors (south rows 20-32, north rows 88-100); unknown east half.
    The goal is east, level with the north door."""
    unobserved = np.zeros((N, N), bool)
    unobserved[:, 61:] = True
    walkable = ~unobserved
    walkable[:, 60] = False
    walkable[20:32, 60] = True
    walkable[88:100, 60] = True
    return walkable, unobserved


def test_exits_lists_both_doors_and_picks_the_goal_side_one():
    walkable, unobserved = _two_door_world()
    p = ew.ExitsPlanner("best")
    ex = p.exits(_ctx(walkable, unobserved), xy(60, 20), xy(94, 110))
    assert len(ex) == 2
    r = p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(94, 110))
    assert r.valid and r.is_exploration and p.target[1] > 0                 # the north door


def test_exits_commits_to_its_exit():
    """After choosing the north door, the robot keeps it although (once the goal moves) the south one now scores better."""
    walkable, unobserved = _two_door_world()
    p = ew.ExitsPlanner("best")
    p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(94, 110))
    first = p.target
    p.plan(_ctx(walkable, unobserved), xy(60, 22), xy(26, 110))            # goal now level with the SOUTH door
    assert p.target == first


def test_exits_drives_the_known_route_when_there_is_one():
    walkable = np.ones((N, N), bool)
    p = ew.ExitsPlanner()
    r = p.plan(_ctx(walkable), xy(60, 20), xy(60, 100))
    assert r.valid and not r.is_exploration and "observed ground" in p.last_note


def test_exits_gives_up_when_nothing_is_left():
    walkable, unobserved = _two_door_world()
    walkable[:, 60] = False                                                 # both doors closed
    p = ew.ExitsPlanner()
    assert not p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(94, 110)).valid
    assert "no exits" in p.last_note


def test_dfs_prefers_the_newest_exit():
    walkable, unobserved = _two_door_world()
    p = ew.ExitsPlanner("dfs")
    ctx = _ctx(walkable, unobserved)
    p.exits(ctx, xy(60, 20), xy(94, 110))                                    # both doors seen at plan 0
    p.n_plans = 5
    walkable2 = walkable.copy()
    walkable2[2:6, 0:30] = True
    unobserved2 = unobserved.copy()
    unobserved2[0:2, 0:30] = True                                            # a NEW opening along the south edge
    walkable2[0:2, 0:30] = False
    r = p.plan(_ctx(walkable2, unobserved2), xy(60, 20), xy(94, 110))
    assert r.valid and p.target[1] < -5.0                                    # it goes to the new (south-edge) exit


def test_bump_memory_expires():
    walkable = np.ones((N, N), bool)
    p = ew.ExitsPlanner()
    b = p.bumper
    b.prev_start, b.prev_wps = (0.0, 0.0), [(0.0, 0.0), (1.0, 0.0)]
    b.note((0.0, 0.0))                                                       # did not move: a bump 0.3 m ahead
    assert p.fs.extra_blocked and abs(p.fs.extra_blocked[0][0] - 0.3) < 1e-6
    b.prev_wps = None
    for _ in range(b.expire_plans + 1):
        b.note((0.0, 0.0))
    assert p.fs.extra_blocked == []
    assert walkable.all()
