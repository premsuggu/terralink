"""Tests for tests/nav/explore_policies.py: the ports of existing exploration strategies used by eval_exploration.py."""
import numpy as np

import explore_policies as ep
from nav.global_planner import MapContext

RES = 0.1
N = 120                    # 12 m x 12 m, centred on (0, 0)


def xy(row, col):
    return ((col - N / 2) * RES, (row - N / 2) * RES)


def _ctx(walkable, unobserved):
    return MapContext(walkable=walkable, resolution=RES, center_x=0.0, center_y=0.0,
                      traversability=np.where(walkable, 1.0, 0.0).astype(np.float32), unobserved=unobserved)


def _room_world():
    """Known free ground on the left (cols < 60), unknown on the right; a known wall at col 60 with a 1.2 m door at rows 54-66."""
    unobserved = np.zeros((N, N), bool)
    unobserved[:, 61:] = True
    walkable = ~unobserved
    walkable[:, 60] = False
    walkable[54:66, 60] = True
    return walkable, unobserved


def test_freespace_plans_through_unknown_space_and_says_so():
    walkable, unobserved = _room_world()
    p = ep.FreespacePlanner()
    r = p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(60, 100))
    assert r.valid and r.has_frontier_segments
    assert "through unknown" in p.last_note
    assert abs(r.waypoints[-1][0] - xy(60, 100)[0]) < 0.2


def test_freespace_uses_the_door_not_the_known_wall():
    walkable, unobserved = _room_world()
    r = ep.FreespacePlanner().plan(_ctx(walkable, unobserved), xy(10, 20), xy(10, 100))
    crossing = [w for a, w in zip(r.waypoints, r.waypoints[1:]) if a[0] < 0.0 <= w[0]]
    assert r.valid and crossing and all(-0.6 <= w[1] <= 0.6 for w in crossing)     # crosses x = 0 inside the door (y in -0.6..0.6)


def test_freespace_gives_up_when_known_walls_enclose_the_goal():
    """A sealed room: once every wall is known there is no route, even through unknown space (the run terminates)."""
    walkable, unobserved = _room_world()
    walkable[54:66, 60] = False                                                   # close the door
    p = ep.FreespacePlanner()
    assert not p.plan(_ctx(walkable, unobserved), xy(60, 20), xy(60, 100)).valid
    assert "no route" in p.last_note


def test_unknown_cost_makes_navfn_prefer_known_ground():
    """Two ways round a block: a short one through unknown, a longer one over known ground. Cost 1 takes the short one, NavFn's ~5 the known one."""
    walkable = np.ones((N, N), bool)
    unobserved = np.zeros((N, N), bool)
    walkable[40:80, 50:70] = False                                                # a known block in the middle
    unobserved[80:110, 40:80] = True                                              # the north way round is unknown
    walkable[80:110, 40:80] = False
    start, goal = xy(70, 20), xy(70, 100)
    short = ep.FreespacePlanner(unknown_cost=1.0).plan(_ctx(walkable, unobserved), start, goal)
    navfn = ep.FreespacePlanner(unknown_cost=5.0).plan(_ctx(walkable, unobserved), start, goal)
    assert short.has_frontier_segments and not navfn.has_frontier_segments


def test_explore_lite_drives_to_the_goal_once_a_known_route_exists():
    walkable = np.ones((N, N), bool)
    p = ep.ExploreLitePlanner()
    r = p.plan(_ctx(walkable, np.zeros((N, N), bool)), xy(60, 20), xy(60, 100))
    assert r.valid and not r.is_exploration and "driving to the goal" in p.last_note


def test_explore_lite_picks_the_bigger_frontier_and_blacklists_after_no_progress():
    """Its cost is 3 * d * res - size * res: a much bigger frontier wins although it is farther."""
    walkable, unobserved = _room_world()                                          # the door: a 12-cell frontier at col 61
    unobserved[:3, :61] = True                                                    # a long strip along the south edge: ~60 cells
    walkable[:3, :61] = False
    p = ep.ExploreLitePlanner()
    ctx = _ctx(walkable, unobserved)
    r = p.plan(ctx, xy(60, 30), xy(60, 100), now=0.0)
    assert r.is_exploration and r.goal_xy[1] < -5.0                               # the long south strip
    p.plan(ctx, xy(60, 30), xy(60, 100), now=10.0)                                # same goal, no progress (robot did not move)
    r2 = p.plan(ctx, xy(60, 30), xy(60, 100), now=45.0)                           # > progress_timeout (30 s): blacklisted
    assert p.blacklist and r2.goal_xy[1] > -5.0                                   # moved on to the door
