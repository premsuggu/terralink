"""Tests for nav/overhead.py: goals under a roof/deck the UAV only saw the top of."""
import numpy as np
import pytest

from nav.overhead import GoalIslandPlanner, compose_with_floor, goal_island_zone

RES = 0.1
N = 200                    # 20 m x 20 m, centred on (0, 0)


def _world(deck=True, ramp=False):
    """Flat ground at 0; a 8x8 m plateau at 3.5 m in the middle, fenced by a lethal ring (the roof edge)."""
    elev = np.zeros((N, N), np.float32)
    trav = np.ones((N, N), np.float32)
    if deck:
        elev[60:140, 60:140] = 3.5
        trav[58:62, 58:142] = trav[138:142, 58:142] = 0.0           # ring: 4 cells (0.4 m) thick on every side
        trav[58:142, 58:62] = trav[58:142, 138:142] = 0.0
    if ramp:                                                        # a gentle ramp joins the plateau to the ground
        elev[95:105, 40:60] = np.linspace(0.0, 3.5, 20)[None, :]
        trav[95:105, 40:62] = 1.0
    valid = np.ones((N, N), np.float32)
    return {"elevation": elev, "traversability": trav, "is_valid": valid, "variance": np.full((N, N), 0.01, np.float32)}


def _walkable(layers):
    return (layers["is_valid"] > 0.5) & (layers["traversability"] > 0.15)


def xy(row, col):
    return ((col - N / 2) * RES, (row - N / 2) * RES)


def test_goal_on_an_elevated_island_gets_a_zone_covering_island_and_ring():
    lay = _world()
    zone = goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(100, 100))
    assert zone is not None
    assert zone[100, 100] and zone[59, 100] and zone[141, 100]              # island, and its lethal ring
    assert not zone[20, 20]                                                 # open ground far away


def test_goal_on_ordinary_ground_has_no_zone():
    lay = _world()
    assert goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(20, 20)) is None


def test_a_small_raised_object_is_not_an_island():
    """A crate (1 m x 1 m top, area 1 m^2) is not a building."""
    lay = _world(deck=False)
    lay["elevation"][100:110, 100:110] = 2.0
    lay["traversability"][98:112, 98:112] = 0.0
    lay["traversability"][100:110, 100:110] = 1.0
    assert goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(105, 105)) is None


def test_a_raised_area_joined_to_the_ground_by_a_ramp_is_not_an_island():
    """Reachable plateaus are the ordinary planner's business: one connected walkable component, not a hidden one."""
    lay = _world(ramp=True)
    assert goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(100, 100)) is None


def test_low_platform_below_min_height_is_not_an_island():
    lay = _world()
    lay["elevation"][60:140, 60:140] = 0.5
    assert goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(100, 100)) is None


def test_compose_hides_the_zone_and_keeps_what_the_ugv_saw():
    lay = _world()
    zone = goal_island_zone(lay["elevation"], _walkable(lay), RES, 0.0, 0.0, xy(100, 100))
    floor = {"elevation": np.zeros((N, N), np.float32), "variance": np.full((N, N), 0.02, np.float32),
             "traversability": np.ones((N, N), np.float32), "is_valid": np.zeros((N, N), np.float32)}
    floor["is_valid"][95:105, 95:105] = 1.0                                  # the UGV has seen a 1 m patch of floor
    floor["traversability"][100, 100] = 0.0                                  # and a wall cell in it
    out = compose_with_floor(lay, zone, floor)
    assert out["is_valid"][70, 70] == 0                                      # unseen roof area: unknown, not "walkable plateau"
    assert out["is_valid"][96, 96] == 1 and out["elevation"][96, 96] == 0.0  # seen floor replaces the roof height
    assert out["traversability"][100, 100] == 0.0
    assert lay["is_valid"][70, 70] == 1 and lay["elevation"][70, 70] == 3.5  # input untouched
    assert out["is_valid"][20, 20] == 1                                      # outside the zone nothing changed


def test_compose_fills_uav_gaps_outside_the_zone_with_ugv_observations():
    lay = _world(deck=False)
    lay["is_valid"][30:40, 30:40] = 0                                        # the UAV never scanned this
    floor = {"elevation": np.full((N, N), 0.2, np.float32), "variance": np.full((N, N), 0.02, np.float32),
             "traversability": np.ones((N, N), np.float32), "is_valid": np.zeros((N, N), np.float32)}
    floor["is_valid"][30:35, 30:40] = 1.0
    out = compose_with_floor(lay, None, floor)
    assert out["is_valid"][32, 32] == 1 and out["elevation"][32, 32] == pytest.approx(0.2)
    assert out["is_valid"][38, 32] == 0                                      # still unknown where the UGV saw nothing


class _FakePlanner:
    """Records the layers it was asked to plan on."""
    def __init__(self):
        self.seen = None

    def plan(self, ctx, start, goal, now=None):
        self.seen = ctx
        return "result"


def test_planner_hides_the_island_for_the_goal_and_also_when_the_robot_is_on_it():
    lay = _world()
    fake = _FakePlanner()
    gp = GoalIslandPlanner(fake)
    ctx_of = lambda layers, use_anomaly=True: layers                                           # make_ctx = identity: the planner "sees" the layers
    gp.plan(lay, None, ctx_of, xy(20, 20), xy(100, 100), RES, 0.0, 0.0)       # goal on the island, robot outside
    assert fake.seen["is_valid"][70, 70] == 0 and "goal is on" in gp.note
    gp.plan(lay, None, ctx_of, xy(100, 100), xy(20, 20), RES, 0.0, 0.0)       # robot on the island, goal outside
    assert fake.seen["is_valid"][70, 70] == 0 and "robot is on" in gp.note
    gp.plan(lay, None, ctx_of, xy(20, 20), xy(180, 20), RES, 0.0, 0.0)        # neither
    assert fake.seen["is_valid"][70, 70] == 1 and "ordinary" in gp.note
