"""Regression tests on a REAL map: a cropped snapshot of `/elevation_map` from
a live `tunnel_demo` run, ~100 s in (`fixtures/tunnel_t100.npz`, 10 m x 10 m
around the tunnel, UGV start at (-3, 0), goal (3, 0)).

Why a real fixture: the synthetic tunnels in the other tests have generous
8-row anomaly bands. The REAL flagged mouths on this map are only about 3 x 3
cells (0.3 m) - narrower than the robot - and an early A* demanded robot
clearance even inside suspected-passage cells, so it answered "no path" on
maps where the old zero-width PRM routed straight through. Found by replaying
recorded runs through both planners (see docs/work-docs/nav/step07).
"""
from pathlib import Path

import numpy as np
import pytest

from nav import astar_planner as ap
from nav import replay

FIXTURE = Path(__file__).parent / "fixtures" / "tunnel_t100.npz"
START, GOAL = (-3.0, 0.0), (3.0, 0.0)


@pytest.fixture(scope="module")
def tunnel():
    snap = replay.load_snapshot(FIXTURE)
    return snap, replay.build_masks(snap, use_frontier=True, use_anomaly=True)


class TestRealTunnelMap:
    def test_the_flagged_mouths_really_are_narrower_than_the_robot(self, tunnel):
        # guards the premise of this file: if the detector ever flags wider
        # mouths this fixture should be refreshed, not silently stop testing
        # what it was made to test
        from scipy import ndimage as ndi

        snap, masks = tunnel
        labels, n = ndi.label(masks.anomaly, structure=np.ones((3, 3)))
        assert n >= 2
        for k in range(1, n + 1):
            rows, cols = np.nonzero(labels == k)
            assert min(np.ptp(rows) + 1, np.ptp(cols) + 1) * snap.resolution < 0.44

    def test_astar_routes_through_the_tunnel(self, tunnel):
        snap, masks = tunnel
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert res.valid
        assert res.has_anomaly_segments  # a tentative route, honestly flagged
        # one contiguous uncertain stretch (it may hold several waypoints), not
        # separate stretches for each mouth
        kinds = res.segment_kinds
        first = kinds.index("anomaly")
        last = len(kinds) - 1 - kinds[::-1].index("anomaly")
        assert all(k == "anomaly" for k in kinds[first : last + 1])

    def test_it_routes_wherever_the_old_planner_does(self, tunnel):
        snap, masks = tunnel
        assert replay.run_plan(snap, masks, START, GOAL, planner="prm").valid
        assert replay.run_plan(snap, masks, START, GOAL, planner="astar").valid

    def test_the_route_crosses_the_tunnel_roughly_along_its_axis(self, tunnel):
        snap, masks = tunnel
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        kinds = res.segment_kinds
        first = kinds.index("anomaly")
        last = len(kinds) - 1 - kinds[::-1].index("anomaly")
        entry, exit_ = res.waypoints[first], res.waypoints[last + 1]
        assert entry[0] < -0.5 and exit_[0] > 0.3  # west of the tunnel -> east of it
        assert abs(entry[1]) < 0.8 and abs(exit_[1]) < 0.8  # through the tunnel, not around it

    def test_a_standoff_point_precedes_the_tentative_stretch(self, tunnel):
        snap, masks = tunnel
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        i = res.segment_kinds.index("anomaly")
        assert i >= 1 and res.segment_kinds[i - 1] == "free"

    def test_the_planner_is_deterministic_on_real_data(self, tunnel):
        snap, masks = tunnel
        a = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        b = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert a.waypoints == b.waypoints

    def test_a_voxel_confirmed_blocked_tunnel_removes_the_route(self, tunnel):
        from nav.resolved_regions import ResolvedRegion

        snap, _ = tunnel
        blocked = [ResolvedRegion(-1.6, -1.0, 1.4, 1.0, passable=False)]
        masks = replay.build_masks(snap, use_frontier=True, use_anomaly=True, resolved=blocked)
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert not res.has_anomaly_segments  # may still find some other route, but never THIS one

    def test_a_voxel_confirmed_passable_tunnel_gives_a_confirmed_route(self, tunnel):
        from nav.resolved_regions import ResolvedRegion

        snap, _ = tunnel
        passable = [ResolvedRegion(-1.6, -1.0, 1.4, 1.0, passable=True)]
        masks = replay.build_masks(snap, use_frontier=True, use_anomaly=True, resolved=passable)
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert res.valid and not res.has_anomaly_segments


PARKED = Path(__file__).parent / "fixtures" / "tunnel_test_parked.npz"


class TestCleanParkedMap:
    """The finished map with the UGV parked at spawn and the UAV patrol done
    (`fixtures/tunnel_test_parked.npz`, 16 m x 16 m). On it BOTH planners used
    to "gamble" through the dividing wall: its top is only 1-2 cells thick, so
    part of it is unobserved (not lethal) and looked like unscanned floor."""

    @pytest.fixture(scope="class")
    def snap(self):
        return replay.load_snapshot(PARKED)

    def test_without_pruning_astar_gambles_through_the_wall(self, snap):
        masks = replay.build_masks(snap, use_frontier=True, use_anomaly=True)
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert res.valid and res.has_frontier_segments  # the wall-seam gamble

    def test_with_pruning_astar_takes_the_direct_tunnel_route(self, snap):
        masks = replay.build_masks(snap, use_frontier=True, use_anomaly=True, frontier_min_width_m=0.44)
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        assert res.valid and res.has_anomaly_segments and not res.has_frontier_segments
        m = replay.path_metrics(res.waypoints, snap, masks)
        assert m.length_m < 6.5  # straight line is 6.0 m
        assert m.sharpest_turn_deg < 30

    def test_the_tentative_stretch_is_the_tunnel_and_a_standoff_precedes_it(self, snap):
        masks = replay.build_masks(snap, use_frontier=True, use_anomaly=True, frontier_min_width_m=0.44)
        res = replay.run_plan(snap, masks, START, GOAL, planner="astar")
        kinds = res.segment_kinds
        i = kinds.index("anomaly")
        assert i >= 1 and kinds[i - 1] == "free"
        assert res.waypoints[i][0] < -0.5  # entry is west of the tunnel
        assert res.waypoints[i + 1][0] > 0.5  # exit is east of it
