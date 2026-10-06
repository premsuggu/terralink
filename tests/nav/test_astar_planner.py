"""Unit tests for nav.astar_planner - pure NumPy, no ROS.

Test grids are SQUARE (the planner, like the rest of `nav`, assumes the
map is square) with the map centred on world (0, 0), so cell (row, col) sits
at world x = (col - n/2) * RES, y = (row - n/2) * RES. `_xy`/`_cell` below do
that conversion so tests can talk in cells and the planner in meters.

Most scenarios pass `mask_rim_m=0` so inflation arithmetic is exactly
"robot radius + margin" and the expected numbers are easy to verify by hand;
the rim behaviour itself has its own test.
"""
import math

import numpy as np
import pytest
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from nav import astar_planner as ap

RES = 0.1


def _xy(row, col, n):
    """World (x, y) of a cell centre on an n x n map centred at the origin."""
    return ((col - n / 2.0) * RES, (row - n / 2.0) * RES)


def _cell(x, y, n):
    return (int(round(y / RES + n / 2.0)), int(round(x / RES + n / 2.0)))


def _plan(walkable, start_cell, goal_cell, **kw):
    n = walkable.shape[0]
    kw.setdefault("mask_rim_m", 0.0)
    return ap.plan(walkable, RES, 0.0, 0.0, _xy(*start_cell, n), _xy(*goal_cell, n), **kw)


def _open(n=60):
    return np.ones((n, n), dtype=bool)


def _segment_min_clearance(walkable, p, q, samples=60):
    """Smallest distance (m) from any point on the straight segment p->q
    (world coords) to a blocked cell centre - used to prove a returned path
    really keeps its clearance, independently of the planner's own masks."""
    n = walkable.shape[0]
    clear = ap.clearance_m(~walkable, RES)
    worst = math.inf
    for t in np.linspace(0.0, 1.0, samples):
        x = p[0] + (q[0] - p[0]) * t
        y = p[1] + (q[1] - p[1]) * t
        r, c = _cell(x, y, n)
        worst = min(worst, clear[r, c])
    return worst


# --------------------------------------------------------------------------
# line_is_clear
# --------------------------------------------------------------------------


class TestLineIsClear:
    def test_clear_horizontal_and_diagonal_lines(self):
        m = np.ones((10, 10), dtype=bool)
        assert ap.line_is_clear(m, 3, 0, 3, 9)
        assert ap.line_is_clear(m, 0, 0, 9, 9)

    def test_single_blocked_cell_on_the_line_blocks_it(self):
        m = np.ones((10, 10), dtype=bool)
        m[3, 5] = False
        assert not ap.line_is_clear(m, 3, 0, 3, 9)

    def test_does_not_slip_between_two_corner_touching_blocked_cells(self):
        # Blocked cells (4,4) and (5,5) touch only at a corner. A line from
        # (4,5) to (5,4) passes exactly through that shared corner. Plain
        # Bresenham can slip through; a robot cannot.
        m = np.ones((10, 10), dtype=bool)
        m[4, 4] = False
        m[5, 5] = False
        assert not ap.line_is_clear(m, 4, 5, 5, 4)

    def test_line_leaving_the_grid_is_not_clear(self):
        m = np.ones((10, 10), dtype=bool)
        assert not ap.line_is_clear(m, 0, 0, 0, 12)

    def test_same_cell_is_clear_iff_passable(self):
        m = np.ones((5, 5), dtype=bool)
        m[2, 2] = False
        assert ap.line_is_clear(m, 1, 1, 1, 1)
        assert not ap.line_is_clear(m, 2, 2, 2, 2)


# --------------------------------------------------------------------------
# Open space, determinism, basic validity
# --------------------------------------------------------------------------


class TestOpenSpace:
    def test_open_room_gives_one_straight_segment(self):
        res = _plan(_open(), (10, 10), (45, 40))
        assert res.valid
        assert len(res.waypoints) == 2
        assert res.segment_kinds == ["free"]
        assert not res.has_anomaly_segments and not res.has_frontier_segments

    def test_waypoints_are_the_start_and_goal_cell_centres(self):
        n = 60
        res = _plan(_open(n), (10, 10), (45, 40))
        assert res.waypoints[0] == pytest.approx(_xy(10, 10, n))
        assert res.waypoints[-1] == pytest.approx(_xy(45, 40, n))

    def test_start_equals_goal_is_a_trivial_valid_plan(self):
        res = _plan(_open(), (20, 20), (20, 20))
        assert res.valid
        assert res.waypoints[0] == pytest.approx(res.waypoints[-1])

    def test_same_input_always_gives_the_identical_path(self):
        rng = np.random.default_rng(3)
        w = rng.random((60, 60)) > 0.2
        w[5, 5] = w[50, 50] = True
        first = _plan(w, (5, 5), (50, 50))
        second = _plan(w, (5, 5), (50, 50))
        assert first.valid == second.valid
        assert first.waypoints == second.waypoints
        assert first.cost == second.cost

    def test_start_or_goal_outside_the_grid_is_invalid(self):
        n = 40
        res = ap.plan(_open(n), RES, 0.0, 0.0, (100.0, 0.0), _xy(10, 10, n), mask_rim_m=0.0)
        assert not res.valid


# --------------------------------------------------------------------------
# Optimality: A* cost must equal an independent Dijkstra on the same graph
# --------------------------------------------------------------------------


def _reference_cost(passable, cost, start, goal):
    """Independent shortest-path cost: scipy Dijkstra over the same
    8-connected, no-corner-cutting graph where entering cell X costs
    step_length * cost[X]. If `_astar` is optimal it must agree exactly."""
    n_rows, n_cols = passable.shape
    idx = lambda r, c: r * n_cols + c  # noqa: E731
    rows, cols, data = [], [], []
    for r in range(n_rows):
        for c in range(n_cols):
            if not passable[r, c]:
                continue
            for dr, dc, length in ap._MOVES:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < n_rows and 0 <= cc < n_cols) or not passable[rr, cc]:
                    continue
                if dr and dc and not (passable[r + dr, c] and passable[r, c + dc]):
                    continue
                rows.append(idx(r, c))
                cols.append(idx(rr, cc))
                data.append(length * cost[rr, cc])
    graph = coo_matrix((data, (rows, cols)), shape=(n_rows * n_cols,) * 2).tocsr()
    return dijkstra(graph, directed=True, indices=idx(*start))[idx(*goal)]


class TestOptimality:
    @pytest.mark.parametrize("seed", range(8))
    def test_astar_cost_matches_dijkstra_on_random_grids(self, seed):
        rng = np.random.default_rng(seed)
        passable = rng.random((30, 30)) > 0.25
        cost = 1.0 + rng.random((30, 30)) * 3.0  # varied, always >= 1
        start, goal = (1, 1), (28, 27)
        passable[start] = passable[goal] = True
        found = ap._astar(passable, cost, start, goal)
        expected = _reference_cost(passable, cost, start, goal)
        if math.isinf(expected):
            assert found is None
        else:
            assert found is not None
            assert found[1] == pytest.approx(expected)

    def test_returned_cells_form_a_connected_path_from_start_to_goal(self):
        passable = np.ones((20, 20), dtype=bool)
        passable[5:15, 10] = False
        cells, _ = ap._astar(passable, np.ones((20, 20)), (10, 2), (10, 18))
        assert cells[0] == (10, 2) and cells[-1] == (10, 18)
        for (r0, c0), (r1, c1) in zip(cells[:-1], cells[1:]):
            assert max(abs(r1 - r0), abs(c1 - c0)) == 1
            assert passable[r1, c1]


# --------------------------------------------------------------------------
# Complex geometry: the planner must find any path that exists
# --------------------------------------------------------------------------


class TestComplexGeometry:
    def test_finds_the_way_around_a_wall_with_a_gap_at_the_end(self):
        w = _open(60)
        w[0:45, 30] = False  # wall with an opening at the bottom
        res = _plan(w, (10, 10), (10, 50))
        assert res.valid
        # must actually pass through the gap (rows >= 45), not through the wall
        ys = [p[1] for p in res.waypoints]
        assert max(ys) >= _xy(45, 30, 60)[1]

    def test_s_curve_corridor(self):
        n = 60
        w = np.zeros((n, n), dtype=bool)
        # three horizontal lanes joined alternately at the right and left end
        w[5:13, 5:55] = True
        w[13:21, 47:55] = True
        w[21:29, 5:55] = True
        w[29:37, 5:13] = True
        w[37:45, 5:55] = True
        res = _plan(w, (8, 8), (41, 50))
        assert res.valid
        assert len(res.waypoints) >= 4  # genuinely bends, not one line

    def test_spiral_corridor_is_solved(self):
        n = 70
        w = np.zeros((n, n), dtype=bool)
        # a square spiral, corridors 9 cells wide
        w[3:12, 3:67] = True
        w[3:67, 58:67] = True
        w[58:67, 3:67] = True
        w[15:67, 3:12] = True
        w[15:24, 3:55] = True
        w[15:55, 46:55] = True
        res = _plan(w, (6, 6), (20, 8))
        assert res.valid
        assert len(res.waypoints) > 4

    def test_walled_off_goal_has_no_path(self):
        w = _open(60)
        w[:, 30] = False  # complete wall
        res = _plan(w, (10, 10), (10, 50))
        assert not res.valid
        assert res.waypoints == []


# --------------------------------------------------------------------------
# Inflation / robot size
# --------------------------------------------------------------------------


class TestInflation:
    def _corridor(self, free_width, n=60):
        """Two rooms joined by a horizontal corridor `free_width` cells wide."""
        w = np.zeros((n, n), dtype=bool)
        w[10:50, 5:20] = True
        w[10:50, 40:55] = True
        mid = n // 2
        top = mid - free_width // 2
        w[top : top + free_width, 20:40] = True
        return w, (mid, 10), (mid, 50)

    def test_corridor_far_wider_than_the_robot_is_used(self):
        w, s, g = self._corridor(10)
        assert _plan(w, s, g).valid

    def test_corridor_narrower_than_the_robot_is_rejected(self):
        # 3 free cells = 0.3 m wide; the robot is 0.44 m wide.
        w, s, g = self._corridor(3)
        assert not _plan(w, s, g).valid

    def test_returned_path_keeps_its_clearance_from_every_wall(self):
        w, s, g = self._corridor(9)
        res = _plan(w, s, g)
        assert res.valid
        for p, q in zip(res.waypoints[:-1], res.waypoints[1:]):
            assert _segment_min_clearance(w, p, q) >= res.inflation_m - 1e-6

    def test_relaxed_inflation_is_used_when_the_full_one_finds_nothing(self):
        # radius 0.2 + margin 0.15 -> full inflation 0.35; a 5-wide corridor's
        # centre line has clearance 0.3, so only the relaxed 0.2 passes.
        w, s, g = self._corridor(5)
        res = _plan(w, s, g, robot_radius_m=0.2, inflation_margin_m=0.15)
        assert res.valid
        assert res.inflation_m == pytest.approx(0.2)

    def test_full_inflation_is_used_when_it_is_enough(self):
        w, s, g = self._corridor(12)
        res = _plan(w, s, g, robot_radius_m=0.2, inflation_margin_m=0.15)
        assert res.inflation_m == pytest.approx(0.35)

    def test_mask_rim_opens_a_corridor_the_plain_inflation_would_close(self):
        # 5-wide corridor: centre clearance 0.3 > full inflation 0.27 anyway,
        # so use a 4-wide one (max clearance 0.2): closed with rim 0, open
        # with rim 0.1 (inflation drops to 0.17, relaxed to 0.12).
        w, s, g = self._corridor(4)
        assert not _plan(w, s, g, mask_rim_m=0.0).valid
        assert _plan(w, s, g, mask_rim_m=0.1).valid

    def test_diagonal_crack_between_corner_touching_walls_is_not_a_door(self):
        n = 50
        w = np.ones((n, n), dtype=bool)
        # a diagonal wall of single cells touching only at corners
        for k in range(n):
            w[k, k] = False
        res = _plan(w, (40, 5), (5, 40), robot_radius_m=0.0, inflation_margin_m=0.0)
        assert not res.valid


# --------------------------------------------------------------------------
# Start / goal handling
# --------------------------------------------------------------------------


class TestStartAndGoal:
    def test_goal_inside_a_hard_obstacle_is_rejected(self):
        w = _open(60)
        w[28:34, 28:34] = False
        assert not _plan(w, (5, 5), (30, 30)).valid

    def test_goal_in_the_inflation_band_snaps_to_nearby_free_space(self):
        w = _open(60)
        w[:, 30] = False
        w[:, 29] = True  # keep the wall thin
        # goal right next to the wall: walkable but inside the inflation band
        res = _plan(w, (10, 10), (30, 29))
        assert res.valid
        assert res.goal_xy != pytest.approx(_xy(30, 29, 60))
        # ...and it moved only a little
        assert math.dist(res.goal_xy, _xy(30, 29, 60)) <= 0.4 + 1e-6

    def test_start_inside_a_polluted_pocket_still_plans(self):
        # the UGV's own chassis can end up marked lethal in the map; the
        # footprint around the start must be treated as free
        w = _open(60)
        w[28:33, 28:33] = False
        res = _plan(w, (30, 30), (50, 50))
        assert res.valid

    def test_force_free_cells_survive_inflation(self):
        n = 60
        w = np.zeros((n, n), dtype=bool)
        w[10:50, 5:20] = True
        w[10:50, 40:55] = True
        w[29:31, 20:40] = True  # a 2-cell-wide crack - far too narrow
        assert not _plan(w, (30, 10), (30, 50)).valid
        forced = np.zeros((n, n), dtype=bool)
        # Confirmed passable by the 3D check. The real verification box is
        # padded (0.5 m), so it also covers the cells where the crack meets
        # each room - those would otherwise be inside the inflation band and
        # leave the forced cells unreachable.
        forced[29:31, 17:43] = True
        assert _plan(w, (30, 10), (30, 50), force_free_mask=forced).valid


# --------------------------------------------------------------------------
# Cost grid
# --------------------------------------------------------------------------


class TestCostGrid:
    def test_cost_is_never_below_one(self):
        clear = ap.clearance_m(np.zeros((20, 20), dtype=bool) | np.eye(20, dtype=bool), RES)
        cost = ap.build_cost_grid(clear, 0.27, None, None, None, 1.0, 0.5, 1.0, 20.0, 2.0)
        assert cost.min() >= 1.0

    def test_no_obstacles_means_flat_unit_cost(self):
        clear = ap.clearance_m(np.zeros((10, 10), dtype=bool), RES)
        cost = ap.build_cost_grid(clear, 0.27, None, None, None, 1.0, 0.5, 1.0, 20.0, 2.0)
        assert np.allclose(cost, 1.0)

    def test_cost_falls_with_distance_from_walls(self):
        blocked = np.zeros((20, 20), dtype=bool)
        blocked[:, 0] = True
        clear = ap.clearance_m(blocked, RES)
        cost = ap.build_cost_grid(clear, 0.2, None, None, None, 1.0, 0.5, 1.0, 20.0, 2.0)
        assert cost[10, 3] > cost[10, 8] >= 1.0

    def test_path_through_a_wide_corridor_is_not_dragged_against_a_wall(self):
        n = 60
        w = np.zeros((n, n), dtype=bool)
        w[10:50, 5:20] = True
        w[10:50, 40:55] = True
        w[22:38, 20:40] = True  # 16-wide corridor, centre row 29.5
        # start/goal are off-centre, so a purely geometric taut line would
        # slide along the corridor's wall; the cost-aware shortening must not
        res = _plan(w, (25, 10), (25, 50), standoff_m=0.0)
        assert res.valid
        worst = min(_segment_min_clearance(w, p, q) for p, q in zip(res.waypoints[:-1], res.waypoints[1:]))
        # clearance is not just barely the inflation radius: the path keeps
        # clear of the wall by more than the hard minimum
        assert worst > res.inflation_m + 0.1

    def test_difficult_terrain_is_avoided_when_a_cheap_detour_exists(self):
        n = 60
        w = _open(n)
        trav = np.ones((n, n))
        trav[20:40, 25:35] = 0.3  # a rough patch straight across the route
        straight = _plan(w, (30, 5), (30, 55))
        avoided = _plan(w, (30, 5), (30, 55), traversability=trav, difficult_weight=5.0)
        assert avoided.valid
        # straight route is a single segment; the avoiding one bends around
        assert len(straight.waypoints) == 2
        assert len(avoided.waypoints) > 2


# --------------------------------------------------------------------------
# Uncertain cells: the tunnel scenario
# --------------------------------------------------------------------------


def _tunnel_world(n=60):
    """Room A | anomaly band | island (tunnel roof) | anomaly band | room B,
    same shape the anomaly detector produces for a real roofed tunnel. Every
    other cell is wall."""
    walkable = np.zeros((n, n), dtype=bool)
    anomaly = np.zeros((n, n), dtype=bool)
    walkable[5:55, 0:20] = True  # room A
    walkable[5:55, 40:60] = True  # room B
    walkable[26:34, 22:38] = True  # island: tunnel roof reads as walkable
    anomaly[26:34, 20:22] = True  # west mouth band
    anomaly[26:34, 38:40] = True  # east mouth band
    return walkable, anomaly


class TestUncertainRoute:
    def test_without_anomaly_mode_the_tunnel_is_a_wall(self):
        w, a = _tunnel_world()
        assert not _plan(w, (30, 5), (30, 55), anomaly_mask=a).valid

    def test_with_anomaly_mode_a_tentative_route_through_it_is_found(self):
        w, a = _tunnel_world()
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        assert res.valid
        assert res.has_anomaly_segments
        assert not res.has_frontier_segments
        assert "anomaly" in res.segment_kinds

    def test_the_whole_tunnel_is_one_uncertain_stretch(self):
        # band / island / band must merge into ONE anomaly segment, not three
        w, a = _tunnel_world()
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        assert res.segment_kinds.count("anomaly") == 1

    def test_entry_and_exit_points_survive_shortening(self):
        # The uncertain stretch runs from where the path first comes within
        # the waive distance (0.5 m) of the uncertain region to where it last
        # does - so entry sits just BEFORE the west mouth (col 20) and exit
        # just AFTER the east mouth (col 39), by at most that distance (plus a
        # cell of rounding slack). Shortening must keep both.
        n = 60
        w, a = _tunnel_world(n)
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        i = res.segment_kinds.index("anomaly")
        entry, exit_ = res.waypoints[i], res.waypoints[i + 1]
        slack = ap.PlannerParams().uncertain_waive_m + RES
        west_mouth_x, east_mouth_x = _xy(0, 20, n)[0], _xy(0, 39, n)[0]
        assert west_mouth_x - slack <= entry[0] <= west_mouth_x
        assert east_mouth_x <= exit_[0] <= east_mouth_x + slack

    def test_a_standoff_point_is_pinned_before_the_entry(self):
        n = 60
        w, a = _tunnel_world(n)
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True, standoff_m=1.0)
        i = res.segment_kinds.index("anomaly")
        entry = res.waypoints[i]
        standoff = res.waypoints[i - 1]
        assert math.dist(standoff, entry) == pytest.approx(1.0, abs=0.2)
        # the segment up to the entry is ordinary confirmed ground
        assert res.segment_kinds[i - 1] == "free"

    def test_no_standoff_when_disabled(self):
        w, a = _tunnel_world()
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True, standoff_m=0.0)
        # start -> entry -> exit -> goal
        assert res.segment_kinds == ["free", "anomaly", "free"]

    def test_the_uncertain_segment_never_leaves_the_corridor(self):
        n = 60
        w, a = _tunnel_world(n)
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        i = res.segment_kinds.index("anomaly")
        ys = [res.waypoints[i][1], res.waypoints[i + 1][1]]
        lo, hi = _xy(26, 0, n)[1], _xy(33, 0, n)[1]
        assert all(lo <= y <= hi for y in ys)

    def test_a_confirmed_route_is_preferred_even_if_far_longer(self):
        # Same tunnel, plus a long open detour around the whole wall block.
        n = 60
        w, a = _tunnel_world(n)
        w[0:5, :] = True  # open strip across the top joins A and B
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        assert res.valid
        assert not res.has_anomaly_segments
        assert set(res.segment_kinds) == {"free"}

    def test_a_goal_inside_the_uncertain_region_is_not_silently_moved_in_strict_mode(self):
        w, a = _tunnel_world()
        # goal on a band cell: unreachable without anomaly mode...
        assert not _plan(w, (30, 5), (30, 21), anomaly_mask=a).valid
        # ...and reachable (tentatively) with it
        assert _plan(w, (30, 5), (30, 21), anomaly_mask=a, allow_anomaly=True).valid

    def test_frontier_cells_are_crossed_tentatively_and_flagged(self):
        n = 60
        w = np.zeros((n, n), dtype=bool)
        f = np.zeros((n, n), dtype=bool)
        w[5:55, 0:20] = True
        w[5:55, 40:60] = True
        f[26:34, 20:40] = True  # unobserved corridor between the rooms
        res = _plan(w, (30, 5), (30, 55), frontier_mask=f, allow_frontier=True)
        assert res.valid and res.has_frontier_segments and not res.has_anomaly_segments
        assert "frontier" in res.segment_kinds
        assert not _plan(w, (30, 5), (30, 55), frontier_mask=f).valid

    def test_a_route_that_only_works_by_assuming_unobserved_neighbours_are_clear_is_tentative(self):
        # REAL FINDING from replaying a saved map: a corridor only 4 cells
        # (0.4 m) wide with UNOBSERVED space on both sides. The strict pass
        # rejects it (it counts unobserved cells as obstacles, so there is no
        # clearance). The optimistic pass accepts it WITHOUT stepping on a
        # single frontier cell - the path is entirely on confirmed ground - but
        # it is still a gamble that the unknown neighbours are clear, so it
        # must NOT be reported as a confirmed route.
        from nav.walkability import compute_frontier_mask

        n = 60
        w = np.zeros((n, n), dtype=bool)
        valid = np.ones((n, n), dtype=bool)
        w[5:55, 0:20] = True
        w[5:55, 40:60] = True
        w[28:32, 20:40] = True  # 4-wide corridor
        valid[:28, 20:40] = False  # unobserved above it
        valid[32:, 20:40] = False  # ...and below it
        f = compute_frontier_mask(w, valid)

        strict_only = _plan(w, (30, 5), (30, 55))
        assert not strict_only.valid  # unknown space counts as an obstacle here

        res = _plan(w, (30, 5), (30, 55), frontier_mask=f, allow_frontier=True)
        assert res.valid
        assert res.has_frontier_segments  # honest: depends on the unknown being clear
        assert "frontier" in res.segment_kinds

    def test_a_corridor_with_real_clearance_is_still_confirmed(self):
        # Same shape but wide: confirmed ground with room to spare, so the
        # optimistic pass is never needed and nothing is flagged.
        from nav.walkability import compute_frontier_mask

        n = 60
        w = np.zeros((n, n), dtype=bool)
        valid = np.ones((n, n), dtype=bool)
        w[5:55, 0:20] = True
        w[5:55, 40:60] = True
        w[22:38, 20:40] = True  # 16-wide
        valid[:22, 20:40] = False
        valid[38:, 20:40] = False
        f = compute_frontier_mask(w, valid)
        res = _plan(w, (30, 5), (30, 55), frontier_mask=f, allow_frontier=True)
        assert res.valid and not res.has_frontier_segments

    def test_anomaly_cells_cost_more_than_frontier_cells(self):
        # equal-length detours: one over anomaly cells, one over frontier
        # cells; the planner must pick the frontier one
        n = 60
        w = np.zeros((n, n), dtype=bool)
        a = np.zeros((n, n), dtype=bool)
        f = np.zeros((n, n), dtype=bool)
        w[5:55, 0:15] = True
        w[5:55, 45:60] = True
        a[10:18, 15:45] = True  # upper corridor: anomaly
        f[40:48, 15:45] = True  # lower corridor: frontier
        res = _plan(
            w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True, frontier_mask=f, allow_frontier=True
        )
        assert res.valid
        assert res.has_frontier_segments and not res.has_anomaly_segments


# --------------------------------------------------------------------------
# Property test: whatever the planner returns, it must be safe
# --------------------------------------------------------------------------


class TestReturnedPathsAreAlwaysSafe:
    @pytest.mark.parametrize("seed", range(10))
    def test_every_segment_clears_every_obstacle_by_the_inflation_radius(self, seed):
        rng = np.random.default_rng(100 + seed)
        n = 60
        w = rng.random((n, n)) > 0.15
        # carve some rooms so there are real structures, not just noise
        w[10:25, 10:25] = True
        w[35:50, 35:50] = True
        w[24:36, 15:20] = True
        w[30:36, 15:45] = True
        res = _plan(w, (15, 15), (45, 45), mask_rim_m=0.0)
        if not res.valid:
            pytest.skip("this random map happens to have no route")
        for p, q in zip(res.waypoints[:-1], res.waypoints[1:]):
            # the start footprint is exempt by design, so only check segments
            # that do not begin inside it
            if math.dist(p, res.waypoints[0]) <= 0.3:
                continue
            assert _segment_min_clearance(w, p, q) >= res.inflation_m - 1e-6


class TestSpeed:
    def test_a_large_map_plans_in_reasonable_time(self):
        import time

        n = 400
        w = np.ones((n, n), dtype=bool)
        w[:, 200] = False
        w[100:300, 200] = True
        res_t0 = time.perf_counter()
        res = _plan(w, (50, 50), (350, 350))
        elapsed = time.perf_counter() - res_t0
        assert res.valid
        assert elapsed < 3.0  # generous bound; measured ~0.2 s


# --------------------------------------------------------------------------
# evaluate_path: re-judging an existing route against the current map
# --------------------------------------------------------------------------


class TestEvaluatePath:
    def _args(self, w):
        return dict(resolution=RES, center_x=0.0, center_y=0.0, mask_rim_m=0.0)

    def test_a_path_the_planner_just_produced_evaluates_as_valid(self):
        w, a = _tunnel_world()
        n = w.shape[0]
        res = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True)
        ok, cost, kinds = ap.evaluate_path(
            res.waypoints, w, RES, 0.0, 0.0, res.waypoints[0], res.mode, res.inflation_m,
            anomaly_mask=a, allow_anomaly=True, mask_rim_m=0.0,
        )  # fmt: skip
        assert ok
        assert kinds.count("anomaly") == 1
        assert cost > 0

    def test_a_new_wall_across_the_path_invalidates_it(self):
        w = _open(60)
        res = _plan(w, (30, 5), (30, 55))
        w2 = w.copy()
        w2[20:40, 30] = False
        ok, cost, _ = ap.evaluate_path(
            res.waypoints, w2, RES, 0.0, 0.0, res.waypoints[0], res.mode, res.inflation_m, mask_rim_m=0.0
        )
        assert not ok and math.isinf(cost)

    def test_waypoint_outside_the_grid_is_invalid(self):
        w = _open(60)
        ok, _, _ = ap.evaluate_path(
            [(0.0, 0.0), (100.0, 0.0)], w, RES, 0.0, 0.0, (0.0, 0.0), "strict", 0.2, mask_rim_m=0.0
        )
        assert not ok

    def test_cost_matches_across_the_same_route_evaluated_twice(self):
        w = _open(60)
        res = _plan(w, (30, 5), (30, 55))
        kw = dict(mask_rim_m=0.0)
        a1 = ap.evaluate_path(res.waypoints, w, RES, 0.0, 0.0, res.waypoints[0], res.mode, res.inflation_m, **kw)
        a2 = ap.evaluate_path(res.waypoints, w, RES, 0.0, 0.0, res.waypoints[0], res.mode, res.inflation_m, **kw)
        assert a1 == a2

    def test_unknown_override_name_is_rejected(self):
        with pytest.raises(TypeError):
            _plan(_open(), (10, 10), (20, 20), not_a_real_option=1)


class TestPlannerParams:
    def test_params_object_and_override_agree(self):
        w, a = _tunnel_world()
        via_override = _plan(w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True, standoff_m=0.0)
        via_params = _plan(
            w, (30, 5), (30, 55), anomaly_mask=a, allow_anomaly=True,
            params=ap.PlannerParams(standoff_m=0.0),
        )  # fmt: skip
        assert via_override.waypoints == via_params.waypoints
