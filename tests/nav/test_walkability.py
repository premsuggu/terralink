"""Unit tests for nav.walkability.compute_walkable_mask.

Pure NumPy - no ROS. Proves the two design rules this module exists to
enforce (see its module docstring): traversability, not absolute elevation,
decides walkability; and an unobserved cell is excluded, never assumed safe.
"""
import numpy as np

from emap.traversability import DIFFICULT, EASY, LETHAL
from nav.walkability import compute_frontier_mask, compute_walkable_mask


class TestComputeWalkableMask:
    def test_easy_and_difficult_observed_cells_are_walkable(self):
        traversability = np.array([[EASY, DIFFICULT]], dtype=np.float32)
        is_valid = np.array([[1.0, 1.0]], dtype=np.float32)
        mask = compute_walkable_mask(traversability, is_valid)
        assert mask.tolist() == [[True, True]]

    def test_lethal_observed_cell_is_not_walkable(self):
        traversability = np.array([[LETHAL]], dtype=np.float32)
        is_valid = np.array([[1.0]], dtype=np.float32)
        mask = compute_walkable_mask(traversability, is_valid)
        assert mask.tolist() == [[False]]

    def test_unobserved_easy_cell_is_not_walkable(self):
        # compute_traversability's own default for an unobserved cell is
        # EASY (a fail-safe for other callers, not a safety claim - see its
        # docstring) - this is the exact case this module must NOT trust.
        traversability = np.array([[EASY]], dtype=np.float32)
        is_valid = np.array([[0.0]], dtype=np.float32)
        mask = compute_walkable_mask(traversability, is_valid)
        assert mask.tolist() == [[False]]

    def test_accepts_boolean_is_valid_as_well_as_float(self):
        traversability = np.array([[EASY, LETHAL]], dtype=np.float32)
        is_valid = np.array([[True, True]])
        mask = compute_walkable_mask(traversability, is_valid)
        assert mask.tolist() == [[True, False]]

    def test_mixed_grid_matches_elementwise_expectation(self):
        traversability = np.array(
            [[EASY, LETHAL, DIFFICULT], [LETHAL, EASY, EASY]], dtype=np.float32
        )
        is_valid = np.array([[1, 1, 0], [1, 1, 1]], dtype=np.float32)
        mask = compute_walkable_mask(traversability, is_valid)
        expected = [[True, False, False], [False, True, True]]
        assert mask.tolist() == expected


class TestComputeFrontierMask:
    """See docs/work-docs/nav/step05_frontier_tunnel_navigation.md for the
    real-world scenario this exists for: a tunnel/culvert whose interior the
    UAV's overhead camera can never observe, even in principle.
    """

    def test_unobserved_cell_next_to_walkable_is_frontier(self):
        # Center cell unobserved, one neighbor (right) walkable - the
        # simplest possible "edge of the known map" case.
        walkable = np.array([[False, False, False], [False, False, True], [False, False, False]])
        is_valid = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
        frontier = compute_frontier_mask(walkable, is_valid)
        assert frontier.tolist() == [
            [False, False, False],
            [False, True, False],
            [False, False, False],
        ]

    def test_unobserved_cell_with_no_walkable_neighbor_is_not_frontier(self):
        # Deep in unexplored territory - nothing walkable borders it, so
        # there's no reason to believe it's ever reachable. Must stay
        # excluded exactly like today, not become a frontier by default.
        walkable = np.zeros((3, 3), dtype=bool)
        is_valid = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
        frontier = compute_frontier_mask(walkable, is_valid)
        assert not frontier.any()

    def test_observed_cell_is_never_frontier_regardless_of_neighbors(self):
        # A cell that HAS been observed (walkable or not) is not "unknown" -
        # it can't be a frontier no matter what surrounds it.
        walkable = np.array([[True, True], [False, False]])
        is_valid = np.array([[1, 1], [1, 1]])  # everything observed
        frontier = compute_frontier_mask(walkable, is_valid)
        assert not frontier.any()

    def test_diagonal_only_neighbor_does_not_count(self):
        # 4-connectivity only (the standard frontier-detection convention) -
        # a walkable cell that's only diagonally adjacent doesn't make the
        # unobserved cell a frontier, since nothing confirms the straight
        # path between them is itself walkable.
        walkable = np.array([[True, False], [False, False]])
        is_valid = np.array([[1, 0], [0, 1]])  # (1,1)="observed" just to isolate (0,1)/(1,0)
        # Only cell (1, 1) is unobserved-and-diagonal-to-walkable here; check it directly.
        walkable2 = np.array([[True, False], [False, False]])
        is_valid2 = np.array([[1, 1], [1, 0]])
        frontier = compute_frontier_mask(walkable2, is_valid2)
        assert frontier.tolist() == [[False, False], [False, False]]

    def test_border_cell_has_no_out_of_bounds_neighbor_crash(self):
        # A frontier cell sitting on the grid's own edge must not need an
        # off-map neighbor to qualify, and must not raise/index out of bounds.
        walkable = np.array([[False, True], [False, False]])
        is_valid = np.array([[0, 1], [1, 1]])
        frontier = compute_frontier_mask(walkable, is_valid)
        assert frontier.tolist() == [[True, False], [False, False]]


class TestFrontierThinStripPruning:
    """A strip of unobserved cells narrower than the robot is not a passage
    (on a top-down map it is nearly always an unscanned thin WALL TOP)."""

    RES = 0.1

    def _world(self):
        # two free rooms either side of a 2-cell-thick unobserved "wall"; the
        # wall runs down into a big unobserved area at the bottom
        n = 60
        walkable = np.zeros((n, n), dtype=bool)
        valid = np.ones((n, n), dtype=bool)
        walkable[:40, :29] = True
        walkable[:40, 31:] = True
        valid[:40, 29:31] = False  # the thin seam
        valid[40:, :] = False  # a big unobserved area below
        return walkable, valid

    def test_without_pruning_the_seam_is_frontier(self):
        w, v = self._world()
        f = compute_frontier_mask(w, v)
        assert f[10, 29] and f[10, 30]

    def test_a_thin_seam_is_dropped_with_pruning(self):
        w, v = self._world()
        f = compute_frontier_mask(w, v, min_unobserved_width_m=0.44, resolution=self.RES)
        assert not f[10, 29] and not f[10, 30]
        assert not f[20, 29]

    def test_a_seam_attached_to_a_big_area_is_still_dropped_away_from_the_junction(self):
        # the component-level thickness would call this seam thick, because it
        # connects to the big area - the check has to be local
        w, v = self._world()
        f = compute_frontier_mask(w, v, min_unobserved_width_m=0.44, resolution=self.RES)
        assert not f[5, 29]  # far up the seam

    def test_real_open_unobserved_space_is_kept(self):
        w, v = self._world()
        f = compute_frontier_mask(w, v, min_unobserved_width_m=0.44, resolution=self.RES)
        assert f[40, 5]  # the unobserved row just below the room's edge
        assert f[40, 50]

    def test_a_wide_gap_is_kept(self):
        n = 60
        w = np.zeros((n, n), dtype=bool)
        v = np.ones((n, n), dtype=bool)
        w[:, :20] = True
        w[:, 40:] = True
        v[:, 20:40] = False  # 2 m of unobserved space between two free areas
        f = compute_frontier_mask(w, v, min_unobserved_width_m=0.44, resolution=self.RES)
        assert f[30, 20] and f[30, 39]

    def test_default_is_unchanged(self):
        w, v = self._world()
        assert np.array_equal(
            compute_frontier_mask(w, v), compute_frontier_mask(w, v, min_unobserved_width_m=0.0, resolution=0.1)
        )
