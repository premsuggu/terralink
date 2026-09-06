"""Unit tests for nav.walkability.compute_walkable_mask.

Pure NumPy - no ROS. Proves the two design rules this module exists to
enforce (see its module docstring): traversability, not absolute elevation,
decides walkability; and an unobserved cell is excluded, never assumed safe.
"""
import numpy as np

from emap.traversability import DIFFICULT, EASY, LETHAL
from nav.walkability import compute_walkable_mask


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
