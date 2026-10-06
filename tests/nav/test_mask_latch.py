"""Unit tests for nav.mask_latch - holding a flickering mask for a while."""
import numpy as np

from nav.mask_latch import MaskLatch


def _mask(*cells, shape=(6, 6)):
    m = np.zeros(shape, dtype=bool)
    for r, c in cells:
        m[r, c] = True
    return m


class TestMaskLatch:
    def test_hold_zero_returns_the_input_object_unchanged(self):
        latch = MaskLatch(0.0)
        m = _mask((1, 1))
        assert latch.update(m, now=0.0) is m

    def test_a_cell_that_disappears_is_held_for_hold_sec(self):
        latch = MaskLatch(10.0)
        latch.update(_mask((2, 2)), now=0.0)
        held = latch.update(_mask(), now=5.0)  # gone from this message
        assert held[2, 2]

    def test_a_cell_is_released_after_hold_sec(self):
        latch = MaskLatch(10.0)
        latch.update(_mask((2, 2)), now=0.0)
        assert not latch.update(_mask(), now=10.5)[2, 2]

    def test_a_cell_seen_again_restarts_its_hold(self):
        latch = MaskLatch(10.0)
        latch.update(_mask((2, 2)), now=0.0)
        latch.update(_mask((2, 2)), now=8.0)
        assert latch.update(_mask(), now=15.0)[2, 2]  # 7 s since last seen
        assert not latch.update(_mask(), now=19.0)[2, 2]  # 11 s

    def test_cells_never_flagged_stay_off(self):
        latch = MaskLatch(10.0)
        held = latch.update(_mask((0, 0)), now=0.0)
        assert held.sum() == 1

    def test_never_flagged_mask_stays_empty(self):
        latch = MaskLatch(10.0)
        assert not latch.update(_mask(), now=0.0).any()

    def test_shape_change_resets_instead_of_crashing(self):
        latch = MaskLatch(10.0)
        latch.update(_mask((1, 1)), now=0.0)
        out = latch.update(_mask(shape=(8, 8)), now=1.0)
        assert out.shape == (8, 8) and not out.any()

    def test_clear_forgets_everything(self):
        latch = MaskLatch(10.0)
        latch.update(_mask((1, 1)), now=0.0)
        latch.clear()
        assert not latch.update(_mask(), now=1.0).any()
