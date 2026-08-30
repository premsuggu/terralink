"""Unit tests for emap.traversability.compute_traversability.

Pure NumPy on small, hand-built synthetic elevation/variance arrays - no
ROS, no Gazebo, no real sensor data. Every expected result here was checked
against the real function before being written down (see
docs/work-docs/emap/step07_traversability.md for the same numbers explained
step by step), not just asserted.

All tests use resolution=0.1 (matching this project's real map resolution)
rather than a round number like 1.0 - a resolution of 1.0 turned out to make
the slope and step-height metrics interfere with each other in a way that's
specific to that choice of numbers (a sustained ramp 1m per cell always
racks up more than max_step's worth of height across a 3-cell window before
its PER-CELL slope alone reaches max_slope), which made it impossible to
build a test case that isolates "slope reached DIFFICULT/LETHAL" from "step
height also reached LETHAL". At the real resolution, the two metrics behave
independently enough to test one at a time.
"""
import numpy as np
import pytest

from emap.traversability import compute_traversability, LETHAL, DIFFICULT, EASY

RESOLUTION = 0.1
MAX_SLOPE = 0.35
MAX_STEP = 0.15
MAX_ROUGHNESS = 0.05


def _compute(elevation, variance, is_valid=None):
    if is_valid is None:
        is_valid = np.ones_like(elevation, dtype=bool)
    return compute_traversability(
        elevation, variance, is_valid, RESOLUTION, MAX_SLOPE, MAX_STEP, MAX_ROUGHNESS
    )


def test_flat_low_variance_ground_is_easy_everywhere():
    elevation = np.zeros((5, 5))
    variance = np.zeros((5, 5))
    result = _compute(elevation, variance)
    assert np.all(result == EASY)


def test_a_sharp_cliff_is_lethal_at_and_near_the_edge_only():
    # Flat on the left, flat (0.4m higher) on the right - a genuine step,
    # not a ramp. 0.4m far exceeds max_step (0.15m).
    elevation = np.zeros((5, 5))
    elevation[:, 2:] = 0.4
    variance = np.zeros((5, 5))

    result = _compute(elevation, variance)

    # Columns 1 and 2 straddle the cliff (within a 3x3 window of the jump) -
    # lethal. Column 0 (flat, far from the edge) and columns 3-4 (flat, atop
    # the raised side, far from the edge) are untouched - still easy.
    expected = np.array([[EASY, LETHAL, LETHAL, EASY, EASY]] * 5)
    assert np.array_equal(result, expected)


@pytest.mark.parametrize(
    "per_cell_slope, expected",
    [
        (0.05, EASY),        # well under half of max_slope, and too gentle to trip step height either
        (0.2, DIFFICULT),    # between max_slope/2 (0.175) and max_slope (0.35)
        (0.5, LETHAL),       # over max_slope
    ],
)
def test_sustained_slope_is_classified_by_steepness(per_cell_slope, expected):
    # A uniform ramp: elevation rises by `per_cell_slope * RESOLUTION` meters
    # for every cell moved along the columns - i.e. a constant slope of
    # exactly `per_cell_slope` (rise/run), verified below via np.gradient
    # before trusting the traversability result.
    cols = np.arange(5)
    elevation = np.tile(cols * per_cell_slope * RESOLUTION, (5, 1))
    variance = np.zeros((5, 5))

    grad_row, grad_col = np.gradient(elevation, RESOLUTION)
    actual_slope = float(np.sqrt(grad_row[2, 2] ** 2 + grad_col[2, 2] ** 2))
    assert actual_slope == pytest.approx(per_cell_slope)

    result = _compute(elevation, variance)
    # Check the interior columns only - np.gradient's one-sided differences
    # at the very first/last column can read slightly differently there,
    # which isn't what this test is about.
    assert np.all(result[:, 1:-1] == expected)


def test_high_variance_alone_makes_flat_ground_lethal():
    # Elevation is perfectly flat - only variance (our roughness proxy) is
    # above threshold. This confirms roughness can condemn a cell on its
    # own, independent of slope or step height.
    elevation = np.zeros((5, 5))
    variance = np.full((5, 5), 0.1)  # > max_roughness (0.05)
    result = _compute(elevation, variance)
    assert np.all(result == LETHAL)


def test_never_observed_cells_are_left_untouched_regardless_of_raw_values():
    # Deliberately absurd elevation/variance values that WOULD be lethal if
    # they were real - the point is that a cell that has never actually been
    # observed (is_valid=False) must not get an opinion formed about it at
    # all, no matter what placeholder numbers happen to be sitting in its
    # elevation/variance layers (see the "don't touch what we haven't earned
    # an opinion about" rule from steps 3/5).
    elevation = np.full((5, 5), 100.0)
    variance = np.full((5, 5), 100.0)
    is_valid = np.zeros((5, 5), dtype=bool)

    result = _compute(elevation, variance, is_valid)

    assert np.all(result == EASY)
