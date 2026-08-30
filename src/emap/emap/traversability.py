"""Analytical traversability: turn the map's `elevation`/`variance` layers
into a per-cell "how easy is this to drive over" score.

This is an ORIGINAL design for this project, not adapted from `src/d1` -
that reference computes traversability with a trained multi-scale CNN
(`traversability_filter.py`: dilated 3x3 convolutions, weights loaded from a
pickle file produced by an offline training run we have no access to). We
have no training data or pipeline, and an opaque trained filter would work
against this whole project's point (from-scratch, explainable, unit-tested
at every step) - so this module builds traversability out of three
standard, well-understood terrain-analysis ideas instead: how STEEP the
ground is (slope), whether there's a sharp LEDGE nearby (step height), and
how NOISY/inconsistent the measurements there have been (roughness).

See docs/work-docs/emap/step07_traversability.md for the full from-scratch
walkthrough, the worked numeric examples behind the test suite, and an
honest discussion of this approach's limitations (particularly the
roughness-as-variance choice, flagged again below where it's used).
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import maximum_filter, minimum_filter

# Traversability scores use a small, explainable 3-tier scale rather than a
# continuous 0..1 number: every decision this module makes can be described
# in one sentence ("too steep", "too rough", "fine") and checked by hand,
# which matters far more for a from-scratch teaching project than a
# smoother-looking but harder-to-reason-about score would.
LETHAL = 0.0      # don't go here
DIFFICULT = 0.3   # possible, but avoid if there's a better way
EASY = 1.0        # no concern


def compute_traversability(
    elevation: np.ndarray,
    variance: np.ndarray,
    is_valid: np.ndarray,
    resolution: float,
    max_slope: float,
    max_step: float,
    max_roughness: float,
) -> np.ndarray:
    """Compute a `{LETHAL, DIFFICULT, EASY}` score for every cell.

    Args:
        elevation: (rows, cols) height per cell, in meters.
        variance: (rows, cols) uncertainty per cell (see below - reused here
            as a stand-in for "roughness").
        is_valid: (rows, cols) boolean/0-1 - has this cell ever actually been
            observed? Cells that haven't are left alone entirely (see the
            end of this function) - this module only forms an opinion about
            ground that's actually been measured.
        resolution: meters per cell - needed to turn a height DIFFERENCE
            between neighboring cells into an actual SLOPE (rise / run).
        max_slope: slope (rise/run, dimensionless) above which a cell is
            lethal.
        max_step: height difference (meters) within a cell's immediate
            neighborhood above which a cell is lethal.
        max_roughness: variance above which a cell is lethal.

    Returns:
        A new (rows, cols) array of traversability scores - callers are
        expected to only copy this into cells where `is_valid` is true (see
        `elevation_mapping_node.py`), matching every other layer's rule of
        never overwriting an unobserved cell's optimistic default.
    """
    # --- Slope: how steep is the ground at each cell? ---
    # np.gradient gives the rate of change of elevation along each axis
    # separately (finite differences - basically "neighbor to the right
    # minus neighbor to the left, divided by 2 cells" for interior cells).
    # `resolution` converts "height difference between adjacent CELLS" into
    # an actual physical slope (rise/run in real meters), since a cell is
    # `resolution` meters wide.
    grad_row, grad_col = np.gradient(elevation, resolution)
    slope = np.sqrt(grad_row**2 + grad_col**2)

    # --- Step height: is there a sharp ledge immediately nearby? ---
    # A smooth gradient can under-react to a single-cell-wide cliff, because
    # np.gradient averages across a cell's two neighbors. Directly comparing
    # the highest and lowest point within each cell's own 3x3 neighborhood
    # catches that case even when the "average" slope through it looks mild.
    local_max = maximum_filter(elevation, size=3)
    local_min = minimum_filter(elevation, size=3)
    step_height = local_max - local_min

    # --- Roughness: how noisy/inconsistent have measurements here been? ---
    # We reuse `variance` directly rather than computing a separate
    # roughness statistic - a cell whose repeated measurements disagree with
    # each other (high variance) is a reasonable proxy for physically rough
    # or unstable ground. HONEST LIMITATION: this also flags a cell that's
    # simply been seen only once or twice (variance starts high and only
    # shrinks with repeated confident measurements - see 00_concepts.md
    # Section 8-9) even if the ground there is perfectly flat. Treating
    # "not confidently measured yet" as "risky" is a defensible conservative
    # choice for a first pass, but it does mean roughness here isn't a pure
    # measure of physical terrain roughness - see step07's docs for more.
    roughness = variance

    lethal = (slope > max_slope) | (step_height > max_step) | (roughness > max_roughness)
    difficult = (slope > max_slope / 2.0) | (step_height > max_step / 2.0)

    result = np.full(elevation.shape, EASY, dtype=np.float32)
    result[difficult] = DIFFICULT
    result[lethal] = LETHAL  # checked last: lethal overrides difficult where both are true

    # Cells that have never actually been observed get no opinion at all
    # from this function - `elevation`/`variance` there are still just
    # step-3's placeholder defaults (0.0 / initial_variance), and computing
    # a slope/step/roughness "score" from placeholder data would be
    # meaningless. Returning EASY here specifically (rather than, say,
    # leaving these cells at whatever this function would have computed
    # anyway) exists purely so callers that forget to mask by `is_valid`
    # fail safe (never-observed ground reads as "easy", not "lethal") -
    # but the real, load-bearing rule is still enforced by the caller: only
    # copy this function's output into is_valid cells (see
    # elevation_mapping_node.py and the corresponding unit test).
    result = np.where(is_valid, result, EASY)
    return result
