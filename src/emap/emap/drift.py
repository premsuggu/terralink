"""Vertical (Z) drift compensation (step 10).

The problem, in plain terms: a real robot doesn't know its own position
directly - it estimates it (odometry/IMU), and every estimate has a tiny
error. Those tiny errors add up over time ("drift"), so after flying around
for a while, the robot may genuinely believe it's higher or lower than it
truly is. Since every mapped point is placed using that belief, drift
silently misplaces terrain - fly over the same spot twice and you can get
two different heights for what is actually one flat patch of ground.

The fix used here: the map itself is the reference. A cell that's been
fused many times and agrees with itself has LOW variance - the map is
confident about it. If a new batch of points lands on those same confident
cells and reads a systematically different height, that's stronger evidence
the SENSOR'S BELIEF ABOUT ITS OWN POSITION drifted than that the ground
itself changed - so the disagreement is used to correct the pose going
forward, not to overwrite the trusted cell.

Scoped to Z only (not X/Y): correcting horizontal drift properly needs
scan-matching/registration against the map (search over 2D offsets for the
best alignment) - a materially bigger algorithm, and this sim has no real
X/Y drift source to verify a horizontal correction against anyway. Z drift
is directly measurable this way because "expected height at this (x, y)" is
exactly what the map already stores.

See docs/work-docs/emap/00_concepts.md ("Pose drift and how to correct it")
and step10_drift_compensation.md for the from-scratch explanation and the
worked synthetic example this module's tests are built from.
"""
from __future__ import annotations

import numpy as np

from emap.elevation_map import ElevationMap


def estimate_vertical_drift(
    emap: ElevationMap,
    points_xyz: np.ndarray,
    min_confidence_variance: float,
    min_matches: int,
    max_reasonable_residual: float | None = None,
) -> float | None:
    """Estimate how far the current pose's Z belief has drifted, using this
    batch of points measured against the map's most-confident cells.

    Args:
        emap: the map whose `elevation`/`variance`/`is_valid` layers serve
            as the trusted reference (see module docstring).
        points_xyz: (N, 3) points already in the map frame - the SAME
            already-transformed points `fuse_points` receives (this
            function only reads `emap` and `points_xyz`; it never modifies
            either).
        min_confidence_variance: a cell only counts as a trustworthy
            reference if its current variance is below this - i.e. it's
            been fused enough times that its stored height is genuinely
            well-established, not just step 3's optimistic-but-unproven
            initial guess.
        min_matches: minimum number of confident-cell matches required
            before returning an estimate at all. Below this, there simply
            isn't enough evidence to conclude the pose has drifted rather
            than just seeing normal per-point noise - returning `None`
            (rather than a noisy guess from 2-3 points) is the safe default.
        max_reasonable_residual: if given, a computed residual larger than
            this (in either direction) is treated as untrustworthy and
            `None` is returned instead. Real pose drift accumulates slowly,
            so a single callback's residual should stay small; a huge one is
            far more likely to be a bad sensor frame (a depth camera
            artifact - e.g. a near-clip-plane glitch or a momentary self-
            reflection off the UAV's own body, both real, observed
            failure modes for this project's camera) than genuine drift.
            Mirrors `fuse_points`' own outlier rejection - the same
            "disagreement too large to be trusted" principle, applied here
            to the aggregate estimate instead of one point.

    Returns:
        The estimated Z bias (meters, `measured - true`: positive means the
        pose currently reads too HIGH) as the median residual across every
        matched point, or `None` if fewer than `min_matches` confident
        matches were found this batch.
    """
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]

    row, col = emap.world_to_grid(x, y)
    inside = emap.in_bounds(row, col)
    row, col, z = row[inside], col[inside], z[inside]
    if row.size == 0:
        return None

    is_valid = emap.layer("is_valid") > 0.5
    variance_layer = emap.layer("variance")
    confident = is_valid[row, col] & (variance_layer[row, col] < min_confidence_variance)
    if np.count_nonzero(confident) < min_matches:
        return None

    elevation_layer = emap.layer("elevation")
    matched_true_h = elevation_layer[row[confident], col[confident]]
    matched_measured_h = z[confident]

    # Median, not mean: a handful of points landing on a real obstacle
    # inside an otherwise-flat confident patch shouldn't drag the estimate
    # off - the median is robust to exactly that kind of minority outlier,
    # the same reasoning fuse_points' own outlier rejection relies on.
    residual = matched_measured_h - matched_true_h
    estimate = float(np.median(residual))

    if max_reasonable_residual is not None and abs(estimate) > max_reasonable_residual:
        return None

    return estimate
