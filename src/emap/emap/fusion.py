"""Bayesian fusion: update an ElevationMap's `elevation`/`variance` layers from
a batch of 3D points (e.g. one camera frame's point cloud, already transformed
into the map's world frame - see step 2's TF pipeline).

The core idea, in plain terms first (full math below): every cell's current
height is a *belief*, not a certainty - it comes with a variance saying how
sure we are. A new sensor measurement is also a belief with its own variance
(farther-away points are noisier). Combining two independent, uncertain
beliefs about the same true value has a standard, principled answer - trust
whichever one is more confident (lower variance) more - and that's what
`fuse_points` computes, rather than just overwriting each cell with whatever
was measured most recently. See docs/work-docs/emap/00_concepts.md (Section 8)
and docs/work-docs/emap/step04_bayesian_fusion.md for the full from-scratch
explanation and worked examples.

Reference (consulted, not copied): the per-point update formula and the
outlier check below come from
src/d1/elevation_mapping_gpu_ros2/.../kernels/custom_kernels.py
(`add_points_kernel`). That GPU kernel uses a two-buffer atomic-accumulate
trick purely to avoid race conditions between many GPU threads writing to the
same cell at once. A single-threaded CPU function has no such race, so this
version reaches the same result more simply, using NumPy's `np.add.at` for
correct one-pass accumulation - see the comment above its use below for why
that specific function is necessary here, not just a nice-to-have.
"""
from __future__ import annotations

import numpy as np

from emap.elevation_map import ElevationMap


def fuse_points(
    emap: ElevationMap,
    points_xyz: np.ndarray,
    sensor_origin: np.ndarray,
    sensor_noise_factor: float,
    mahalanobis_thresh: float,
    outlier_variance: float,
    min_valid_distance: float,
) -> None:
    """Fuse one batch of points into `emap`, updating it in place.

    Args:
        emap: the map to update (its `elevation`/`variance`/`is_valid` layers
            are modified in place; nothing is returned).
        points_xyz: (N, 3) array of point positions, in the SAME world frame
            the map's `center_x`/`center_y` are expressed in (i.e. already
            transformed out of the sensor's own frame - that transform is the
            TF lookup step 2 already verified, not this function's job).
        sensor_origin: (3,) array - where the sensor was when it took this
            batch of points, in that same world frame. Used only to compute
            how far each point is from the sensor (farther = noisier).
        sensor_noise_factor: scales how quickly measurement noise grows with
            distance (`variance = sensor_noise_factor * range^2`) - a
            property of the sensor, not of any one point cloud.
        mahalanobis_thresh: how many "standard deviations" a measurement is
            allowed to disagree with a confident cell before it's treated as
            an outlier instead of being fused in.
        outlier_variance: how much to inflate a cell's variance by every time
            an outlier lands in it (an outlier is evidence the cell might be
            less well-understood than we thought - e.g. the terrain changed -
            even though we don't trust the outlier's specific height value).
        min_valid_distance: points closer than this to the sensor are dropped
            outright (real depth sensors are unreliable at very short range;
            this also incidentally protects against a sensor measuring a
            point essentially on top of itself, which would make `range` used
            in the noise formula meaninglessly small).
    """
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    sensor_origin = np.asarray(sensor_origin, dtype=np.float64)

    # --- Step 1: distance from sensor to each point, and the min-range filter ---
    offset = points_xyz - sensor_origin  # (N, 3): vector from sensor to each point
    range_sq = np.sum(offset * offset, axis=1)  # (N,): squared distance - avoids an
    # unnecessary sqrt for every point; both the min-distance check and the
    # noise formula below only ever need range^2, never the range itself.
    far_enough = range_sq >= (min_valid_distance * min_valid_distance)

    # --- Step 2: per-point measurement variance (noisier the farther away it is) ---
    measurement_variance = sensor_noise_factor * range_sq

    # --- Step 3: which grid cell does each point land in, and is that cell real? ---
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    row, col = emap.world_to_grid(x, y)
    inside = emap.in_bounds(row, col)

    keep = far_enough & inside
    if not np.any(keep):
        return  # nothing left to fuse - e.g. an empty or fully-out-of-range cloud

    row, col, z = row[keep], col[keep], z[keep]
    point_variance = measurement_variance[keep]

    # --- Step 4: look up what each surviving point's cell currently believes ---
    # Fancy indexing here (unlike the accumulation below) is safe even with
    # repeated (row, col) pairs, because we're only READING - every point
    # just needs its own cell's current value, independent of the others.
    elevation_layer = emap.layer("elevation")
    variance_layer = emap.layer("variance")
    prior_h = elevation_layer[row, col]
    prior_v = variance_layer[row, col]

    # --- Step 5: outlier test ---
    # If this measurement disagrees with the cell's current belief by more
    # than `mahalanobis_thresh` standard deviations (sqrt of variance is a
    # standard deviation; multiplying the threshold by variance directly,
    # as the reference kernel does, is a slightly cheaper equivalent scaling
    # that avoids a sqrt per point - it changes the effective threshold's
    # units but not the comparison's correctness for a fixed, tuned constant),
    # treat it as an outlier: don't let it move the height, but do inflate
    # the cell's variance, since something unexpected was just observed there.
    is_outlier = np.abs(prior_h - z) > (prior_v * mahalanobis_thresh)

    if np.any(is_outlier):
        # np.add.at is a "scatter-add": it correctly ADDS into repeated
        # indices one at a time, instead of the last write silently winning
        # (which is what plain `variance_layer[row[is_outlier], col[is_outlier]]
        # += outlier_variance` would do if the SAME cell appears twice in this
        # batch - a real, easy-to-miss NumPy pitfall, not a hypothetical one:
        # a dense camera point cloud routinely puts many points in one cell).
        np.add.at(variance_layer, (row[is_outlier], col[is_outlier]), outlier_variance)

    # --- Step 6: fuse every inlier's own estimate against the prior we already
    # gathered in step 4 (not against each other - see the module docstring's
    # note on why this matches the reference kernel's behavior) ---
    inlier = ~is_outlier
    if not np.any(inlier):
        return

    v = point_variance[inlier]
    prior_h_in = prior_h[inlier]
    prior_v_in = prior_v[inlier]
    z_in = z[inlier]

    # The Bayesian "combine two Gaussian beliefs" formula: the fused mean is
    # a variance-weighted average (whichever input has smaller variance pulls
    # the result closer to itself), and the fused variance is always smaller
    # than either input alone (combining two independent observations can
    # only make us more certain, never less).
    new_h = (prior_h_in * v + z_in * prior_v_in) / (prior_v_in + v)
    new_v = (prior_v_in * v) / (prior_v_in + v)

    rows_in, cols_in = row[inlier], col[inlier]

    # Several points can land in the same cell within one batch (very common
    # with a dense camera point cloud). We can't just write new_h/new_v
    # straight into the map with fancy-index assignment - like the outlier
    # case above, a repeated (row, col) pair would silently keep only the
    # LAST point's result and throw the others away. Instead we accumulate a
    # sum and a count per cell with np.add.at, then divide once at the end -
    # i.e. every point that landed in a cell contributes equally to that
    # cell's final fused value for this batch.
    sum_h = np.zeros(emap.shape, dtype=np.float64)
    sum_v = np.zeros(emap.shape, dtype=np.float64)
    count = np.zeros(emap.shape, dtype=np.float64)
    np.add.at(sum_h, (rows_in, cols_in), new_h)
    np.add.at(sum_v, (rows_in, cols_in), new_v)
    np.add.at(count, (rows_in, cols_in), 1.0)

    touched = count > 0
    elevation_layer[touched] = (sum_h[touched] / count[touched]).astype(elevation_layer.dtype)
    variance_layer[touched] = (sum_v[touched] / count[touched]).astype(variance_layer.dtype)
    emap.layer("is_valid")[touched] = 1.0
