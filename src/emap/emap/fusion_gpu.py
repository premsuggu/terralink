"""GPU port of `fusion.py`'s `fuse_points` (step 9).

This is the SAME algorithm as `fuse_points` - same 6 steps, same outlier
rule, same variance-weighted Bayesian combine - just run on the GPU via
CuPy instead of NumPy, because `ElevationMap`'s per-point/per-cell math
(distance, masks, scatter-accumulate over tens of thousands of points every
callback) is exactly the kind of "same operation, many data points" work a
GPU is built for. See `fusion.py`'s own docstring and
docs/work-docs/emap/00_concepts.md for the full explanation of the math
itself - this file only documents what's DIFFERENT about running it on a
GPU, not the math again.

`ElevationMap` itself stays plain NumPy (see step09 doc for why that scope
was deliberately not widened): this function copies just the two layers it
needs (`elevation`, `variance`) to the GPU once at the start, does the whole
batch of work there, and copies the result back once at the end. From the
outside, this function has the EXACT same contract as `fuse_points` - it
mutates `emap`'s NumPy layers in place - so callers (and every other part of
this codebase) never need to know or care which one ran.
"""
from __future__ import annotations

import cupy as cp
import numpy as np

from emap.elevation_map import ElevationMap


def fuse_points_gpu(
    emap: ElevationMap,
    points_xyz: np.ndarray,
    sensor_origin: np.ndarray,
    sensor_noise_factor: float,
    mahalanobis_thresh: float,
    outlier_variance: float,
    min_valid_distance: float,
    max_valid_range: float | None = None,
) -> None:
    """GPU version of `fusion.fuse_points` - same arguments, same effect
    (including `max_valid_range` - see fusion.py's docstring for why that
    filter exists: a real, observed depth-camera far-clip clamp artifact,
    not a hypothetical one)."""
    # --- Host -> device: only the two layers this function actually reads
    # or writes, plus this batch's points. Everything else about `emap`
    # (world_to_grid, in_bounds, shape) is cheap scalar/int math done on the
    # CPU exactly as before - only the big per-point/per-cell arrays need to
    # live on the GPU. ---
    points_xyz = cp.asarray(points_xyz, dtype=cp.float64)
    sensor_origin = cp.asarray(sensor_origin, dtype=cp.float64)

    # --- Step 1: distance from sensor to each point, and the min-range filter ---
    offset = points_xyz - sensor_origin
    range_sq = cp.sum(offset * offset, axis=1)
    far_enough = range_sq >= (min_valid_distance * min_valid_distance)
    not_clamped = (
        range_sq <= (max_valid_range * max_valid_range)
        if max_valid_range is not None
        else cp.ones_like(far_enough)
    )

    # --- Step 2: per-point measurement variance ---
    measurement_variance = sensor_noise_factor * range_sq

    # --- Step 3: which grid cell does each point land in, and is it real? ---
    # world_to_grid/in_bounds are plain NumPy-vectorized math on small
    # (N,) arrays of Python floats/ints - cheap enough on the CPU that
    # moving them to the GPU too would only add transfer overhead for no
    # benefit, so points_xyz's x/y are pulled back to host just for this.
    x = cp.asnumpy(points_xyz[:, 0])
    y = cp.asnumpy(points_xyz[:, 1])
    row, col = emap.world_to_grid(x, y)
    inside = emap.in_bounds(row, col)

    far_enough_host = cp.asnumpy(far_enough) & cp.asnumpy(not_clamped)
    keep = far_enough_host & inside
    if not np.any(keep):
        return

    row, col = row[keep], col[keep]
    z = points_xyz[:, 2][cp.asarray(keep)]
    point_variance = measurement_variance[cp.asarray(keep)]
    row_gpu, col_gpu = cp.asarray(row), cp.asarray(col)

    # --- Step 4: look up each surviving point's cell's current belief ---
    elevation_layer = emap.layer("elevation")
    variance_layer = emap.layer("variance")
    elevation_gpu = cp.asarray(elevation_layer, dtype=cp.float64)
    variance_gpu = cp.asarray(variance_layer, dtype=cp.float64)
    prior_h = elevation_gpu[row_gpu, col_gpu]
    prior_v = variance_gpu[row_gpu, col_gpu]

    # --- Step 5: outlier test (identical rule to the CPU version) ---
    is_outlier = cp.abs(prior_h - z) > (prior_v * mahalanobis_thresh)

    if bool(cp.any(is_outlier)):
        # cupy.add.at is a direct, verified-identical port of numpy's - see
        # fusion.py's comment on why plain fancy-index assignment is wrong
        # here (repeated indices in one batch must ACCUMULATE, not
        # last-write-wins).
        cp.add.at(variance_gpu, (row_gpu[is_outlier], col_gpu[is_outlier]), outlier_variance)

    # --- Step 6: fuse every inlier against the prior gathered in step 4 ---
    inlier = ~is_outlier
    if not bool(cp.any(inlier)):
        variance_layer[:] = cp.asnumpy(variance_gpu).astype(variance_layer.dtype)
        return

    v = point_variance[inlier]
    prior_h_in = prior_h[inlier]
    prior_v_in = prior_v[inlier]
    z_in = z[inlier]

    new_h = (prior_h_in * v + z_in * prior_v_in) / (prior_v_in + v)
    new_v = (prior_v_in * v) / (prior_v_in + v)

    rows_in, cols_in = row_gpu[inlier], col_gpu[inlier]

    sum_h = cp.zeros(emap.shape, dtype=cp.float64)
    sum_v = cp.zeros(emap.shape, dtype=cp.float64)
    count = cp.zeros(emap.shape, dtype=cp.float64)
    cp.add.at(sum_h, (rows_in, cols_in), new_h)
    cp.add.at(sum_v, (rows_in, cols_in), new_v)
    cp.add.at(count, (rows_in, cols_in), 1.0)

    touched = count > 0
    elevation_gpu[touched] = sum_h[touched] / count[touched]
    variance_gpu[touched] = sum_v[touched] / count[touched]

    # --- Device -> host: write the two updated layers back once. ---
    elevation_layer[:] = cp.asnumpy(elevation_gpu).astype(elevation_layer.dtype)
    variance_layer[:] = cp.asnumpy(variance_gpu).astype(variance_layer.dtype)
    touched_host = cp.asnumpy(touched)
    emap.layer("is_valid")[touched_host] = 1.0
