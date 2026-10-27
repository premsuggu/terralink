"""Offline UAV-scan simulator: turns a TRUE height field into the noisy, partial `ElevationMap` a real downward-looking
depth-camera pass would have produced - without Gazebo.

Why it exists: the classifier must be judged on what the live map looks like (one noisy observation per cell, range-dependent
variance, holes where nothing was seen), not on the noiseless ground truth. A live UAV scan of a 60 m world takes many minutes
of simulation; this takes seconds and is repeatable. It reuses the REAL `fuse_points` and the live camera model
(models/iris_quad/model.sdf: 320 x 240, horizontal FOV 1.047 rad, clip 0.1-20 m), so the variance layer behaves as in the node.

Simplifications (written down so nobody over-trusts the numbers): the camera looks straight down and does not roll; the
height field is 2.5D (no overhang under a roof - the roof IS the surface, as for the real UAV); depth noise is Gaussian
`sigma = noise_a + noise_b * range^2` (the real sim camera is almost noiseless: ~3 mm on flat ground, measured on
tests/nav/fixtures/tunnel_test_parked.npz); there is no pose drift.
"""
from __future__ import annotations

import numpy as np

from emap.elevation_map import ElevationMap
from emap.fusion import fuse_points

# Live-node fusion parameters (config/elevation_mapping.yaml) - kept in one place so the evaluation matches the node.
FUSION = dict(sensor_noise_factor=0.01, mahalanobis_thresh=2.0, outlier_variance=1.0, min_valid_distance=0.2, max_valid_range=19.8)
HFOV = 1.047
IMG_W, IMG_H = 320, 240


def lawnmower(x0: float, x1: float, y0: float, y1: float, altitude: float, overlap: float = 0.5, hfov: float = HFOV):
    """Poses (x, y) of a back-and-forth survey over the rectangle. Lane spacing = footprint width * (1 - overlap)."""
    foot = 2.0 * altitude * np.tan(hfov / 2.0)
    step = max(foot * (1.0 - overlap), 0.5)
    lo, hi = y0 + min(foot / 2, (y1 - y0) / 2), y1 - min(foot / 2, (y1 - y0) / 2)       # lanes: the footprint stays inside the area
    ys = np.linspace(lo, hi, max(int(np.ceil((hi - lo) / step)) + 1, 1))
    xs = np.linspace(x0, x1, max(int(np.ceil((x1 - x0) / step)) + 1, 2))                # a pose every <= `step` along a lane
    poses = []
    for i, y in enumerate(ys):
        row = xs if i % 2 == 0 else xs[::-1]
        poses += [(float(x), float(y)) for x in row]
    return poses


def _cast(height: np.ndarray, res: float, x0: float, y0: float, origin: np.ndarray, dirs: np.ndarray, far: float, dt: float):
    """March each ray (origin + t*dir) until it dips below the height field; return the hit points (N,3) and a hit mask."""
    n = dirs.shape[0]
    t = np.full(n, 0.1)
    hit = np.zeros(n, bool)
    alive = np.ones(n, bool)
    ny, nx = height.shape
    out = np.zeros((n, 3))
    while alive.any():
        idx = np.flatnonzero(alive)
        p = origin + t[idx, None] * dirs[idx]
        c = np.round((p[:, 0] - x0) / res).astype(int)
        r = np.round((p[:, 1] - y0) / res).astype(int)
        inside = (c >= 0) & (c < nx) & (r >= 0) & (r < ny)
        h = np.where(inside, height[np.clip(r, 0, ny - 1), np.clip(c, 0, nx - 1)], -np.inf)
        below = inside & (p[:, 2] <= h)
        hit[idx[below]] = True
        q = p[below].copy()
        q[:, 2] = h[below]                      # snap to the surface: a march step overshoots by up to half a step (a -2 cm bias)
        out[idx[below]] = q
        alive[idx[below]] = False
        alive[idx[~inside | (t[idx] > far)]] = False
        t[idx] += dt
        if t.max() > far + dt:
            break
    return out, hit


def scan(height: np.ndarray, res: float, x0: float, y0: float, poses, altitude: float, *, map_length: float,
         center: tuple[float, float] = (0.0, 0.0), noise_a: float = 0.002, noise_b: float = 0.0, seed: int = 0,
         stride: int = 2, clearance: float | None = None):
    """Fly `poses` at `altitude` above the highest surface (or `clearance` over the local height if given) and fuse every frame.

    `height[row, col]` is the true surface at the CENTRE of cell (row, col) = (x0 + col*res, y0 + row*res) - the ElevationMap
    convention (row <-> y, col <-> x), so `x0 = center_x - n/2*res`. Returns the filled `ElevationMap`.
    `stride` subsamples the 320 x 240 image (2 -> 160 x 120 rays) to keep the simulation fast; the point density per 0.1 m
    cell stays above 1 for altitudes up to ~5 m.
    """
    rng = np.random.default_rng(seed)
    emap = ElevationMap(res, map_length, initial_variance=10.0)
    emap.center_x, emap.center_y = center
    f = (IMG_W / 2) / np.tan(HFOV / 2)
    u = (np.arange(0, IMG_W, stride) + 0.5 - IMG_W / 2) / f
    v = (np.arange(0, IMG_H, stride) + 0.5 - IMG_H / 2) / f
    uu, vv = np.meshgrid(u, v)
    d = np.stack([uu.ravel(), vv.ravel(), -np.ones(uu.size)], 1)       # camera looks down (-z)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    for (px, py) in poses:
        z = altitude if clearance is None else float(np.nanmax(height)) + clearance
        origin = np.array([px, py, z])
        pts, ok = _cast(height, res, x0, y0, origin, d, 19.8, res / 2.5)
        if not ok.any():
            continue
        pts = pts[ok]
        rng_m = np.linalg.norm(pts - origin, axis=1)
        pts[:, 2] += rng.normal(0.0, noise_a + noise_b * rng_m**2)
        fuse_points(emap, pts, origin, **FUSION)
    return emap
