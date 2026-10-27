"""Rule-based terrain analysis, v2: slope / step / roughness / confidence / continuous cost from the 2.5D map.

Replaces the three weak spots of `traversability.compute_traversability` (kept unchanged for the 3-tier API and its tests):

  1. ROUGHNESS WAS THE VARIANCE LAYER. Variance is how unsure the fusion is about a cell's height (0.01 * range^2 per
     observation), not how rough the ground is: a flat cell seen once from 6 m scored 0.36 and was lethal. Here roughness is the
     residual of the elevation around a best-fit plane in a 0.5 m window - a property of the TERRAIN. The variance now only
     feeds a separate CONFIDENCE layer.
  2. UNOBSERVED CELLS WERE REAL CELLS. They hold a placeholder height (0.0), so the border of an observed area at z = 1.5 m
     (a deck, a mound) read as a 1.5 m cliff. Here only observed cells take part in any window; a cell with too few observed
     neighbours gets low confidence instead of a made-up number.
  3. NOISE WAS TRUSTED. Central-difference slope amplifies height noise by ~7x/m, and max-min over 9 cells by ~3x. Here slope is
     a least-squares PLANE over 5 x 5 cells (noise gain ~1.4x/m) and step height is a robust spread (2nd-highest minus
     2nd-lowest of the 3 x 3 window) so one noisy cell cannot make a ledge. A real spike is NOT hidden: if the raw
     max-min exceeds `spike_gate` the raw value is used (a 30 cm post is never noise).

The output is a continuous HAZARD in [0, 1+) per cell (1 = at the limit of what the vehicle can drive; see `max_*` and the
vehicle experiments in docs/work-docs/nav/step11_terrain_classification.md), a CONFIDENCE in [0, 1], and the old
{LETHAL, DIFFICULT, EASY} tiers as a derived view so every consumer of the `traversability` layer keeps working.

Pure NumPy/SciPy, no ROS: runs in the unit tests and in the offline evaluation (scripts/eval_terrain.py).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import correlate

from emap.traversability import DIFFICULT, EASY, LETHAL


@dataclass(frozen=True)
class TerrainParams:
    """Vehicle limits and analysis scales. The step limit is MEASURED (experiment X-A1, step 11 doc): the simulated Jackal crosses
    a 9 cm step and is blocked by 10 cm = its wheel radius (0.098 m). The slope limit is a policy margin: the same robot
    climbed every ramp up to 35 deg (slope 0.70) at friction 1.0, which real wet ground will not give."""
    max_slope: float = 0.35          # rise/run at which the vehicle should not drive (~19 deg; measured capability >= 0.70)
    max_step: float = 0.09           # m, height jump over 0.3 m above which the vehicle cannot cross (measured: 9 cm ok, 10 cm blocked)
    max_roughness: float = 0.06      # m, residual std around the local plane (0.5 m window); not yet measured on the vehicle
    difficult_fraction: float = 0.5  # hazard above this fraction of a limit = DIFFICULT (same as the old "half the limit")
    plane_window_m: float = 0.5      # window of the plane fit (slope + roughness)
    step_window_cells: int = 3       # window of the step test (0.3 m at 0.1 m cells)
    spike_gate: float = 0.30         # m; raw max-min above this is never treated as noise
    surface_gate: float = 0.30       # m; plane-fit cells further than this from the centre height are another surface
    min_plane_cells: int = 8         # observed cells needed in the plane window, else slope/roughness are unknown
    var_ref: float = 0.1             # m^2; cell variance at which the measurement confidence is 0.5


@dataclass
class TerrainAnalysis:
    slope: np.ndarray        # rise/run from the plane fit (0 where unknown)
    step: np.ndarray         # m, robust step height (0 where unknown)
    roughness: np.ndarray    # m, residual std around the plane (0 where unknown)
    confidence: np.ndarray   # 0..1; 0 for unobserved cells
    hazard: np.ndarray       # 0..: max(slope/max_slope, step/max_step, roughness/max_roughness)
    tier: np.ndarray         # {LETHAL, DIFFICULT, EASY} (float32), the old 3-tier view
    known: np.ndarray        # bool: observed AND enough neighbours for the plane fit


def _kernels(k: int, res: float):
    h = k // 2
    d = (np.arange(k) - h) * res
    dx = np.tile(d, (k, 1))                      # column offset -> world x
    dy = dx.T                                    # row offset -> world y
    return dx, dy


def plane_fit(elevation: np.ndarray, observed: np.ndarray, res: float, window_m: float, min_cells: int, surface_gate: float):
    """Least-squares plane z = a + b*x + c*y over the cells of a (k x k) window around every cell that are OBSERVED and on the
    SAME SURFACE as the centre cell (|z - z_centre| <= `surface_gate`).

    The gate matters: without it a window that straddles a wall or a ledge fits one plane through two surfaces, and every cell
    within k/2 of the edge gets a huge "roughness" - a lethal band twice as wide as the edge test makes it (measured on W2:
    16 % lethal instead of 9 %). With it, a cell next to a wall sees only its own floor; the wall itself is the STEP test's job.

    Returns (slope, roughness, count): `roughness` = sqrt(residual sum of squares / (n - 3)); slope/roughness are NaN where fewer
    than `min_cells` cells qualify or the window is degenerate (all cells on one line).
    """
    k = max(3, int(round(window_m / res)) | 1)             # odd window size in cells
    h = k // 2
    ny, nx = elevation.shape
    z0 = float(np.mean(elevation[observed])) if observed.any() else 0.0
    z = np.where(observed, elevation - z0, np.nan)         # centred heights keep the normal equations well conditioned
    zp = np.pad(z, h, constant_values=np.nan)
    names = ("s0", "sx", "sy", "sxx", "sxy", "syy", "sz", "sxz", "syz", "szz")
    S = {n: np.zeros((ny, nx)) for n in names}
    for i in range(k):
        for j in range(k):
            zs = zp[i:i + ny, j:j + nx]                    # neighbour at row offset i-h, col offset j-h
            m = np.isfinite(zs) & np.isfinite(z) & (np.abs(zs - z) <= surface_gate)
            dxv, dyv = (j - h) * res, (i - h) * res
            zz = np.where(m, zs, 0.0)
            S["s0"] += m
            S["sx"] += m * dxv
            S["sy"] += m * dyv
            S["sxx"] += m * dxv * dxv
            S["sxy"] += m * dxv * dyv
            S["syy"] += m * dyv * dyv
            S["sz"] += zz
            S["sxz"] += zz * dxv
            S["syz"] += zz * dyv
            S["szz"] += zz * zz
    n = ny * nx
    f = {k_: v.ravel() for k_, v in S.items()}
    A = np.empty((n, 3, 3))
    A[:, 0, 0], A[:, 0, 1], A[:, 0, 2] = f["s0"], f["sx"], f["sy"]
    A[:, 1, 0], A[:, 1, 1], A[:, 1, 2] = f["sx"], f["sxx"], f["sxy"]
    A[:, 2, 0], A[:, 2, 1], A[:, 2, 2] = f["sy"], f["sxy"], f["syy"]
    rhs = np.stack([f["sz"], f["sxz"], f["syz"]], axis=1)
    det = np.linalg.det(A)
    ok = (f["s0"] >= min_cells) & (np.abs(det) > 1e-9 * (res ** 4) * np.maximum(f["s0"], 1.0) ** 3)
    sol = np.full((n, 3), np.nan)
    if ok.any():
        sol[ok] = np.linalg.solve(A[ok], rhs[ok][..., None])[..., 0]
    a_, b_, c_ = sol[:, 0], sol[:, 1], sol[:, 2]
    rss = f["szz"] - (a_ * f["sz"] + b_ * f["sxz"] + c_ * f["syz"])
    rough = np.sqrt(np.maximum(rss, 0.0) / np.maximum(f["s0"] - 3.0, 1.0))
    return np.hypot(b_, c_).reshape(ny, nx), rough.reshape(ny, nx), S["s0"]


def robust_step(elevation: np.ndarray, observed: np.ndarray, cells: int, spike_gate: float):
    """Height spread of the observed cells in a (cells x cells) window: 2nd-highest minus 2nd-lowest (robust to one noisy cell),
    replaced by the raw max-min whenever that exceeds `spike_gate` (a real spike is never noise). 0 with < 3 observed cells."""
    h = cells // 2
    ny, nx = elevation.shape
    pad = np.full((ny + 2 * h, nx + 2 * h), np.nan)
    pad[h:h + ny, h:h + nx] = np.where(observed, elevation, np.nan)
    stack = np.stack([pad[i:i + ny, j:j + nx] for i in range(cells) for j in range(cells)], axis=0)
    s = np.sort(stack, axis=0)                              # NaN sorts last
    cnt = np.sum(~np.isnan(stack), axis=0)
    idx = lambda f: np.clip(np.round(f * (cnt - 1)).astype(int), 0, cells * cells - 1)
    take = lambda i: np.take_along_axis(s, i[None], axis=0)[0]
    lo_r, hi_r = take(idx(np.full(cnt.shape, 0.125))), take(idx(np.full(cnt.shape, 0.875)))
    raw = take(np.maximum(cnt - 1, 0)) - s[0]
    robust = hi_r - lo_r
    step = np.where(raw > spike_gate, raw, robust)
    return np.where(cnt >= 3, np.nan_to_num(step), 0.0)


def analyze_terrain(elevation: np.ndarray, variance: np.ndarray, is_valid: np.ndarray, resolution: float,
                    params: TerrainParams = TerrainParams()) -> TerrainAnalysis:
    """Slope, step, roughness, confidence, hazard and the 3-tier view for every cell. Unobserved cells: confidence 0, tier EASY
    (the old fail-safe contract - the CALLER masks by `is_valid`, see walkability.compute_walkable_mask)."""
    p = params
    elevation = np.asarray(elevation, dtype=np.float64)
    observed = np.asarray(is_valid).astype(bool)
    slope, rough, cnt = plane_fit(elevation, observed, resolution, p.plane_window_m, p.min_plane_cells, p.surface_gate)
    known = observed & np.isfinite(slope)
    slope = np.where(known, slope, 0.0)
    rough = np.where(known, rough, 0.0)
    step = np.where(observed, robust_step(elevation, observed, p.step_window_cells, p.spike_gate), 0.0)

    k = max(3, int(round(p.plane_window_m / resolution)) | 1)
    coverage = np.clip(cnt / float(k * k), 0.0, 1.0)
    meas = 1.0 / (1.0 + np.maximum(np.asarray(variance, dtype=np.float64), 0.0) / p.var_ref)
    confidence = np.where(observed, coverage * meas, 0.0)

    hazard = np.maximum.reduce([slope / p.max_slope, step / p.max_step, rough / p.max_roughness])
    # An observed cell without enough same-surface neighbours (an isolated wall-top fragment, the rim of a hole) has no slope or
    # roughness evidence: it is NOT assumed flat. The old classifier got this right by accident (the unobserved neighbour's
    # placeholder height 0.0 made a cliff); now it is explicit: hazard 1.0 = at the limit, i.e. LETHAL, until more is seen.
    hazard = np.where(observed & ~known, np.maximum(hazard, 1.0 + 1e-6), hazard)
    hazard = np.where(observed, hazard, 0.0)
    tier = np.full(elevation.shape, EASY, dtype=np.float32)
    tier[hazard > p.difficult_fraction] = DIFFICULT
    tier[hazard > 1.0] = LETHAL
    tier[~observed] = EASY
    return TerrainAnalysis(slope, step, rough, confidence, hazard, tier, known)


class IncrementalTerrain:
    """Keeps the analysis of a growing map up to date by recomputing only what changed.

    A full `analyze_terrain` of a 400 x 400 map costs ~0.2 s; the live node fuses a cloud ~10 times a second, and one cloud
    changes a patch of ~100 x 100 cells. Every output cell depends only on the cells within `reach` (the 5 x 5 plane window), so
    after each update only the bounding box of the changed cells grown by `reach` is recomputed (from an input block grown by
    another `reach`, so that every window is complete). The result is IDENTICAL to a full recompute (tested): same arrays, same
    values, up to float rounding of the plane-fit centring.

    A change of map shape - or a first call - recomputes everything. A rolling map that re-centres every callback changes almost
    every cell each time, so it falls back to a full recompute by itself.
    """

    def __init__(self, resolution: float, params: TerrainParams = TerrainParams()):
        self.res = resolution
        self.params = params
        self.reach = max(int(round(params.plane_window_m / resolution)) | 1, 3) // 2 + 0
        self.reach = max(self.reach, params.step_window_cells // 2)
        self._prev: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self.result: TerrainAnalysis | None = None

    def update(self, elevation: np.ndarray, variance: np.ndarray, is_valid: np.ndarray) -> TerrainAnalysis:
        e = np.asarray(elevation, dtype=np.float64)
        v = np.asarray(variance, dtype=np.float64)
        m = np.asarray(is_valid).astype(bool)
        if self._prev is None or self._prev[0].shape != e.shape or self.result is None:
            self.result = analyze_terrain(e, v, m, self.res, self.params)
        else:
            pe, pv, pm = self._prev
            changed = (pe != e) | (pv != v) | (pm != m)
            if changed.any():
                rows = np.flatnonzero(changed.any(axis=1))
                cols = np.flatnonzero(changed.any(axis=0))
                ny, nx = e.shape
                R = self.reach
                o0, o1 = max(rows[0] - R, 0), min(rows[-1] + R + 1, ny)          # output block (rows)
                c0, c1 = max(cols[0] - R, 0), min(cols[-1] + R + 1, nx)          # output block (cols)
                i0, i1 = max(o0 - R, 0), min(o1 + R, ny)                           # input block, one window wider
                d0, d1 = max(c0 - R, 0), min(c1 + R, nx)
                sub = analyze_terrain(e[i0:i1, d0:d1], v[i0:i1, d0:d1], m[i0:i1, d0:d1], self.res, self.params)
                a, b = o0 - i0, o1 - i0
                c, d = c0 - d0, c1 - d0
                for name in ("slope", "step", "roughness", "confidence", "hazard", "tier", "known"):
                    getattr(self.result, name)[o0:o1, c0:c1] = getattr(sub, name)[a:b, c:d]
        self._prev = (e.copy(), v.copy(), m.copy())
        return self.result
