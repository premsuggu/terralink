"""Unit tests for emap.terrain_analysis (rule-based terrain analysis v2). Pure NumPy, no ROS/Gazebo.

Each test encodes one weakness of the old classifier (emap.traversability) or one property the new one must keep; the
comment on each says which. Resolution 0.1 m everywhere, as in the real map.
"""
import numpy as np
import pytest

from emap.terrain_analysis import IncrementalTerrain, TerrainParams, analyze_terrain, plane_fit, robust_step
from emap.traversability import DIFFICULT, EASY, LETHAL

RES = 0.1
N = 60


def _flat(noise=0.0, seed=0, var=0.002, n=N):
    rng = np.random.default_rng(seed)
    e = rng.normal(0.0, noise, (n, n)) if noise else np.zeros((n, n))
    return e, np.full((n, n), var), np.ones((n, n), bool)


def _run(e, v, m, **kw):
    return analyze_terrain(e, v, m, RES, TerrainParams(**kw))


def test_flat_ground_with_high_variance_is_easy():
    """W-1: variance is measurement uncertainty, not roughness. A flat cell seen once from 6 m (variance 0.36) must NOT be
    lethal - the old rule (variance > 0.05) called all of it lethal."""
    e, v, m = _flat(noise=0.005, var=0.36)
    r = _run(e, v, m)
    assert np.all(r.tier[3:-3, 3:-3] == EASY)
    assert r.confidence[10:-10, 10:-10].max() < 0.3          # ... but the low certainty is reported, in its own layer


def test_confidence_rises_with_certainty_and_is_zero_where_unobserved():
    e, _, m = _flat()
    m[:, :20] = False
    lo = _run(e, np.full(e.shape, 0.5), m).confidence
    hi = _run(e, np.full(e.shape, 0.005), m).confidence
    assert np.all(hi[:, 30:50] > lo[:, 30:50]) and hi[30, 40] > 0.9
    assert np.all(hi[:, :20] == 0.0)                           # unobserved: no confidence
    assert hi[30, 21] < hi[30, 40]                             # near the unobserved region: half the window is empty -> lower


@pytest.mark.parametrize("slope,tier", [(0.10, EASY), (0.25, DIFFICULT), (0.30, DIFFICULT), (0.45, LETHAL), (0.80, LETHAL)])
def test_ramp_slope_is_measured_and_classified(slope, tier):
    e, v, m = _flat()
    e = e + slope * RES * np.arange(N)[None, :]                # rises along +x
    r = _run(e, v, m)
    assert r.slope[30, 30] == pytest.approx(slope, abs=0.005)
    assert r.tier[30, 30] == pytest.approx(tier)               # tier is float32: compare scalars with approx


def test_slope_survives_height_noise():
    """W-?: np.gradient amplifies 1 cm noise to ~0.07 slope per cell (bigger than the DIFFICULT band); the plane fit does not."""
    e, v, m = _flat(noise=0.01, seed=3)
    e = e + 0.15 * RES * np.arange(N)[None, :]
    r = _run(e, v, m)
    core = r.slope[5:-5, 5:-5]
    assert abs(core.mean() - 0.15) < 0.01 and core.std() < 0.03


def test_a_real_step_is_lethal_in_a_band_no_wider_than_the_old_rule():
    e, v, m = _flat()
    e[:, 30:] = 0.4
    r = _run(e, v, m)
    lethal_cols = np.flatnonzero((r.tier[30] == LETHAL))
    assert set(lethal_cols) == {29, 30}                         # the two cells that touch the edge (old rule: the same two)
    assert np.all(r.tier[30, :27] == EASY) and np.all(r.tier[30, 33:] == EASY)


def test_a_drivable_curb_is_difficult_not_lethal():
    e, v, m = _flat()
    e[:, 30:] = 0.08                                            # 8 cm: the Jackal crosses it (step 10 drive test)
    r = _run(e, v, m)
    assert r.tier[30, 29] == pytest.approx(DIFFICULT) and r.tier[30, 30] == pytest.approx(DIFFICULT) and not np.any(r.tier == LETHAL)


def test_a_wall_does_not_widen_the_lethal_band_on_the_floor():
    """The plane fit must not straddle a wall (surface gate): a floor cell 2 cells from a 2 m wall is plain floor."""
    e, v, m = _flat()
    e[:, 30:33] = 2.0
    r = _run(e, v, m)
    assert set(np.flatnonzero(r.tier[30] == LETHAL)) == {29, 30, 32, 33}   # the cells touching a wall face only
    assert r.tier[30, 27] == EASY and r.tier[30, 36] == EASY


def test_one_noisy_cell_is_not_a_ledge_but_a_real_post_is():
    e, v, m = _flat()
    e[30, 30] = 0.08                                            # a single 8 cm spike: noise
    e[10, 10] = 0.5                                             # a single 50 cm post: real
    r = _run(e, v, m)
    assert r.tier[30, 30] != LETHAL and np.all(r.tier[29:32, 29:32] != LETHAL)
    assert np.all(r.tier[9:12, 9:12] == LETHAL)


def test_rubble_is_rough_and_smooth_ground_is_not():
    e, v, m = _flat(noise=0.002)
    rng = np.random.default_rng(5)
    e[:, 30:] += rng.uniform(-0.09, 0.09, (N, N - 30))          # +-9 cm bumps (W3's rubble field)
    r = _run(e, v, m)
    assert np.all(r.tier[10:-10, 5:20] == EASY)
    assert np.mean(r.tier[10:-10, 36:-6] != EASY) > 0.9         # nearly all rubble cells are DIFFICULT or LETHAL
    assert r.roughness[30, 45] > 0.03 > r.roughness[30, 10]


def test_unobserved_cells_are_not_cliffs():
    """The old classifier read a placeholder 0.0 next to an observed 1.5 m deck as a 1.5 m cliff. Here an observed plateau only
    sees observed cells; its interior is EASY and unobserved cells keep the fail-safe EASY + confidence 0."""
    e, v, m = _flat()
    e[:] = 0.0
    m[:] = False
    m[10:50, 10:50] = True
    e[10:50, 10:50] = 1.5
    r = _run(e, v, m)
    assert np.all(r.tier[15:45, 15:45] == EASY)
    assert np.all(r.tier[~m] == EASY) and np.all(r.confidence[~m] == 0.0)


def test_an_isolated_observed_fragment_is_not_assumed_flat():
    e, v, m = _flat()
    m[:] = False
    m[30, 30] = m[30, 31] = m[31, 30] = True                    # 3 observed cells, nothing around them
    r = _run(e, v, m)
    assert np.all(r.tier[m] == LETHAL) and not r.known[m].any()


def test_hazard_is_continuous_and_ordered():
    e, v, m = _flat()
    e = e + 0.25 * RES * np.arange(N)[None, :]
    r = _run(e, v, m)
    assert r.hazard[30, 30] == pytest.approx(0.25 / 0.35, abs=0.02)


def test_plane_fit_and_robust_step_helpers():
    e, _, m = _flat()
    e = e + 0.2 * RES * np.arange(N)[None, :]
    slope, rough, cnt = plane_fit(e, m, RES, 0.5, 8, 0.3)
    assert slope[30, 30] == pytest.approx(0.2, abs=1e-6) and rough[30, 30] == pytest.approx(0.0, abs=1e-6) and cnt[30, 30] == 25
    assert robust_step(e, m, 3, 0.3)[30, 30] == pytest.approx(0.04, abs=1e-6)   # 2nd-highest minus 2nd-lowest of a 0.2 slope window


def test_incremental_update_equals_a_full_recompute():
    rng = np.random.default_rng(1)
    n = 90
    e, v, m = np.zeros((n, n)), np.full((n, n), 10.0), np.zeros((n, n), bool)
    inc = IncrementalTerrain(RES)
    for k in range(8):
        r, c = rng.integers(0, n - 30, 2)
        h, w = rng.integers(5, 30, 2)
        e[r:r + h, c:c + w] = rng.normal(0, 0.01, (h, w)) + (0.4 * (np.arange(w) > w // 2) if k % 3 == 0 else 0)
        v[r:r + h, c:c + w], m[r:r + h, c:c + w] = 0.1, True
        a, f = inc.update(e, v, m), analyze_terrain(e, v, m, RES)
        for name in ("slope", "step", "roughness", "confidence", "hazard", "tier", "known"):
            np.testing.assert_allclose(getattr(a, name), getattr(f, name), atol=1e-6, err_msg=f"{name} after patch {k}")


def test_incremental_recomputes_everything_when_the_shape_changes():
    inc = IncrementalTerrain(RES)
    e, v, m = _flat(n=40)
    inc.update(e, v, m)
    e2, v2, m2 = _flat(n=50)
    assert inc.update(e2, v2, m2).tier.shape == (50, 50)


@pytest.mark.parametrize("step_m,tier", [(0.06, DIFFICULT), (0.085, DIFFICULT), (0.10, LETHAL), (0.14, LETHAL)])
def test_step_limit_is_the_measured_wheel_limit(step_m, tier):
    """X-A1 (Gazebo, Jackal model): a 9 cm step is crossed, a 10 cm step blocks the robot (wheel radius 0.098 m). The old limit
    (0.15) called a 10-14 cm ledge passable; the new default must call it lethal and still allow the 8 cm curb."""
    e, v, m = _flat()
    e[:, 30:] = step_m
    r = _run(e, v, m)
    assert r.tier[30, 29] == pytest.approx(tier) and r.tier[30, 30] == pytest.approx(tier)
    assert TerrainParams().max_step == pytest.approx(0.09)


# ------------------------------------------------------------------------------------------- noise compensation (step 11b)
from emap.terrain_analysis import estimate_noise_sigma


def test_noisy_flat_ground_is_easy_once_the_noise_floor_is_known():
    """sigma = 3 cm of height noise makes the plane fit read ~5 cm slope and the 3x3 step ~5.5 cm: half the limits, i.e. DIFFICULT.
    With the noise floor subtracted in the soft zone those cells are EASY again."""
    e, v, m = _flat(noise=0.03, seed=7)
    raw = _run(e, v, m)
    sg = estimate_noise_sigma(raw)
    assert sg == pytest.approx(0.03, rel=0.25)
    comp = _run(e, v, m, noise_sigma=sg)
    inner = (slice(5, -5), slice(5, -5))
    assert np.mean(raw.tier[inner] != EASY) > 0.5 and np.mean(comp.tier[inner] != EASY) < 0.05


def test_noise_compensation_never_hides_a_lethal_step():
    e, v, m = _flat(noise=0.03, seed=7)
    e[:, 30:] += 0.14                                          # a 14 cm ledge: the vehicle cannot climb it
    comp = _run(e, v, m, noise_sigma=0.03)
    assert np.all(comp.tier[10:-10, 29:31] == LETHAL)


def test_noise_estimate_is_capped_and_ignores_a_few_rough_cells():
    e, v, m = _flat(noise=0.004, seed=2)
    e[20:25, 20:25] += np.random.default_rng(0).uniform(-0.1, 0.1, (5, 5))          # a patch of rubble
    assert estimate_noise_sigma(_run(e, v, m)) == pytest.approx(0.004, rel=0.3)
    rough, v2, m2 = _flat(noise=0.2, seed=2)
    assert estimate_noise_sigma(_run(rough, v2, m2)) <= 0.05                       # a really rough map is not "compensated" away


def test_incremental_auto_noise_recalibrates_and_equals_a_full_recompute():
    e, v, m = _flat(noise=0.03, seed=4)
    inc = IncrementalTerrain(RES, TerrainParams(), auto_noise=True)
    a = inc.update(e, v, m)
    assert inc.params.noise_sigma == pytest.approx(0.03, rel=0.3)
    e2 = e.copy(); e2[10:20, 10:20] += 0.2                                             # a patch changes afterwards
    a2 = inc.update(e2, v, m)
    f = analyze_terrain(e2, v, m, RES, inc.params)
    for name in ("slope", "step", "roughness", "hazard", "tier"):
        np.testing.assert_allclose(getattr(a2, name), getattr(f, name), atol=1e-6, err_msg=name)
