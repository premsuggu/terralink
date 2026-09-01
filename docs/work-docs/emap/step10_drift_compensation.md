# Step 10: Vertical (Z) Drift Compensation

**Package**: `src/emap/`
**Goal**: Correct accumulated vertical pose-drift error using low-variance ("confident") cell statistics - scoped to Z only (see `00_concepts.md` Section 15 for why), wired into the live node with an injectable synthetic drift so the correction is actually observable, since Gazebo's own TF never drifts on its own.
**Status**: ✅ Complete and verified - including two real bugs found and fixed during live verification (see Section 2), a live 45-second stability test, and a live before/after comparison showing the correction roughly halving accumulated error against a continuous synthetic drift ramp.
**Read first**: [`00_concepts.md`](00_concepts.md) Section 15 (Pose drift and how to correct it).

---

## 1. The design

`emap/drift.py`'s `estimate_vertical_drift(emap, points_xyz, min_confidence_variance, min_matches, max_reasonable_residual)`: match incoming points against cells that are both `is_valid` and confident (`variance < min_confidence_variance`), take the **median** of `measured_z - cell_elevation` across those matches (median, not mean, for the same outlier-robustness reason `fuse_points`' own outlier logic exists), and return `None` if there aren't at least `min_matches` confident matches to trust.

`elevation_mapping_node.py` keeps a running `self._z_bias_estimate` (starts at 0). Each callback: **apply** the current estimate to this batch's points first, **then** measure the residual against the (already-corrected) data, then fold `drift_correction_gain * residual` into the running estimate for next time. `tests/emap/test_drift.py::test_damped_gain_converges_toward_true_offset_without_overshoot` proves this recurrence converges to a fixed **step** offset without overshoot.

## 2. Two real bugs found and fixed during live verification

Both were caught by actually running the live sim and watching the numbers - not by inspection.

**Bug 1 - wrong measurement order caused exponential runaway.** The first version measured the residual against the RAW (not-yet-corrected) pose, then applied the bias correction afterward. Live, this made the global map's elevation reading run away from a correct ~1.5m to **~660m within about 15 seconds** of hovering with drift compensation on (synthetic drift rate = 0, i.e. no injected drift at all - this shouldn't have moved at all). Traced by hand: measuring against the raw pose compares fresh data to a map that already reflects *previous* corrections, so each callback's "error" is contaminated by the correction itself, not by anything real - `bias_k = bias_{k-1} * (1 + gain)`, textbook positive feedback, confirmed by re-deriving the recurrence and matching the observed ~15s time-to-blowup at gain 0.3. **Fix**: apply the running estimate to the batch first, then measure the residual from that already-corrected data (Section 1) - this makes a fully-converged estimate leave ~zero residual (a stable fixed point), not a moving target.

**Bug 2 - no defense against a single bad sensor frame.** After fixing Bug 1, an offline reproduction of the exact node logic (1000s of realistic synthetic point-cloud callbacks, `scratchpad/repro_drift_bug.py`) showed the corrected control loop itself was completely stable (`z_bias_estimate` stayed within ~0.002m the whole run). But live, with drift compensation on and **zero** synthetic drift, the map's center cell still occasionally spiked to ~10-11m before dropping back to the correct ~1.5m. Checking the raw `/camera/points` at the altitude where this happened (1.69m) showed every point at a depth of ~0.05-0.11m - a real, already-documented category of depth-camera artifact for this project (near-clip-plane / self-body-reflection glitches, same family as the `+inf`-at-close-range issue found in step 2). A single bad frame like that, if it happens to land ≥`drift_min_matches` points on already-confident cells, produces a genuinely large (and genuinely *measured*, not buggy) residual - the estimator was working correctly on bad input. **Fix**: `max_reasonable_residual` (config: `drift_max_reasonable_residual: 1.0`) - a residual larger than this in one callback is rejected as an untrustworthy sensor glitch rather than applied, on the same "too big to trust" principle `fuse_points`' own outlier rejection already uses. Real pose drift accumulates slowly; it doesn't jump by meters in a single frame.

Both fixes are covered by unit tests (`test_damped_gain_converges_toward_true_offset_without_overshoot` for Bug 1's ordering, `test_implausibly_large_residual_is_rejected_as_a_sensor_glitch` for Bug 2's safeguard).

## 3. Verification

- `tests/emap/test_drift.py`: a synthetic scenario (since live Gazebo TF is ground truth and never drifts on its own) - fuse the same true height into a block of cells repeatedly until confident, then feed a new batch shifted by a known Z offset and assert the estimate recovers it (both positive and negative signs); too few confident matches → `None`; a minority of outlier points doesn't skew the median; an implausibly large residual is rejected; the damped-gain update converges to a step offset without overshoot. All pass, part of the full 41-test suite.
- **Live, 45 seconds, zero synthetic drift**: with both GPU fusion (step 9) and drift compensation on, the map's elevation at the true bump peak stayed at **exactly 1.5071m for the entire 45 seconds**, zero deviation - confirming both bugs above are actually fixed, not just patched around.
- **Live, 45 seconds, `synthetic_drift_z_rate=0.05` (m/s)** - a direct before/after comparison, same hover, same duration:

  | | Reading at t=45s | Error vs. true 1.5m |
  |---|---|---|
  | No correction (`enable_drift_compensation:=false`) | 3.87m | 2.37m |
  | With correction (`enable_drift_compensation:=true`) | 2.57m | 1.07m |

  Correction roughly **halved** the accumulated error over 45 seconds of continuous drift. It does not eliminate it entirely, which is an honest, expected property of this design, not a shortfall to hide: a **step** (one-time, fixed) offset is fully cancelled (proven by the unit test), but a **continuously growing ramp** - which is what `synthetic_drift_z_rate` simulates, and what slow real-world sensor bias often looks like - always leaves some steady-state lag behind a damped, integrator-style corrector chasing a moving target. A more aggressive `drift_correction_gain` would close more of that gap at the cost of reacting more sharply to noise; `0.3` was kept as a reasonably conservative default.
- Separately (not a step 10 bug): live testing also surfaced that this camera clips at very low altitude (~1.7m produced only near-clip-plane garbage, exactly the step 2-documented failure mode) - purely an altitude/testing-procedure note, unrelated to steps 9 or 10, worked around here by hovering above ~2m for verification.

## Follow-ups for later steps

- `drift_correction_gain` and `drift_max_reasonable_residual` are both plain config values, worth revisiting once real (non-Gazebo-ground-truth) odometry is ever used, where actual drift characteristics would inform better defaults than this session's synthetic-ramp testing could.
- Horizontal (X/Y) drift compensation remains out of scope (see `00_concepts.md` Section 15) - would need scan-matching/registration, a materially larger undertaking than this step's confident-cell comparison.
