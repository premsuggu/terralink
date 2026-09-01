# Step 11: Phantom High-Altitude Terrain + Runaway-Climb Fixes

**Package**: `src/emap/`
**Goal**: Fix a real bug the user found live: flying the UAV above a certain altitude corrupted the elevation map into a tall, obviously-wrong "tower" of terrain (screenshots showed a correct Gaussian bump at low altitude, then a spike reaching up toward the drone after climbing higher).
**Status**: ✅ Complete and verified live - including catching my own INITIAL diagnosis being wrong, and correcting it with more evidence before shipping a fix.
**Read first**: this doc stands alone; it's a bugfix, not a new mapping concept.

---

## 1. The report and the first (wrong) theory

The user published one `ros2 topic pub --once /cmd_vel ... {z: 0.6}` and watched the map. It correctly showed the bump, then - after the UAV kept climbing - showed a tall purple spike growing up from the ground toward the drone.

Investigating live, the UAV was found to be climbing indefinitely (Ignition's `MulticopterVelocityControl` plugin has no command timeout - it holds the last `/cmd_vel` forever). At ~30m altitude, raw `/camera/points` showed 76,800 pixels total, only 52 finite (correct - most of the ground is beyond the camera's 20m `<far>` clip, so `+inf`, already filtered), and those 52 survivors all read a camera-frame depth of ~19.94m - suspiciously close to the 20.0m far clip. **First theory**: a depth-buffer clamp-to-far-clip artifact. A `max_valid_range` filter was added to `fuse_points`/`fuse_points_gpu` (reject points within 0.2m of the configured far clip) to catch it.

## 2. Re-testing exposed the first theory was incomplete

Verifying that fix at a DIFFERENT altitude (~28m instead of ~30m), the corruption was still there - but the raw depth reading had changed to ~17.96m, comfortably under the new `max_valid_range=19.8` filter, which therefore didn't catch it. If this were really "clamped to the far clip," the value should have stayed ~20m regardless of altitude. It didn't, so the far-clip theory was wrong (or at least incomplete).

Comparing both readings against their respective altitudes was the actual tell: `30.02 - 19.94 = 10.08` and `28.03 - 17.96 = 10.07` - nearly identical. The phantom points weren't at a fixed *range* from the sensor; they were at a fixed **world height, ~10m**, regardless of the drone's altitude. That pointed at something IN THE SCENE at world z=10 - and `worlds/bump_test.world`'s directional light (`<light type="directional" name="sun">`) was posed at exactly `0 0 10 0 0 0`.

## 3. The real root cause

Under this environment's software-rendered (llvmpipe, via `LIBGL_ALWAYS_SOFTWARE=1` - see step01's docs for why that's forced here) Ogre2 depth camera, the light's pose was occasionally misread as physical, depth-occluding geometry - producing a phantom "hit" at that exact world position. Since it sat at z=10, well inside the elevation map's operational volume, those phantom points got fused in as real terrain: a fake ~10m-tall column exactly where the light was, growing as the UAV (still climbing, per Section 4) kept re-observing it from increasing altitude.

**The fix**: a directional light's illumination in SDF depends ONLY on `<direction>`, never `<pose>` - it models parallel rays from infinitely far away, so the light's position is otherwise meaningless. Moving it (`worlds/bump_test.world` and `worlds/uav_test.world`, both had the identical light block) to `0 0 500 0 0 0` changes nothing about how the scene looks or is lit, but means if this render glitch ever recurs, the resulting phantom point lands far outside the map's finite extent (40m) and gets silently, correctly dropped by `fuse_points`' already-existing, already-tested `in_bounds` check - no new mapping-code logic needed for what was really a simulator rendering quirk, not a mapping bug.

`max_valid_range` (Section 1) was kept anyway - real depth-camera far-clip clamp artifacts are a documented, plausible failure mode in their own right (this project's own step 2 already found the mirror case at the *near* clip), even though it turned out not to be what actually happened here. It's cheap, well-justified, symmetric with the existing `min_valid_distance` filter, and does not reject genuine near-far-clip readings (see its test: a point at 19.5m range is still fused; only points within 0.2m of the configured 20.0m far clip are rejected).

## 4. The other real bug: the UAV never stops climbing

Independent contributing cause: Ignition's `MulticopterVelocityControl` plugin holds the LAST `/cmd_vel` message forever - a single `ros2 topic pub --once` (or any short-lived publisher) makes the UAV move indefinitely, not just for a moment. This is what let the drone reach an altitude (30m+) far outside anything this project was ever meant to test at, which is what exposed the light-pose bug above in the first place.

**Fix**: `emap/cmd_vel_watchdog.py`, a new node - subscribes to `/cmd_vel`, and if no new command arrives within `timeout_sec` (default 1.0s, launch arg `cmd_vel_timeout`), publishes one all-zero `Twist` to stop the vehicle. Enabled by default in `uav_sim.launch.py` (launch arg `cmd_vel_watchdog`, default `true`).

**A real bug found in the watchdog itself, live**: because it both publishes AND subscribes to `/cmd_vel` (the stop has to go out on the same topic the bridge listens to), its own zero-Twist stop message looped back into its own subscription callback and was treated as "fresh operator input" - resetting the timer, which then fired again one timeout later, forever (observed live: the warning kept re-firing every ~6.4s indefinitely with nothing actually driving the UAV). **Fixed** by only treating a message as "the operator is still actively commanding" if it has some nonzero component - an all-zero Twist (ours or a legitimate operator stop) doesn't need to re-arm anything, since the vehicle is already at rest either way.

## 5. Verification

- `tests/emap/test_fusion.py`/`test_fusion_gpu.py`: `max_valid_range` rejects a point at the observed clamp-artifact range (19.94m) while still fusing a point that's genuinely farther out but clearly below the boundary (19.5m) - the exact "don't reject a real 19.5 while missing something worse" requirement. All 44 tests (including the pre-existing 41) pass.
- **Live, the exact reported scenario**: sent one `ros2 topic pub --once /cmd_vel {z: 1.5}` and did nothing else. With the watchdog: the UAV climbed briefly, the watchdog fired exactly once (`no /cmd_vel for 1.5s ... publishing zero velocity`), and the UAV held steady at ~2.4m afterward - confirmed by re-reading `/odom` every 2s for 12s with no further increase. Confirmed the watchdog's own re-trigger bug was gone too (no repeated warnings).
- **Live, deliberately past both old failure thresholds**: climbed to ~32m (past the 20m far clip AND the old light-artifact zone) and let the watchdog stop it there. Raw `/camera/points`: 0 finite points (everything correctly beyond sensor range now that the light isn't sitting inside it). The elevation map: 0 cells above 3.0m anywhere, peak still located exactly at the true bump's cell.

## Follow-ups for later steps

- This session's low-altitude (~1.7m) near-clip-plane garbage (documented separately, step02/step10) is a different, still-open, lower-priority issue - not touched by this fix.
- If a future world file ever needs a light closer than z=500 for a specific visual reason, re-check for this same class of artifact before assuming it's safe.
