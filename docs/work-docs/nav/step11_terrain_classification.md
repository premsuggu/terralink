# Step 11 - Rule-based terrain classification, v2 (2026-10-27)

What this step did: replaced the weak parts of the cell classifier with measured, noise-robust rules, built an offline scan
simulator so the classifier can be judged on what a real scan looks like, measured what the simulated Jackal can actually
climb, and wired the result into the live elevation node. **The user decided (2026-10-27) that the rule-based classifier is
the way to go and a segmentation model is the expensive choice, so the semantic phases of `cell_classification_plan.md` are
parked.** Phase A of that plan is what this document reports. The 3D refinement idea (classify again from the UGV's own 3D
data while it drives) is the next step, see section 8.

Files: [terrain_analysis.py](../../../src/emap/emap/terrain_analysis.py) (the rules),
[scan_sim.py](../../../src/emap/emap/scan_sim.py) (offline UAV-scan simulator),
[eval_terrain.py](../../../src/nav/scripts/eval_terrain.py), [eval_planning.py](../../../src/nav/scripts/eval_planning.py), [eval_refine.py](../../../src/nav/scripts/eval_refine.py) and [eval_ground.py](../../../src/nav/scripts/eval_ground.py) (experiments),
[elevation_mapping_node.py](../../../src/emap/emap/elevation_mapping_node.py) (integration),
tests: `tests/emap/test_terrain_analysis.py` (27), `tests/emap/test_scan_sim.py` (6), `tests/nav/test_limits_world.py` (9);
experiment X-A1: `tests/nav/integration/make_limits_world.py`, `ugv_limits.py`, `ugv_limits_test.world`.

## 1. The old rules, exactly (so the changes can be judged)
`emap/traversability.py: compute_traversability`, 0.1 m cells, one of three scores per observed cell:

| Number | How | Limit |
|---|---|---|
| slope | `sqrt(gx^2 + gy^2)` of the elevation, `np.gradient` (central difference, 0.2 m baseline) | lethal > 0.35, difficult > 0.175 |
| step | max - min of the elevation in the 3 x 3 window (0.3 m) | lethal > 0.15 m, difficult > 0.075 m |
| roughness | **the variance layer, copied** | lethal > 0.05 |

`LETHAL if slope > 0.35 or step > 0.15 or variance > 0.05; DIFFICULT elif slope > 0.175 or step > 0.075; else EASY`. Unobserved
cells are EASY and masked by `is_valid` by the caller (`nav/walkability.py`). The planner only uses `is_valid and tier != LETHAL`
plus `+difficult_weight` for DIFFICULT cells.

## 2. What was wrong, in numbers
Weaknesses W-1..W-7 are listed in `cell_classification_plan.md`. The offline scan simulator (section 4) turned the first one into a
measurement: **on a simulated real scan, the old default rule (`max_roughness` 0.05) calls 100 % of the observed cells lethal in all
five worlds.** The variance layer is `0.01 * range^2` per observation (0.16 for one look from 4 m), so no cell is ever "smooth" in
that sense unless it was seen dozens of times; the stress-world scans only worked because the launch file overrides
`max_roughness:=0.5`. Two more weaknesses were found while building v2:
* **Unobserved cells were real cells.** They hold the placeholder height 0.0; the border of an observed 1.5 m deck read as a
  1.5 m cliff (accidentally "right" for wall tops, wrong for decks and mounds).
* **Noise was amplified.** Central-difference slope turns 1 cm of height noise into ~0.07 slope per cell (the "difficult" band
  is 0.175), and max - min over 9 cells turns one noisy cell into a ledge.

## 3. The v2 rules (`terrain_analysis.analyze_terrain`)
Everything works on a window around each cell and only uses OBSERVED cells.

1. **Slope = least-squares plane over 5 x 5 cells (0.5 m)**, `z = a + b*x + c*y`, `slope = hypot(b, c)`. Noise gain ~1.4 per metre
   instead of ~7. Only cells on the **same surface as the centre cell** take part (`|z - z_centre| <= surface_gate` 0.30 m);
   without that gate a window straddling a wall fits one plane through two surfaces and the lethal band around every wall doubles
   (measured: W2 16 % lethal instead of 9 %).
2. **Roughness = residual std of that plane fit** (`sqrt(RSS / (n - 3))`): a property of the terrain, not of the sensor. Lethal
   at 0.06 m (not yet measured on the vehicle).
3. **Step = robust spread of the 3 x 3 window**: 2nd-highest minus 2nd-lowest observed cell, so one noisy cell cannot make a
   ledge. A real spike is never hidden: if the raw max - min exceeds `spike_gate` (0.30 m) the raw value is used (a 30 cm post is
   not noise). Lethal at 0.09 m, **measured** (section 5).
4. **Confidence** = `coverage * 1 / (1 + variance / 0.1)`; coverage is the fraction of the 5 x 5 window that is observed. 0 for an
   unobserved cell. It never makes a cell lethal; it is its own layer for the planner, the explorer and the 3D refinement.
5. **Hazard** (continuous) = `max(slope/0.35, step/0.09, roughness/0.06)`. 1.0 = at the vehicle's limit. The old 3 tiers are a
   derived view: `LETHAL` above 1.0, `DIFFICULT` above 0.5, else `EASY`, so every consumer of the `traversability` layer works unchanged.
6. **An observed cell without enough same-surface neighbours** (< 8 of 25: an isolated wall-top fragment, the rim of a hole) has no
   slope evidence and is **not assumed flat**: hazard just above 1 (lethal) until more is seen.

```python
from emap.terrain_analysis import analyze_terrain, TerrainParams
r = analyze_terrain(elevation, variance, is_valid, 0.1, TerrainParams())
r.tier        # {LETHAL 0.0, DIFFICULT 0.3, EASY 1.0}, the old layer
r.hazard      # continuous, 1.0 = limit
r.confidence  # 0..1
```

**Speed.** A full analysis of a 400 x 400 map takes 0.2 s (1200 x 1200: 1.9 s), the old rule 9 ms. The node fuses a cloud ten
times a second, so `IncrementalTerrain` recomputes only the bounding box of the cells that changed (plus a window): ~1-2 ms per
cloud, and a unit test proves the result equals a full recompute. The rolling local map re-centres every callback, so it is
re-analysed from the publish timer every 4th publish (0.5 Hz) instead of per cloud.

## 4. The offline scan simulator and the experiments
`emap/scan_sim.py` flies a lawn-mower survey over a TRUE height field with the live camera model (320 x 240, 60 deg, clip 20 m),
casts the rays, adds depth noise and fuses every frame with the REAL `fuse_points` and live parameters. Seconds instead of a
30-minute live scan; repeatable. Limits, written down so nobody over-trusts it: the camera looks straight down, the height field is
2.5D (a roof is the surface, like for the real UAV), the noise is Gaussian (the Gazebo depth camera has almost none: 3 mm measured
on a real scan fixture), no pose drift. A bug found while testing it: ray-march hits land below the surface by up to half a step
(a -1.8 cm bias on flat ground), fixed by snapping the hit to the height field.

**X-C1** (`eval_terrain.py`): the truth classification of the noiseless height field vs the classification of the scanned map,
on the cells observed in the scan and whose whole 5 x 5 neighbourhood was observed ("interior"). Noise sigma 1 cm, altitude 7 m.

| World | old default (0.05): agree | old with `max_roughness` 0.5: agree / false-lethal / missed-lethal | **new: agree / false-lethal / missed-lethal** |
|---|---|---|---|
| W1 house site | 8.0 % | 98.2 % / 1.0 % / 3.9 % | **99.4 % / 0.1 % / 2.6 %** |
| W2 container yard | 7.4 % | 98.4 % / 1.2 % / 0.0 % | **98.9 % / 0.0 % / 0.2 %** |
| W3 excavation | 15.3 % | 97.1 % / 0.1 % / 2.1 % | **98.8 % / 0.0 % / 1.2 %** |
| W4 decks | 2.5 % | 99.6 % / 0.4 % / 0.0 % | **100.0 % / 0.0 % / 0.1 %** |
| W5 door gauntlet | 5.0 % | 98.7 % / 1.3 % / 0.0 % | **100.0 % / 0.0 % / 0.0 %** |

**High noise (11b, the production default with noise compensation, section 4b).** Sigma 0.03 + 0.001 r^2 (up to ~7 cm at 7 m),
interior cells, agree / false-lethal / missed-lethal:

| World | old with `max_roughness` 0.5 | new (noise compensated) |
|---|---|---|
| W2 | 38.8 % / 15.9 % / 0.1 % | **95.4 % / 2.1 % / 0.2 %** |
| W3 | 38.5 % / 20.1 % / 9.4 % | **88.8 % / 3.5 % / 2.2 %** |
| W4 | 48.2 % / 12.0 % / 0.0 % | **95.7 % / 1.4 % / 0.1 %** |
| W5 | 27.9 % / 20.4 % / 0.0 % | **94.8 % / 3.3 % / 0.0 %** |

The old rule with the override is as good as v2 when the data is clean; v2 needs no override and does not fall apart when the noise
grows. The remaining interior misses of v2 at low noise are slopes within ~0.01 of the limit (0.34 read for a true 0.36), i.e.
threshold jitter. At very high noise the residual false-lethal (1-3 %) is a sensor limit: a 9 cm ledge cannot be told from 7 cm of noise,
and the lethal decision is deliberately not "compensated" (that would hide real 10 cm ledges).

**X-C2** (`eval_planning.py`): A* from S to every goal marker on the truth map, on the old-classifier scan map and on the v2 scan map.
Feasibility (path / no path) agrees with the truth on all 30 goals for both classifiers and no route of either touches a truth-lethal
cell. The routes differ: v2 reproduces the truth route (W2: A 31.7 m vs truth 31.6, C 45.8 vs 45.8, **F 28.1 vs 28.0**; W3, W5: within
0.1 m), the old rule detours (W2 A 35.0, **F 37.2**, C 47.2; W5 doors 0.4-0.9 m longer each, W3 up to 1.8 m at higher noise).

**4b. Noise compensation (added after the first results).** Looking at the confusion matrix of a noisy scan (W2, sigma 0.02 + 0.0005 r^2):
no lethal errors, but 7.5 % of flat cells came out DIFFICULT: on flat ground with height noise sigma the plane fit reads a slope of
~1.77 sigma, the robust 3 x 3 step ~1.85 sigma, the residual ~sigma. `estimate_noise_sigma` takes sigma from the map itself (median plane
residual of the supported cells; most cells of a site are flat; capped at 5 cm) and the DIFFICULT zone subtracts those expected readings;
**the LETHAL decision (raw hazard > 1) is untouched.** Effect (agreement with the truth): W2 sigma~1.4 cm 93.0 -> 99.2 %, W5 93.4 -> 99.9 %, W3 sigma~1.5 cm
91.6 -> 96.7 %, sigma~2.6 cm 54.0 -> 88.8 %, W4 sigma~2.1 cm 61.8 -> 95.7 %; at low noise (3 mm) it costs 0.3 % (W3 99.1 -> 98.8 %).
Strength 1.0 beat 0.5 and 1.5. `IncrementalTerrain(auto_noise=True)` (the node's default, parameter `terrain_auto_noise`) re-estimates every
20 updates and recomputes once when sigma moved by > 20 %.

**Known limitation - the rim.** At the edge of the observed area a step to an UNOBSERVED cell cannot be known by any rule: the far
side of a wall seen only from above is never scanned, so its top looks like flat ground. v2 misses 8 % (W2) and 23 % (W4) of the
truth-lethal cells there (the old rule caught them by accident through the placeholder height). They are not reachable: the lethal
ring on the observed side of the wall disconnects them from the floor, so A* never routes onto them, and the frontier logic keeps its
inputs (the rim stays walkable, so the frontier cells next to a tunnel mouth survive). A rule such as "rim cells are lethal" would
have broken exploration. The 3D refinement (section 8) is the proper fix: the UGV looks at those walls from the ground.

## 5. X-A1 - what the simulated Jackal can really climb
`ugv_limits_test.world` has twelve lanes, one obstacle each; `ugv_limits.py` teleports the UGV to a lane, drives at it at 0.5 m/s for
26 simulated seconds and reports height gained, maximum roll/pitch up to the highest point and a verdict. Friction mu = 1.0
(the model's). Results (two runs: a coarse one and one narrowed down around the limit; a 4 m ramp, then a 3 m plateau):

| Obstacle | Result | max pitch |
|---|---|---|
| ramp 10, 15, 20, 25, 30, **35 deg (slope 0.70)** | **all crossed**, no roll | 13, 17, 22, 29, 36, 42 deg |
| step 4, 8, **9 cm** | crossed | 9-20 deg |
| step **10, 11, 12, 14, 16, 20, 24 cm** | **blocked** (the wheel stops at the edge) | - |

* **Step limit = the wheel radius (0.098 m): 9 cm crossed, 10 cm blocked.** The old `max_step` 0.15 called a 10-14 cm ledge passable
  (DIFFICULT) although the vehicle cannot climb it. The v2 default is now **0.09 m** (a 9 cm ledge reads exactly at the limit: DIFFICULT;
  10 cm: lethal; unit test `test_step_limit_is_the_measured_wheel_limit`).
* **Slope: no failure up to 35 deg (0.70), twice the planner's 0.35.** The limit stays 0.35 as a policy margin: friction 1.0 is the
  best case (wet gravel or mud is closer to 0.5 - a slip limit of ~27 deg), side slopes tip a vehicle earlier than a straight climb,
  and the map's slope has errors. If routes need steeper ramps, raise it with a friction experiment (C-02), not on this evidence alone.
* Driving off the end of a ramp flips the robot (a 0.7 m drop): the first run had no plateau and "flipped" at 10 deg. The experiment,
  not the robot, was wrong; the world now has plateaus.
* Roughness (rubble, ruts) is still unmeasured on the vehicle: `max_roughness` 0.06 m is a guess; the next experiment (N-02) drives W3's rubble field.

## 6. Integration in the live node
* `ElevationMap` has two new layers, `confidence` and `hazard` (published on `/elevation_map` like the others; nav snapshots save them;
  the lidar floor map publishes them too, hazard 2.0 for an occupied cell).
* New node parameters: `terrain_classifier` (`v2` default, `v1` = the old rule for A/B tests), `max_step_v2` (0.09), `max_roughness_m`
  (0.06). The old `max_roughness` (variance) and the launch argument of the same name only affect `v1` now; the stress-world scripts
  still pass `max_roughness:=0.5`, which is harmless.
* Live check (`nav_sim` room_maze, UAV autopilot, 130 s, `/elevation_map`): all six layers present, 9568 observed cells, confidence median 0.94,
  tiers 8100 easy / 385 difficult / 1083 lethal (repeated after adding `max_step_v2`; processes checked clean afterwards).
* **Not yet re-run live:** the full stress-world missions (they use saved snapshots with the old classifier's layer, so they do not exercise
  v2; a mission with a real UAV scan, `run_world_mission.sh`, takes ~30 min). The saved world maps (`maps/*_uavview.npz`) and the world
  generator's ground truth (`scene.py`) still use the v1 constants (0.15 step); regenerating them with v2 changes the "design" feasibility
  table, so it is a separate decision.

## 7. Pitfalls found
* `tests/emap`: running the folder collects as "1 skipped" because `test_fusion_gpu.py` fails to import cupy against this numpy and the
  module-level skip swallows the whole run. Run `python3 -m pytest -q $(ls test_*.py | grep -v gpu)` (77 pass; `tests/nav`: 445 pass, 14 skipped - 9 of those are the world-generator tests that need the Majadroid asset pack, which was deleted from the repo in commit `646071c`).
* A float32 tier compared with a Python `0.3` is `False` for a scalar (True for an array): compare scalars with `pytest.approx`.
* A ramp test without a plateau tests the fall, not the climb.
* Ray-marched hits must be snapped to the surface, or the simulated map has a -2 cm bias.
* The 5 x 5 plane fit must be gated to the centre's surface (section 3, item 1) or walls get twice the lethal band.

## 8. The UGV's own 3D data (user idea, 2026-10-27): first offline evidence
Design agreed in the discussion: one classifier, two data sources; every cell keeps class + confidence + source; a UGV observation
overrides the UAV prior only where its confidence is higher; hysteresis so classes do not flicker; re-plan only when the next 10-15 m of the
path is affected (event-driven, not a timer). `scan_sim.ugv_pass` simulates the Jackal's front camera (the real model's: 34 deg down, 0.1-8 m,
so the floor is seen only ~0.1-0.9 m ahead: a ~1 m strip along the route) and fuses it with the same `fuse_points`.

**X-C3** (`eval_refine.py`, UAV scan -> A* route -> UGV drives it, UGV cloud fused into the SAME map, noise 0.02 + 0.0005 r^2): on the cells the UGV measured the
agreement with the truth rises 95.3 % -> 99.9 % *without* noise compensation; with the compensation of 4b the UAV-only map is already at
99.7 % (W2), 96.3 % (W3), 99.8 % (W5) there and the UGV lifts it to 100 / 96.7 / 100 %. Over the whole map nothing changes (the strip is 1 m wide).
So after 4b, refining the cells the UAV already saw is a small gain; the remaining W3 misses are map-representation effects (a cell holds the
MEAN of its points, a rubble bump of +-9 cm on a 0.5 m lattice is not the cell-centre height the truth uses), which no rule fixes.

**X-C4** (`eval_ground.py`, W4: the floor view is the truth the UGV drives on; the UAV view has the decks): routes to the five feasible goals,
ground layer = a separate map fed only by the UGV camera. On 8816 cells the UGV measured, the ground layer is 100 % correct; the UAV map is
99.2 % (98.8 % under roofs). **902 cells have no UAV opinion at all (573 of them under a roof) and the ground layer classifies them 100 %
correctly.** No floor-lethal cell under a roof was called walkable by the UAV map on these routes (the planner's routes avoid walls, so the strip rarely
contains one) - so the measurable gain of a ground layer on a driven route is *new knowledge* (unobserved -> known), not corrections.

**Conclusion and what is NOT built.** The accuracy gains came from the rules (sections 3-4b), not from the UGV data; the UGV refinement is worth
having for unobserved/occluded cells (and the near-range noise), but it needs the `ground` layer in the node (a second map fed by
`/ugv/camera/points` with a gate on height, the combination `where(ground_valid, ground_tier, uav_tier)`, and the planner re-plan trigger);
that is the next step and is NOT implemented live. Note `tunnel_demo.launch.py` currently disables the UGV cloud in the elevation map when
`overhead_islands:=true` (a UGV cloud fused into the SAME 2.5D cell would fight with the deck height) - a separate ground map avoids that conflict.
Sensor caveat: a camera 34 deg down sees a 1 m strip; a 3D lidar or a less tilted camera would see much more.
