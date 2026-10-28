# TerraLink - TODO and experiment plan

Last updated 2026-10-27. This file is the living list of what we still have to do and which experiments we will run.
Status per item: `[ ]` open, `[~]` in progress, `[x]` done. Finished work is documented in `docs/work-docs/`; this file only
points to it. Rules of the project that apply to everything below: unit test per step, no push without approval, no stale
background processes after a run (kill, check, cap the run time, report progress), reuse an existing solution before building
one, document every step.

## 0. Scope and decisions (fixed)
* **Static world.** Moving objects (people, vehicles) are out of scope; another team is working on them (user, 2026-10-26).
* **Simulation first** (Ignition Gazebo Fortress, ROS 2 Humble, WSL2); real-hardware items are listed but not scheduled.
* **Reuse before build** (user, 2026-10-26): for every open item we first look for a maintained solution and report what it
  achieved (research done so far: exploration - section 3; cell classification - section 2).
* **UGV = Clearpath Jackal-class model** (0.51 x 0.43 x 0.25 m, 4-wheel skid-steer) - done, `docs/work-docs/nav/step10_jackal_ugv_model.md`.
* The intended mission is UAV-first: a UAV scan gives the global 2.5D map, the UGV drives on it. Autonomous exploration of
  space the UAV cannot see (inside a roofed structure) is the secondary, harder case.

## 1. Where we stand (honest status of the five blocks)
| Block | State | Evidence / gap |
|---|---|---|
| Path planner (A*, route memory) | **solid** on a good map | W1-W4 reached with a complete map (`step09`); deterministic |
| Navigation (Nav2 + follower) | works; not yet tuned for rough ground | circular 0.22 m footprint for a 0.51 m-long robot; DWB; 0.5 m doors fail |
| UAV 2.5D map (`emap`) | good, **not consistent** | height drift (0.3 m over a patrol), variance used as roughness, noisy single-observation cells |
| UGV 3D voxel map | good for now | radius-limited insertion; used only for tunnel/anomaly verification |
| Cell classification | **weak - the main gap** | geometry thresholds only (slope, step, roughness); no ground type, no semantics |
| Exploration without a UAV map | **open** | works offline (simulator), never reached live (W4 goal B) |

## 2. Cell classification (the biggest gap) - research done, plan below
**What exists today** (`emap/traversability.py`, `nav/walkability.py`): per-cell slope, step height and roughness (height variance)
from the 2.5D map -> traversability in [0, 1] -> walkable mask. Known weaknesses: variance is a *measurement* uncertainty
(0.01 x range^2 per observation) used as a *roughness* (a cell seen once from 6 m reads 0.36 = "lethal" - we needed
`max_roughness:=0.5` for scans); cannot tell soft ground / mud / water / vegetation from hard ground; cannot tell a 3.5 m deck
from a ground plane except by its height; no confidence (a seen-once cell and a seen-30-times cell look the same);
no handling of negative obstacles (holes, trench edges) beyond the step threshold.

**What the literature does (summary of the research, sources in the chat log of 2026-10-26):**
| Approach | Examples | What it achieved | Fit for us |
|---|---|---|---|
| Geometric traversability from an elevation map (slope/step/roughness, filters) | ETH `traversability_estimation`, `grid_map_filters`; `elevation_mapping_cupy` (leggedrobotics, ROS 2 ports exist for Humble: iit-DLSLab, amilearning) | the standard for legged/wheeled robots; real deployments | **what we have**; the ROS 2 port of elevation_mapping_cupy also fuses semantic layers and has a learned traversability filter |
| Self-supervised from the robot's own experience | Wild Visual Navigation (ETH/Oxford, RSS 2023, extended 2024): pre-trained DINO features + online learning from where the robot actually walked; ScaTE; Schmid et al. 2022 | traversability in forests/parks/grass learned in **< 5 min** of in-field training on ANYmal, 1.4 km footpath; code released | **good fit for "soft vs hard ground"**: needs a camera + the UGV driving; in a static world the labels come cheaply |
| Supervised semantic segmentation of RGB (off-road) | GANav; datasets RUGD (24 classes, Husky camera), RELLIS-3D (adds mud, rubble, puddles, deep water), GOOSE (64 classes), **GOOSE-Ex (excavator / construction)** | mIoU 54.9 on GOOSE-Ex after fine-tuning, 45.6 on RELLIS-3D; no pretrained model specific to construction sites | fine-tuning data exists; needs GPU and a sim-to-real gap plan |
| Aerial (UAV) traversability | URA* (aerial CNN -> traversal probability -> uncertainty-aware A*, 2023); Guastella 2017 (UAV elevation -> costmap -> D*); "Seeing Where to Deploy" (2026, UAV metric RGB -> geometric-semantic traversability for UAV-deployed UGV) | closest to our architecture | **the same shape as our pipeline**: UAV RGB semantic layer on top of the geometric layer |

**Recommended plan** (to discuss before starting; each step is testable offline):
* [x] **C-01 Fix the geometric classifier first (no new dependency) - DONE 2026-10-27, `step11_terrain_classification.md`.** Separate *measurement uncertainty* from *roughness*:
  roughness = local height variation of the elevation (e.g. std of the neighbourhood), confidence = number of observations.
  Add a confidence layer and "unknown vs lethal" distinction. Unit tests with synthetic terrain (ramp, step, rubble, pit).
  Success: the stress scans work with the default `max_roughness` (no 0.5 override).
  **Result:** `emap/terrain_analysis.py` (plane-fit slope, robust step, roughness from the elevation, confidence + hazard layers, incremental
  update); on simulated real scans 97.6-100 % agreement with the truth classification (old default: 2.5-15 %), 0.0-0.1 % false-lethal, routes equal
  to the truth routes; step limit measured with the Jackal model: 9 cm crossed / 10 cm blocked -> `max_step` 0.09 (was 0.15); slope: all ramps up
  to 35 deg crossed (limit kept at 0.35 as a policy margin). Known limitation: the rim of the observed area (needs C-06).
* [x] **C-06 (first version done 2026-10-27, step11 sections 8-10) 3D refinement of the classes from the UGV's own data (user idea, 2026-10-27).** One classifier, two data sources; class + confidence +
  source per cell; UGV overrides the UAV prior where more confident; hysteresis; re-plan only when the next 10-15 m of the path is affected.
  First step: fuse the UGV depth-camera cloud into the same map through the node's `secondary_points_topic` (v2 reclassifies incrementally),
  then a `ground` layer (floor under a roof) + `clearance` layer from the UGV's points. Evaluate prior vs refined against the W1-W5 truth maps.
  Sensor caveat: the Jackal has a 2D lidar + front depth camera only. See `step11` section 8. **Offline evidence done (X-C3/X-C4): a separate ground layer
  classifies 100 % correctly the cells the UAV never saw (902 on W4, 573 under roofs); on cells the UAV already saw the gain is small after the noise
  compensation. Built: `ground_layer:=true` (separate ground map in the node, `nav/ground_layer.py` combination, `plan_blocked` event-driven re-plan, self-pollution fix
  at the start, camera tilt 0.6 -> 0.35 rad = 4x wider floor strip). **Live: W4 goal B (hidden dead-end room) reached for the first time (0.13 m, 878 sim s) - but 366 m driven, 89 re-plans:
  open = efficiency (EX-03: commit to targets, hysteresis on re-planning).** Still open: hysteresis for classes, `ground` clearance layer, a 3D lidar option.
* [ ] **C-07 Measure the roughness limit on the vehicle** (drive W3's rubble field / ruts; `max_roughness_m` 0.06 is a guess) and the slope limit
  on low-friction ground (needs C-02 friction patches). (The world maps and `scene.py` ground truth were regenerated with v2 on 2026-10-27, `scripts/reclassify_maps.py`; feasibility tables unchanged.)
* [ ] **C-02 Terrain-test worlds with ground types.** Add visual + friction/softness differences to the worlds (gravel, mud/soft
  patch, water puddle, grass) so classification has something to classify; keep the geometric truth map and add a *class truth map*.
* [ ] **C-03 Semantic layer from RGB (UAV and UGV).** Candidates in order of effort: (a) a pre-trained off-road segmenter
  (GANav on RUGD/RELLIS/GOOSE) -> class -> cost table; (b) WVN-style self-supervised traversability from the UGV's driving.
  First check: do weights/code run under ROS 2 Humble and on this GPU (if any)? CPU-only is a hard constraint to confirm.
* [ ] **C-04 Fuse geometry + semantics into one traversability layer** (`emap` layer, with confidence), consumed unchanged by
  the A* planner (the planner only needs the walkable/cost masks).
* [ ] **C-05 Compare with `elevation_mapping_cupy`'s ROS 2 port on the same worlds** (it already provides fusion + traversability
  filters): decide keep-ours / adopt-theirs for the UAV map and classification.
* Experiments: **X-C1** classifier ablation on the four worlds + new terrain-test world (metric: fraction of ground-truth
  traversable cells classified walkable / non-traversable cells classified walkable, per class); **X-C2** end-to-end: same
  goals, success and path cost with geometry-only vs geometry+semantics.

## 3. Exploration without a UAV map - research done
**Do we need it?** Realistic indoor/disaster cases yes (collapsed or unmapped buildings; rooms the UAV cannot see), but not as
the primary mission. So: **reuse a standard explorer; keep our hidden-island code only as the UAV-to-UGV handover.**

**Who has achieved it (all lidar-based, built the map as they go):**
| System | Achievement | Notes |
|---|---|---|
| TARE (CMU) | DARPA SubT Urban circuit, 886 m in 1458 s fully autonomous; RSS 2021 best paper; Science Robotics 2023 | two-level (local detailed + global coarse) tour; ROS 1 (Melodic/Noetic) per its page - ROS 2 branch to verify |
| GBPlanner / GBPlanner2 (NTNU) | in the SubT-winning CERBERUS team; 4 ANYmal robots in the Prize Round | graph sampling in free space; ROS 1 |
| FAEL (SYSU, RA-L 2023) | large-scale Gazebo worlds + a real-robot video | ROS Noetic, Jackal simulator |
| STGPlanner (2024) | beats TARE/FAEL/GBPlanner2 in 4 sim worlds (-19.8 % time, -15.6 % path); real office 731 m2 in ~350 s | skeleton/topology graph: finish a branch before leaving; code link in the paper (not verified live) |
| explore_lite / m-explore-ros2 | classic frontier explorer on Nav2 + slam_toolbox, Humble | simplest to try; known to dither at doors |
**Lessons for us:** commit to a target/branch (STGPlanner/TARE); plan one tour over all frontiers rather than re-picking one block;
use the lidar's 12 m (longer legs); treat a Nav2 failure as map information; none of them has our "roof looks like floor" problem.

**2026-10-27: X-E1 done offline - see `docs/work-docs/nav/step12_exploration.md`.** Winner: the FREESPACE ASSUMPTION (D* Lite /
Nav2 `allow_unknown`, unknown cost 1) - W4 real scan: all feasible goals, 0 target switches, B in 99 m (ours: not reached),
sealed C given up at 113 m. explore_lite lost (B 284 m / 41 switches; stalls on a single big frontier). Also found and FIXED: the
ground layer switched off the hidden-island rule (`planner_node` merged it first; live W4 log: 29 of 31 decisions "ordinary ground").
* [x] **EX-01 explore_lite** - its decision rule ported and compared offline (`tests/nav/explore_policies.py`); clearly worse than
  freespace and than ours on W4 (goal-free, centroid targets). Live package run not recommended (ask the user).
* [x] **EX-02 TARE** - has a ROS 2 Humble/Jazzy branch, but runs inside CMU's own stack on registered 3D scans and is goal-free
  coverage exploration: not tested (would replace Nav2 + our map; our UGV has a 2D lidar).
* [x] **EX-03 Adopt freespace planning under roofs** - done 2026-10-28: `nav/freespace.py`, `island_strategy:=freespace` (default
  still `viewpoints`), ground-layer confidence gate (`nav/ground_layer.py`, 0.15). Offline: W4 real scan, B in 98 m, 0 switches.
  **Live: 2 runs, room B NOT reached** (71 m stuck / 443 m, reached the room next to B in 300 s): the camera ground layer's false
  obstacles (flying pixels at door jambs) closed B's 1.0 m door. step12 section 7; videos `media/runs/w4_structure_decks_mission_freespace_*.mp4`.
* [ ] **EX-05 Flying-pixel filter on the UGV depth image** (drop points at depth discontinuities before fusing the ground layer,
  like PCL's ShadowPoints), then ONE live W4-B run. Awaiting approval.
* [ ] **EX-04 STGPlanner skeleton idea on our 2D floor map** (corridors/doors): read its code, decide port vs. reimplement.
* Experiment **X-E1 (done offline, step12):** W4 goals A-F, both maps, ground layer on/off, + W1/W2/W3/W5 regression:
  ours / freespace (cost 1, 2, 5) / island-only freespace / explore_lite (`src/nav/scripts/eval_exploration.py`).

## 4. Navigation and the vehicle
* [x] **N-00 Realistic UGV model** (Jackal-class, 4-wheel skid-steer) - `step10`.
* [x] **N-01 Footprint** (2026-10-26, `step10` section 6). Nav2 costmaps now use the Jackal's 0.51 x 0.43 m rectangle and a steeper
  inflation (10.0 / 0.45 m). New test world **W5 door gauntlet** (doors 1.2 ... 0.5 m). Measured minimum door for A* + Nav2: **0.7 m clean
  (15 s per door), 0.6 m marginal (195 s, 8 failed attempts), < 0.6 m refused by A***. W2 C, W3 A, W4 B (oracle) re-run: no regressions.
* [ ] **N-02 Rough-terrain stability.** Build W3-like terrain with bumps/ruts (not only smooth slopes); drive with DWB vs
  Nav2's MPPI controller; metric: tilt (roll/pitch RMS from the pose), time, slip, failures. Slope limits of the Jackal model.
* [ ] **N-03 Skid-steer fidelity.** The 0.49 m effective separation was measured on flat ground; check on slopes and gravel
  (friction scenarios from C-02).
* [ ] **N-04 A* vs Nav2 disagreement on narrow gaps.** Reduced by N-01 but not closed: A* accepts a 0.6 m door that Nav2 passes only after 195 s.
  Options: make A* refuse / price doors < 0.7 m (then re-check W2's 0.8 m lane), switch DWB's `BaseObstacle` to a footprint critic or MPPI,
  mark a gap closed after repeated local failure. Test bed: W5 (goals 5-8).
* [ ] **N-05 Check controller tolerance** for the UGV's higher mass/inertia (accelerations, `velocity_smoother`), now that the
  vehicle is 16 kg instead of 2 kg.

## 5. UAV map consistency (`emap`)
* [ ] **U-01 Height drift.** `drift_compensation` slid the map by ~0.3 m over a long patrol -> turn it off (workaround) or fix.
  Decide: keep ours or adopt `elevation_mapping_cupy`'s drift handling (C-05).
* [ ] **U-02 Multi-pass consistency.** Same area scanned in two passes / two altitudes must give the same elevation (metric:
  RMS height difference per cell). A fixed patrol pattern per world, saved.
* [ ] **U-03 Scan quality settings.** Lane spacing / altitude / stride that give a usable map (6 m, 2 m lanes worked; higher or
  sparser gave 57 % "lethal"). Document as a recommended mission profile.
* [ ] **U-04 UAV map -> UGV hand-off check:** UGV-observed cells vs. UAV map disagreement (e.g. the 0.125 m camera error of the
  old model, step10 pitfall 3) - add a consistency test.

## 6. UGV 3D voxel map
* [ ] **V-01** Nothing known to be broken. Re-validate on the tunnel world with the new camera mount (the depth camera now sees
  0.17-0.74 m of floor from 0.155 m height; was 0.05 m). Re-check the radius-limited insertion on a long mission.

## 7. Localisation and mapping on a real robot (not in the original list - flagged by us)
The simulation uses **ground-truth odometry** (`/ugv/odom_ground_truth`) for the UGV, the voxel map and the floor map. A real robot
needs lidar-inertial odometry and UAV-to-UGV map alignment.
* [ ] **L-01 Research and reuse**: FAST-LIO2 / LIO-SAM / slam_toolbox (2D) - ROS 2 Humble status, what they achieved, CPU needs.
* [ ] **L-02 Experiment X-L1:** replace ground-truth odometry by slam_toolbox (2D lidar) in one world; measure the drift against
  ground truth and its effect on goal-reaching error. Then UAV-map alignment (initial pose + scan matching to the 2.5D map).

## 8. Other items we may have forgotten (candidates; to be prioritised with the user)
* [ ] **S-01 Safety layer:** stop on a collision/cliff, geofence, battery-aware return, e-stop topic.
* [ ] **S-02 Communication loss** UAV <-> UGV (the UGV must finish on the last map).
* [ ] **S-03 Mission layer:** a goal list / patrol / return-to-base interface instead of one `goal_x, goal_y`.
* [ ] **S-04 Payload tasks** (what the UGV carries or does at the goal) - needs the use-case list below.
* [ ] **S-05 Real-time performance** of the whole stack on target hardware (CPU fusion is ~0.25x real time today).
* Out of scope: dynamic obstacles (other team).

## 9. Use cases and readiness (to finish with the user; see the chat analysis of 2026-10-26)
| Use case | Today | Blocking items |
|---|---|---|
| Material/tool delivery over a construction site from a UAV map | closest to ready | L-01, C-01..04, N-01, S-01 |
| Repeated site inspection patrols | mostly ready for fixed routes | L-01, S-03 |
| Earthmoving-site support (slopes, trenches, spoil) | partly | C-01..05, N-02 |
| Search and rescue / disaster entry | not ready | EX-01..04, C-03, N-02, S-01, S-02 |
| Yard/warehouse logistics | ready in sim (static) | L-01, S-03 |
| Indoor mapping without UAV | not ready | EX-01..04 (reuse) |

## 10. Housekeeping
* [x] **R-01 Re-ran W2, W3, W4 (complete map) and W1 with the Jackal model**; videos regenerated with the to-scale Jackal drawing (results in
  `step10` section 6.5; W1 row there).
* [ ] **R-02 Regenerate the technical report** (`docs/build_report.py`) after R-01 and the classification work.
* [ ] **R-03 Write the learning-style tutorial** for the lidar floor map / hidden islands / skid-steer (docs/work-docs/learning/).
* [ ] **R-04 Untested snippet** in `docs/RUN.md` (offline simulator) - run it once.
* Done and committed: stress worlds + videos + hidden-interior experiment (commit `1b6fdf2`, not yet pushed).
* Done and committed: Jackal UGV model (`d8c59bb`; `step10`, `tests/nav/test_ugv_model.py`, `tests/nav/integration/`).
* Done and committed: footprint fix N-01 (`118c8a8`).
* Done and committed (2026-10-27): C-01 terrain classification v2 (`10fa2ee`, `bf7a9be`). Done, uncommitted: ground layer, camera tilt, event-driven re-plan, maps reclassified (`step11` sections 9-11).

## 11. Proposed order of work (for discussion)
1. ~~N-01 footprint + R-01 re-runs~~ (done 2026-10-26).
2. ~~C-01 rule-based classifier v2~~ (done 2026-10-27, user decided: rules, not a segmentation model). Next: C-06 (3D refinement from the UGV), C-07 (roughness/friction limits), C-02 (terrain worlds with friction patches).
3. EX-01 (explore_lite baseline) - one hour tells us whether to reuse or fix.
4. C-03/C-05 (semantic layer; elevation_mapping_cupy comparison) once GPU availability is known.
5. L-01/L-02 localisation.
