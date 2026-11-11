# TerraLink - TODO and experiment plan

Last updated 2026-11-11 (checklist trimmed to what matters for finalising: EX-05, EX-08, N-04, C-07, U-01, S-01..S-03, R-02, R-04; semantic layer, SLAM, hardware, extra terrain worlds dropped). This file is the living list of what we still have to do and which experiments we will run.
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
| Navigation (Nav2 + follower) | works; Jackal footprint, rubble limit measured | min door 0.7 m clean (0.6 m marginal); crosses bumps up to 8 cm (C-07); opt-in safety gate + mission layer (step14) |
| UAV 2.5D map (`emap`) | good | drift fix on slopes done (unit-tested, not re-measured on a long live patrol); noise compensation in place |
| UGV 3D voxel map | good for now | radius-limited insertion; used only for tunnel/anomaly verification |
| Cell classification | rule-based v2, **geometry only** | slope / step / roughness measured on the vehicle; no ground type or semantics (dropped from scope 2026-11-11) |
| Exploration without a UAV map | **works offline, open live** | freespace everywhere: 19/30 offline, 0 target switches; W4 room B never reached live (3 runs) |

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
* [x] **C-07 Roughness limit on the vehicle** - measured 2026-11-11 in Gazebo (8 rubble lanes): bumps up to 8 cm crossed at full speed, from 10 cm blocked (= the 9 cm step limit); `max_roughness` 0.06 kept (0.04 would flag 3 cm map noise). `media/terrain/`, step14 section 6.
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
* [x] **EX-05 Flying-pixel filter on the UGV depth image** - tried 2026-11-11, **discarded**: live W4 goal B still not reached (318 m, stuck along the north wall within 5 m), so no benefit; code removed. step14 section 5.
* [x] **EX-06 Wall following / backtracking (user's idea, 2026-10-29), offline** - Bug1, Bug2 and a lidar "exit list" (best
  first and depth-first) in `tests/nav/explore_wallfollow.py`, run in the simulator with NO UAV map and with W4's real map.
  No UAV map, 10 reachable goals: freespace 10/10 (525 m), exit list best-first 10/10 (538 m), DFS 10/10 (1105 m), today's
  planner 7/10 (gave up at W5 doors, stuck at W2-C), Bug1 1/10, Bug2 6/10. With a UAV map today's planner stays best. step13.
* [x] **EX-07 No-UAV mode = freespace everywhere (code + offline results done 2026-11-11)** (step13 P1): today's planner falls back to viewpoints when there is no UAV
  map. Awaiting approval; then one live run without the UAV.
* [x] **EX-08 unknown-cell cost 1.5 ("skip a building with no opening")** - tried 2026-11-11, **discarded**: W4-D 48.6 -> 27.5 m but +240 m wasted on unreachable goals, same 19/30 goals reached. step14 section 4.
* Experiment **X-E1 (done offline, step12):** W4 goals A-F, both maps, ground layer on/off, + W1/W2/W3/W5 regression:
  ours / freespace (cost 1, 2, 5) / island-only freespace / explore_lite (`src/nav/scripts/eval_exploration.py`).

## 4. Navigation and the vehicle
* [x] **N-00 Realistic UGV model** (Jackal-class, 4-wheel skid-steer) - `step10`.
* [x] **N-01 Footprint** (2026-10-26, `step10` section 6). Nav2 costmaps now use the Jackal's 0.51 x 0.43 m rectangle and a steeper
  inflation (10.0 / 0.45 m). New test world **W5 door gauntlet** (doors 1.2 ... 0.5 m). Measured minimum door for A* + Nav2: **0.7 m clean
  (15 s per door), 0.6 m marginal (195 s, 8 failed attempts), < 0.6 m refused by A***. W2 C, W3 A, W4 B (oracle) re-run: no regressions.
* [x] **N-04 A* vs Nav2 on narrow doors** - decided 2026-11-11: **accepted limitation** (min door 0.7 m clean, 0.6 m marginal, 0.55 m refused). step14 section 7.
* [x] **U-01 Height drift** - fixed 2026-11-11: only flat cells (3x3 relief < 5 cm) are references + 1 cm deadband; phantom drift on slopes removed (unit tests); not re-measured on a long live patrol. step14 section 2.
* [x] **S-01 Safety layer** - done 2026-11-11 (`nav/safety.py`, `safety_node`, opt-in `enable_safety:=true`; geofence, tilt, obstacle stop with velocity gate, driving-budget return). step14 section 3.
* [x] **S-02 Communication loss** - done 2026-11-11: link-lost flag after 5 s, finish on the last map, RETURN after 120 s (`nav/safety.py`).
* [x] **S-03 Mission layer** - done 2026-11-11 (`nav/mission.py`, `mission_node`, `mission_goals:=...`); live on W2. step14 sections 3 and 8.
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
* [x] **R-02 Technical report regenerated** 2026-11-11 (`python3 docs/build_report.py`); note: the generator's text was not extended with the step14 results - see `step14_finalisation.md`.
* [x] **R-04 Untested snippet in `docs/RUN.md`** - run 2026-11-11 exactly as written: goal reached, 120 m, 560 sim s.
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
