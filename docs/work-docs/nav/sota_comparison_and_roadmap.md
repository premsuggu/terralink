# TerraLink vs. the state of the art, what is left to do, and where compute goes

**Written:** 2026-10-07, after step 07 (A* planner). **Nature of this document:** a positioning and planning note, not a build log. Nothing here changes code.

> **How far to trust the "others" column.** The related-work facts below come from web-search result summaries (titles, abstracts, project pages) and general knowledge of these projects. I did **not** read the papers in depth or run their code. Treat every statement about another system as "orientation, check before citing". Statements about TerraLink are measured in this repo (numbers in §4 were measured on 2026-10-07 on the saved fixture).

---

## 1. Where TerraLink stands (honest summary)

| Layer | TerraLink today | Verdict against open-source practice |
|---|---|---|
| Mapping | From-scratch 2.5D GPU/CPU Bayesian elevation map (`emap`), drift compensation, traversability | Same idea as `elevation_mapping_cupy`; **less mature**, no learned traversability |
| Occlusion handling | Anomaly detector + bounded 3D voxel verification + resolved-region memory | **No direct equivalent found** in the systems surveyed (not a survey claim, an "I did not find one") |
| Global planning | A* with free/blocked/**uncertain** cell classes, route memory, standoff points | Same algorithm class as Nav2 Smac 2D A*; ours adds uncertain-cell semantics, theirs is faster and far better tested |
| Exploration | Anomaly viewpoints, then frontier viewpoints, thin-strip pruning | **Far behind** TARE / FUEL / GBPlanner (hierarchical, information-gain based) |
| Execution | Nav2 DWB, one `/goal_pose` per waypoint | **Behind**: no continuous path following, docking edge open |
| Evidence | Simulation, one main world, n = 3 (clean) / n = 1-2 (full) | **Behind**: no real hardware, no benchmark suite |

Plain reading: our *planning decisions* are competitive for the narrow case we built them for; we are behind on proof and on execution.

## 2. Comparison table

| System | Perception / map | Global planning | Exploration | Compute profile | Validation | TerraLink relative to it |
|---|---|---|---|---|---|---|
| **elevation_mapping_cupy** (ETH RSL; [paper](https://arxiv.org/pdf/2204.12876), [repo](https://github.com/swiss-mile/elevation_mapping_cupy)) | GPU 2.5D map, drift compensation, visibility cleanup, learned traversability | not included | not included | GPU | Real legged robots, DARPA SubT (CERBERUS ANYmal) | `emap` re-implements the idea; **behind** on maturity, learned traversability |
| **Nav2 Smac planners** ([paper](https://arxiv.org/abs/2401.13078v1)) | Costmap2D | Cost-aware 2D A*, Hybrid-A*, State Lattice | none | CPU, C++ | Widely deployed | Same class as our A*; **theirs faster / better tested**, ours adds uncertain-cell handling |
| **Nav2 Regulated Pure Pursuit** ([paper](https://arxiv.org/pdf/2305.20026)) | n/a | n/a | n/a | n/a | Widely deployed | **Behind**: not adopted; DWB is the likely cause of the goal-docking stall |
| **TARE** ([RSS 2021](https://roboticsproceedings.org/rss17/p018.html)) | dense local + sparse global | hierarchical | hierarchical coverage planning; reported lower compute than prior SOTA | CPU | Real ground + aerial robots | **Far behind** on exploration |
| **FUEL** ([paper](https://arxiv.org/pdf/2010.11561)) | incremental frontier structure | hierarchical | reported 3-8x faster than prior work | CPU, UAV-oriented | Real UAV | **Far behind** on exploration (UAV-only, not directly comparable) |
| **GBPlanner / GBPlanner2** ([wiki](https://github.com/ntnu-arl/gbplanner_ros/wiki)) | graph over the known map | graph search | frontier vertices, reduced-compute mode for MAVs | CPU | DARPA SubT, legged + aerial | Behind on exploration |
| **UAV-assisted UGV navigation** (e.g. [ColAG](https://arxiv.org/html/2310.13324v1), [aerial elevation planning](https://www.scitepress.org/PublishedPapers/2017/62983/pdf/index.html)) | UAV SLAM or top-down occupancy / elevation | mostly standard A* / RRT on a shared map | usually none | varies | mostly sim, some field | **Roughly on par** at the concept level |
| **TerraLink** | 2.5D elevation + bounded 3D voxel check | A* (free / blocked / uncertain), route memory, standoff | anomaly then frontier viewpoints | Python planner, 64 ms / plan (§4) | Sim, 1 main world | see §1 |

## 3. What is left to work on

| # | Item | Why it matters | Size |
|---|---|---|---|
| 1 | ~~Run A* on `room_maze`~~ **done in step 08** (live, 3 runs: 3/3 reached; also 3 more tunnel runs: 2/3). Still open: the construction-site map. Also fix the tunnel failure (thin wall seam) | We have **one world, one goal**; thin-strip pruning (0.44 m) is untested on other wall thicknesses | small (offline replay makes it cheap) |
| 2 | Replace DWB one-goal-per-waypoint with continuous `FollowPath` + Regulated Pure Pursuit, tunnel-sized inflation (`docs/resource/05` phases 1 and 3) | Biggest remaining lever on useless motion; likely fixes the E2 docking stall (stopped 0.306 m from goal) | medium |
| 3 | Fix the anomaly detector (accumulate over time) instead of masking flicker with `anomaly_hold_sec` | Root cause, not a workaround | medium |
| 4 | Decide defaults (`planner_type`, `wait_for_mapping`) after more samples | Currently PRM default; n = 1-2 in full mode | decision |
| 5 | Closed loop with the UAV: re-observe uncertain regions during the run | The UAV currently scans once; frontier/anomaly regions are only resolved by the UGV driving there | large |
| 6 | Hierarchical exploration (TARE/FUEL ideas) | Our exploration is greedy single-viewpoint | large |
| 7 | UAV-to-UGV relative localization | Simulation uses ground-truth/odometry; no GNSS-denied story | large |
| 8 | Semantic layer (`terralink_semantic`) | Geometry-only traversability cannot tell mud from gravel | large, not started |
| 9 | Real hardware / more statistics | Sim-only evidence | large |
| 10 | 2D-only limitation (overlapping walkable levels) | Bridges, multi-storey not representable | design question |

## 4. Compute: measured, then proposed

### 4.1 Measured (2026-10-07, `tests/nav/fixtures/tunnel_test_parked.npz`, 160x160 cells at 0.1 m, start (-3,0), goal (3,0), anomaly + frontier masks, 0.44 m pruning, CPU, single thread)

| Stage | Time |
|---|---|
| Mask building (`replay.build_masks`) | 3.5 ms |
| **A\* plan** (`GlobalPlanner`, memory off) | **64 ms** |
| PRM plan, same map | 1,050 ms |

Profile of the A* plan (10 plans, cProfile): 3 `_astar` searches per plan account for ~65 % of the time (strict, optimistic, then the relaxed-inflation retry), `_postprocess` / `_shorten` ~18 %, building the grids ~12 %.

**Not measured:** `emap` fusion cost in a live run, octomap / voxel-map cost, Nav2 + Gazebo cost, how often the node actually replans, or timings on a larger map than the 16 m crop. The planner is small next to those in absolute terms, so the largest real saving may be elsewhere; this has to be profiled before optimising.

### 4.2 Proposed (ordered by expected payoff; none implemented)

1. **Replan only when something relevant changed**: skip planning when no cell inside the route corridor changed and the stored route is still valid. Needs a measurement of the current replan rate first.
2. **Run the passes lazily**: search `optimistic` / `relaxed` only if `strict` fails, instead of preparing all grids.
3. **Bound the search**: stop A* at the goal and restrict it to a window around the start-goal corridor on large maps.
4. **Compile the hot loop** (numba or a small Cython/C++ core) for the ~65 % in `_astar`. Adds a build dependency, so it needs your approval (AGENTS.md dependency rule).
5. **Adaptive UAV scan**: stop scanning when observed-area growth saturates (`map_growth.py` already measures this); the 95 s scan is by far the largest time item in full mode.
6. **Profile mapping and voxel map** before touching them (stride and eviction were already tuned, see report Table II).

## 5. Sources
- Miki et al., *Elevation Mapping for Locomotion and Navigation using GPU*: https://arxiv.org/pdf/2204.12876
- Macenski et al., *Open-Source, Cost-Aware Kinematically Feasible Planning* (Smac): https://arxiv.org/abs/2401.13078v1
- Macenski et al., *Regulated Pure Pursuit*: https://arxiv.org/pdf/2305.20026
- Cao et al., *TARE*, RSS 2021: https://roboticsproceedings.org/rss17/p018.html
- Zhou et al., *FUEL*: https://arxiv.org/pdf/2010.11561
- GBPlanner wiki: https://github.com/ntnu-arl/gbplanner_ros/wiki
- ColAG: https://arxiv.org/html/2310.13324v1
- *A Global Path Planning Strategy for a UGV from Aerial Elevation*: https://www.scitepress.org/PublishedPapers/2017/62983/pdf/index.html
