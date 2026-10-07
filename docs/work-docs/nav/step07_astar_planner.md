# Step 07 - A* global planner, route memory, exploration, and start-at-planning replay

**Status:** built and tested (368 tests in `tests/nav/`), **opt-in** (`planner_type:=astar`; default stays `prm`).
**Live result:** with the right options A* reaches the goal in **~16 s from first movement** on a straight-ish 5.8 m route with no stalls, versus PRM's ~28 s / 6.5 m (clean mode, 3 runs each) and 62–112 s / 10–17 m (full mode, scan included). It only works with the **thin-unobserved-strip pruning** (without it: 0 of 3 runs reached the goal). See §5 for every number, the caveats, and what is not yet proven.

---

## 1. Why this step exists

`prm_planner` (a port of d3's PRM) hurt a UGV that should not make useless motions:

| Problem | Why it matters |
|---|---|
| Random sampling | Two plans for nearly the same map can differ; a narrow passage may get no sample. |
| Zero-width line of sight, no obstacle inflation | Robot size was left entirely to Nav2's costmap. |
| Replans from scratch every time | Nothing stops it flipping between two almost-equal routes as the map flickers. |

## 2. Concepts (read this first)

### 2.1 A* in one paragraph
Treat the map as graph paper. Every free square gets `g` = cheapest known cost from the start, `h` = a guess of the cost still to go (straight-line distance, which never over-estimates), and `f = g + h`. Repeatedly expand the open square with the lowest `f`. Because `h` never over-estimates, the first time the goal is taken off the queue the path is the cheapest possible; if the queue empties first, **no path exists on this grid** (complete up to the resolution - PRM is not). Moves are 8-directional (diagonals cost √2; a diagonal may not cut a wall's corner). The result is a staircase of 45° steps, hence the straightening stage. Code: `nav/astar_planner.py::_astar`, `dijkstra_all` (same search to every cell, used by exploration).

### 2.2 The pipeline in `astar_planner.plan()`
1. **Inflate** obstacles by robot radius + margin, minus `mask_rim_m` (§2.3).
2. **Cost grid**: free = 1; cells near the inflation edge, DIFFICULT terrain, and uncertain cells cost more.
3. **Search in passes**, so a confirmed route always beats a guess: `strict` (free cells only) → `optimistic` (+ anomaly/frontier cells, penalised), each at full then relaxed inflation.
4. **Straighten** with line-of-sight shortening, per stretch (free vs uncertain) so entry/exit of an uncertain region survive. Free stretches also pass a cost test; uncertain stretches use line of sight alone.
5. **Standoff**: pin a waypoint ~1 m before an uncertain stretch - where the UGV should stop and look.

### 2.3 What the real maps taught us (each fixed, each has a test)
* **The map's lethal rim already counts as clearance.** `emap` marks a step's 3×3 neighbourhood lethal, so a real 0.15 m wall is 0.3 m wide in the mask (measured). Inflating by the full robot radius on top double-counts one cell and closes doorways. `mask_rim_m` (0.1) is subtracted.
* **Shortening must respect cost** on free ground, or paths get dragged back across rough ground / against walls.
* **Optimistic routes that merely run alongside unobserved space are still gambles**, so they are flagged tentative (`near_kind`), not "confirmed".
* **Line of sight must be conservative** (supercover): plain Bresenham slips between two blocked cells that touch at a corner.
* **Unobserved space is not an obstacle for exploration**, only a place to look.
* **Suspected-passage cells are exempt from the clearance test** (`uncertain_waive_m` = 0.5 m, optimistic pass only). *I got this one wrong first.* The live detector flags only ~3×3 cells (0.3 m, narrower than the robot) at each tunnel mouth; my first version demanded robot clearance there and answered "no path" on maps where PRM's zero-width line of sight crossed the tunnel. I had wrongly written that the live wandering "was not a planner problem" - replaying PRM and A* side by side on recorded maps proved otherwise (`tests/nav/test_real_tunnel_fixture.py` pins it on real data). The route is still flagged tentative and gets a standoff point; the 3D voxel check and Nav2's costmap are the safety net.
* **Thin unobserved strips are walls, not passages** (`frontier_min_width_m`, §2.6) - the single biggest finding.

### 2.4 Decisions around the planner (`nav/global_planner.py`)
One class, used by **both** the live node and the offline replay, so what you see offline is what the node does.

| Piece | Module | What it does | Param |
|---|---|---|---|
| Route memory | `plan_memory.py` | Keep the route being driven unless it became invalid, a confirmed route replaced a tentative one, or a new one is ≥15 % cheaper. Returns the **original waypoint list unchanged** so the follower's 5 cm "same plan" test sees no change and Nav2's goal isn't preempted. | `enable_plan_memory`, `plan_switch_margin` |
| Exploration | `exploration.py` | No route (or only a frontier gamble): go and *look*. Suspected passages (anomaly clusters) first, then frontier viewpoints; visited spots blacklisted. | `enable_exploration` |
| Anomaly hold | `mask_latch.py` | Keep a flagged anomaly cell "on" N s after the detector last flagged it (it flickers while the map is built). | `anomaly_hold_sec` |
| Wait for mapping | `map_growth.py` | While the observed area is still growing, don't commit to *speculative* targets; specific signals are still acted on. Capped. | `wait_for_mapping`, `mapping_*` |
| Stuck feedback | `stuck_feedback.py` | Handed a route but not moving for ~18 s → drop the stored route and penalise the stretch **ahead** of the robot for 60 s. | `stuck_feedback`, `stuck_window_sec` |

All **off by default**. `planner_type:=astar` alone is "A* + route memory + the frontier pruning below".

### 2.5 Bugs fixed that also affect the original planner / follower
* **Failed request overwrote the plan flags.** `planner_node` published `plan_has_frontier/anomaly = False` for a request that found *no path*; the follower reads that as "my plan is no longer tentative" and stops periodic replanning (a ~45 s stall seen live). Now only a valid result updates them. **Note: every PRM number in this document ran with this fix** - it is not the originally shipped PRM.
* **A dropped goal was never resent.** In start-at-planning mode a plan is ready ~9 s after launch, while Nav2 may still be coming up; the first `/goal_pose` was silently dropped. Stuck recovery then asked for a plan, a deterministic planner returned the identical route, `_waypoints_effectively_equal` called it "no change" and returned without publishing - the UGV sat at its start for the whole run (seen as the first matrix attempt's "never moved"). Stuck recovery now resends the current goal even when the route is unchanged (`_goal_to_resend`). In the matrix this shows as three runs whose first move came ~20 s late; they then ran normally.

### 2.6 Thin unobserved strips (`frontier_min_width_m`)
A *frontier* is an unobserved cell next to free ground, treated as "might be a passage". But a real 0.15 m wall is only 1–2 cells wide on a 0.1 m map, so part of it is **unobserved rather than lethal** - on the live map the whole dividing wall appears as a thin frontier line. Both planners kept "gambling" through it (seen in every scan-mode run: the UGV drove to the wall, bumped, re-routed to another seam, ...). A strip of unobserved cells narrower than the robot cannot be a passage regardless of what is inside, so frontier cells whose *local* unobserved neighbourhood cannot hold a robot-sized disc are dropped (`walkability.compute_frontier_mask(min_unobserved_width_m=…)`). Local, not per-component: the seam runs into the large unobserved region beyond the map, so a component-level check calls it thick.

`frontier_min_width_m < 0` (default) = automatic: the robot diameter (0.44 m) for `astar`, off for `prm` (the original planner is unchanged). With it A* plans the direct tunnel route (5 waypoints, 6.03 m for a 6.00 m straight line, standoff point before the mouth); PRM given the same pruned mask finds *no path* (random sampling cannot thread the flagged mouth).

## 3. Offline replay and start-at-planning mode

The UAV needs ~95 s to scan before planning is interesting, and the finished map is the same every run.

**Offline (no simulator at all):**
```bash
python3 src/nav/scripts/save_map_snapshot.py --out maps/tunnel_test.npz         # once, from a live run
python3 src/nav/scripts/replay_plan.py --snapshot maps/tunnel_test.npz --start -3 0 --goal 3 0 \
    --frontier --anomaly --frontier-min-width 0.44 --png /tmp/plan.png --crop -4 4 -3 3
python3 src/nav/scripts/replay_plan.py ... --planner prm         # the old planner, same map
python3 src/nav/scripts/replay_plan.py ... --resolved -0.8 -0.7 0.8 0.7 1   # inject a voxel verdict
python3 src/nav/scripts/replay_sequence.py --run runs/astar3 --goal 3 0 --planner astar --hold 30 \
    --explore --wait-for-mapping --frontier-min-width 0.44 --fixed-start -3 0   # a whole recorded run
```
Offline replay is **open-loop**: it exercises the planner, not Nav2 or the follower.

**Live, but skipping the scan (closed-loop):**
```bash
ros2 launch nav tunnel_demo.launch.py headless:=true planner_type:=astar \
    map_snapshot:=$PWD/tests/nav/fixtures/tunnel_test_parked.npz
```
`map_replay_node` publishes the saved map as `/elevation_map` and the UAV patrol + `elevation_mapping_node` are not started; Nav2, planner, follower, voxel map and the UGV physics all run for real. `start_follower:=false` keeps the UGV parked - used to capture the clean map (UGV at spawn the whole scan, so its chassis is not fused in mid-route). What it does not reproduce: the map never changes (no UAV updates, no UGV camera filling in the tunnel interior).

Code: `nav/replay.py`, `nav/map_replay_node.py`, scripts in `src/nav/scripts/`. Fixtures: `tests/nav/fixtures/tunnel_t100.npz` (real ~100 s map, 10 m crop) and `tunnel_test_parked.npz` (clean finished map, 16 m crop).

## 4. Parameters
`planner_type` (`prm`|`astar`), `robot_radius_m` 0.22, `inflation_margin_m` 0.05, `mask_rim_m` 0.1, `footprint_radius_m` 0.3, `standoff_m` 1.0, `uncertain_waive_m` 0.5 (in `PlannerParams`), `anomaly_penalty` 20, `frontier_penalty` 2, `clearance_weight` 1, `difficult_weight` 1, `frontier_min_width_m` -1 (auto), `enable_plan_memory` true, `plan_switch_margin` 0.15, `enable_exploration` false, `exploration_gain_weight` 1, `anomaly_hold_sec` 0, `wait_for_mapping` false (+ `mapping_window_sec` 10, `mapping_growth_frac` 0.01, `mapping_max_wait_sec` 150), `stuck_feedback` false (+ `stuck_window_sec` 18). `tunnel_demo.launch.py` args: `planner_type`, `enable_exploration`, `anomaly_hold_sec`, `wait_for_mapping`, `stuck_feedback`, `frontier_min_width_m`, `map_snapshot`, `start_follower`; `nav_sim.launch.py`: `planner_type`.

## 5. Live results (tunnel_test, headless, goal (3,0), ground-truth `/ugv/odom_ground_truth`)
"Move→arrive" = first movement to within 0.3 m of the goal. "Stalled" = 5 s windows with <0.1 m net movement between first move and arrival. Straight start→goal is 6.0 m. Arrival is checked against 0.3 m while Nav2's own goal tolerance is 0.25 m, so a run that stops at 0.30–0.31 m counts as "no" here.

### 5.1 Start-at-planning mode (clean map, UAV scan skipped) - 3 runs per configuration
| Config | Reached | Move→arrive (s) | Driven (m) | Stalled (s) |
|---|---|---|---|---|
| A  PRM (as shipped + the §2.5 fixes) | 3/3 | 27.6, 28.7, 27.6 (mean 28.0) | 6.55, 6.54, 6.45 | 5, 5, 5 |
| B  A*, frontier pruning off | **0/3** | - | 3.8, 3.9, 3.2 (stopped at the wall) | 120, 115, 130 |
| C  A* (defaults) | 3/3 | 16.6, 15.4, 15.4 (mean 15.8) | 5.77, 5.77, 5.77 | 0, 0, 0 |
| D  A* + exploration + stuck feedback | 3/3 | 16.5, 16.7, 19.4 (mean 17.5) | 5.78, 5.78, 6.52 | 0, 0, 0 |
No >120° direction reversals in any run. Config C/D "driven" is under 6.0 m because arrival is counted 0.3 m short of the goal. Run-to-run spread is small, so these differences are real for this world/map.
* A* with pruning is ~44 % faster from first movement and drives ~11 % less than PRM; PRM's route is a slight detour with one short stall.
* Without the pruning A* fails outright: it routes through the dividing wall's unscanned top and stays there (goal resend fired repeatedly; a wall is not fixable by resending).
* Exploration + stuck feedback (D) bought nothing here (static map, nothing to explore) - they stay off by default. D3's slower route coincides with a first move ~18 s late (the dropped-goal case, §2.5).

### 5.2 Full mode (UAV scan included), final code
| Run | Config | Reached | Arrived (s) | 1st move (s) | Move→arrive (s) | Driven (m) | Stalled (s) |
|---|---|---|---|---|---|---|---|
| E1 | A* (defaults) | yes | 107 | 32.8 | 74.2 | 16.3 | 0 |
| E2 | A* (defaults) | **no** (stopped 0.306 m from goal, 210 s idle) | - | 33.2 | - | 13.5 | 210 |
| F1 | A* + `anomaly_hold_sec:=30 wait_for_mapping:=true` | yes | 110 | 93.7 | **16.3** | **5.77** | 0 |
| F2 | same | yes | 97 | 68.2 | **28.8** | 6.88 | 0 |
| G1 | PRM | yes | 147 | 34.5 | 112.5 | 17.3 | 45 |
Earlier PRM full-mode run (before several fixes): arrived 98 s, move→arrive 62 s, 10.5 m, 10 s stalled.
* With A* defaults the UGV starts at ~33 s on a tentative plan while the UAV is still scanning and wanders (74 s / 16 m, one run stalled just short of the goal). With `wait_for_mapping` it holds still until the map stops growing (~70–95 s), then drives a clean route: **16–29 s, 5.8–6.9 m, no stalls** - the same as the clean mode. Total arrival time (97–110 s) is no worse than PRM's (98–147 s).
* **Small sample (n=1–2 per row); recommended full-mode config: `planner_type:=astar wait_for_mapping:=true anomaly_hold_sec:=30`.** Not made a default - that is your call after more samples.

### 5.3 Earlier scan-mode runs (before the fixes above) - history, confounded
Six runs and one PRM baseline, before the clearance waiver, frontier pruning and goal-resend fix existed: A* variants 106–148 s / 11–20 m, one failed in a Nav2 DWB stall; PRM 98 s / 10.5 m. They were taken in full mode (scan flicker included), which is exactly the condition that muddied them, and they led to the wrong first diagnosis noted in §2.3. Kept only because the diagnosis path (replays on recorded runs) is the method §3 packages.

### 5.4 What is and is not proven
* Proven (n=3, clean mode): A* + pruning beats PRM on time-from-first-move, distance and stalls on this map; A* without pruning fails.
* Reasonably shown (n=1–2, full mode): `wait_for_mapping` gives the same clean behaviour with the scan included.
* **Not proven:** other worlds/goals (one world, one goal here; **update:** step 08 adds 3 maze runs - A* reached the goal 3 of 3 - and 3 more full-run tunnel runs, of which A* arrived in 2 and failed once, see `step08_path_trace_images.md`); that `anomaly_hold_sec`, exploration or stuck feedback help (hold/wait were run together; neither exploration nor stuck feedback showed a benefit); robustness of the 0.44 m pruning width on other wall thicknesses. E2's stop 6 mm outside the arrival threshold shows the goal-docking edge (Nav2's DWB near the goal) is still a loose end.

## 6. Files
| New / edited | Purpose | Tests |
|---|---|---|
| `nav/astar_planner.py` | inflation, cost grid, A*, supercover LOS, shortening, standoff, waiver, `evaluate_path` | `test_astar_planner.py` |
| `nav/plan_memory.py`, `exploration.py`, `global_planner.py` | route memory; anomaly+frontier viewpoints; the shared decision | `test_plan_memory.py`, `test_exploration.py`, `test_global_planner.py` |
| `nav/mask_latch.py`, `map_growth.py`, `stuck_feedback.py` | hold / growth / stuck | `test_mask_latch.py`, `test_map_growth.py`, `test_stuck_feedback.py` |
| `nav/walkability.py` (edited) | `compute_frontier_mask(min_unobserved_width_m=…)` | `test_walkability.py` |
| `nav/replay.py`, `map_replay_node.py`, 4 scripts | offline replay, start-at-planning node, snapshot capture | `test_replay.py`, `test_map_replay_node.py` |
| `nav/planner_node.py` (edited) | `planner_type`, params, flag fix | `test_planner_node_astar.py` (ROS sourced) |
| `nav/waypoint_follower.py` (edited) | resend goal on stuck recovery | `test_waypoint_follower.py` |
| `nav/resolved_regions.py` (edited) | `passable_mask()` | `test_resolved_regions.py` |
| fixtures | real maps | `test_real_tunnel_fixture.py` |
Run: `source /opt/ros/humble/setup.bash && cd tests/nav && python3 -m pytest -v`. Environment note: `tests/emap` collects as "1 skipped" when run as a folder here (the GPU test module's cupy import fails against this numpy); the 44 CPU files pass run individually.

## 7. Limitations and next steps
* One world, one goal. Wall thickness vs the 0.44 m pruning width should be checked on `room_maze` and a construction-site map (the offline replay makes that cheap: capture a snapshot, replay).
* The anomaly detector flags only ~3×3 cells per mouth and flickers during mapping; a better fix is at the detector (accumulate over time), not the planner. `anomaly_hold_sec` only masks it.
* Goal docking (E2 stopped at 0.306 m, idle) and Nav2's DWB in the 1.0 m tunnel remain; `docs/resource/05` phases 1 and 3 (continuous `FollowPath`, Regulated Pure Pursuit, inflation 0.40) are the next biggest lever and are **not** part of this step.
* Only the follower's goal-resend was fixed; it still sends one goal per waypoint (stop-and-go).
* 2D only: overlapping walkable levels (bridges, multi-storey) are not representable.
* Compute (measured later, see `sota_comparison_and_roadmap.md` §4): A* plan ~64 ms vs PRM ~1.05 s on the fixture map; 3 searches per plan = ~65 % of A* time, so lazy passes / a compiled search loop are the cheap wins. Whole-pipeline cost (mapping, voxel map, Nav2) is not profiled.
* Default stays `prm`; flipping it, and making `wait_for_mapping` the default for `astar`, are decisions for you.
