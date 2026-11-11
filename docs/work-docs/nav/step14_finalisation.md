# Step 14 - Finalisation round (2026-11-11): what was kept, what was thrown away

**Rule used for every item:** try it, measure it, keep it only if the numbers improve; otherwise discard it and write down why.
The pictures and raw logs are in `media/exploration/`, `media/terrain/` and `media/missions/`.

| Item | Result | Decision |
|---|---|---|
| EX-07 no UAV map = freespace everywhere | 16 -> 19 of 30 goals reached, target switches 788 -> 0 | **kept** |
| U-01 UAV map height drift | phantom drift on slopes removed (unit-tested) | **kept** |
| S-01 safety layer, S-02 link loss, S-03 mission layer | built, 17 unit tests, one live run (W2) | **kept** |
| EX-08 unknown cells cost 1.5 instead of 1.0 | W4-D 48.6 -> 27.5 m, but +240 m wasted on unreachable goals, same goals reached | **discarded** |
| EX-05 flying-pixel filter on the UGV depth image | live W4 goal B still not reached (same as without) | **discarded** |
| C-07 roughness limit on the vehicle | measured in Gazebo; limit 0.06 m kept | **measured, no change** |
| N-04 A* accepts 0.6 m doors that Nav2 struggles with | not changed | **accepted limitation** |
| R-04 `docs/RUN.md` offline snippet | ran as written (goal reached, 120 m) | done |
| LLM / VLM exploration (earlier this session) | abandoned by the user; code and notes removed | removed |

---

## 1. EX-07 - no UAV map: plan through unseen space everywhere  (kept)

**Concept (for a newcomer).** The UGV plans with a map. Where nobody has looked yet the map says "unknown". The *freespace assumption*
says: plan as if unknown ground is open, drive, and re-plan when the lidar finds a wall. Until now we used that only under roofs
(`island_strategy:=freespace`). Without any UAV map the UGV used a slower "go and look at the edge of the unknown" method that changes its
mind often (it picked a new target 18-75 times per goal).

**Change.** `IslandStrategyPlanner` (`src/nav/nav/freespace.py`) now also uses `FreespacePlanner` in the open when the UAV observed less
than `no_uav_fraction` = 2 % of the grid. `GoalIslandPlanner` (`src/nav/nav/overhead.py`) stores that fraction in `last_uav_fraction`;
`planner_node` exposes the parameter `no_uav_fraction` (0 = old behaviour). With a real UAV map nothing changes.

**Result (offline simulator, no UAV map, all five worlds, 30 goals; `media/exploration/ex07_no_uav_freespace.png`):**

| | before | after |
|---|---|---|
| goals reached | 16 | 19 |
| target switches | 788 | 0 |

Per world: W4 5/6 -> 5/6 (switches 156 -> 0), W5 1/8 -> 3/8, W2 4/7 -> 5/7, W1 1/3 -> 1/3, W3 5/6 -> 5/6. W3 goal A: 116.5 m -> 21.2 m.
Unit tests: `tests/nav/test_freespace.py` (3 new). **Limits:** offline only (no live run of this exact switch); the simulator's doors are
about 0.4 m narrower than Gazebo's, so some goals can never be reached there (W5 goals 4-8).

## 2. U-01 - UAV map height drift  (kept)

**Concept.** `emap/drift.py` compares new points with cells the map is already sure about. If they differ by a constant, the UAV's height
estimate is probably off, so the map is corrected. **The bug:** on a slope or at a step edge a cell stores the *average* height of the
cell, while a point inside it is higher or lower than that average. If a pass sees mostly one flank of a ramp the average error is not
zero, so the code "corrects" a drift that does not exist, and over a long patrol the whole map slid (about 0.3 m).

**Change.** `estimate_vertical_drift(..., max_local_relief=0.05, deadband=0.01)`: only cells whose 3x3 neighbourhood spans less than 5 cm of
elevation count as references, and an estimate below 1 cm is treated as 0. Node parameters `drift_max_local_relief` and `drift_deadband`
(`src/emap/config/elevation_mapping.yaml`, `elevation_mapping_node.py`). Tests (`tests/emap/test_drift.py`, 3 new): a ramp seen from one
flank gives 0.125 m of phantom drift with the old rule and nothing with the new rule; real drift of 0.4 m on flat ground is still found;
a 5 mm estimate becomes 0. **Limit:** proven on a synthetic ramp, not re-measured on a long live patrol (that takes about 30 minutes of
wall time per world); the node parameter `drift_max_local_relief:=0` restores the old behaviour.

## 3. S-01 / S-02 / S-03 - safety, link loss, mission  (kept)

All three are pure Python state machines first (no ROS, fully unit-tested), plus thin ROS nodes, wired into `tunnel_demo.launch.py` as
**opt-in** (defaults off, so every old command behaves as before).

* `src/nav/nav/safety.py` (`tests/nav/test_safety.py`, 9 tests). `SafetyMonitor.update()` returns OK / STOP / RETURN:
  geofence, tilt (above 35 deg), a lidar return closer than 0.30 m in front -> STOP (released only after the way has been clear 1.5 s,
  so it does not flicker); a driving-distance budget that no longer covers the way home x 1.3 + 5 m -> RETURN (battery-aware return; the
  simulator has no battery, so it is a budget in metres); no UAV map for 5 s -> `link_lost` flag, and RETURN only after 120 s (S-02: until
  then the UGV simply finishes on the last map it has, which the planner keeps anyway).
* `src/nav/nav/safety_node.py`: also a **gate**. Nav2's velocity is remapped to `/nav2_cmd_vel_raw`; the node republishes it on
  `/nav2_cmd_vel` (the topic the bridge sends to the wheels), as zero while STOP. A second publisher on the same topic would be out-voted
  by Nav2, a gate cannot be.
* `src/nav/nav/mission.py` (`tests/nav/test_mission.py`, 8 tests): goal list, `once` / `patrol` (loops, 0 = forever), dwell time at a goal,
  unreachable goals skipped and remembered, `return_to_base()`, `abort()`. `mission_node.py` publishes the current goal on `/mission/goal`;
  `waypoint_follower` got the opt-in parameter `goal_topic` (restarts planning for each new goal) and reports a goal with no route after
  `goal_failed_after` = 8 failed attempts on `/mission/goal_failed`.
* Launch: `enable_safety:=true safety_fence:=xmin,ymin,xmax,ymax battery_budget_m:=90 mission_goals:=x1,y1,x2,y2 mission_mode:=once|patrol
  mission_loops:=N base_x:= base_y:=`.

**Live check (W2 container yard, saved UAV map, `media/missions/`):** mission "D (5, -24) -> E (19, -21) -> home (-3, -26)", safety gate on,
fence and a 90 m budget. See section 7 for the numbers. **Honest limits:** one run, one world; the stop and tilt checks were
proven by unit tests and by the gate carrying normal driving, but no live obstacle / geofence violation was staged; the very first launch
of this run sat still (Nav2's known start-up flake - the stale-process check in the run script did not know the new nodes and left them
alive, so the scripts' process lists should include `safety_node|mission_node`).

## 4. EX-08 - a mild extra cost for unknown cells  (discarded)

Idea: W4 goal D (outside, south of the building) wastes 21-27 m because freespace dives into the building and turns back. Giving unknown
cells a cost of 1.2 or 1.5 (instead of 1.0) makes the planner prefer known ground a little. Sweep (offline, no UAV map, W2/W3/W4):
1.5 gave D = 27.5 m (shortest 28.2) and changed nothing else on W2/W3/W4 A/B/E/F. But over all 30 goals the same 19 goals were reached,
the 19 reached goals needed 793 m instead of 815 m (-3 %), while the 11 goals that cannot be reached cost 1191 m of searching instead of 950 m
(W1 goal B: gives up after 242 m instead of 75 m). Net negative -> kept 1.0. Raw sweep in `media/exploration/`. (The sweep policies
`freespace12` / `freespace15` stay in `eval_exploration.py` for anyone who wants to repeat it.)

## 5. EX-05 - flying-pixel filter on the UGV depth image  (discarded)

Idea: depth cameras report "in between" ranges for pixels on a depth jump (door jamb); fused into the ground map these phantom points looked
like a wall in W4's 1.0 m door. A filter removed a pixel whose range differs from **both** opposite neighbours by more than 5 %
(real edges keep one similar neighbour, a slanted floor changes by about 0.3 % per pixel); 5 unit tests passed.
**Live, W4 goal B, saved UAV scan, `ground_layer:=true island_strategy:=freespace`, filter on: not reached.** 700 simulated seconds,
318 m driven, the UGV spent about 650 s along the building's north wall within 5 m of the goal without finding the door of room B -
the same outcome as the two runs without the filter (step12 section 7). So the false obstacles were not (only) flying pixels, and the
filter showed no benefit -> code and tests removed. Video of the run: `media/exploration/live_w4B_flying_pixel_filter_NOT_reached.mp4`.
**Still open (named honestly):** W4 room B is reached offline (97 m) but never live; the cause is somewhere in the camera ground layer or in
how the door is perceived, and was not found in this round.

## 6. C-07 - the roughness limit on the vehicle  (measured, limit kept)

New world `tests/nav/integration/ugv_rough_test.world` (`make_rough_world.py`, driver `ugv_rough.py`): 8 lanes, each a 6 m field of
15 cm boxes on a 35 cm grid with random heights up to H. The UGV drives each lane at 0.5 m/s (Gazebo, real physics).

| H (cm) | 2 | 4 | 6 | 8 | 10 | 12 | 16 | 20 |
|---|---|---|---|---|---|---|---|---|
| map roughness, median (m) | 0.005 | 0.011 | 0.019 | 0.020 | 0.029 | 0.035 | 0.045 | 0.056 |
| result | crossed | crossed | crossed | crossed | **blocked** | blocked | blocked | blocked |
| max pitch (deg) | 5 | 10 | 16 | 18 | 22 | 24 | 12 | 4 |

Crossing time was the same as on flat ground (about 19-21 s for 9.3 m). Fields with boxes up to 8 cm are crossed at full speed; from 10 cm
the wheels (radius 9.8 cm) are stopped - exactly the 9 cm step limit measured in X-A1. So **the binding limit is the single highest bump
(the step rule, 0.09 m), not the roughness number.** The classifier already flags 13-65 % of the cells of the blocked fields (hazard
>= 1) and none of the crossable ones with the current `max_roughness` 0.06. Lowering it to 0.04 looked tempting, but it made the
noise-compensation test fail: a real UAV map carries about 3 cm of height noise, which 0.04 would start to call rough. **Decision: no
change (0.06).** Raw output: `media/terrain/c07_rubble_lanes_raw.txt`. Limit: one random field per H, flat ground (friction 1.0), straight
driving - rubble with loose rocks or soft ground is not covered.

## 7. N-04 - A* and Nav2 disagree on narrow doors  (accepted limitation)

A* accepts a 0.6 m door; Nav2 passes it only after 195 s (step10 section 6). Options were: refuse doors below 0.7 m, price them, or change the
controller. Refusing would make goals behind a 0.6 m door "unreachable" although the robot does get through, pricing needs a live
re-measurement per world, and the controller change is a large risk for a marginal case. **Decision:** keep the behaviour, state the honest
minimum door width (0.7 m passes cleanly; 0.6 m marginal; 0.55 m refused) and move on.

## 8. Live mission result

W2 container yard, saved UAV map, real Gazebo + Nav2, safety gate on, fence (-12,-32)-(32,-8), 90 m budget, base (-3, -26).

| leg | result |
|---|---|
| start -> D (5, -24) | reached at 20 sim s, 7.9 m |
| D -> F (20, -15) | reached at 56 sim s (27.7 m cumulative) |
| F -> base | back at (-2.9, -25.8) at 105 sim s, 55.4 m cumulative; mission state DONE; 56.1 m in total |

The robot never left the fence. A second run with goal E (unreachable on this map) shows the skip rule: "no path found" 8 times, then the
mission skips E and drives home. **Three honest notes:** (1) the very first attempt did not move at all - Nav2 start-up flake (a diagnostic relaunch of the identical stack
worked at once); (2) the run script's cleanup did not know `safety_node` / `mission_node` and left them running, so two copies existed in
the next launch; `run_world_scenario.sh` / `run_world_mission.sh` now include them; (3) the recorder also overwrote `media/runs/*_run.mp4`; the
originals were restored with `git checkout` and the new videos live in `media/missions/`.
