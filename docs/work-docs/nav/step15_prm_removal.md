# Step 15 - The PRM planner is removed (2026-11-22)

**Why.** The project had two global planners: the original PRM (random sample points joined into a roadmap) and the A* grid planner
that replaced it in step 07. The PRM was only the default because A* was "opt-in until verified live". Since step 09-14, every world
run (W1-W5, the missions, the replays) uses A*. Keeping the PRM meant two code paths to test and explain, so it was deleted
("we don't need redundant things"). The old code stays in git history: last present in commit `efd5408`
(`git show efd5408:src/nav/nav/prm_planner.py`).

**Concept for a newcomer.** A planner answers "which route goes from here to the goal on this map?". A PRM drops random points on
free cells and links the ones that can see each other, so two runs can give different routes and a narrow gap may get no point at
all. A* searches the grid cell by cell, so it always finds a route if one exists on the grid, and the same map gives the same route.

## What changed

| Place | Change |
|---|---|
| `nav/prm_planner.py` | **deleted** |
| `nav/astar_planner.py` | now owns the `PlanResult` dataclass (it was the only piece A* took from the PRM file); docstring rewritten |
| `nav/planner_node.py` | PRM branch, `planner_type`, `num_samples`, `connect_radius_m` and the fixed random seed removed; `frontier_min_width_m` automatic value is always the robot diameter. The plan trace still writes `"planner": "astar"` so old readers work |
| `nav/replay.py` | `run_plan(..., planner=)` and `replay_sequence(..., planner=)` lost the `planner` argument |
| `scripts/` | `--planner` flag removed from `replay_plan.py`, `replay_sequence.py`; `planner="astar"` removed from the other scripts; `planner_type:=astar` removed from the two `run_world_*.sh` |
| launch files | `planner_type` launch argument removed from `nav_sim.launch.py` and `tunnel_demo.launch.py` (A* is now what you get) |
| `waypoint_follower.py` + launches + `run_world_mission.sh` | ROS node renamed `prm_waypoint_follower` -> `ugv_waypoint_follower`. **Not** plain `waypoint_follower`: Nav2 already runs a node with that name, so it would collide |
| tests | `test_prm_planner.py` deleted; PRM-vs-A* comparison tests removed or turned into A*-only tests; `planner_type:=astar` overrides removed |

**Not changed on purpose:** older step docs (`step01`-`step09`, `docs/resource/`), the PRM-vs-A* comparison pictures in
`media/figures/path_comparison/` and the `"prm"` label colouring in `trace_render.py` (so those saved runs still draw). They describe
what was true when written. Long historical comments in `waypoint_follower.py` that tell the story of live bugs found with the PRM
still say "PRM" where that is what happened.

## Verification

* `tests/nav`: 485 passed, 14 skipped (was 512; the 27 fewer are the PRM tests deleted or merged). `colcon build --packages-select nav`
  succeeds; both launch files load.
* Offline replay on `tests/nav/fixtures/tunnel_test_parked.npz`: route found, 5 waypoints, 6.03 m, as in RUN.md.
* **Live maze run (`nav_sim.launch.py headless:=true autonomous_uav:=true`, 480 s cap): the goal (1.7, -0.5) was NOT reached.**
  The UAV mapped, A* found a 7-waypoint path, the UGV reached waypoint 4, then stalled near (-1.3, -3.7) with Nav2's DWB
  reporting "Trajectory Hits Obstacle".
* **Control run on the unmodified commit** (git worktree, `planner_type:=astar`): same 7-waypoint path, reached waypoint 4, then no
  progress (UGV near (-2.1, -1.5) after ~5 minutes). So the stall is **not caused by this change**. Logged as R-06 in `TODO.md`.

## Maze stall (R-06): investigation, 2026-11-22 (no fix made yet)

**What was measured.** 6 more live maze runs (headless, `autonomous_uav:=true`), with a recorder saving `/elevation_map` every 2 s and every
`/plan_trace` line. Outcomes: 3 reached the goal (first plan 4-5 waypoints), 2 stalled (7 waypoints), 1 not finished (2 waypoints).
The true wall positions were read from `room_maze.world` (the collision boxes) to check the plans.

**The failing plan.** (-0.7,-4.5) -> (-1.7,-4.2) -> (-1.8,-3.0) -> (-1.8,1.0) -> (-1.2,1.7) -> (-0.6,1.7) -> (1.7,-0.5). It goes up the west
corridor and crosses the inner wall (x=-0.95, y from -3.97 to 2.03, 2.5 m tall) near its north end. The earlier stalled runs crossed the same
wall further south (waypoint 4 -> 5 jumped from x=-1.64 to x=-0.30).

**Why the map allowed it.** At the moment of that plan the cells x=-0.95..-0.75, y=1.25-1.95 held elevation 0.0-0.2 and were walkable: the
wall is 0.15 m thick (1-2 cells at 0.1 m) and those cells were first filled with floor readings (floor next to the wall base) instead of the wall top.
They were unobserved until ~20 s before the plan and still wrong 34 s after it.

**Why the first plan is the wrong one.** A* answers the moment ANY route over observed walkable cells exists. In the passing runs that
moment is when the UAV has scanned the east corridor (~+90 s). In the failing runs a route through a false wall opening appears first.
On maps saved after the scan finished (17 snapshots of one run, node run with launch defaults) every plan was the correct east route, 0 wall crossings.

**Why it is never corrected.** `nav_sim.launch.py` starts `waypoint_follower` with every re-plan option off (`enable_blocked_replan`,
`enable_stuck_recovery`, `enable_frontier_replan`, `enable_anomaly_replan` all default False) and `planner_node` without `wait_for_mapping`.
`run_world_mission.sh` and `tunnel_demo.launch.py` turn these on, which is why the world runs do not show this stall.

**Not proven:** that the stall in every failing run is this same mechanism (the first failing plan was not recorded; the second failing plan
was, and its geometry matches). The outcome of the recorded failing run was not watched to its end.

**Correction to my first idea.** I first proposed `wait_for_mapping`. Reading the code showed it only holds back *speculative* routes through
unobserved space; the failing plan's six segments were all "free" (confirmed cells), so it would not have helped. The mechanism built for
"the planner keeps returning the same failing route" is `stuck_feedback` (`nav/stuck_feedback.py`), together with the follower's stuck/blocked
re-plan options.

## The fix and its measurement (2026-11-22)

`nav_sim.launch.py`: `planner_node` gets `stuck_feedback: True`; `ugv_waypoint_follower` gets `enable_stuck_recovery: True` and
`enable_blocked_replan: True` (the same options the world run scripts and `tunnel_demo` use). Guard test: `tests/nav/test_launch_nav_sim.py`.

**Protocol:** 8 live runs in a row, `nav_sim.launch.py headless:=true autonomous_uav:=true`, goal (1.7, -0.5), 330 s cap; success = ground-truth
position within 0.4 m of the goal. (A throw-away harness in the session scratch folder; not part of the repo.)

| | before (5 conclusive + 1 control) | after |
|---|---|---|
| reached the goal | 3 of 6 | **8 of 8** |
| started with the bad 7-waypoint plan | 3 (all stalled) | 2 (both recovered) |
| time to goal, good first plan | ~120-160 s | 123-158 s (6 runs) |
| time to goal, bad first plan | never | 252 s and 282 s |

**How the bad-plan runs recovered (run 4 log):** the 7-waypoint plan at +87 s; 57 s later "The plan crosses cells the map now knows are blocked"
(`blocked replan`) once the UAV had scanned the wall; the planner then handed out the 4-waypoint east route. The stuck-recovery path
itself never fired (0 times in 8 runs); blocked re-plan did the work.

**Weak points, stated plainly**
* Only 2 of the 8 runs began with a bad plan, so the evidence for the recovery is 2 successes, on top of 6 runs that never needed it.
  8 of 8 against about 50 % before is unlikely by luck (0.4 % if the old rate still held), but it is not a large sample of recoveries.
* Recovery is slow and noisy: after the bad plan the planner flipped between routes while the map settled (run 8: 15 blocked re-plans, plans of
  7, 3, 4, 8 waypoints). Route memory kept the old route several times ("kept stored route 73.6 vs new 74.3").
  Not fixed; a candidate for a later look (hysteresis on blocked re-plans).
* The first plan can still be wrong. The fix revises it once the map knows better; it does not stop the UGV from setting off on it.
  In the studio the user decides when the map is good enough, which avoids this by design.
* Only the maze world, one goal.
