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
  progress (UGV near (-2.1, -1.5) after ~5 minutes). So the stall is **not caused by this change**. The maze was last run live
  before the step 10 Jackal model and footprint change (step 08: A* 3/3); it has apparently not been re-checked since. Logged as an
  open item in `TODO.md`.
