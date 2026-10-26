# TerraLink — Current Project Status

High-level architecture (how it works)

UAV (iris_quad, Ignition Gazebo)
  → downward RGB-D camera → PointCloud2
  → emap: Bayesian fusion into a persistent 2.5D elevation grid (elevation, variance, is_valid, traversability layers)
  → published as /elevation_map (grid_map_msgs/GridMap)

nav: walkable-mask classification (traversability != LETHAL & is_valid)
  → PRM planner (random sampling + line-of-sight + Dijkstra) → waypoint path
     (or, opt-in: A* planner + route memory/exploration - see step07)
  → Nav2 (DWB controller) drives the UGV (diff-drive) along that path

Two independently working packages, chained together:

- emap (src/emap/) — the elevation-mapping half. Fully built, 12 roadmap steps done: UAV deployment, depth-camera pipeline, CPU elevation map data structure, Bayesian fusion, map shifting, ROS node + live publishing, traversability classification, persistent global map (never forgets terrain once scanned) alongside a rolling local map, GPU (CuPy) fusion (~25% faster), vertical drift compensation, a couple of real bugs fixed (phantom terrain from a mis-placed light, runaway climb from a stale cmd_vel), and a hand-built custom construction-site world (pit/mound heightmap + crane/excavator/people vendored from Fuel).
- nav (src/nav/) — the navigation half, built on top of emap. Classifies emap's traversability layer into walkable space, plans a path with a from-scratch PRM/Dijkstra port of d3's proven algorithm, and drives it with Nav2. Also has an "autonomous demo mode" — the UAV self-patrols 5 waypoints so you can watch the whole pipeline run with zero manual control.

What we're capable of right now

- Fly a UAV in simulation (manually or via a scripted patrol) and build a real, drift-corrected elevation map of a room in real time.
- Classify that live map into walkable/non-walkable terrain automatically (no hardcoded color thresholds like the d3 baseline).
- Plan a collision-free path for a UGV through that map and have Nav2 actually drive it, with real obstacle avoidance (lidar-based local costmap).
- Run the whole thing hands-off: ros2 launch nav nav_sim.launch.py headless:=false autonomous_uav:=true — UAV patrols and maps, UGV plans and drives to a goal, no /cmd_vel input from a human at any point. This was live-verified last session (UGV drove (2.64,1.13)→(3.61,1.49)→(3.63,2.55) with real continuous motion, get_plan succeeding in ~15s).
- A parked UGV won't lock itself out of planning even if the UAV's camera "sees" it as terrain (footprint-clearing fix).
- 382 unit/node tests in `tests/nav/` (+ 44 CPU unit tests in `tests/emap/`, run file-by-file - the folder-level run collects as "skipped" here because the GPU test module's cupy import fails against this numpy). All pure-Python algorithm logic; the node-level tests need ROS sourced but no Gazebo.
- **Opt-in A* planner** (`planner_type:=astar`, default still PRM) with route memory, exploration, anomaly hold, wait-for-mapping, stuck feedback, and an **offline / start-at-planning replay** workflow (`docs/work-docs/nav/step07_astar_planner.md`). Live in `tunnel_demo` (one world, one goal): A* reaches the goal ~16 s after first movement vs PRM ~28 s (clean mode, 3 runs each); it requires the thin-unobserved-strip pruning (without it 0 of 3 reached the goal). Second world (maze) and run-by-run path images added in step 08: A* 3/3 in the maze, 2/3 in the tunnel (one failure at a thin wall seam); PRM 2/3 in the maze, 3/3 (slow) in the tunnel - 3 runs per cell, simulation only (`docs/work-docs/nav/step08_path_trace_images.md`, images in `media/figures/path_comparison/`). Measured planning cost on the saved tunnel map: A* ~64 ms/plan vs PRM ~1.05 s.
- **Four stress-test worlds built and run once each (step 09):** W1 house construction site (CC0 Majadroid pack layout, 100 x 100 m), W2 container yard with narrow lanes/chicane/dead end/sealed pocket, W3 excavation with ramp, slopes, rubble and a berm saddle, W4 structure under overhead decks with rooms, dead ends and a sealed room. Each ships a `.world`, a **ground-truth 2.5D map** in nav snapshot format (so the UAV scan is skipped and A* is tested directly), the world as a plain self-contained 3D page of the real meshes (`media/worlds/<w>_world.html`, open in a browser) and the planner map as a PLY (`media/3d_maps/<w>_elevation_uavview.ply`, open with `media/3d_maps/viewer.html`), with one shared colour code (green walkable / yellow difficult / red lethal). The UAV-view map is the one A* will be given. Generator + conversion tooling in `src/nav/scripts/worldgen/` (`build_world.py`), 23 new tests (408 nav total). First run done (2026-10-15): W2, A* on the UAV-view map - offline sweep matches the feasibility table on all 7 goals, live run to goal C (46 m, chicane + 0.8 m lane) reached the goal; video `media/runs/w2_container_yard_run.mp4`. Hidden interiors (W4 rooms under the deck) are NOT solved: the planner/map logic reaches 5 of 6 goals in an offline simulator but the live runs never reached goal B (details and open problems in the step09 doc, section "Hidden interiors"). W3 (pit ramp), W1 (105 m) and W4 (under the deck, oracle map) also run live with videos in `media/runs/` - all reached their goals (table in step09 doc). See `docs/work-docs/nav/step09_stress_worlds.md`.

Known limitations / what we can't do yet

- **Positioning vs. published systems** (see `docs/work-docs/nav/sota_comparison_and_roadmap.md`): roughly on par at the concept level for UAV-assisted UGV navigation; clearly behind on hierarchical exploration (TARE/FUEL/GBPlanner), on continuous path following (Nav2 Regulated Pure Pursuit vs per-waypoint DWB goals), and on real-robot / multi-world evidence. A* is opt-in, validated on one world and one goal; default stays PRM.
- Open next steps (ordered): A* on the maze and construction-site snapshots; continuous `FollowPath` + RPP and the goal-docking stall; anomaly detector accumulation; compute reductions (replan only on change, lazy search passes, adaptive UAV scan) - none implemented yet.

- Only tested in a plain maze/room world (room_maze.world, ported from d3), not the custom construction-site world — that integration is still on hold. The construction site exists and its elevation is verified correct, but no UGV/PRM navigation has been run on it.
- Single UGV, single fixed goal per run — goal_x/goal_y is one static target passed at launch, not a mission/task queue, and any custom goal has to be manually checked against a live-flown map first (no automatic "is this point reachable" pre-check) — a bad default goal near a wall has actually bitten us once already.
- No true multi-robot coordination — the UAV patrol and UGV path planning don't communicate beyond "the UAV happens to have scanned enough of the room." There's no active "go scan wherever the UGV wants to go next" behavior.
- A rolled-over/crashed UAV has no recovery — if it tips past ~90°, no controller logic can right it; this is mitigated by spawning near-ground and starting the autopilot early, not solved in general.
- Elevation mapping has no object/robot segmentation — a parked UGV can genuinely get fused into the map as fake terrain; we only work around the resulting planning deadlock (via footprint-clearing), we don't prevent the map corruption itself.
- Fuel/network dependency risk was designed around, not eliminated — all construction-site assets had to be vendored locally because live Fuel downloads stall in this sandbox; any new asset would hit the same problem.
- A known cosmetic Nav2 log warning (Failed to create a plan from potential when a legal potential was found) recurs periodically without blocking movement — never root-caused since it isn't actually breaking anything, but not explained either.
- GPU fusion's speedup is modest (~25% at realistic point-cloud sizes), not a dramatic scaling win — CPU path is still the default/reference implementation.

Repo state

Cleanup 2026-10-07: the reference directories `src/d1` (GPU elevation-mapping reference) and `src/d3` (OpenCV+PRM baseline) and their walkthrough docs (`docs/elevation_map_ref`, `docs/uav_ugv_nav_ref`) were deleted; nothing in `emap`/`nav` built or ran against them (verified: `colcon build --packages-select emap nav` succeeds, `colcon list` shows only `emap` and `nav`). They remain in git history up to commit `8a86dab`. `docs/SETUP.md` and `docs/RUN.md` were rewritten for `emap` + `nav` only.

Working tree: `docs/build_report.py` updated and compiled to `media/reports/TerraLink_Project_Report.docx` with IEEE two-column formatting, 4 tables (including Table IV 12-run maze benchmark), 6 embedded figures (including Figs 5-6 PRM vs A* path traces), SOTA literature benchmarking (Elmakis 2022, ETH elevation_mapping_cupy, Nav2 Smac/RPP), offline snapshot replay diagnostic framework, 12 documented defect analyses, and verification against 426 tests (382 nav, 44 emap CPU; the report predates step 09, which brings nav to 405).