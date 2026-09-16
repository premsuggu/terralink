# TerraLink — Current Project Status

High-level architecture (how it works)

UAV (iris_quad, Ignition Gazebo)
  → downward RGB-D camera → PointCloud2
  → emap: Bayesian fusion into a persistent 2.5D elevation grid (elevation, variance, is_valid, traversability layers)
  → published as /elevation_map (grid_map_msgs/GridMap)

nav: walkable-mask classification (traversability != LETHAL & is_valid)
  → PRM planner (random sampling + line-of-sight + Dijkstra) → waypoint path
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
- 77 unit tests passing across both packages (pure-Python algorithm logic: fusion, drift, traversability, PRM, walkability — no ROS/Gazebo needed to run them).

Known limitations / what we can't do yet

- Only tested in a plain maze/room world (room_maze.world, ported from d3), not the custom construction-site world — that integration is still on hold. The construction site exists and its elevation is verified correct, but no UGV/PRM navigation has been run on it.
- Single UGV, single fixed goal per run — goal_x/goal_y is one static target passed at launch, not a mission/task queue, and any custom goal has to be manually checked against a live-flown map first (no automatic "is this point reachable" pre-check) — a bad default goal near a wall has actually bitten us once already.
- No true multi-robot coordination — the UAV patrol and UGV path planning don't communicate beyond "the UAV happens to have scanned enough of the room." There's no active "go scan wherever the UGV wants to go next" behavior.
- A rolled-over/crashed UAV has no recovery — if it tips past ~90°, no controller logic can right it; this is mitigated by spawning near-ground and starting the autopilot early, not solved in general.
- Elevation mapping has no object/robot segmentation — a parked UGV can genuinely get fused into the map as fake terrain; we only work around the resulting planning deadlock (via footprint-clearing), we don't prevent the map corruption itself.
- Fuel/network dependency risk was designed around, not eliminated — all construction-site assets had to be vendored locally because live Fuel downloads stall in this sandbox; any new asset would hit the same problem.
- A known cosmetic Nav2 log warning (Failed to create a plan from potential when a legal potential was found) recurs periodically without blocking movement — never root-caused since it isn't actually breaking anything, but not explained either.
- GPU fusion's speedup is modest (~25% at realistic point-cloud sizes), not a dramatic scaling win — CPU path is still the default/reference implementation.

Repo state

Working tree is clean; recent commits since my last tracked session (not made by me) added unit tests for nav/gridmap utilities, the tower crane model + construction-site world, a renaming pass, and an FBX→SDF conversion script — all consistent with what's documented above, nothing conflicting found.