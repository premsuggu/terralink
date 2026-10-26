# Step 09 - Four stress-test worlds for the A* planner (built, run once each on a complete map; hidden interiors NOT solved)

**Update 2026-10-26:** a fifth world, W5 "door gauntlet" (8 doors, 1.2 ... 0.5 m, walls only), was added in the footprint follow-up, and the four worlds were re-run live with the new Jackal-class UGV and costmap: see `step10_jackal_ugv_model.md` sections 6.2-6.5. Every number below was measured with the OLD 0.3 m box UGV.

**Status (2026-10-17):** worlds, ground-truth maps and 3D review pages are built, unit-tested and approved by the user. Each world has had ONE live run with A* given a complete map (sections "First run" and "Runs of W3, W1 and W4": all reached their goals, videos in `media/runs/`). Those runs do NOT test discovering an interior the UAV cannot see; that experiment is the last section ("Hidden interiors") and is **not solved live**. Reading order: this header -> "First run" -> "Runs of W3, W1 and W4" -> "Hidden interiors".
**Why:** all planner evidence so far is from two small, tame worlds (maze, tunnel) with one goal each. `construction_site.world` (emap step 12) is too simple to count (a pit, a mound, a few props, no UGV/Nav2 setup). We need worlds that look and behave like real sites and contain the cases that break planners: clutter, narrow passages, slopes, dead ends, sealed spaces, overhead decks.
**Scope decision (user):** we test the *planner* (A*), so the UAV scan is skipped: the planner gets a **ground-truth 2.5D map** computed from the world itself. The voxel map stays a UGV-side concern for later.

---

## 1. What was built

| World | Size | Source of realism | What it stresses |
|---|---|---|---|
| **W1** `w1_house_site` | 100 x 100 m yard (map 120 x 120 m) | the CC0 *Majadroid LowPoly House Construction Site* pack, its own layout: 55 x 45 m building, 2 tower cranes, trucks, container stacks, material piles, two spoil mounds, fence with two openings (~19 m east, ~9 m west) | long routes, one huge solid obstacle, clutter of every size, a 1200 x 1200 cell map (~39x the 160 x 160 map A* was profiled on) |
| **W2** `w2_container_yard` | 60 x 60 m | pack containers/trucks/piles + concrete barriers | lanes of 2.5 / 1.3 / 0.8 / 0.5 m, chicane (1.2 m gaps), dead-end alley, sealed 6 x 6 m pocket (infeasible goal), wall with 0.35 m and 0.9 m gaps, 4 m gate |
| **W3** `w3_excavation_slopes` | 60 x 60 m terrain mesh | procedural terrain + pack trucks/barrels | 3 m pit entered only by a 12 m ramp (slope 0.25 = difficult), lethal pit walls, 3 mounds (easy ~0.16 / difficult ~0.28 / lethal ~0.47 with an unreachable flat top), rubble field (mostly difficult), berm with one saddle |
| **W4** `w4_structure_decks` | 50 x 50 m | primitives + pack props | 32 x 24 m structure under a deck at z = 3.1-3.5 m, 4 x 3 rooms with doors of 1.2 / 1.0 / 0.7 / 0.5 m, dead-end rooms, a sealed room, a half-blocked door, a scaffold-covered walkway; **UAV view hides everything under the deck** (the occlusion problem) |

**Review files** (open with the project's own viewer, `media/3d_maps/viewer.html`: *Open .ply File* or drag and drop; nothing else is generated in `media/`):

| File | What it is |
|---|---|
| `media/worlds/<w>_world.html` | the world as a plain, self-contained 3D page (orbit/pan/zoom, no panels; three.js from a pinned CDN) embedding the real triangle meshes of the scene with their palette colours and simple lighting - not a point cloud (`worldgen/html.py`), over a ground plane. Everything that exists is there, including crane jibs and decks. |
| `media/3d_maps/<w>_elevation_uavview.ply` | **the planner map (UAV view)**: one point per grid cell at its elevation, coloured by the project-wide convention from `nav/ply_export.py`: **green = walkable/easy, yellow = difficult, red = lethal/blocked**, grey = unobserved. Identical colours and legend for every world (and for maps exported from live runs). W1's grid is block-reduced 2x for display. |

Both are in world coordinates (Z up). The map PLY carries a `comment point_spacing` header line that `viewer.html` uses to pick a point size.

**Decision (user, 2026-10-15): A* is given the UAV-view map**, so only that map is exported to the viewer. The oracle variant stays as data (`src/nav/maps/<w>_groundtruth.npz`, sees under decks) for diagnostics only. Earlier review artefacts (Gazebo top-down PNGs, ground-truth figures, HTML viewers, feasibility text files) were removed as redundant.

## 2. Concepts

**One description, three outputs.** A world is a Python module (`src/nav/scripts/worldgen/worlds/wN_*.py`) that places things into a `Scene`. The same `Scene` writes (1) the `.world`, (2) the ground-truth map, (3) the images. The map and the simulated world therefore cannot drift apart (`scene.py`: `world_sdf`, `ground_truth`).

**Ground-truth map = what an ideal overhead depth sensor would measure**, with one choice: surfaces whose lowest point is at or above a **ceiling** (default 3.0 m) are "overhead" and dropped (a crane jib 50 m up, a deck over a corridor). A wall that starts on the ground is kept at its full height. Two variants are written for every world:

* **Oracle** `<w>_groundtruth.npz` (ceiling 3 m): sees *under* decks. In W4 this makes the rooms, doors and sealed room visible to the planner, i.e. a plain maze.
* **UAV view** `<w>_uavview.npz` (ceiling 5 m = just above the live patrol altitude, 3.0-4.5 m in `tunnel_demo`/`nav_sim`): what the UAV camera would see. Decks and walkway roofs are solid, so in W4 everything under the deck is *hidden* (every goal inside the structure looks blocked) and the occlusion problem is back - the UGV's voxel map would have to resolve it. Crane jibs 40 m up are still excluded (a UAV flying at 5 m never sees them). In W1/W2/W3 the two variants are nearly identical (W1: the building interior shows as a ~3.9 m plateau under the first floor slab; both are ringed by the same lethal walls).

**Which map A* gets is an open choice for the user** (recommended: UAV view, because the voxel map is meant to be the UGV's job).

**Traversability is not re-implemented**: `emap.traversability.compute_traversability` is called with the live node's thresholds (`max_slope 0.35`, `max_step 0.15`, `max_roughness 0.05`, resolution 0.1 m).

**The rasteriser is conservative** (`worldgen/raster.py`) because this grid is what A* is judged against: (a) fill at 4x the target resolution, then max-pool, so a 4 cm wall still occupies its 10 cm cells; (b) steep/vertical triangles also draw their plan-view edges (a vertical fence panel has zero plan area and would otherwise vanish). Tested in `tests/nav/test_worldgen.py::TestRasterCoverage` and `test_fence_geometry_is_never_dropped_by_the_rasteriser`.

**Effective door width.** The lethal band eats ~0.15 m per side (conservative edge + 3x3 step test). Measured: largest passable robot radius = gap/2 - 0.15 m (0.5 m gap -> r 0.1; 0.9 -> 0.3; 1.2 -> 0.4). Design gaps accordingly; our UGV is ~0.5 m wide.

## 3. Pipeline and files

```
src/nav/scripts/build_world.py <world>        # builds everything for one world (ROS sourced)
src/nav/scripts/worldgen/
    fbx.py          binary-FBX reader (no Blender/trimesh/assimp binding on this machine, Fortress cannot load FBX)
    assets.py       pack -> OBJ+MTL+palette PNG model folders (texture next to the OBJ, bare filename - emap step 12 lesson)
    raster.py       conservative top-down rasteriser (height grid + colour)
    scene.py        Scene: add_asset / add_box / add_terrain, world_sdf, ground_truth
    layout.py       wall / seg_wall / rect_walls (door gaps) / place / export_props
    feasibility.py  independent "is any route possible at robot radius r?" check (flood fill on clearance)
    ply.py          PLY export of the world and of the planner map (for media/3d_maps/viewer.html)
    worlds/         w1_house_site.py  w2_container_yard.py  w3_excavation_slopes.py  w4_structure_decks.py
outputs: media/worlds/<w>_world.html  media/3d_maps/<w>_elevation_uavview.ply  src/nav/worlds/<w>.world   src/nav/models/site_*/ , w3_*_terrain/   src/nav/maps/<w>_uavview.npz (the planner map) , <w>_groundtruth.npz (oracle)
```
Maps use the nav snapshot format (`layer_elevation/variance/is_valid/traversability`, `resolution`, `center_x/y`), so `replay_plan.py`, `replay_sequence.py` and `map_replay_node` take them unchanged. `setup.py` now also installs `maps/`.

Rebuild: `source /opt/ros/humble/setup.bash && python3 src/nav/scripts/build_world.py w2_container_yard` (3-12 s per world). The pack is read from `tests/worlds/gazebo_world_conversion/...` (CC0; credit in `src/nav/models/SITE_PACK_LICENSE.md`).

## 4. Design-intent feasibility (flood fill on the ground-truth map, from `S`)

`yes` = a route exists for a robot of that radius (r = 0.15 / 0.25 / 0.35 / 0.45 m). This is *not* the planner - it is the check that the worlds contain the cases we meant them to.

| World | Reachable at every radius | Only small robots | Never reachable (infeasible goal: A* must say "no path") |
|---|---|---|---|
| W2 | A open yard, B chicane, D dead-end alley floor, F beyond the wall (0.9 m gap) | C (0.8 m lane): r <= 0.25 | G (0.5 m lane), E (sealed pocket) |
| W3 | A pit floor (ramp), C rubble, D beyond berm (saddle), E gentle mound, F medium mound | - | B (top of the steep mound) |
| W4 | A far room, D under walkway, E outside east, F SE room | B dead-end room (1.0 m door): r <= 0.35 | C (sealed room) |

These tables are for the **oracle** map. On the **UAV-view map (the one A* gets)** W4 is mostly blocked - A, B, C, D, F all lie under the deck/walkway roof and are unreachable; only E (outside) stays reachable - while W1-W3 are unchanged. So W4 scenarios under the deck can only be solved with the UGV's own voxel map, by design. Pinned by `test_generated_world_is_valid_and_matches_its_design`. W1 has no marker set yet (scenarios to be chosen with the user).

**How this relates to A*'s inflation.** `planner_node` inflates obstacles by `robot_radius_m 0.22 + inflation_margin_m 0.05 = 0.27 m`, and each pass retries at the bare 0.22 m. So "only small robots" cases (W2-C passes at r <= 0.25, W4-B at r <= 0.35) are passable for A* **only in the relaxed 0.22 m pass** (W2-C) or in the full pass (W4-B); W2-G (0.5 m lane, r <= 0.1) is closed even relaxed. Expect W2-C to be where the strict/relaxed fallback shows up.

## 5. Pitfalls found while building (all fixed)

1. **FBX part files are a prop library**, every part authored at the origin; the placed layout exists only in the `_AllInOne` file. W1 uses the all-in-one, W2-W4 place parts themselves.
2. **ASCII PLY must stay small**: the viewer parses it in the browser, so the world file is sampled to ~450 k points (the first attempt, 2.5 M points / 104 MB for W1, was unusable) and W1's map is block-reduced.
3. **A ground plane at z = 0 hid and blocked W3's pit** (visually and as collision). `Scene.ground_z` now sits below the terrain (-3.5 m); W1 uses -0.3 m because its roads are at -0.2 m.
4. **Feasibility check caught three design errors** (before anyone saw an image): W2's dead-end alley overlapped a closed container so goal D was sealed in; W2's goal C was in a cross-gap, not in the 0.8 m lane; W4's "behind the 0.7 m doorway" room was reachable by other routes.
5. **Flat tops are islands.** Container roofs, the building roof and mound tops are flat, so they classify as EASY plateaus ringed by a lethal band - exactly what a real emap map would show. Harmless for A* from the ground, but it is a likely trigger for the island-based anomaly detector (nav step 06 v3); watch for false "tunnel" flags in W1/W2.

## 6. Not done / decisions for the user

* **No runs.** Next: choose scenarios per world (W2-W4 `MARKERS` are proposals; W1 has none), then offline A* sweeps on the saved maps (`replay_plan.py`), then live UGV runs. Live runs also need per-world launch files (UGV spawn on a free cell, ground-truth `OdometryPublisher`, `bridge.yaml`, Nav2 map handling, **identity spawn transform** - the two bugs of step 08) and a check that the small UGV can physically drive the slopes (its real climb limit is unmeasured; map `max_slope` 0.35 is a planner threshold, not a measured vehicle limit).
* **Collision cost is unmeasured.** Pack meshes (building ~10 k, trucks ~2 k triangles) are used as collision meshes; Gazebo real-time factor with the UGV on them has not been measured.
* **Ceilings (3 m oracle, 5 m UAV view)** are modelling choices (the UAV one is tied to the real patrol altitude); each world module has `CEILING` and `UAV_CEILING`.
* W1 stays faithful to the pack; extra obstacles (barriers, chicanes) can be added if it proves too easy.
* A two-level overpass (a known 2.5D limit, roadmap item 10) is intentionally not included.

---

## First run: W2 container yard, A* on the UAV-view map (2026-10-15)

**Offline sweep** (`replay_plan.py`, start S = (-3, -26), map `src/nav/maps/w2_container_yard_uavview.npz`). A* agrees with the independent feasibility flood-fill on every goal:

| goal | what it tests | A* result |
|------|---------------|-----------|
| A open yard | baseline | route 31.6 m (straight 28.8 m) |
| B through the chicane | tight turns | 24.2 m |
| C inside the 0.8 m lane | narrowest feasible passage (needs the relaxed 0.22 m retry) | 45.8 m, 8 waypoints |
| D dead-end alley floor | dead end | 8.3 m |
| F beyond the near-blocked wall | gaps 0.35 / 0.9 m | 28.0 m |
| G inside the 0.5 m lane | too narrow | **no path** (correct) |
| E sealed pocket | unreachable | **no path** (correct) |

**Live run, goal C** (the hardest feasible one: 46 m through the chicane, past the near-blocked wall and into the 0.8 m lane). One run is enough: the plan is deterministic (the 8-waypoint route was identical in all three live runs; driving time varied 93-113 s because Nav2's controller is not deterministic).
Result: reached the goal (final error 0.15 m), 46.6 m driven vs the 45.8 m A* route, no recoveries needed. Video: `media/runs/w2_container_yard_run.mp4` (96 s, real-time playback).

How a live run works (all new, in this step):
* `build_world.py` also writes `worlds/<w>_run.world` = the world + the UGV at marker S (OdometryPublisher = true pose, DiffDrive) + a 150 m-high straight-down camera covering the map (`worldgen/run_world.py`).
* `tunnel_demo.launch.py` got two args: `world:=<file in share/nav/worlds>` and `nav2_params:=<file in share/nav/config>`. `nav2_params_large.yaml` is `nav2_params.yaml` with a 130 x 130 m global costmap (the default 20 x 20 m would not contain the goals).

```bash
source /opt/ros/humble/setup.bash && source install/local_setup.bash
python3 src/nav/scripts/record_world_run.py --out /tmp/w2_run --goal -12 13.8 &      # start FIRST so no motion is missed
ros2 launch nav tunnel_demo.launch.py world:=w2_container_yard_run.world nav2_params:=nav2_params_large.yaml \
    headless:=true autonomous_uav:=false planner_type:=astar \
    map_snapshot:=$PWD/src/nav/maps/w2_container_yard_uavview.npz goal_x:=-12.0 goal_y:=13.8
# after the recorder prints "recorded ..." (it stops itself shortly after arrival):
python3 src/nav/scripts/make_run_video.py --run /tmp/w2_run --world w2_container_yard --start -3 -26 --goal -12 13.8 \
    --out media/runs/w2_container_yard_run.mp4 --title "W2 ..."
```

Video layout (`make_run_video.py`): left = whole world from the overhead camera with the A* route (yellow), path driven (cyan), start, goal; top right = 14 m window following the UGV; bottom right = the UGV's own forward camera (real footage). The software renderer delivers only ~2 overhead frames per simulated second, so the robot is drawn to scale at its 30 Hz ground-truth pose on a "clean plate" (median of the overhead frames with the robot masked out) - the world and the UGV camera are real renders, the UGV marker on the overhead panels is an overlay. Frames are stamped with simulation time, so playback speed is exact.

Findings: (1) the default Nav2 global costmap (20 x 20 m) cannot hold the stress worlds - needs the large variant; (2) a recorder started after launch misses the first ~12 s of motion; (3) a 70 m camera shows visible parallax on 2.6 m containers (route looks shifted against roofs) - 150 m removes it.

---

## Runs of W3, W1 and W4 (2026-10-15)

Same recipe as W2 (offline sweep, then ONE live run + video; driver: `src/nav/scripts/run_world_scenario.sh <world> <sx> <sy> <gx> <gy> "<title>" [uavview|groundtruth]`). All three reached the goal. Videos in `media/runs/`.

| world / goal | map given to A* | A* route | driven | sim time | final error | video |
|---|---|---|---|---|---|---|
| W2 C (chicane + 0.8 m lane) | UAV view | 45.8 m | 46.6 m | 93 s | 0.15 m | `w2_container_yard_run.mp4` |
| W3 A (down the ramp into the pit) | UAV view | 20.6 m | 26.9 m | 45 s | 0.15 m | `w3_excavation_slopes_run.mp4` |
| W1 A (105 m around the building) | UAV view | 105.1 m | 105.2 m | 189 s | 0.17 m | `w1_house_site_run.mp4` |
| W4 B (under the deck, dead-end room) | **oracle** (see below) | 72.1 m | 76.0 m | 198 s | 0.26 m | `w4_structure_decks_run.mp4` |

**Offline sweeps** (all agree with the feasibility tables): W3 from S=(4,-6): A pit floor 20.6 m, C rubble field 33.5 m, D beyond the berm via the saddle 28.8 m, E 24.0 m, F 17.1 m, B steep mound top **no path**. W1 from S=(-40,5) (markers added in this step): A (30,-45) 105.2 m (detour x1.22), B inside the building and C outside the double fence **no path** (both correct; the building's ring wall and the fences are closed). W4 on the UAV-view map: every interior goal **no path** (under the deck), only E reachable.

**W3 detail.** A* planned a straight 2-waypoint line down the middle of the ramp (slope 0.25, class "difficult"). The UGV climbed down fine, but Nav2 (which re-plans on its own lidar costmap between waypoints) took the south edge of the ramp: 26.9 m driven vs 20.6 m planned. The elevation along its track runs smoothly 0 -> -3.0 m. So the physical ramp limit is not reached at 0.25; the steeper "lethal" ground (mound B, slope > 0.35) was not driven.

**W1 detail.** 13-waypoint live route vs 11 offline (the live start pose differs slightly); driven length matches the plan within 0.1 m. The run is slow in wall time (25 min for 189 sim s) because of the 120 m map, 44 k-triangle collision meshes and a 2000 px camera - the video is real-time because frames are stamped with simulation time.

**W4 caveat (important).** On the UAV-view map (the map A* normally gets) the interior is unreachable by design, so this run gives A* the **oracle map** - the map the UGV's own sensors would produce once it has seen under the deck. It tests planning + driving through 1.2 m / 1.0 m doorways and corridors (72 m, detour x4.25) but NOT the UAV-then-UGV mapping handover; that is the voxel-map step. The run world draws the deck 80 % transparent (visual only; `RUN_SEE_THROUGH` in the world module, `see_through_kinds` in `Scene.world_sdf`) so the overhead camera can show the UGV under it.

Findings: (1) goals must be passed to the follower as floats (`30` -> `30.000`, the ROS parameter is a double; the driver script formats them); (2) Nav2 sometimes drives a longer path than A* gives it (W3: x1.3) - A* only hands over waypoints, Nav2's costmap decides the local path; (3) `OdometryPublisher` with `dimensions 2` gives z = 0, so the pose log has no height.

---

## Hidden interiors: the real-pipeline experiment (2026-10-16) - NOT solved yet

The runs above gave A* a complete map. For goals INSIDE a roofed structure (W4's rooms under the deck) that is not a test of the framework: the UAV sees only the roof, so the planner must discover the interior with the UGV's own sensors. This section is that experiment: what was built, what was found, and where it stands. **Update 2026-10-26:** a fifth world, W5 "door gauntlet" (8 doors 1.2 ... 0.5 m, walls only), was added in the footprint follow-up - see `step10_jackal_ugv_model.md` section 6. All runs below used the OLD 0.3 m box UGV; the four worlds were re-run with the Jackal-class model (numbers in step10 section 6.5).

**Status: works in the simulator, not yet live (W4 goal B never reached).**

### What the first real-pipeline run showed
Real UAV scan (patrol, `emap`, no map handed over) + A* + frontier/anomaly exploration + voxel verification: the UGV never reached goal B in 900 simulated seconds. Reason: the UAV map shows the 800 m^2 deck as one flat "walkable" island fenced by a lethal ring, with NO unknown cells (so frontier exploration finds nothing), and the anomaly detector only flags small islands (a tunnel roof). The planner chased meaningless frontiers on the map edge. A 25-minute run is a bad way to find that out, hence the offline simulator below.

### What was built
| Piece | File | Idea |
|---|---|---|
| Hidden-island rule | `nav/overhead.py` (`goal_island_zone`, `compose_with_floor`, `GoalIslandPlanner`) | If the goal (or the robot) is on an elevated island - a connected walkable component >= 1.5 m above the ground around it, >= 3 m^2 - the UAV map there is not trusted: the island is "unknown" until the UGV has seen it. Only the goal's / robot's island, because a container roof looks identical from above. No "confirmed route" shortcut: a UAV-map route into an island squeezes over a 1-cell lethal ring and is physically impossible. Anomaly targets are off in this mode. |
| UGV floor map | `nav/floor_map_node.py` (`integrate_scan`) | Lidar log-odds occupancy on the UAV map's grid -> `/ugv_floor_map`. The lidar (360 rays, 0.3-12 m), NOT the depth camera: that is tilted down and only sees a 0.1-0.9 m strip of floor ahead. Returns nearer than 0.3 m (-inf) are ignored. |
| Planner hook | `planner_node.py` params `enable_overhead_islands`, `uncertain_waive_m`, `viewpoint_standoff_m`, `viewpoint_block_m`, `stuck_progress_window_sec` | launch args of the same names in `tunnel_demo.launch.py` (+ `overhead_islands`) |
| Exploration changes | `nav/exploration.py`, `global_planner.py` | `viewpoint_standoff_m`: look at unknown space from >= 0.8 m (the lidar is blind inside 0.3 m); `viewpoint_block_m`: one viewpoint per 3 m block of a long frontier (a frontier round a whole building is ONE connected cluster, one visit blacklisted all of it); `mark_failed`: blacklist an unreachable target |
| Stuck detection | `nav/stuck_feedback.py` | new "no progress toward the target for N s" test: the old one needs < 0.25 m of movement, but the UGV jitters +-0.7 m in front of a door it cannot pass |
| Offline simulator | `tests/nav/explore_sim.py` | UGV with the node's own lidar integration against a ground-truth map, same planner classes; ~10-40 s per goal instead of a 40-minute live run. Models the 0.3 m blind zone, a 0.27 m body radius and ~10 s overhead per new goal |
| Live tooling | `scripts/run_world_scenario.sh` (+ `EXTRA` launch args), `run_world_mission.sh` (UAV scan + mission), `record_world_run.py`, `make_run_video.py` | `<w>_scan.world` = run world + UAV and `_patrol.txt` (`worldgen/run_world.py: lawnmower`) |

### Simulator results (W4, ideal UAV map `w4_structure_decks_uavview.npz`, true floor in `_groundtruth.npz`)
| goal | result | driven | time incl. overhead |
|---|---|---|---|
| A far room | reached | 109 m | |
| B dead-end room (oracle route 76 m) | reached | 120 m | 560 s |
| D under the walkway | reached | 28 m | |
| E outside | reached | 62 m | |
| F SE room | reached | 81 m | |
| C sealed room | not reached (correct): explores 306 m, then gives up | 306 m | |
`blocks` of 6 and 10 m instead of 3 m made B fail in the simulator (not understood yet).

### Live results (Gazebo, same map, goal B) - all NOT reached
| run | what happened |
|---|---|
| real UAV scan + camera floor map (900 s) | ended outside, 29 m away |
| lidar floor map, #4-#6 (500-900 s) | sweeps the walls in 3 m steps (15 s each: 0.2 m/s instead of 0.6, ~10 s overhead per new goal), enters the east door at ~340 s, then dithers 2 minutes in front of a 0.5 m door (too narrow) at (8, 9); `uncertain_waive_m 0` did not change it |
| #7, + progress stuck detector | dithers back and forth along the north wall instead; stopped at 475 s |

### Bugs found on the way (all fixed unless noted)
* emap drift compensation slid the map height by ~0.3 m over a long patrol (`drift_compensation:=false`).
* `voxel_map_node` grew to 10 GB during a UAV scan: it inserted every point, now only within its radius of the UGV; it also centred on wheel odometry (starts at 0,0 wherever the UGV spawns), now `/ugv/odom_ground_truth`.
* `max_roughness` compares the height VARIANCE (model 0.01 x range^2 per observation) to 0.05: a cell seen once from 6 m reads 0.36 = "lethal", although the simulated depth noise is ~6 mm. Scans of the stress worlds use `max_roughness:=0.5`. (Not a fix of the model - a limit of the scan-quality of the current emap.)
* Scans: 6 m altitude with 2 m lanes; flying higher/thinner gave an unusable map (57 % of the ground "lethal"). `emap_point_stride:=16` speeds the CPU fusion.
* A stale `floor_map_node` from an earlier run kept publishing next to the new one (kill lists in the run scripts did not include it).
* The depth camera cannot map interiors (see above); a viewpoint touching a wall is invisible to the lidar.

### Honest assessment
Planning logic works offline; live it does not yet. What is missing: (1) the UGV spends ~10 s per exploration goal and explores the facade in 3 m steps -> minutes of wall-hugging before it finds a door; (2) Nav2's local planner and the global map disagree about narrow doors (a 0.5 m door is "free" on the map, impassable for the robot) and nothing converts a repeated local failure into "this gap is closed" except the new stuck detector, which in turn made the target choice oscillate; (3) target choice has no hysteresis between far-apart blocks. Next steps, in order: commit to an exploration target until reached or declared unreachable; mark a gap closed after a Nav2 failure; fewer, longer exploration legs (the lidar sees 12 m). Rerun with `scripts/run_world_scenario.sh` (EXTRA args listed in this section's launch args).
