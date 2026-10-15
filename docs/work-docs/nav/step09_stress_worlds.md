# Step 09 - Four stress-test worlds for the A* planner (built, NOT yet run)

**Status (2026-10-14):** worlds, ground-truth maps and 3D review pages are built and unit-tested. **No planner or simulation run has been done on them yet** - that waits for the user's review of the images in `media/worlds/`.
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
