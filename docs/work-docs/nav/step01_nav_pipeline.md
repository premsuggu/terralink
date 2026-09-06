# Step 1: UAV-mapped elevation → walkable classification → PRM path → UGV follows

**Package**: `src/nav/`
**Goal**: replace `src/d3/my_bot`'s hardcoded pixel-color walkability classification with `emap`'s own traversability layer, keeping d3's proven PRM planning and Nav2-based low-level following otherwise.
**Status**: mapping → classification → planning is built, live-verified end to end (a real PRM path was returned from a live-flown map). The final low-level following stage (Nav2/DWB → `/cmd_vel_smoothed` → UGV motion) is wired but not yet live-verified - Nav2 isn't installed in this environment and the install is pending on the user (see below).

---

## 1. The design question: how to classify walkable space

Discussed explicitly before building anything (see the two `AskUserQuestion` exchanges this step's conversation opened with):

- **Walkability = `is_valid & (traversability != LETHAL)`, not an elevation threshold.** `emap.traversability.compute_traversability` (step 7) already scores every cell from slope/step-height/roughness *relative to its neighbors*. An absolute-elevation rule ("flat ground = walkable") was rejected because `emap`'s own construction-site world already proved it wrong once - that world's flat ground sits at world z=+1.5, not 0.
- **Unobserved cells are excluded, not assumed safe.** `traversability.py`'s `EASY` default for `is_valid=False` cells is documented there as a fail-safe for *other* callers, not a safety claim - `nav.walkability.compute_walkable_mask` checks `is_valid` itself.

`src/nav/nav/walkability.py` is the one-line result: `is_valid & (traversability != LETHAL)`.

## 2. What was built

| Piece | File | Adapted from |
|---|---|---|
| Walkable-space classifier | `nav/walkability.py` | new (see Section 1) |
| PRM planner | `nav/prm_planner.py` | `d3/my_bot/includes/processImage.cpp`'s `GridSpace` - same algorithm (random sampling restricted to free space, line-of-sight edges, shortest-path search), ported to NumPy/SciPy (`scipy.sparse.csgraph.dijkstra`), using `emap`'s exact grid coordinate math instead of d3's fixed-altitude camera-projection assumption |
| GridMap decoder | `emap/utils/gridmap_utils.py::decode_gridmap` | new - the missing inverse of the existing `encode_layer_to_multiarray`, so `nav` has one authoritative decoder instead of re-deriving the column-major layout by hand |
| Planning service | `nav/planner_node.py` | `d3/my_bot/src/waypoints_server.cpp` - same role, standard `nav_msgs/srv/GetPlan` instead of a vendored custom `.srv` |
| Path follower | `nav/waypoint_follower.py` | `d3/my_bot/src/waypoints_client.cpp` - identical distance-threshold waypoint-advancement logic, ported to Python |
| Test world | `nav/worlds/room_maze.world` | `d3/my_bot/worlds/roomWithObstacles.world`'s wall geometry, ported onto `emap`'s own Ignition Fortress boilerplate (Classic material scripts stripped, light moved to z=500 per step 11's already-known bug) |
| UGV | `nav/models/nav_ugv/` | shape/component list from `d3/my_bot/description/robot_core.xacro` + `lidar.xacro` (chassis + 2 driven wheels + caster + planar lidar), expressed as plain Ignition SDF (matching `models/iris_quad`'s own convention) rather than URDF/xacro, since d3's actuation/sensor plugins are Gazebo-Classic-only |
| Nav2 config | `nav/config/nav2_params.yaml` | `d3/my_bot/config/nav2_params.yaml` - same DWB/costmap tuning values, frame names corrected (see Section 4) |

## 3. Two things learned the hard way, before any of this could be trusted

**Nav2 isn't installed here.** Only `ros-humble-nav2-msgs` was present - none of `nav2_bringup`/`nav2_controller`/`nav2_planner`/`nav2_bt_navigator`/etc, meaning `d3`'s own `navigation_launch.py` couldn't actually run in this environment either. The same category of discovery that made `emap` abandon porting `terralink_elevation` onto Gazebo Classic. Flagged to the user (installing the full stack is a dozen+ new packages, well past AGENTS.md's "never install without asking" threshold) with two options - install Nav2, or write a from-scratch follower instead (matching this project's own "explainable over black-box" pattern used for traversability). **Nav2 install was chosen** (`sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup`) - pending the user actually running it (no sudo in this sandbox).

**`DiffDrive`'s topic names and odom-frame semantics differ from iris_quad's plugins, confirmed live, not assumed.** Two separate findings from actually launching `room_maze.world` and inspecting `ign topic -l`/`-e`:
- iris_quad's `MulticopterVelocityControl`/`OdometryPublisher` auto-scope their topics under `/model/iris_quad/...`. `nav_ugv`'s `DiffDrive` plugin does NOT - its `<topic>cmd_vel</topic>`/`<odom_topic>odometry</odom_topic>` become the LITERAL gz topics `/cmd_vel`/`/odometry`, unscoped. A bridge built on the assumed-scoped name would have silently connected to nothing - caught by actually publishing a test `Twist` to a guessed topic and watching `/odometry` for a real position change.
- iris_quad's `OdometryPublisher` reports **world-absolute** pose under `iris_quad/odom` (confirmed: after falling from its spawn altitude and bouncing, its Y position tracked its true spawn Y to 7 decimal places, and never read anywhere near 0). `nav_ugv`'s `DiffDrive` odometry is **dead-reckoning from spawn** instead (confirmed: its very first reading was ≈(0,0) despite spawning at world (-0.7,-3.0)) - a completely normal, correct convention for a real odom frame, just a different one than iris_quad's plugin happens to use, and one that would have silently produced a planner working in the wrong frame if assumed away.

**Fix**: one static transform, `iris_quad/odom -> nav_ugv/odom`, translation = `nav_ugv`'s actual spawn pose in `worlds/room_maze.world` (`-0.7 -3.0 0.0` - both are documented as a single source of truth that must stay in sync, see `launch/nav_sim.launch.py`'s comment on the transform). This ties `nav_ugv`'s tree into the exact frame `emap`'s map already publishes in (`iris_quad/odom` - already effectively world-absolute, matching `emap`'s own `map_frame` parameter convention), rather than inventing a separate "map" frame neither package otherwise uses.

Nav2's own `cmd_vel` output chain (`controller_server` → `cmd_vel_nav` → `velocity_smoother` → `cmd_vel_smoothed`) was also traced from d3's `navigation_launch.py`/`launch_sim.launch.py` rather than assumed: d3's real final output topic is `/cmd_vel_smoothed`, arbitrated further by `twist_mux` (for joystick input `nav` doesn't need). `nav`'s bridge reads directly from `/cmd_vel_smoothed`, skipping `twist_mux` entirely.

## 4. Live verification

With `room_maze.world` running (headless) and every non-Nav2 node from `launch/nav_sim.launch.py` started individually (Nav2 itself skipped - not installed yet):

- `/elevation_map` published correctly (400x400 global map, `frame_id: iris_quad/odom`).
- `iris_quad/odom -> nav_ugv/odom -> nav_ugv/base_link` resolved via `tf2_echo`, translation exactly `(-0.7, -3.0, 0.0)` as designed.
- After flying the UAV over part of the room (a simple ascend + 4-direction sweep, ~40s, explicit stop sent - the step 11 lesson about always stopping applied from the start this time), `/elevation_map` showed 7,262 observed cells with 6,097 walkable - and, tellingly, the point (-0.7, -3.0) (right next to an interior wall corner) correctly read `traversability=LETHAL` while a point 1.2m away read `EASY` - real proof `compute_traversability` is reacting to the actual wall geometry, not coincidence.
- `ros2 service call /get_plan` between two confirmed-walkable points 3.2m apart returned a genuine 3-waypoint path (`(0.5,-3.0) → (2.7,-2.8) → (3.0,-1.0)`) - not a straight line, meaning the roadmap's sampling and line-of-sight edges actually did the work, not a degenerate direct connection.
- A request with a start point inside `traversability=LETHAL` space (immediately next to a wall) was correctly rejected (`valid=False`, matching `prm_planner.plan`'s explicit design decision to reject rather than silently snap elsewhere - see that module's docstring).
- `colcon build --packages-select nav emap` succeeds; all 60 unit tests pass (44 emap + 16 new: `test_gridmap_utils.py`, `test_walkability.py`, `test_prm_planner.py`).

## 5. What's not yet verified

The low-level following stage (`planner_node`'s path → `prm_waypoint_follower` → `/goal_pose` → Nav2's `bt_navigator`/DWB → `/cmd_vel_smoothed` → bridge → `nav_ugv` actually moving) needs Nav2 installed first. Once installed, `ros2 launch nav nav_sim.launch.py` brings up the full stack; the UAV still needs to be flown manually first to build enough map coverage before a useful plan can be requested (this milestone doesn't yet include any "explore automatically" behavior - the UAV side is still flown the same way every prior `emap` step was).
