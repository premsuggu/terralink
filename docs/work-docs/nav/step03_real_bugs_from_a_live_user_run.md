# Step 3: Four real bugs found from an actual user run

**Package**: `src/nav/`
**Goal**: fix why `nav_ugv` never moved when you actually ran `ros2 launch nav nav_sim.launch.py`.
**Status**: four independent, confirmed bugs found and fixed, each verified against the live running system (not assumed) - ending in the UGV actually driving under Nav2 control.

---

## What "doesn't move at all" actually was

Your `cli.txt` showed `planner_node: get_plan: no path found between the requested start and goal` repeating continuously, alongside `Message Filter dropping message: frame 'nav_ugv/base_link/lidar' ...`. These pointed at two of four real, independent problems that all had to be fixed together before anything would move - not one bug, four:

### Bug 1: `nav_ugv` spawns directly on a non-walkable cell

`(-0.7, -3.0)` (the original spawn pose, picked by eyeballing the wall model's own reference pose rather than actually checking against a flown map) reads `traversability=LETHAL` - close enough to an interior wall corner (Wall_18/Wall_28) for a real slope/step-height reading. `planner_node.py` always plans a route **from the UGV's own current position** - and `prm_planner.plan` deliberately rejects a query whose start cell isn't walkable (see that module's docstring on why: silently snapping to the nearest walkable point would be worse - see step01). Since the UGV can only ever move via a valid plan, and no valid plan can ever start from a rejected cell, this was a permanent deadlock, confirmed live: `walkable((-0.7,-3.0)) == False` after a real flown map.

**Fix**: moved the spawn to `(-0.7, -4.5)` - confirmed live to be walkable with a full ~0.7m clear margin in every direction, not just the exact point. `worlds/room_maze.world`'s `<include><pose>` and `launch/nav_sim.launch.py`'s `ugv_odom_static_tf` translation both had to change together (documented as a single source of truth in both places).

### Bug 2: a missing static transform for the lidar

Echoing `/scan` directly showed `frame_id: nav_ugv/base_link/lidar` (Ignition's auto-derived `<model>/<link>/<sensor>` naming) - but no transform from `nav_ugv/base_link` to that frame was ever published. Nav2's costmaps had nowhere to place the lidar's own points relative to the robot, so every single scan was silently dropped forever (`Message Filter dropping message... timestamp earlier than all the data in the transform cache` - a misleading message: the real problem was a **missing** transform, not a stale one). The exact same gap `camera_static_tf` already fixes for the UAV's camera - just missed for the UGV's lidar.

**Fix**: a new static transform, `nav_ugv/base_link -> nav_ugv/base_link/lidar`, matching the lidar `<sensor>`'s own `<pose>` in `models/nav_ugv/model.sdf` (`0.1 0 0.095`, no rotation).

### Bug 3: the installed Nav2's final `cmd_vel` output would have collided with the UAV's

Checked directly against the **installed** `nav2_bringup`'s `navigation_launch.py` (not d3's own vendored copy, which turned out to be an older/different version): it remaps `velocity_smoother`'s output straight to the literal, unnamespaced topic `cmd_vel` - `[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]`. That would collide directly with `emap_bridge`'s own `/cmd_vel` (the UAV's command topic) - the UGV's velocity commands would either vanish into the UAV's controller or the two would fight over the same name. Confirmed the installed file has no `namespace=`/`PushRosNamespace` actually applied to its nodes either (its `namespace` launch argument only feeds `RewrittenYaml`'s root key, not real topic namespacing), so passing a namespace wouldn't have fixed it. Confirmed fixed live too: every Nav2 process now shows `-r /cmd_vel:=/nav2_cmd_vel` in its own argv.

**Fix**: `launch/nav_sim.launch.py` wraps the Nav2 `IncludeLaunchDescription` in a `GroupAction` with `SetRemap(src='/cmd_vel', dst='/nav2_cmd_vel')` - the standard, documented way to redirect a topic used inside a launch file you don't own, without forking it. `nav/config/bridge.yaml` bridges from `/nav2_cmd_vel` accordingly.

### Bug 4 (found while testing the fix for the other three): a UAV that free-falls can tip over and get physically stuck

Discovered while live-testing bugs 1-3's fix with `autonomous_uav:=true`: the UAV started, but ended up **86m outside the room**, having flown off in a completely wrong direction. Its odometry showed why: `orientation` was `(0.87, 0, 0, 0.49)` - roughly **120 degrees of yaw**, not level. Root cause, in two parts:

- `MulticopterVelocityControl` gives the UAV zero thrust until something actually commands a velocity (documented in the README's "Controlling the UAV" section) - so however many seconds pass before the autopilot node starts, the UAV free-falls uncontrolled from its spawn altitude.
- `nav.autopilot.compute_hold_command` computes velocity in WORLD frame, but the plugin expects BODY frame. Every prior manual `/cmd_vel` test in this whole project happened to work anyway, purely because the vehicle was always level (yaw=0) when commanded - a coincidence the autonomous demo mode broke for the first time, since a bad landing can leave it rotated.

Chasing the fix further surfaced something worse: after fixing the frame conversion and shrinking the free-fall window, one run still left the UAV **completely stuck**, orientation showing a ~90 degree **roll** (resting on its side), position frozen no matter what was commanded. A rolled-over multirotor's own thrust axis no longer points anywhere near "up" - no controller, however correct, can fly out of that. The real, structural fix had to be upstream of any control logic: **don't let it free-fall far enough to tip over in the first place**. Every other `emap` world already spawns `iris_quad` right at/near ground level (z=0.1) for exactly this reason - `room_maze.world` had deviated from that already-proven-safe convention (spawning at z=3.5 "for room to see the maze"), which is what made a violent tip-over possible at all.

**Fix, in three parts**:
1. `nav.autopilot.world_to_body_xy` - a proper world-to-body-frame rotation using the UAV's live yaw (not computed once, but every control tick, so it stays correct even through a momentarily-rotating recovery).
2. `launch/nav_sim.launch.py`: `uav_autopilot_node` now starts alongside the bridges (t=5s) instead of waiting for the whole Nav2 stack (t=9s) - it only ever needed `/odom`/`/cmd_vel`, nothing Nav2-related - shrinking the uncontrolled window as much as the launch structure allows.
3. `worlds/room_maze.world`: `iris_quad`'s spawn altitude restored to `z=0.1`, matching every other `emap` world. A fall of a few centimeters can't generate enough energy to tip the vehicle over, so there's nothing left to recover from regardless of timing - `autonomous_uav`'s autopilot then climbs to altitude itself, under **controlled** flight, which is what actually delivers "start it in the air" safely (an uncontrolled mid-air spawn isn't actually safer than a low one - the fall happens either way if nothing's holding it up).

4 new unit tests (`test_world_to_body_xy`) verify the rotation directly - including the exact 90-degree case that would have caught this before it ever ran live. 75 tests total now pass.

## A fifth thing, not a code bug: leftover processes

While reproducing this, several **stale processes from earlier test runs** (including a full previous `ros2 launch nav nav_sim.launch.py` that never fully exited) were found still running and actively fighting a fresh launch - e.g. an old `static_transform_publisher` still publishing the pre-fix `(-0.7, -3.0)` spawn offset alongside the new, correct one, with `tf2` resolving to whichever one it happened to receive first. If a launch is ever interrupted uncleanly, check for leftovers before the next run:

```bash
pgrep -af "ros2 launch|ign gazebo|parameter_bridge|static_transform_publisher"
```

and kill anything unexpected before relaunching.

## Also fixed alongside these: a retry busy-loop

`prm_waypoint_follower` re-requested a plan on **every single odom message** while waiting (nav_ugv's odometry publishes at ~30Hz), so a genuinely-not-yet-available plan produced dozens of `get_plan`/log lines per second - which is what made the original failure look like a crash loop rather than "still waiting for the map." A `retry_interval_sec` cooldown (default 2.0s) now throttles this.

## Live verification - the UGV actually drives

`ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true goal_x:=1.0 goal_y:=-1.0`, from a verified-clean process state, with all four fixes and Nav2 installed:

- No more `Timed out waiting for transform` spam (step 2's staggering fix holding up), no more `Message Filter dropping message` spam (bug 2 fixed).
- The UAV climbed cleanly from its safe near-ground spawn and settled at `(-0.79, -3.00, 4.43)` - the first patrol waypoint - with orientation essentially identity (`w≈1`, everything else ≈0). No tip-over, this time or on retest.
- `get_plan: found a 4-waypoint path` - a real plan, once the UAV had scanned enough of the area.
- `prm_waypoint_follower: Reached waypoint 0 / 1, publishing next` - the sequencer advancing correctly.
- `controller_server: Received a goal, begin computing control effort` - Nav2's DWB controller actually engaged.
- `nav_ugv`'s own odometry sampled three times in a row: `(2.27, 3.36) -> (1.39, 3.90) -> (1.29, 3.96)` - **real, continuous physical motion**, not a stationary robot with a satisfied planner.

One remaining, lower-priority rough edge: `planner_server: Failed to create a plan from potential when a legal potential was found. This shouldn't happen.` recurs periodically during navigation - a known Nav2/NavFn internal quirk (not something in this project's own code) that didn't block the controller from continuing to drive in this run. Worth a closer look in a future pass if it turns out to matter more once the UGV is driving longer routes, but out of scope for "the UGV wasn't moving at all," which is now resolved.
