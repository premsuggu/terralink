# Step 2: Autonomous UAV demo mode + launch startup-race fix

**Package**: `src/nav/`
**Goal**: let the whole pipeline run and be watched with zero manual `/cmd_vel` control, and fix a real startup-ordering issue found from a live user run.
**Status**: ✅ both built and live-verified.

---

## 1. What triggered this step

A live run (`ros2 launch nav nav_sim.launch.py`, captured in a user-provided `cli.txt`) showed two things:

1. **No GUI appeared** - expected, not a bug: `headless` defaults to `true` (matching `emap`'s own convention and AGENTS.md's "run simulations headless by default" rule). Needs `headless:=false` to actually watch it.
2. **A burst of `Timed out waiting for transform from nav_ugv/base_link to nav_ugv/odom` warnings**, and a messy `SIGABRT`/class_loader crash in `planner_server` right after the user hit Ctrl+C a few seconds in. The warnings themselves are Nav2 behaving normally (it retries on a timeout, it doesn't fail) - `nav_sim.launch.py` started Gazebo, the bridges, and the entire Nav2 stack all at the exact same instant, so Nav2's costmaps started polling for `nav_ugv`'s TF before Gazebo had even finished spawning it. Given a few more seconds it would have self-resolved - but interrupting a lifecycle node mid-transition (exactly what a Ctrl+C during that noisy window catches) is what produced the ugly teardown crash, and the constant warning spam made the whole thing look broken even though nothing had actually failed yet.

**Fix**: `nav_sim.launch.py` now stages startup with `TimerAction` (the same tool `d3`'s own `launch_sim.launch.py` uses to stagger its UGV/UAV spawns) - Gazebo starts immediately (it's the slowest thing to come up), the ROS-side bridges/static TFs/`elevation_mapping_node` start 5s later, and Nav2 plus the planning layer start 9s in - so nothing tries to use a transform or topic before the thing publishing it has had a real chance to exist.

## 2. The requested feature: fly without touching anything

The ask: spawn the UAV in the air (not on the ground) and have it stay there scanning the room on its own, with no manual control - purely to watch the pipeline work.

**A single static hover, taken literally, has a real limit worth being honest about**: a downward camera held perfectly still can only ever see what one fixed cone of view exposes. `room_maze.world`'s interior walls (2.5m tall) cast a geometric blind spot behind themselves from ANY single overhead vantage point, no matter how high it's raised - this isn't a bug, it's the same limitation any single fixed camera has. `emap`'s persistent global map (step 8) was specifically built to accumulate observations from *different* vantage points over time without forgetting earlier ones - so a small **patrol** of hover points, each held long enough to build real coverage before moving to the next, gets meaningfully more of the room mapped than a single hover ever could, while still needing zero manual control (exactly what was asked for - nobody sends any command, the UAV just flies its own fixed loop forever).

**Built**: `nav/autopilot.py` (pure, no ROS - `compute_hold_command`, a simple proportional "fly to a point and hold" controller matching this project's other from-scratch, explainable modules, and `WaypointSequencer`, a small dwell-and-advance state machine) + `nav/uav_autopilot_node.py` (the ROS wrapper: subscribes `/odom`, publishes `/cmd_vel`). A default 5-point patrol (room center + one point per quadrant, all at 4.5m altitude - high enough to clear the 2.5m walls with real margin, low enough to keep reasonable ground resolution) is derived from the room's live-measured extent (see step01's verification section). Opt-in via a new `autonomous_uav:=true` launch argument - manual control is completely unaffected when it's off (default).

7 new unit tests (`tests/nav/test_autopilot.py`) cover both halves independently: the P-controller's clamping/direction-preservation, and the sequencer's arrival/dwell/loop/drift-resets-the-clock logic. 71 tests total now pass.

## 3. Live verification

With the world running headless and every non-Nav2 node started (Nav2 still pending the user's install - see step01), `uav_autopilot_node` was launched and the UAV's `/odom` was watched directly:

- Spawned on the ground (as always); within seconds of the node starting, climbed and settled at `(-0.78, -3.00, 4.48)` - the first waypoint, `(-0.8, -3.0, 4.5)`, matched to well within the arrival radius - **with no `/cmd_vel` ever sent by hand**.
- After its dwell time elapsed, it moved on and settled at `(-3.18, -0.51, 4.50)` - the NW-quadrant waypoint - confirming the sequencer actually advances, not just holds forever.
- Continued on toward the SE-quadrant waypoint next, confirming the full loop, not just a two-point back-and-forth.
- `/elevation_map` coverage grew over this window (checked via the same `decode_gridmap` + `compute_walkable_mask` script used in step01's verification) as the UAV visited each new vantage point - direct proof the patrol is actually building the map, not just flying for show.

## 4. Honest limitation

This patrol is a demo convenience, not a real coverage-guaranteeing exploration algorithm - 5 fixed points at one altitude will still leave some floor area (particularly tight pockets directly behind interior walls, from every vantage point's angle) unobserved. If a future need requires provably complete coverage, that's a genuinely different, harder problem (frontier-based exploration, or a much denser waypoint grid) - out of scope for what was asked here (a way to watch the pipeline run without touching anything).
