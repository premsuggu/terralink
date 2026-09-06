# Step 4: the UGV corrupting its own map, and a bad default goal

**Package**: `src/nav/`
**Goal**: fix why the UAV moved but the UGV still didn't, on a second live run.
**Status**: two more real bugs found and fixed - a structural one (a parked robot corrupting its own elevation map) and a data one (a bad example coordinate). UGV confirmed driving again, faster than before.

---

## Symptom: `get_plan: no path found` for over two minutes, despite the UAV visibly flying

Your second `cli.txt` showed the same warning as step 3, but now persisting far longer (100+ retries over 2+ minutes) even though the UAV was visibly patrolling. Two independent problems, both in data the planner was trusting, not in the planning algorithm itself:

### Bug 5: a parked UGV gets fused into its own elevation map

Checked live: `nav_ugv`'s own spawn cell, `(-0.7, -4.5)` - the exact point step 3 fixed and confirmed walkable - now read `traversability=LETHAL`. Nothing about the world had changed. What had changed: the UGV had been sitting there, parked, while the UAV's patrol kept flying back over that same spot (the room-center waypoint is only ~1.5m away, well within camera view). `emap`'s depth camera has no notion of "that's a robot, not terrain" - it fuses whatever it sees into the elevation map, including the UGV's own chassis. Once enough of those readings accumulate, the map genuinely believes there's a small step/obstacle exactly where the UGV is standing.

This is structural, not a bad-coordinate problem: **wherever** the UGV parks, it will eventually see itself this way if the UAV flies back over it. Since `planner_node.py` always plans a route from the UGV's own current position, and the old `prm_planner.plan` rejected any query whose start cell wasn't walkable, a parked UGV could get permanently locked out of ever planning a route away from wherever it happened to be sitting - it doesn't get stuck *next to* an obstacle, it can *become* one.

**Fix**: standard costmap practice (Nav2's own costmaps do exactly this) - always treat the robot's own current footprint as passable, regardless of what the map says there. `nav.prm_planner._clear_footprint_around` forces a small disk (`footprint_radius_m`, default 0.3m) around the START point walkable before anything else runs; the GOAL is deliberately NOT given this exemption (a genuinely blocked goal should still be rejected - "trust that I'm not sitting inside a wall right now" only makes sense for a live position report, not for something you're being asked to drive into). 4 new/updated tests cover this, including one confirming the exemption is local and can't accidentally bridge all the way through a genuinely thick wall.

### Bug 6: the documented default goal, `(4.0, 0.0)`, sits ~0.1m from a real wall

Checked live: `(4.0, 0.0)` reads `traversability=LETHAL`. `room_maze.world`'s east boundary wall sits at world x≈4.10 (see that file's own docstring for the wall-layout math) - `(4.0, 0.0)` was never actually inside the room at all, practically speaking. It was picked as a generic "far corner" example before the room world existed, and never checked against the real geometry once it did - the same category of mistake as the original `nav_ugv` spawn point in step 3.

**Fix**: the default `goal_x`/`goal_y` (in `waypoint_follower.py`'s parameter declaration, `nav_sim.launch.py`'s launch arguments, and the README's example commands) is now `(1.7, -0.5)` - confirmed live to be walkable with real clearance, and it sits inside the UAV's own default patrol's NE-quadrant waypoint, so it gets scanned early instead of needing a long wait. Documented clearly in all three places: any custom goal needs to be checked against a live-flown `/elevation_map` first, since a point too close to any wall will fail exactly this way regardless of how long you wait.

## Live verification

Same launch, this time with no goal override (using the new default): `ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true`.

- UAV took off clean and level again (no regression from step 3's fix).
- `get_plan: found a 2-waypoint path` in about **15 seconds** - dramatically faster than step 3's ~460 seconds, because the goal no longer needed the UAV to scan all the way to a wall edge that would never actually succeed.
- `nav_ugv`'s odometry sampled three times in a row: `(2.64, 1.13) -> (3.61, 1.49) -> (3.63, 2.55)` - real, continuous motion again.

Same known, low-priority rough edge as step 3: `planner_server: Failed to create a plan from potential when a legal potential was found` recurs periodically during navigation without blocking movement - a Nav2/NavFn-internal quirk, not this project's own code, still not investigated further since it isn't blocking anything.
