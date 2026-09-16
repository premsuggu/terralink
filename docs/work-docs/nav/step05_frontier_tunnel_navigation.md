# Step 5: frontier-aware tunnel navigation - a prototype, tested once, and an honest limitation found live

**Package**: `src/nav/` (+ one additive extension to `src/emap/`)
**Goal**: let the UGV route through terrain the UAV's overhead camera can never observe from above (the concrete example: a semi-cylindrical tunnel/culvert that's the only way to a goal), instead of that region reading identically to a solid wall forever.
**Status**: a prototype, built and live-tested exactly once per instruction, **not yet promoted to be the default behavior**. The four architectural pieces below all work and are unit-tested. Live-testing against the literal "hollow tube" scenario surfaced a real, deeper limitation that this pass does **not** solve - documented honestly below rather than papered over, per this project's standing discipline.

---

## Why this exists

`emap`'s UAV maps the world from directly overhead. `nav.walkability.compute_walkable_mask` treats any cell the UAV has never gotten a measurement for (`is_valid=False`) as non-walkable - deliberately, so unscanned ground is never assumed safe (see `nav/walkability.py`'s original docstring). But that same rule means a region the UAV **cannot even in principle** observe - because something opaque sits between the camera and the true ground - reads identically to a real wall, forever, no matter how long the surrounding area gets scanned. If that region is the only route to a goal, the UGV is permanently stuck with no way to route around a "wall" that isn't actually solid.

Research into how this class of problem is handled elsewhere (frontier-based exploration - Yamauchi 1997; heterogeneous UAV/UGV teams like DARPA SubT's CSIRO/NeBula work; risk-aware planning over occluded aerial imagery) converged on a design with four parts, all built here:

1. **Three-state classification** (`nav.walkability.compute_frontier_mask`): a cell that's unobserved but borders a known-walkable cell is a "frontier" - worth tentatively routing through and investigating, distinct from both confirmed-safe and confirmed-lethal.
2. **Frontier-aware PRM** (`nav.prm_planner.plan`, new `frontier_mask`/`allow_frontier` params, default off): if no all-walkable path exists, a path that only needs to cross frontier cells is returned as a **tentative** result (`PlanResult.has_frontier_segments`) instead of failing.
3. **The UGV becomes a second mapping sensor** (`nav_ugv`'s new `front_camera_link`, `emap.elevation_mapping_node`'s new `secondary_points_topic`): closes the actual sensing gap by letting the UGV's own onboard camera feed the *same* shared `/elevation_map`, fusing in real ground-truth for terrain it physically drives past that the UAV never saw.
4. **Investigate-and-replan loop** (`nav.waypoint_follower`, new `enable_frontier_replan`): while following a tentative plan, periodically re-request a plan from the current position - once the UGV's own sensor resolves the frontier region with real data, the route gets confirmed or corrected instead of trusting one static guess forever.

All four are **additive and off by default** - every existing parameter/behavior in `nav`/`emap` is byte-for-byte unchanged unless a caller explicitly opts in (`enable_frontier_mode`, `allow_frontier`, `enable_frontier_replan`, `secondary_points_topic`). `tunnel_demo.launch.py` is the one place in the project that turns all of it on.

## What was built

- `nav/walkability.py`: `compute_frontier_mask(walkable_mask, is_valid)` - 4-connected "unobserved but adjacent to walkable" detection.
- `nav/prm_planner.py`: `plan()` gained `frontier_mask`/`allow_frontier` params and `PlanResult.has_frontier_segments`. Roadmap *nodes* are still only ever sampled from confirmed-walkable cells; frontier cells are eligible only as pass-through corridor cells for a line-of-sight edge.
- `nav/planner_node.py`: `enable_frontier_mode` parameter; publishes whether the last plan was tentative on a new `plan_has_frontier` topic (a `std_msgs/Bool` side channel, since `nav_msgs/srv/GetPlan`'s response has no room for a custom field).
- `nav/waypoint_follower.py`: `enable_frontier_replan`/`frontier_replan_interval_sec` - periodically re-requests a plan while the one being followed is flagged tentative; a failed *replan* attempt no longer discards a plan already working (a real bug caught while building this - the original retry-on-failure logic assumed failure only ever meant "no plan has succeeded yet").
- `emap/elevation_mapping_node.py`: `secondary_points_topic` parameter (default `""`, disabled) - an optional second `PointCloud2` subscription, fused into the **global** map only (never the local rolling one, which has no well-defined "recenter on which robot" semantics with two vehicles). Refactored the primary callback's TF-lookup-then-transform logic into a shared `_read_and_transform_cloud` helper so both sensors use identical code, not two hand-written copies.
- `nav/models/nav_ugv/model.sdf`: a new `front_camera_link` - a forward-facing, 34°-downward-tilted `rgbd_camera`, 8m range.
- `nav/worlds/tunnel_test.world`: a new test world - two rooms connected only by a 7-segment, box-approximated semi-cylindrical tunnel (radius 0.7m, length 1.5m), with green/red flat visual-only markers at the UGV's start and the goal.
- `nav/launch/tunnel_demo.launch.py`: a dedicated, hands-off demo launch (defaults `headless:=false`, `autonomous_uav:=true`) turning on all four frontier pieces.
- Tests: `tests/nav/test_walkability.py` (+5), `tests/nav/test_prm_planner.py` (+4), `tests/nav/test_waypoint_follower.py` (new, +4 for the pure replan-cooldown helper). **All 43 nav tests and all 44 runnable emap tests pass** (1 emap test skips for a pre-existing, unrelated reason - no CUDA/cupy in this environment, see step09's own doc).

## Two real bugs found live while building this

1. **UAV/UGV spawn collision.** An early version of `tunnel_test.world` spawned both robots at the exact same `(x, y)`. Their collision geometry overlapped at spawn, and the resulting contact impulse tipped the UAV over (confirmed live: its odometry orientation read a huge roll, `~(x=0.97, y=-0.18, z=0.16, w=0.02)`, not a level hover) while `/cmd_vel` kept commanding a normal climb the whole time - a rolled multirotor's thrust axis no longer points up, so no control logic could recover it (same class of bug as `room_maze.world`'s z=3.5-spawn free-fall issue). Fixed by giving the UAV 1.5m of clearance in Y.
2. **Visual markers read as real 1m-tall geometry by the depth camera.** The start/goal markers were originally 1m-tall cylinders with no `<collision>`. No collision only means the *physics* engine ignores them - the depth *camera* still renders and measures them like anything else. Confirmed live: `/elevation_map` read elevation ≈1.0 at both markers' exact coordinates, contaminating the room-center readings used for verification. Fixed by flattening both markers to a 2cm-tall disc (well under the traversability classifier's own step-height threshold).

## The honest finding: this does NOT yet solve the literal "hollow tube" case

Live-testing `tunnel_demo.launch.py` against the tunnel world revealed something the design didn't anticipate, confirmed by decoding the real, live `/elevation_map`:

**A solid, opaque tunnel roof does not produce `is_valid=False` cells underneath it.** A depth camera - simulated or real - reports the distance to whatever surface it actually hits. Looking straight down at an opaque roof, it correctly reports the roof's own height, not "nothing." A live scan-line through the tunnel's centerline (`y=0`, resolution 0.1m/cell) after the UAV's patrol had covered the area showed exactly this:

```
x=-1.0  elev=0.001  [walkable]   <- real room-A floor
x=-0.8  elev=0.721  [LETHAL]     <- a ~0.72m "cliff" - the false transition onto the roof reading
x=-0.6..0.2  elev≈0.726  [walkable]  <- the tunnel's ENTIRE interior reads as a flat, EASY, elevation-0.73m
                                       "plateau" - because the map fused the ROOF's height, not the
                                       floor's, and the roof itself is genuinely flat/consistent along its length
x=0.4   elev=0.726  [LETHAL]     <- transition back down
x=0.6..4.0  elev≈0.001..0.02  [walkable]  <- real room-B floor
```

Every cell under the tunnel is `is_valid=True` - genuinely, confidently measured - just measuring the wrong surface. `compute_frontier_mask` (built entirely around `is_valid=False`) correctly does **not** flag any of this as frontier, because none of it was ever actually unobserved. The two single-cell `LETHAL` bands are real, in the sense that `compute_traversability` correctly detected a ~0.7m step discontinuity there - it just doesn't know that discontinuity is a data-fusion artifact (two different real surfaces competing for one grid cell) rather than an actual cliff.

The practical result, confirmed over a full live run (patrol completed, both rooms and the tunnel scanned, `enable_frontier_mode=true` the whole time): **`get_plan` never once found a path** - 0 successes across the entire run. The user's underlying problem (a UGV genuinely blocked from reaching its goal) is faithfully reproduced by this world. But the *mechanism* this prototype built to solve it - tentatively routing through unobserved space - doesn't apply here, because the blocking cells were never unobserved in the first place.

This is a deeper, structural limitation than originally scoped: **a 2.5D single-height-per-grid-cell elevation map cannot represent two different surfaces (a roof and the floor beneath it) at the same `(x, y)` at all**, regardless of how the sensing/classification logic around it is designed. This is true of any overhead 2.5D elevation-mapping system, not a quirk of this specific implementation.

What the prototype *does* correctly solve, verified live and separately from the tunnel: **genuinely unscanned space bordering already-explored ground** - the classical, literature-standard meaning of "frontier," and arguably the more common real-world case (a UAV patrol simply hasn't reached an area yet, as opposed to a solid roof specifically blocking a straight-down view). Checked live at two points in time during the same run:

- **Before** the UAV's patrol reached room B (~45s in): `room_B_center (3,0)` and `room_B_far (5,0)` both read `is_valid=False`, exactly as expected for genuinely unscanned ground.
- **After** the full patrol (~150s in): `room_B_center` had flipped to `is_valid=True, walkable=True`; `room_B_far (5,0)` - just past where the patrol's camera footprint reaches - correctly still read `is_valid=False`, and was flagged `frontier=True` in the live-decoded map (adjacent to the now-confirmed-walkable cell at `x=4.8`ish). This is exactly the mechanism working as designed.

## Recommendation - what this means for "replace the main code"

**Do not promote this to be the default.** The instruction was to test it once and promote only if it works perfectly; it does not, for the specific scenario it was commissioned to solve. Recommend instead:

1. **Keep all four pieces as opt-in, off-by-default extensions** (already how they're built - nothing about existing `nav`/`emap` behavior needs to be touched or reverted). They are real, working, unit-tested capability for the "unscanned space at the exploration boundary" case, which is valuable independent of the tunnel scenario.
2. **The literal "opaque roof occludes the ground" case needs different, harder machinery**, out of scope for this pass: most plausibly, a classifier that treats an extreme, spatially narrow step-discontinuity bordered by walkable-on-multiple-opposing-sides as *suspicious* (worth investigating) rather than confidently lethal - conceptually similar to what's built here, but reasoning about a possible **data conflict** rather than **missing data**, which is a materially different (and riskier - it must not misfire on a real thin wall) inference to get right. This needs its own dedicated design and validation pass, not a rushed addition on top of an already-large change.
3. Alternatively, accept the representational limit and reframe what "the UAV can't see" means for future demo/test worlds: dense canopy, unexplored areas beyond the current patrol boundary, or anything that produces genuine non-returns - all of which the frontier mechanism built here already handles correctly - rather than a fully solid, opaque roof, which no 2.5D elevation map can see past by construction.

## How to run it

```bash
colcon build --packages-select nav emap
source install/local_setup.bash
ros2 launch nav tunnel_demo.launch.py   # headless:=false, autonomous_uav:=true by default
```

Watch: the UAV patrols both rooms and directly over the tunnel; the green/red markers show the UGV's start and goal; `get_plan` warnings in the log show the planner correctly detecting no route exists (matching the live finding above, honestly - this demo currently shows the LIMITATION clearly, not a success story).
