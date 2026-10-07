# Step 08 - Path-trace images: where the UGV really went, and every plan it got (PRM vs A*, two worlds)

**Status:** built, tested, and run (12 live runs). Images are in `media/figures/path_comparison/`.
**What this step is:** a way to *see* a run - the UGV's true track, every plan the planner handed out, where it paused to look - for both planners, so they can be compared on more than a single number. It also produced the first **second-world** evidence for the A* planner, and found three bugs.

---

## 1. The pictures

| File | What it shows |
|---|---|
| `media/figures/path_comparison/tunnel_prm_vs_astar.png` | Tunnel world, PRM (left) vs A* (right), 3 runs overlaid per planner, table of per-run numbers |
| `media/figures/path_comparison/maze_prm_vs_astar.png` | Same for the maze world |
| `media/figures/path_comparison/runs/<world>_<planner>_run<N>.png` | One run in detail (12 images) |
| `media/figures/path_comparison/data/<world>_<planner>_<N>/` | The raw record of each run (`trace.json`, `map.npz`) so any image can be redrawn |
| `media/figures/path_comparison/*_metrics.json` | The numbers behind the tables |

**How to read a detail image** (`runs/*.png`)
* Thick coloured line = where the UGV really was (ground truth), coloured by **seconds since it first moved** (dark = early).
* Thin grey line with dots = a plan the planner returned, numbered at the point where the UGV *asked* for it. **Orange dashed** = a tentative stretch (through a suspected passage or unseen space), **purple dashed** = an exploration detour (go and look instead of heading to the goal).
* Red ring = it stayed within 15 cm for 3 s or more ("waited at start", or "paused N x, total T s" if it kept pausing in one spot).
* Green plus = a 3D clearance check (octomap) gave a verdict. Green square = start, red star = goal.
* Background: dark = obstacle, white = free, light grey = **never observed** (no valid measurement; the planner does not treat it as walkable). Walls show up grey where the UAV never got a valid reading of their top: a real wall is 0.15 m thick, i.e. only 1-2 cells on the 0.1 m map, so a top-down camera often misses part of it - e.g. the thin vertical grey line between the two tunnel rooms. Large grey areas outside the room were never scanned at all. In frontier mode the planner treats unobserved cells next to free space as *possible* passages (the orange tentative routes), which is exactly the wall-seam trap described below. (The background is the final map of that run.)
* "N 'no path' replies" = how many times the planner answered *no route*. These are not drawn (there is nothing to draw); they are mostly the minutes while the UAV is still scanning.

## 2. How it works (and how to repeat it)

```
planner_node --(JSON per get_plan reply)--> /plan_trace --+
Gazebo dynamic_pose/info --(own ros_gz_bridge)--> true pose --+--> scripts/trace_run.py --> trace.json + map.npz
/goal_pose, /voxel_verification, /elevation_map -------------+
trace.json + map.npz --> scripts/render_run_trace.py (nav/trace_render.py) --> PNG
```
* `nav/planner_node.py` now publishes **`plan_trace`** (a JSON string per `get_plan` request, valid or not: planner name, start, goal, waypoints, per-stretch kind, exploration flag). Nobody in the stack subscribes to it; it changes nothing about planning (`TestPlanTraceTopic` in `test_planner_node_astar.py`).
* `src/nav/scripts/trace_run.py` records one run. It starts its **own** `ros_gz_bridge` for Gazebo's `/world/<world>/dynamic_pose/info`, so the true pose needs no change to any world or bridge file. It stops when the UGV has stayed within 0.3 m of the goal for 3 s (or after `--duration`).
* `src/nav/scripts/render_run_trace.py detail|compare` draws the images; the numbers (first move, arrival, move->arrive, driven, stalled, distinct plans, "no path" replies, turn-backs) come from `nav/trace_render.py::track_metrics` (`tests/nav/test_trace_render.py`).

```bash
# one run (launch it first, e.g. ros2 launch nav tunnel_demo.launch.py headless:=true planner_type:=astar wait_for_mapping:=true anomaly_hold_sec:=30)
python3 src/nav/scripts/trace_run.py --out-dir runs/tunnel_astar_1 --world tunnel_test --goal 3 0 --planner astar
# draw
python3 src/nav/scripts/render_run_trace.py detail  --run runs/tunnel_astar_1 --out run.png
python3 src/nav/scripts/render_run_trace.py compare --group "PRM=runs/p1,runs/p2,runs/p3" --group "A*=runs/a1,runs/a2,runs/a3" --out cmp.png
```
Metric definitions follow `step07_astar_planner.md` (move->arrive = first movement to within 0.3 m of the goal). Stalled time is now measured with sliding 5 s windows (any 5 s stretch with < 0.1 m net progress), so a pause is counted whatever its phase; step 07's table used fixed windows, so the two stalled columns are not strictly the same measure.

## 3. Results (simulation, headless, full run including the UAV scan, 3 runs per cell)

Settings: tunnel - goal (3, 0), `planner_type:=prm` (default) vs `planner_type:=astar wait_for_mapping:=true anomaly_hold_sec:=30`. Maze - goal (1.7, -0.5), `nav_sim.launch.py autonomous_uav:=true planner_type:=prm|astar`. "First move" is seconds after the logger started (the sim needs ~25 s to come up, so subtract that for time since launch).

### Tunnel world (straight line 6.0 m)
| Planner | Run | First move (s) | Arrived (s) | Move->arrive (s) | Driven (m) | Stalled (s) | Distinct plans | "No path" replies |
|---|---|---|---|---|---|---|---|---|
| PRM | 1 | 39 | 281 | 242 | 13.1 | 192 | 15 | 11 |
| PRM | 2 | 39 | 102 | 63 | 11.3 | 29 | 2 | 12 |
| PRM | 3 | 40 | 181 | 141 | 12.3 | 105 | 12 | 19 |
| A* | 1 | 73 | 103 | 31 | 8.4 | 0 | 1 | 29 |
| A* | 2 | 99 | 116 | 17 | 5.6 | 0 | 1 | 43 |
| A* | 3 | 61 | **did not arrive** | - | 7.6 | 247 | 3 | 25 |

### Maze world (straight line 4.7 m)
| Planner | Run | First move (s) | Arrived (s) | Move->arrive (s) | Driven (m) | Stalled (s) | Distinct plans | "No path" replies | Turn-backs |
|---|---|---|---|---|---|---|---|---|---|
| PRM | 1 | 101 | 121 | 19 | 6.3 | 0 | 1 | 44 | 0 |
| PRM | 2 | 105 | 208 | 103 | 7.3 | 81 | 1 | 45 | 0 |
| PRM | 3 | 98 | **did not arrive** | - | 79.5 | 32 | 1 | 40 | 11 |
| A* | 1 | 101 | 119 | 18 | 6.4 | 0 | 1 | 43 | 0 |
| A* | 2 | 103 | 120 | 17 | 6.3 | 0 | 1 | 44 | 0 |
| A* | 3 | 102 | 120 | 18 | 6.4 | 0 | 1 | 44 | 0 |

### What the pictures show
* **Maze:** A* drove the same route three times (17-18 s, 6.3-6.4 m, no stalls). PRM was clean once (run 1), once stalled for 81 s on the same east route (run 2), and once never arrived (run 3). In run 3 the PRM planner's only plan cut straight across the vertical wall at x of about -1 (in the run's final map that stretch is solid wall, 4 cells thick; I only saved the final map, so why PRM thought it passable at plan time is not established - most likely the wall was not fully scanned yet), and the UGV then circled in front of that wall for ~5 minutes (79.5 m driven, 11 turn-backs) without ever being given a different plan. All three PRM runs received exactly one plan each, but not the same one, because PRM samples random points; A*, being deterministic, produced the same route every time.
* **Tunnel:** PRM reached the goal 3 of 3 times but slowly (63-242 s from first move, 11-13 m, 29-192 s stalled); every PRM run spent a long time at the same spot just south of the tunnel. A* reached it 2 of 3 times, quickly and without stalls (17-31 s, 5.6-8.4 m). **A* run 3 failed**: it started moving earlier than the other two A* runs (first move at 61 s vs 73 s and 99 s; I did not determine why), drove to the dividing wall south of the tunnel and spent the rest of the run there offering tentative routes through the wall (247 s stalled, until the 330 s cap).
* **The south-of-tunnel wall seam is a trap for both planners.** In 4 of the 6 tunnel runs the UGV drove to about (-0.5, -1.3), where the dividing wall is only 1-2 cells thick and partly unscanned (see step 07 section 2.6), and lost time there. A* went there once (run 3) and did not get out; PRM went there in all 3 runs and got out each time, after a stall of 29-192 s (the stalls in the PRM table).
* **A* waits longer to start** (first move 61-99 s vs PRM's 39-40 s in the tunnel) because `wait_for_mapping` holds it still while the map grows. Its *arrival* time was still no later than PRM's in the two A* runs that arrived (103/116 s vs 102/181/281 s).
* Both planners spend the first ~100 s in the maze, ~40-100 s in the tunnel with no route at all ("no path" replies): the UAV has not yet scanned enough to connect start and goal.

### What this does and does not prove
* Supports: A* is more consistent than PRM in the maze (3 of 3 identical, vs 1 of 3 clean), and when A* works in the tunnel it is faster with less driving and no stalls. It also adds a second world to the evidence (step 07 only had one).
* Does **not** support "A* is simply better": it **failed once in its six runs here** (tunnel run 3). Counting the two earlier full-run tunnel runs with the same settings (F1 and F2 in step 07), 4 of the 5 tunnel runs arrived. PRM failed once in its six runs (maze run 3). In the tunnel the A* failure is the same thin-wall gamble that step 07 set out to remove. n = 3 per cell, one goal per world, simulation only. These images show behaviour; they are not a statistical comparison.
* The default planner stays `prm`.

### Why do A* results differ between runs if A* is deterministic? (checked 2026-10-08)
* A* **is** deterministic: replaying it twice on the same saved map returned identical routes for all 6 runs' maps (checked offline).
* What differs between runs is **the map it is given**, not the algorithm. The first valid plan each A* run actually received: maze - three plans within about 10 cm of each other (same route, same result 3 of 3); tunnel - three different routes (run 1 swings up to y = 1.9 around the tunnel exit, run 2 goes straight, run 3 heads for the wall seam south of the tunnel). Replaying A* on the three tunnel runs' *final* maps also gives three different routes. The UAV scan finishes at a slightly different moment every run (the simulator runs under CPU load), and the planner is first asked at the moment a route exists, so each run starts from a different half-built map.
* A* returns the cheapest route **for the map at that moment**; the numbers in the tables (time, metres driven, stalls) are what happened afterwards in the physics and in Nav2's local controller, which are not deterministic. They are not A*'s path cost.
* Two more time-dependent inputs: the `wait_for_mapping` gate and the anomaly hold both use clocks, and route memory keeps an earlier route unless a new one is at least 15 % cheaper, so the plan also depends on what was handed out before.
* Caveat: replays on the final maps are not what the planner saw (final maps include things like the UGV's own chassis fused at the goal; maze run 1's final map even gives "no route"). The map at each plan was not recorded. Recording a map snapshot with every plan would settle this (not done).
* The grey in the images is "never observed", not "free": see the legend note in section 1.

## 4. Bugs found while doing this (all fixed, none caused by the new code)
1. **`anomaly_hold_sec:=30` killed `planner_node`.** `tunnel_demo.launch.py` passed the launch argument through as text; `30` reached the node as an INTEGER and the node declares a double (`InvalidParameterTypeException`), so the planner died at startup and the UGV never moved (it showed up as 0 plans in the first trial). Fixed with `ParameterValue(..., value_type=float)` for `anomaly_hold_sec` and `frontier_min_width_m`. The command in `docs/RUN.md` and step 07 (`anomaly_hold_sec:=30`) now works as written.
2. **The maze world was broken by the shared `bridge.yaml`.** `config/bridge.yaml` takes `nav_ugv`'s transform from `/model/nav_ugv/pose`, which only exists when the world has the ground-truth `OdometryPublisher` plugin. The tunnel world got that in step 06; `room_maze.world` never did, so in the maze Nav2 never saw `nav_ugv/odom -> nav_ugv/base_link` and the UGV never moved (first maze trial: 420 s, 0 movement). Fixed by giving `room_maze.world` the same plugin pair as the tunnel world (DiffDrive's own odometry moved to unused frames).
3. **Spawn offset applied twice in the maze.** With the plugin in place the pose is already in world coordinates (measured: base_link at (-0.7, -4.5) in `nav_ugv/odom`), but `nav_sim.launch.py`'s static transform still added the spawn offset (-0.7, -4.5), so the planner's start point was (-1.4, -9.0), outside the map ("no path" 202 times). The static transform is now the identity, as `tunnel_demo.launch.py`'s already was.

Bugs 2 and 3 mean **the maze demo had not been working since step 06** (it was last verified before that); it works again (A* reached the goal in all 3 runs, PRM in 2 of 3).

## 5. Limits of the images / next steps
* The background is the *final* map of each run, so it shows more than the planner knew when it made early plans.
* "Distinct plans" merges routes within 0.15 m; the PRM planner returns slightly different roadmaps on each replan, so its count is a measure of replanning churn as well as of real route changes.
* Not done: the construction-site world (no UGV/Nav2 setup exists there), more runs per cell, other goals.
* The tunnel A* failure is the next thing worth fixing: the planner should not keep offering tentative routes through a 1-2 cell wall seam it has already tried. `stuck_feedback` (step 07) is meant for this but was off in these runs; running the same matrix with `stuck_feedback:=true` would show whether it helps.
