# Step 10 - A realistic UGV: the Clearpath Jackal-class model (2026-10-26)

**Status:** done and verified in simulation (drive tests, terrain tests, live runs of W2, W3, W4 and W5, 445 tests). Replaces the old 0.3 m box rover. Section 6 (footprint, door gauntlet, Nav2 inflation) is the follow-up of the same day.

## 1. Why, and what was chosen
The old `nav_ugv` was a white 0.3 x 0.15 m box on two wheels and two casters: it worked, but it looked like a toy next to the mesh-based UAV and it was not a vehicle anyone would deploy. Decision (user, 2026-10-26): **reuse an existing, proven robot description instead of building one** - the Clearpath **Jackal** (BSD licence, github.com/jackal/jackal, `jackal_description`). The same robot class the published ground-exploration planners (FAEL, STGPlanner) were tested on. Only three meshes were vendored (`models/nav_ugv/meshes/`: base, fender, wheel, 190 kB) plus the licence notice. The URDF was not used directly (Gazebo Fortress needs SDF; the official Humble simulation package targets Gazebo Classic): the **numbers** were taken from it and written as SDF.

| | old | Jackal-class (now) |
|---|---|---|
| size (L x W x H) | ~0.3 x 0.44 x 0.15 | **0.51 x 0.43 x 0.25 m** (fenders, wheel outside edges) |
| drive | 2 driven wheels + 2 ball casters | **4 driven wheels, skid-steer** |
| mass | 2 kg | **16.5 kg chassis + 4 x 0.477 kg** (inertia from the URDF) |
| wheel radius | 0.05 | **0.098**; track 0.3756, wheelbase 0.262 |
| lidar | 0.22 m above ground, inside the volume of the body | **0.29 m, on the top deck** (same 360 rays, 0.3-12 m, 10 Hz) |
| front depth camera | 0.05 m above the ground (!) | **0.155 m, on the front of the body**, same 0.6 rad tilt |
| video camera | inside the chassis volume | above the top deck, looking forward |

What did **not** change: model name `nav_ugv`, frame `base_link` (still 0.125 m above ground), topics (`nav_ugv/scan`, `/front_camera`, `/video_camera`), joint names `left_wheel_joint` / `right_wheel_joint` (now the front wheels; `rear_*` added), so Nav2, the planner and the follower needed no change.

## 2. Concept: skid-steer and the "effective wheel separation"
A 4-wheel robot with fixed wheels cannot turn without its wheels **slipping sideways**. Gazebo's `DiffDrive` plugin computes wheel speeds assuming no slip: `v_left = v - w * sep/2`. With the true track (0.3756 m) the robot turned at only ~77 % of the commanded rate (the rest is lost to slip). The standard remedy (the real Jackal driver does the same with a multiplier) is to tell the controller a **wider, effective separation**: `0.3756 / 0.77 = 0.49 m`. Measured in the test world (spin 1 rad/s for 3 s, then 0.4 m/s with 0.5 rad/s for 4 s):

| separation given to DiffDrive | spin (ideal 172 deg) | arc (ideal 115 deg) | straight 0.5 m/s x 3 s |
|---|---|---|---|
| 0.3756 (geometric), mu2 = 0.6 | 87 deg | (erratic) | 1.5 m |
| 0.3756, mu = mu2 = 1.0 | 132 deg (77 %) | 88 deg (77 %) | 1.5 m |
| **0.49, mu = mu2 = 1.0 (used)** | **167 deg** | **118 deg** | **1.5 m** |

The worlds therefore carry `<wheel_separation>0.49</wheel_separation>` with a comment; the unit test `tests/nav/test_ugv_model.py` fails if a world forgets it. (The plugin's own wheel odometry is not used anywhere: localisation is ground truth - see TODO.md, "localisation".)

## 3. Pitfalls found (all fixed, all now covered by a test)
1. **SDF poses have two reference frames.** The poses of a model's *links* (wheels, camera links) are relative to the **model origin = the ground**, while visuals/collisions/sensors *inside* `base_link` are relative to `base_link` (0.125 m up). Writing a camera link at `z = 0.03` put it inside the floor: the depth image was 94 % `-inf`. The static TFs in the launch files are relative to `base_link`, so `z_tf = z_link - 0.125`.
2. **A sensor inside the body shell sees the shell.** A depth camera inside the front face of the body returned nothing; a lidar below the body top would hit the robot. All three sensors now sit outside the shell (checked in tests).
3. **The old camera was 0.05 m above ground, not 0.175 m** (it was written in the model frame, the TF said base_link frame): the old `/elevation_map` contributions of this camera were 0.125 m too high. With the Jackal both agree. Not re-validated on the tunnel world yet (see TODO.md E-05).
4. A spawn-and-drive test script that runs `pkill -f "<process name>"` kills the shell that carries the text of that pattern in its own command line: use PIDs or run the script from a file.

## 4. Verification (all numbers measured)
* Straight drive: 1.99 m of 2.00 m ideal (0.5 m/s x 4 s). Lidar: no returns from the robot itself (minimum finite range 7.80 m = the wall 8 m ahead minus the lidar offset), 0 `-inf`.
* Depth camera: all 240 x 320 pixels valid, 0.17-0.74 m slant range to the floor (the geometry predicts 0.18-0.74 m).
* Terrain (test world with a curb and a ramp): crosses an **8 cm curb** and climbs a **20 deg ramp** at 0.5 m/s; it fell off the end of the 0.68 m-high ramp, as expected, which is a property of the test, not a result.
* **Live pipeline** (W2 container yard, goal C - the 46 m route through the 0.8 m lane, A* + Nav2, `run_world_scenario.sh`): reached, **46.9 m driven (46.6 m with the old robot), final error 0.14 m, no recovery behaviours, 107 s** (old robot 93-113 s over three runs). The 0.8 m lane is passed with a 0.43 m-wide robot.
* Tests: 441 pass (`tests/nav/test_ugv_model.py` new: model/world consistency).

## 5. Known limits / next (written before section 6; the footprint item is resolved there)
* (Resolved in section 6: Nav2 now uses the rectangle.) The A* planner still uses a **circular** footprint of 0.22 m (+0.05 m margin). The Jackal's half-width is 0.215 m, so sideways it is right, but it is 0.51 m long: turning in place needs 0.33 m of clearance (circumscribed radius). W2's lane passed anyway; a polygon footprint in Nav2 and a check in the tight W4 doors are in TODO.md (N-01).
* The fenders are visual only; collision is the URDF's 0.42 x 0.31 box (plus the wheel spheres).
* Only W2 was re-run live with the new model; W1, W3 and W4 need re-runs for the record (TODO.md R-01). The videos in `media/runs/` other than W2 still show the old robot as a dot (the dot/arrow marker was replaced by a to-scale top view of the Jackal in `make_run_video.py`).
* The drive test is in the repo: `tests/nav/integration/ugv_drive.py` + `ugv_drive_test.world` / `ugv_terrain_test.world` (needs Gazebo; the header of the script lists the commands); the geometry rules are unit-tested in `tests/nav/test_ugv_model.py`.

## 6. Follow-up: the footprint and "how narrow a door can it really pass?" (N-01, 2026-10-26)

### 6.1 What was wrong
Nav2 still described the robot as a **circle of 0.22 m** (`robot_radius`). The Jackal is a 0.51 m x 0.43 m rectangle: its half-width (0.215 m) matches the circle, but its length does not, and nobody had measured what door a 0.43 m-wide robot can actually get through with A* + Nav2. Earlier the guess was "effective door width = nominal - 0.3 m".

### 6.2 New test world: W5 door gauntlet (`worldgen/worlds/w5_door_gauntlet.py`)
A 44 m hall with 8 cross walls 4 m apart; each has one door, the widths shrink 1.2, 1.0, 0.9, 0.8, 0.7, 0.6, 0.55, 0.5 m and the doors alternate between the north (+3 m) and south (-3 m) side, so the robot must swing across and line up with every door. Goal `k` is 2 m behind door `k`. Built by `build_world.py w5_door_gauntlet` (walls only, no asset pack). Two surprises while building it: (1) **the A* planner assumes SQUARE maps** (a 44 x 18 m map gave "no path" to every goal - the bounds are now 44 x 44 m); (2) the offline feasibility table (`RADII` 0.15 ... 0.45 m) is in `tests/nav/test_worldgen.py` (`DESIGN`).

Offline, with the planner's real parameters, A* accepts doors down to **0.6 m (goal 6)** and says "no path" for 0.55 m and 0.5 m (goals 7, 8).

### 6.3 Measurement (goal 6, behind the 0.6 m door; same route in all three runs; time per door in simulated seconds)
| configuration | door 1 1.2 | 2 1.0 | 3 0.9 | 4 0.8 | 5 0.7 | 6 0.6 | reached at | driven | "Failed to make progress" |
|---|---|---|---|---|---|---|---|---|---|
| circle 0.22 m, inflation 3.0 / 0.55 m (before) | 36 | 15 | 15 | **62** | 14 | **71** | 222 s | 66.7 m | 5 |
| rectangle, inflation 3.0 / 0.55 m | 14 | 15 | 14 | **61** | **41** | **82** | 237 s | 56.8 m | 10 |
| **rectangle, inflation 10.0 / 0.45 m (adopted)** | 34 | 15 | 15 | **15** | **14** | **195** | 297 s | 70.3 m | 8 |
(Door 1's time includes the start-up delay of the stack; "per door" = time between crossing the previous wall and this one.)

What the log showed at the stalled 0.8 m door: four DWB "Failed to make progress" (the progress checker wants 0.5 m in 10 s), then Nav2's **spin recovery aborts with "Collision Ahead"**, wait, back-up, and only then the squeeze through - 40+ s with the robot standing still.

### 6.4 Why, and what was changed (`config/nav2_params.yaml`, `nav2_params_large.yaml`, both costmaps)
* **`robot_radius: 0.22` -> `footprint: [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]`** - the real outline (fenders, wheel edges). This is what Nav2's own collision checks (the spin recovery, the costmap's inscribed/circumscribed radii 0.215 / 0.333 m) use.
* **Inflation `cost_scaling_factor` 3.0 -> 10.0 and `inflation_radius` 0.55 -> 0.45 m.** The polygon alone changed nothing for the door: the DWB controller's `BaseObstacle` critic scores only the cost of the cell under the robot's centre. With the old gentle slope the centre line of a 0.8 m door (0.4 m from each wall) costs 252 * exp(-3 * (0.4 - 0.215)) = **144** of 252, which DWB treats as "dangerous" and answers by not moving; with 10.0 it costs 252 * exp(-10 * 0.185) = **39**. The radius stays above the circumscribed radius 0.333 m (a unit test checks that). Result: doors down to 0.7 m now take the same 15 s as the wide ones.
* **Honest minimum door width (this robot, A* + Nav2, static world): 0.7 m passes cleanly (1.6 x the body width); 0.6 m (1.4 x) is passed but marginally - 195 s and 8 failed attempts in the run; 0.55 m / 0.5 m are refused by A* ("no path", correct).** The planner still accepts 0.6 m doors that Nav2 passes only with great effort: aligning the planner's acceptance threshold with the controller's (e.g. refusing < 0.7 m, or pricing narrow doors) is open item **N-04**, together with a controller that scores the full footprint (`ObstacleFootprint` critic, or MPPI).

### 6.5 Regression runs with the final configuration (live, complete map given to A*)
| world / goal | before (old robot) | now (Jackal, rectangle, inflation 10 / 0.45) |
|---|---|---|
| W2 C (chicane + 0.8 m lane) | 46.6 m, 93 s, 0.15 m | **46.7 m, 88 s, 0.14 m, 0 failures** (first Jackal run, old costmap: 46.9 m, 107 s) |
| W3 A (ramp into the pit) | 26.9 m, 45 s, 0.15 m | **28.6 m, 58 s, 0.06 m, 0 failures** |
| W4 B (under the deck, oracle map) | 76.0 m, 198 s, 0.26 m | **72.8 m, 137 s, 0.19 m, 0 failures** |
| W1 A (105 m around the building) | 105.2 m, 189 s, 0.17 m | **105.2 m, 184 s, 0.16 m, 0 failures** (23 min of wall time for 184 sim s: the 120 m map and the 2000 px camera) |
All videos in `media/runs/` show the robot drawn to scale (`w1`..`w4`, plus `w5_door_gauntlet_run.mp4`).

### 6.6 Pitfalls
* A run that "did not move" (driven 0.0 m for 230 s) was a **Nav2 start-up failure** (`change_state` timeout, the lifecycle manager never activated the stack), not a navigation result: always check that the log contains "Received a goal" and that the first poses change before judging a run. Rerun.
* `make_run_video.py` keeps going after a failed run and **overwrites the committed video** - restore with `git checkout media/runs/<file>` after a failed run.
* Not yet done: W4's real interior (hidden-interior exploration) has not been re-run with the new costmap; the exploration problems are a separate topic (TODO.md section 3).
