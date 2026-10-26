# Step 10 - A realistic UGV: the Clearpath Jackal-class model (2026-10-26)

**Status:** done and verified in simulation (drive tests, terrain tests, one live W2 run, 441 tests). Replaces the old 0.3 m box rover.

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

## 5. Known limits / next
* The planner still uses a **circular** footprint of 0.22 m (+0.05 m margin). The Jackal's half-width is 0.215 m, so sideways it is right, but it is 0.51 m long: turning in place needs 0.33 m of clearance (circumscribed radius). W2's lane passed anyway; a polygon footprint in Nav2 and a check in the tight W4 doors are in TODO.md (N-01).
* The fenders are visual only; collision is the URDF's 0.42 x 0.31 box (plus the wheel spheres).
* Only W2 was re-run live with the new model; W1, W3 and W4 need re-runs for the record (TODO.md R-01). The videos in `media/runs/` other than W2 still show the old robot as a dot (the dot/arrow marker was replaced by a to-scale top view of the Jackal in `make_run_video.py`).
* The drive test is in the repo: `tests/nav/integration/ugv_drive.py` + `ugv_drive_test.world` / `ugv_terrain_test.world` (needs Gazebo; the header of the script lists the commands); the geometry rules are unit-tested in `tests/nav/test_ugv_model.py`.
