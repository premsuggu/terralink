# emap: From-Scratch Elevation Mapping - Implementation Plan

**Status**: Active development line, having fully superseded the earlier `terralink_elevation` attempt (removed - see git history if ever needed).
**Package**: `emap` (`src/emap/`)
**Reference (read-only)**: `src/d1/elevation_mapping_gpu_ros2/` — stays untouched; consult for algorithms and lessons learned, never edit.
**Docs**: `docs/work-docs/emap/` — this file is the roadmap; [`00_concepts.md`](00_concepts.md) is a from-scratch primer on ROS 2/Gazebo/SDF basics (read once); one `stepNN_*.md` per completed step, written as a beginner-friendly walkthrough of that step's new code and concepts, building on `00_concepts.md`.
**Tests**: `tests/emap/` (created once step 2 introduces testable algorithm code)

## Why restart

`terralink_elevation` targeted Gazebo Classic (`gazebo_ros`, `spawn_entity.py`, `libgazebo_ros_camera.so`). That stack **is not installed** on this machine — only **Ignition Gazebo Fortress** (`gz sim` 6.18) with `ros_gz_sim`/`ros_gz_bridge` is. `terralink_elevation`'s own launch file even documents Gazebo Classic being unreliable under WSL2, which is what this machine runs. Rather than keep fighting an unavailable/unreliable stack, `emap` targets the stack that is actually installed and is the current recommended pairing for ROS 2 Humble.

Environment facts that shape every step below:
- WSL2, Ignition Gazebo Fortress (`gz sim` 6.18) + `ros_gz_sim` + `ros_gz_bridge`. No `gazebo_ros`/Classic.
- WSLg is present (`DISPLAY`, X socket) so a GUI can be shown, but default to headless per AGENTS.md.
- No CUDA/CuPy installed yet in this checkout — GPU work (later steps) needs that set up first; algorithms should stay CPU/NumPy-correct before any GPU port, same discipline `terralink_elevation` used.
- Outbound network is inconsistent: GitHub (raw/API) works reliably; `fuel.gazebosim.org` (the Ignition Fuel model registry) accepts a TLS connection but stalls indefinitely on actual downloads in this sandbox. **Don't depend on Fuel at runtime or during setup** — vendor any external assets as local files instead (see step 1).

## Roadmap

1. **UAV in Gazebo** ✅ — a physically-simulated, controllable quadrotor spawns in Ignition Gazebo and is flyable from ROS 2. See `step01_uav_gazebo_deployment.md`.
2. **Depth sensor + point cloud pipeline** ✅ — a downward-facing depth/RGB-D camera is mounted on the UAV, its `PointCloud2` reaches ROS 2, and a verified TF tree (`iris_quad/odom → base_link → camera_link/rgbd_camera`) lets real points be transformed to the world frame and land at the true ground height. See `step02_depth_camera_pointcloud.md`.
3. **Elevation map core data structure (CPU)** ✅ — multi-layer NumPy grid (elevation, variance, is_valid, traversability), vectorized coordinate-transform helpers, unit tested in isolation (no ROS, no Gazebo). See `step03_elevation_map_data_structure.md`.
4. **Bayesian fusion (CPU)** ✅ — point cloud → height/variance update per cell (variance-weighted combination of prior belief + new measurement), outlier rejection, unit tested against hand-computed expected values. See `step04_bayesian_fusion.md`.
5. **Map shifting (UAV-centric)** ✅ — recenter the grid as the UAV moves (`ElevationMap.move_to`), unit tested for the exact axis-swap/edge-blanking-direction pitfalls this kind of code is prone to. See `step05_map_shifting.md`.
6. **ROS 2 node integration** ✅ — `elevation_mapping_node` subscribes to the point cloud, calls `move_to`/`fuse_points` with live TF data, and publishes a `grid_map_msgs/GridMap` on `/elevation_map`. Verified live: cells under the hovering UAV read the true ground height to within a fraction of a micron, and the map correctly follows the UAV after a lateral move. See `step06_ros_node_integration.md` (includes two real bugs found and fixed during verification).
7. **Traversability + visualization** ✅ — an original analytical slope/step-height/roughness classification (`src/d1` turned out to use a trained CNN, not a formula we could adapt), a real Gaussian-bump test world (`worlds/bump_test.world`, `scripts/generate_bump_heightmap.py`), and an RViz config. Verified flying over the real bump: peak elevation measured at 1.4978m against a true 1.5m, with traversability correctly lethal on the steep slope and easy on flat ground. See `step07_traversability.md` (includes a real Gazebo shader-crash bug found and fixed).
8. **Persistent global map + local rolling map** ✅ — the roadmap's original rolling-only map turned out to be the wrong shape of memory for this project's actual end goal (autonomous UGV navigation, which needs a map that never forgets terrain outside the current sensor view). `elevation_mapping_node` now keeps two `ElevationMap` instances fed by the same sensor data: the existing local rolling map (unchanged, kept for future local-reaction use), and a new persistent global map that's never re-centered, published on `/elevation_map` (the local map moved to `/elevation_map_local`). No changes needed to `ElevationMap`/`fuse_points`/`compute_traversability` themselves. Verified live: a bump stayed correctly remembered in the global map long after the UAV flew far enough away that the local map had genuinely lost it. See `step08_persistent_global_map.md`.
9. **GPU acceleration (CuPy)** ✅ — `fuse_points`'s exact algorithm ported to CuPy (`emap/fusion_gpu.py`), verified to numerically match the CPU reference on a randomized point cloud. Getting CuPy working required fixing two environment issues (missing CUDA headers, a numpy2/scipy ABI conflict). Measured ~25% faster than CPU at a realistic point-cloud size (76,800 points) - a real but modest win, honestly reported rather than assumed. See `step09_gpu_acceleration.md`.
10. **Drift compensation** ✅ — vertical (Z)-only correction (`emap/drift.py`) comparing new measurements against the global map's already-confident cells, wired into the live node with an injectable synthetic drift for visibility (Gazebo's own TF never drifts). Verification caught and fixed two real bugs: a measurement-ordering mistake that caused exponential runaway, and a missing safeguard against a single bad sensor frame producing an implausible correction. Live-verified stable for 45s with zero injected drift, and roughly halving accumulated error over 45s of continuous injected drift. See `step10_drift_compensation.md`.
11. **Bugfix: phantom high-altitude terrain + runaway climb** ✅ — a real bug found live: flying high enough corrupted the map into a fake tower of terrain. Root cause turned out NOT to be the first, plausible-looking theory (a depth-camera far-clip clamp artifact) but a scene light posed inside the map's operational volume being misread as geometry by this environment's software-rendered depth camera - fixed by moving the light (harmless for a directional light) plus a defensive `max_valid_range` filter in `fuse_points`/`fuse_points_gpu`. A second, independent bug (the UAV's velocity command never expires, letting one stray `cmd_vel` climb forever) was fixed with a new `cmd_vel_watchdog` node, itself found to have a self-referential re-triggering bug during verification. See `step11_far_range_and_flight_safety_fixes.md`.

Each step gets its own `stepNN_*.md` in this folder covering: what was built, why (concept + reference-code line links where relevant), how it was verified, and pitfalls hit.

## Ground rules (carried over from AGENTS.md)

- CPU-first, GPU only after CPU is proven.
- Every step is independently testable before moving to the next.
- Reference `src/d1` for ideas; never modify it.
- No test/debug files in the repo root — everything under `tests/emap/`.
- Run simulations headless by default; use the GUI only to sanity-check visually.
