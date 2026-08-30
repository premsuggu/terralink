# emap: From-Scratch Elevation Mapping - Implementation Plan

**Status**: Active rebuild, replacing `terralink_elevation` as the working line of development.
**Package**: `emap` (`src/emap/`)
**Reference (read-only)**: `src/d1/elevation_mapping_gpu_ros2/` and `src/terralink_elevation/` — both stay untouched; consult for algorithms and lessons learned, never edit.
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
3. **Elevation map core data structure (CPU)** — multi-layer NumPy grid (elevation, variance, is_valid, traversability), coordinate-transform helpers, unit tested in isolation (no ROS, no Gazebo) — same discipline as `terralink_elevation`'s step 3.
4. **Bayesian fusion (CPU)** — point cloud → height/variance update per cell, outlier rejection, unit tested against hand-computed expected values.
5. **Map shifting (UAV-centric)** — recenter the grid as the UAV moves, unit tested for the axis-swap/padding pitfalls `terralink_elevation` documented running into.
6. **ROS 2 node integration** — wire steps 2-5 into a live node subscribing to the point cloud, publishing a `grid_map_msgs/GridMap`, verified in the Gazebo world from step 1/2.
7. **Traversability + visualization** — slope/step/roughness classification layer, RViz config, verified visually against a known test terrain (e.g. a Gaussian bump world, reused/adapted from the old `worlds/gaussian_bump.world` pattern).
8. **GPU acceleration (CuPy)** — only after CPU correctness is locked in and CUDA/CuPy are installed and verified in this environment; port the fusion kernel, verify GPU output matches the CPU reference numerically.
9. **Drift compensation** — correct accumulated pose-drift error using low-variance cell statistics, unit tested against a synthetic drift scenario.

Each step gets its own `stepNN_*.md` in this folder covering: what was built, why (concept + reference-code line links where relevant), how it was verified, and pitfalls hit.

## Ground rules (carried over from AGENTS.md)

- CPU-first, GPU only after CPU is proven.
- Every step is independently testable before moving to the next.
- Reference `src/d1` and `src/terralink_elevation` for ideas; never modify either.
- No test/debug files in the repo root — everything under `tests/emap/`.
- Run simulations headless by default; use the GUI only to sanity-check visually.
