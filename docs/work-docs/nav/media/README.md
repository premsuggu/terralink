# Recordings and 3D map exports

> **Note**: All visual and media assets have been organized into the top-level `media/` directory:
> - **Videos**: `media/videos/` - `*_prm.mp4` were recorded with the original PRM planner (`overhead_view_prm`, `overhead_view_labeled_prm`, `uav_downward_view_prm`, `ugv_pov_prm`); `*_astar.mp4` with the A* planner (`overhead_view_astar`, `overhead_view_labeled_astar`) - see "A* planner videos" below
> - **3D Map Exports & Viewer**: `media/3d_maps/` (`elevation_map.ply`, `voxel_map.ply`, `viewer.html`)
> - **Figures & Architecture Diagrams**: `media/figures/` (`system_flow_diagram.svg`, `system_architecture_diagram.png`, etc.)
> - **Reports**: `media/reports/` (`TerraLink_Project_Report.docx`)

Generated from a real, ground-truth-verified `tunnel_demo.launch.py` run
(headless mode this time - see "Second pass" below for why) - the UGV
starting in room A, crossing the tunnel, and settling at the goal in room
B. See `../step06_hybrid_3d_voxel_navigation.md` for the full technical
history behind this scenario.

## Path-comparison images (located in `media/figures/path_comparison/`)

How the UGV really moved and which plans it received, PRM vs A*, in both worlds (12 live runs, 3 per planner per world, full runs including the UAV scan). Made with `scripts/trace_run.py` + `scripts/render_run_trace.py`; full explanation, legend, result tables and the bugs found along the way are in `../step08_path_trace_images.md`.

- **`tunnel_prm_vs_astar.png`**, **`maze_prm_vs_astar.png`** - planners side by side, 3 runs overlaid per planner, per-run numbers underneath.
- **`runs/<world>_<planner>_run<N>.png`** - one run in detail: track coloured by time since first movement, every distinct plan numbered where the UGV asked for it (orange dashed = tentative stretch, purple dashed = exploration detour), pauses as red rings, 3D clearance checks as green crosses.
- **`data/<world>_<planner>_<N>/`** - `trace.json` + `map.npz` for each run, so any image can be redrawn; `tunnel_metrics.json` / `maze_metrics.json` hold the numbers.

Headline (simulation, small n): maze - A* identical 3 of 3 (17-18 s from first move), PRM 2 of 3 reached; tunnel - PRM 3 of 3 but slow, A* 2 of 3 fast with one failure at a thin wall seam.

## Videos (located in `media/videos/`)

- **`overhead_view_prm.mp4`** *(PRM planner; renamed from `overhead_view.mp4`)* - full-room top-down bird's-eye view captured from a high static ceiling camera (`0 0 7.65m`, pitched straight down - reframed, see "Third pass" below). Shows the entire environment: Room A, the divider wall, the tunnel in the center, and Room B, with the UAV flying overhead and the UGV crossing through the tunnel from start to goal.
- **`overhead_view_labeled_prm.mp4`** *(PRM planner; renamed from `overhead_view_labeled.mp4`)* - labeled version of the above overhead view (`scripts/overlay_labels.py`, color-detection based): START/GOAL/UGV/UAV text labels drawn directly on the video plus a small legend in the bottom-left corner. The large fixed brown box straddling the wall is the TUNNEL STRUCTURE itself, not a robot.
- **`uav_downward_view_prm.mp4`** *(PRM planner; renamed from `uav_downward_view.mp4`)* - the UAV drone's own downward-facing sensor camera (2.5m altitude), showing the close-up terrain scanned beneath the drone.
- **`ugv_pov_prm.mp4`** *(PRM planner; renamed from `ugv_pov.mp4`)* - the UGV's own dedicated recording camera (`video_camera_link` in `models/nav_ugv/model.sdf`), mounted higher and tilted much less than the UGV's real sensing camera (`front_camera_link`, which stays close-range/floor-facing on purpose - see that link's own comment) - so this one actually shows the room instead of just close-up floor.

All four are real H.264-in-MP4 files (`avc1` fourcc, confirmed live via
OpenCV/FFmpeg), each at its own measured actual frame rate (not a flat
assumption) - `overhead_view*.mp4` and `uav_downward_view.mp4` at ~6.3-6.9
fps, `ugv_pov.mp4` at ~3.5 fps, all lower than the cameras' 10Hz nominal
rate because this sandbox does CPU-only software rendering under real
load. Play with any video player (VLC, mpv, the OS's default player, a
browser) - trimmed to ~224s (the UGV's actual crossing plus ~15s settled
at the goal), not the full multi-minute raw capture.

## A* planner videos (2026-10-07)

Same overhead camera, same tools (`scripts/record_run.py`, then `scripts/overlay_labels.py`), same world and goal as the PRM videos above, recorded from one headless `tunnel_demo` run with the A* planner:

```bash
ros2 launch nav tunnel_demo.launch.py headless:=true planner_type:=astar wait_for_mapping:=true anomaly_hold_sec:=30.0
python3 src/nav/scripts/record_run.py --out-dir /tmp/rec --duration 400      # started a few seconds after launch
python3 src/nav/scripts/overlay_labels.py --in /tmp/rec/overhead_view.mp4 --out /tmp/rec/overhead_view_labeled.mp4
```

- **`overhead_view_astar.mp4`** - the whole run (562 frames, ~114 s at ~4.9 fps, H.264): the UAV flies over the rooms and maps while the UGV waits at the green START marker (`wait_for_mapping` holds it until the map is worth planning on), then the UGV drives through the tunnel (hidden under its roof from above) and settles on the red GOAL marker.
- **`overhead_view_labeled_astar.mp4`** - the same video with START/GOAL/UGV/UAV labels (the UAV label is only detected in ~64 of 562 frames - it relies on spotting the drone's orange rotors, the same limitation as the PRM labeled video).

Ground-truth numbers for this run (`/ugv/odom_ground_truth`): first movement at 57.6 s, within 0.3 m of the goal at 101.3 s (43.7 s later), 8.7 m driven for a 6.0 m straight-line trip. That is slower and longer than the best A* runs in `step07_astar_planner.md` (16-29 s, 5.8-6.9 m, n=2) - recording the cameras adds rendering load and this was a single run, so treat the video as an illustration of the behaviour, not as a benchmark. Only the overhead videos were kept for A*; no `uav_downward`/`ugv_pov` A* versions were made.

## Second pass (2026-09-24) - what was actually wrong and how it was found

A first delivery of these videos was reported by the user as "shows
nothing" and was correct to be rejected on rewatch: `ugv_pov.mp4` had 0
real frames (a bridge config pointed at a gz topic - `.../front_camera/
image` - that a plain rgbd camera doesn't publish anything useful to for
a room view; see below), and the overhead video's frames were, on direct
visual inspection, extracted and viewed as images (not just re-opened by
OpenCV, which only confirms a file is *decodable*, not that its content is
meaningful) - all showing the UGV in the same place near the tunnel wall
regardless of timestamp.

Getting a genuinely correct replacement took several real fixes, in order:

1. **UGV POV showed only flat color bands** - `front_camera_link`'s
   0.6 rad downward tilt at 5cm height is deliberately close-range/
   floor-facing (real elevation-mapping sensor, see its own SDF comment) -
   not a video-showing-the-room camera. Added a SEPARATE
   `video_camera_link` sensor purely for recording, mounted higher with a
   gentler tilt.
2. **That new sensor's bridge was silently connecting to nothing** -
   confirmed live via `ign topic -l` that a plain `type="camera"` sensor
   does NOT get "/image" appended to its `<topic>` the way `rgbd_camera`
   does; `nav/config/bridge.yaml` was pointed at the real, un-suffixed
   topic name.
3. **Ground-truth verification was reading the wrong topic** - `/ugv/odom`
   is DiffDrive-plugin odometry (see `bridge.yaml`'s own top-of-file
   comment) for this world, not ground truth, and it can silently freeze/
   diverge under load. The UGV's real position (used to confirm arrival
   before finalizing this video) is `/model/nav_ugv/odometry` (Ignition's
   own physics ground truth) - two runs that looked "stuck far from goal"
   while being monitored via `/ugv/odom` turned out, on switching to the
   real ground-truth topic, to have already reached the goal.
4. **`mp4v` fourcc and a flat assumed 10fps** - `mp4v` (MPEG-4 Part 2)
   decodes fine in OpenCV's own re-read but many browsers/editors won't
   play it at all - a real, separate contributor to "shows nothing" for
   anyone not using OpenCV to check. Switched to `avc1` (real H.264).
   Cameras also deliver frames slower than their declared 10Hz under this
   sandbox's CPU-bound rendering, so a flat 10fps assumption played
   everything 1.5-3x too fast; `record_run.py` now measures the actual
   average arrival rate per camera and remuxes to that.

See `scripts/record_run.py`'s own docstring/class comments for the full
technical detail on point 4, and `models/nav_ugv/model.sdf`/
`config/bridge.yaml` for points 1-2.

## Third pass (2026-09-24) - "wider view which captures all elements ... shows the simulation properly"

Two more fixes, on top of the second pass above:

1. **Reframed the static overhead camera.** It was centered 10.5m up with
   a footprint (18.5m x 13.9m) much larger than the actual ~12m x 8m room
   - the room only filled about 65% of the frame, with everything in it
   (robots, markers, the tunnel) smaller than necessary. Lowered to 7.65m
   (same FOV, still pitched straight down, nothing newly cropped out) for
   a ~13.5m x 10.1m footprint that fills nearly the whole frame. A
   resolution bump (640x480 -> 1280x960) was tried first but reverted -
   it quadrupled that single camera's render cost and dropped its actual
   frame rate to ~0.6Hz (choppier, not clearer) under this sandbox's
   CPU-bound rendering; the height/FOV reframing is the real, load-free
   win.
2. **Added on-screen labels.** `scripts/overlay_labels.py` post-processes
   the recorded overhead video and draws START/GOAL/UGV/UAV text labels
   directly on it, using HSV color detection (not camera-pose math, which
   this project has been burned by getting backwards before) - the start/
   goal markers are self-illuminated pure green/red, the UGV chassis is
   pure white, the UAV's rotors are bright orange, all reliably
   thresholded. A persistent legend in the bottom-left corner spells out
   the color key. START/GOAL are detected once and held fixed for the
   whole video (so the label doesn't disappear when the UGV parks on top
   of one); UGV/UAV are detected fresh every frame and simply have no
   label when genuinely not visible (e.g. the UGV inside the tunnel -
   correct, not a bug).

Regenerate the label overlay on any freshly recorded `overhead_view.mp4`:

```bash
python3 src/nav/scripts/overlay_labels.py --in /path/overhead_view.mp4 --out /path/overhead_view_labeled.mp4
```

## System Architecture & Flow Diagrams

- **`media/figures/system_flow_diagram.svg`** - high-resolution, vector flow diagram showing the full autonomous navigation pipeline: sensing, elevation mapping, PRM planning, all 4 decision pathways (confirmed, frontier, anomaly, blocked), and 3D voxel headroom verification.
- **`SYSTEM_FLOW.md`** - detailed Markdown specification with full Mermaid flowchart and complete technical breakdown of every branch and edge case (located in this directory).

## 3D map exports (PLY point clouds, located in `media/3d_maps/`)

- **`elevation_map.ply`** - the UAV-built 2.5D elevation map, one colored
  point per observed grid cell at its real (x, y, elevation). Colored by
  traversability: green = easily walkable, yellow = difficult, red =
  lethal/blocked, gray = never observed.
- **`voxel_map.ply`** - the UGV's bounded 3D occupancy map (see step06
  Phase 1), one colored point per OCCUPIED voxel. Unlike the elevation map,
  this can represent the tunnel's roof and floor as two distinct surfaces
  at the same (x, y) - colored by height (blue = low, red = high).
- **`viewer.html`** - local, interactive 3D WebGL viewer for opening and orbiting both `.ply` point clouds directly in any web browser with zero installation (placed alongside `.ply` files in `media/3d_maps/`).

Open either with `viewer.html` or any standard point-cloud/mesh viewer that reads PLY:
**MeshLab** (free, cross-platform - probably the easiest), **CloudCompare**,
**Blender** (File > Import > Stanford PLY), or many browser-based PLY
viewers. No proprietary format, nothing project-specific needed to view them.

## Regenerating these

With a `tunnel_demo.launch.py` (or `nav_sim.launch.py`) session already
running:

```bash
# Video (run for as long as you want to capture, Ctrl+C to stop early)
python3 src/nav/scripts/record_run.py --out-dir /some/output/dir --duration 240

# Elevation map (2.5D, colored by traversability or height)
python3 src/nav/scripts/export_elevation_map.py --out /some/path/elevation_map.ply
python3 src/nav/scripts/export_elevation_map.py --out /some/path/elevation_map.ply --color height

# Voxel map (3D, colored by height)
python3 src/nav/scripts/export_voxel_map.py --out /some/path/voxel_map.ply
```
