# Recordings and 3D map exports

Generated from a real, ground-truth-verified `tunnel_demo.launch.py` run
(headless mode this time - see "Second pass" below for why) - the UGV
starting in room A, crossing the tunnel, and settling at the goal in room
B. See `../step06_hybrid_3d_voxel_navigation.md` for the full technical
history behind this scenario.

## Videos

- **`overhead_view.mp4`** - full-room top-down bird's-eye view captured from a high static ceiling camera (`0 0 7.65m`, pitched straight down - reframed, see "Third pass" below), reframed to fill nearly the whole frame with the room instead of leaving a large wasted margin. Shows the entire environment: Room A, the divider wall, the tunnel in the center, and Room B, with the UAV flying overhead and the UGV crossing through the tunnel from start to goal. **Labeled**: START/GOAL/UGV/UAV text labels are drawn directly on the video (`scripts/overlay_labels.py`, color-detection based) plus a small legend in the bottom-left corner - no more guessing what's what. The large fixed brown box straddling the wall is the TUNNEL STRUCTURE itself, not a robot.
- **`uav_downward_view.mp4`** - the UAV drone's own downward-facing sensor camera (2.5m altitude), showing the close-up terrain scanned beneath the drone.
- **`ugv_pov.mp4`** - the UGV's own dedicated recording camera (`video_camera_link` in `models/nav_ugv/model.sdf`), mounted higher and tilted much less than the UGV's real sensing camera (`front_camera_link`, which stays close-range/floor-facing on purpose - see that link's own comment) - so this one actually shows the room instead of just close-up floor.

All three are real H.264-in-MP4 files (`avc1` fourcc, confirmed live via
OpenCV/FFmpeg), each at its own measured actual frame rate (not a flat
assumption) - `overhead_view.mp4` and `uav_downward_view.mp4` at ~6.3-6.9
fps, `ugv_pov.mp4` at ~3.5 fps, all lower than the cameras' 10Hz nominal
rate because this sandbox does CPU-only software rendering under real
load. Play with any video player (VLC, mpv, the OS's default player, a
browser) - trimmed to ~224s (the UGV's actual crossing plus ~15s settled
at the goal), not the full multi-minute raw capture.

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

- **`system_flow_diagram.svg`** - high-resolution, vector flow diagram showing the full autonomous navigation pipeline: sensing, elevation mapping, PRM planning, all 4 decision pathways (confirmed, frontier, anomaly, blocked), and 3D voxel headroom verification.
- **`SYSTEM_FLOW.md`** - detailed Markdown specification with full Mermaid flowchart and complete technical breakdown of every branch and edge case.

## 3D map exports (PLY point clouds)

- **`elevation_map.ply`** - the UAV-built 2.5D elevation map, one colored
  point per observed grid cell at its real (x, y, elevation). Colored by
  traversability: green = easily walkable, yellow = difficult, red =
  lethal/blocked, gray = never observed.
- **`voxel_map.ply`** - the UGV's bounded 3D occupancy map (see step06
  Phase 1), one colored point per OCCUPIED voxel. Unlike the elevation map,
  this can represent the tunnel's roof and floor as two distinct surfaces
  at the same (x, y) - colored by height (blue = low, red = high).
- **`viewer.html`** - local, interactive 3D WebGL viewer for opening and orbiting both `.ply` point clouds directly in any web browser with zero installation.

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
