# Step 16 - TerraLink Studio: P1 backend + P2-P4 browser viewer, controls, goal/plan and saved-map mode (2026-11-23)

**Goal of the whole studio (agreed 2026-11-22).** A browser page where a person flies the UAV with an Xbox controller, watches the elevation map
grow, clicks a target for the UGV, and optionally drives the UGV by hand and changes the settings. Viewer style: a MuJoCo-like interactive 3D view
in the browser (option 1: Gazebo stays as the headless physics/sensor backend, nothing new to install; three.js will be vendored locally in P2).
First world: W2 (container yard). The controller is plugged into Windows and read by the browser's Gamepad API (WSL2 has no `/dev/input`).

| Phase | Content | State |
|---|---|---|
| P0 | maze stall fix (step15) | done |
| P1 | backend: data stream, UAV commands, safety limits | done (sections below) |
| **P2** | **3D viewer in the browser: 7 camera views, 1/2/4-pane layouts, live elevation surface, vehicles, trails, camera footprint** | **done (section "P2-P4")** |
| **P3** | **keyboard + Xbox-controller control, UAV/UGV control target, e-stop, UAV flight limits** | **done; real controller NOT yet tried (only a simulated one)** |
| **P4** | **click-to-set goal, plan preview, auto drive, manual UGV, cancel, saved-map mode (`map:=uavview/groundtruth`)** | **done** |
| P5 | settings card: UAV/UGV speed limits + flight ceiling, presets (gentle/normal/fast) | done 2026-11-23 |
| P6 | demo video, final polish (the one-command launcher `studio.launch.py` already exists) | partly |

## How it fits together (for a newcomer)

```
 browser  <--- websocket /ws --->  studio_node (ROS 2 + tornado web server)  <--- ROS topics --->  Gazebo + emap + nav
  (page)      JSON  : state, commands                 limits, dead-man, e-stop                    (unchanged)
              binary: elevation map
```
The browser never talks to ROS. It sends *normalised* stick values; the server turns them into velocities and applies the limits, so a bug in the
page cannot command more than the configured maximum.

## Files

| File | Role |
|---|---|
| `src/nav/nav/studio/protocol.py` | wire format: map as one binary frame (int16 mm height + flag byte, cropped to the observed area), JSON helper |
| `src/nav/nav/studio/commands.py` | command parsing/validation, UAV limits, dead-man, e-stop (`UavCommandState`) |
| `src/nav/nav/studio/server.py` | tornado server: static page + websocket, drops frames for slow clients, same-origin check, 127.0.0.1 only |
| `src/nav/nav/studio/studio_node.py` | the ROS node (`ros2 run nav studio_node`), parameters `port`, `max_xy`, `max_z`, `max_yaw`, `deadman_sec`, ... |
| `src/nav/studio_web/` | the browser app (see "P2-P4"): `index.html`, `style.css`, `js/{app,protocol,terrain,vehicles,views,input}.js`, `vendor/three.min.js` (three.js r149, MIT, vendored) |
| `src/nav/launch/studio.launch.py` | ONE command: simulator + nav stack + `studio_node`; `map:=live|uavview|groundtruth|<file.npz>` |
| `src/nav/scripts/make_studio_world.py` | writes `worlds/<world>_studio.world` = the run world + the UAV (`worlds/w2_container_yard_studio.world`) |
| `tests/nav/test_studio_{protocol,commands,commands_ugv,server,launch}.py` | unit tests, no ROS/simulator needed (backend, limits, sticky events, launch wiring, the live-scan route fixture) |
| `tests/nav/integration/studio_live.py` | live check of the backend against the running simulator (8 checks) |
| `tests/nav/integration/studio_browser.py` | drives the page in a headless Chromium (keys, simulated gamepad, mouse clicks, screenshots of every view): 12 checks |
| `tests/nav/integration/studio_fake_backend.py` | a pretend `studio_node` (real server + saved map + two toy robots) so the page can be tested without Gazebo |

## Facts measured on the simulator (W2, headless, 2026-11-23)

* `/cmd_vel` is a **body-frame** velocity: after yawing 86 degrees, +vx moved the UAV north and +vy moved it to its left. So: forward, left, up
  are positive; +yaw is counter-clockwise. The gamepad mapping in P3 follows this.
* **Simulation speed (real-time factor): 0.54 idle, 0.57-0.63 while flying/scanning.** A 29 simulated-second flight took 51 wall seconds. The
  controls therefore feel about half as responsive as real time; that is a property of software rendering on this machine, not of the studio.
* `/elevation_map` is published at about 2 Hz as a 560 x 560 grid with 6 float32 layers = 7.5 MB per message; the studio reduces that to at most
  about 0.9 MB per frame (observed area only, 3 bytes per cell) at 1 Hz. After a short flight (14 k observed cells) frames are far smaller.
* The UAV climbs at the commanded 1 m/s and overshoots the target height by about 0.5 m when told to stop.

## Verification

* Unit tests: protocol round trip (1 mm accuracy, crop, empty map, clipping, malformed frames), command validation (NaN, strings, booleans, unknown
  commands, out-of-range sticks), limits (no input can exceed them), dead-man, e-stop, and the websocket server with real clients (hello,
  late joiner gets the latest state and map, validation errors, refusals, oversized message closes the link, broadcast to several clients).
* Live (`studio_live.py`, W2 headless, node + simulator): **8/8 checks passed** - page served; hello + 10 states/s; a streamed climb raised the UAV
  (z 0.10 -> 11.39 m); 23 map frames while flying, a frame decoded to exactly the observed area the state reported (14 155 cells, 141.6 m^2);
  **dead-man**: after the page went silent the commanded velocity fell to 0; **e-stop** held against a stuck stick that kept streaming and a
  release started from zero; a garbage message got an error frame and the link stayed up.

## Bugs found by the tests while building (fixed)

1. **E-stop could resume the UAV.** A `uav_vel` that arrived while stopped was still remembered, so releasing the e-stop within the dead-man time
   (0.5 s) restarted it with that old command. Now commands during an e-stop are dropped and a release always starts from zero.
2. **Map frames were dropped by state traffic.** The "skip while busy" rule for slow clients treated all frames alike, so a map sent right after a
   state frame was dropped (at 10 state frames per second most maps would never arrive). The busy flag is now per frame kind.
3. **Unclean shutdown.** `kill` left the node alive for seconds and aborted with "terminate called without an active exception": rclpy was shut
   down while the executor thread was still running. Now SIGINT/SIGTERM stop the loop, the UAV is sent zero velocity, the executor is stopped and
   joined, then rclpy is shut down (exits in 0.4 s, clean log, 2 of 2 trials).

## Known gaps / decisions for later phases

* **Flight limits (done in P3):** floor 0.6 m, ceiling 14 m, and a fence at the edge of the elevation map less 2 m; they remove only the outward part of a
  command (`commands.apply_uav_bounds`, unit-tested). They do NOT stop the UAV flying into a container (the UAV has no obstacle sensing): fly with care.
* **Follower "wait for a goal" (done in P4):** `waypoint_follower` has two new opt-in parameters, `wait_for_goal` (default False) and the topic
  `/mission/cancel`; `tunnel_demo.launch.py studio:=true` turns them on. Every other launch is unchanged.
* The map is streamed whole every second (about 0.9 MB on W2, fine on localhost); a delta/tiled format is not needed so far.
* **Not done:** P5 (changing framework settings from the page); a real Xbox controller and the Windows browser have not been tried (see "What was NOT tested").

## P2-P4: the browser app, controls, goal and saved-map mode

### What you can do now
```
ros2 launch nav studio.launch.py                 # fly the UAV yourself (live scan)       -> open http://localhost:8765
ros2 launch nav studio.launch.py map:=uavview    # skip the scan: saved "what the UAV sees" map, straight to planning
ros2 launch nav studio.launch.py map:=groundtruth  # same with the oracle (ground-truth) map
ros2 launch nav studio.launch.py map:=/path/to/snapshot.npz
```
`map:=live` runs the real elevation mapping, so the map is only what you have scanned. Any saved map replaces it with `map_replay_node` (the existing
"start at planning" mode); the planner, Nav2 and the UGV physics are the real ones, only the 95-second scan is skipped. The page shows which source is
active ("map: live scan" or "map: saved - <name>"). The map source is chosen when the stack is launched; switching at run time would mean stopping and
starting the mapping node, which I did not build.

### The page
* **Views (7):** free orbit, top-down (north up, orthographic: the best view for choosing a goal), UAV chase, UAV first-person, UAV camera (down), UGV
  chase, UGV driver. **Layouts:** one big pane, big pane + minimap (default), or four panes; every pane has its own view menu and its own camera.
  Click a pane's `UGV` / `UAV` / `map` button to centre it there. The robots are tiny on a 60 m map, so a "robot size" slider (default x3) draws
  them bigger and each carries a cone beacon that keeps a readable size on screen.
* **Terrain:** a height-field mesh built from the newest map frame (only cells that were observed are drawn, so unscanned ground shows as the dark
  checkered floor); colour by **Height**, **Drivable** (green = the planner will drive there, red = blocked, exactly the planner's walkable mask) or
  **Plain**. The map's outer rectangle is drawn so you can see where planning stops. Also drawn: trails of both robots, the UAV camera footprint (an
  approximate rectangle: 60 degree horizontal field of view, 320x240; which image axis lines up with which body axis is assumed), the goal flag, and the
  plan as a coloured line (cyan = confirmed ground, yellow = suspected anomaly, orange = unobserved frontier) with dots at the waypoints.
* **HUD:** link / map source / scanned area / simulation time with real-time factor / frames per second / controller / mission state chips, positions of
  UAV, UGV, goal and the mouse cursor (with "drivable / blocked / not scanned" under the cursor), and a log of messages from the server.
* **Goal:** press `G` (or the button), click a spot; the cursor ring is green (drivable), red (blocked: the click is refused) or grey (not scanned: allowed
  with a warning, the planner may find no route). The page sends `{"t":"goal"}`; the server publishes `/mission/goal`; the follower (now idle until it
  hears a goal) asks the planner for a plan and drives. State chip: planning, driving, NO ROUTE, ARRIVED. `Esc` / "Cancel goal" stops it.
* **You control: UAV | UGV (manual).** Picking UGV cancels the goal, hands the wheels to you (`/nav2_cmd_vel`, same dead-man as the UAV, max 1.0 m/s and
  1.5 rad/s) and freezes the UAV (it hovers); picking UAV gives the UGV back to the planner. Goals are refused while you drive.
* **Input:** keyboard (WASD/RF/QE for the UAV, WASD or arrows for the UGV, Shift = slow, Space = e-stop, G, Esc, 1-7 views, L layout, M control target,
  H help) and an Xbox-style controller through the browser Gamepad API (left stick climb/turn, right stick fly, LT slow, B e-stop, A release, Y switch
  UAV/UGV, LB/RB view, Start layout, Back goal tool). Sticks are sent 20 times a second **including zeros**, so the server's dead-man (0.5 s) stops the
  UAV if the page ever freezes, closes or loses the link. Buttons are read from a timer, not the drawing loop (see bugs below).

### Server-side additions (all unit-tested)
`ugv_vel`, `mode`, `cancel_goal` commands; `UgvCommandState` (dead-man); `UavBounds` + `apply_uav_bounds` (floor/ceiling/fence); sticky events (the
newest `plan` is replayed to a page opened later); `plan_trace` -> `plan` events; state fields `mode`, `nav`, `uav_limited`; `hello` carries the map source.

### How it was verified (2026-11-23)
| Check | Result |
|---|---|
| unit tests | 575 nav passed (14 skipped as before), 80 emap passed (the GPU test is skipped: no CuPy here); pyflakes clean on all new files |
| browser test vs the pretend backend (`studio_browser.py`) | 12/12, re-run after the LAST code change: page loads with no JS errors, all 3 layouts and all 7 views draw, keyboard climb, e-stop (held key ignored), simulated Xbox pad (sticks, B, A), click -> goal exact to 0.00 m, blocked click refused, plan arrives, **e-stop cancels the goal and the same goal can be sent again**, Cancel, manual UGV, reconnect |
| same test vs the REAL stack (`studio.launch.py map:=uavview`, `--real`) | 12/12, also re-run after the last change (the follower log shows the same goal accepted, cancelled, accepted again) |
| end to end, saved map | clicked a goal 12 m from the UGV: real A* plan, Nav2 drove it, state reached ARRIVED in 66 wall seconds (about 21 simulated) |
| end to end, **live scan** | in the page: climbed to 5.5 m, flew a sweep over the UGV's spot and 12 m north (72 m2 scanned), clicked a goal on scanned ground; the UGV planned and drove about 11 m and ARRIVED (132 wall seconds in all) |
| ground-truth map | loads and draws (checked with the pretend backend; not driven) |
Screenshots of every view are written by the browser test (`--out`).

### Bugs / findings while building
1. **Live scan: "no route" from the UGV's own spot.** A UAV flown low over the parked UGV sees it like a table: its flat top reads as drivable and its edges
   form a blocked ring ~0.4 m out. The planner's "start area is free" circle (`footprint_radius_m`, default 0.3 m) never touches that ring, so the blob
   clearing never fires and the UGV is boxed in. Measured on a saved live scan (`tests/nav/fixtures/studio_live_corridor.npz`, start -3,-26): radius
   0.3 -> no path to any of 3 goals; 0.4, 0.5, 0.6, 0.8 -> routes of 3.0 / 6.0 / 11.1 m. (Raising the blob-size limit instead did nothing: the ring is not
   connected to the circle.) Kept: new launch argument `footprint_radius_m` (default 0.3, so all other launches are unchanged); the studio uses 0.5. The
   saved-map runs never showed it because those maps come from the offline simulator, not from a camera that sees the robot.
2. **A short button tap could be lost on a slow machine.** Gamepad buttons were read in the drawing loop; with software rendering (a few frames per
   second) a 0.3 s tap on A (release e-stop) fell between frames. Now read from the 50 ms input timer.
3. `THREE.Object3D.DefaultUp` was renamed `DEFAULT_UP` in the vendored version: the page died at start (found by the browser test, fixed).
4. Layout: on a narrow window the control panel wrapped to two rows and covered the south of the map, so clicks never reached it. The panel is now one
   row (speed/size/trails moved under a gear button) and the small captions hide on windows narrower than 1300 px.
5. Stale process: a six-hour-old `ign gazebo` from the maze runs was still alive when this phase began (my earlier "nothing left running" was wrong). Killed
   by PID; the check script now lists leftovers by process name before each launch.

6. **A cancelled goal could not be sent again** (found in review). `waypoint_follower` ignores a goal equal to the one it already holds (the mission layer
   repeats its goal every second), and cancel did not forget it, so after Cancel / e-stop / driving by hand, clicking the SAME spot did nothing and the page
   showed "planning" forever. Cancel now resets the stored goal to NaN (never equal to anything). Covered by browser check 8 on the real stack.
7. **E-stop now also cancels the UGV's goal.** It first stopped only the UAV and the manual UGV, leaving a robot that Nav2 was driving still moving. The
   pilot just clicks the goal again after releasing. (Change `studio_node.on_command` if you want the old split.)
8. **A very short button tap can be missed on a slow page.** Gamepad buttons are read by a 50 ms timer; when the page draws at a few frames per second
   (software rendering) one frame can block that timer for longer than a quick tap. Keyboard keys are events and are never missed, and the page warns when
   drawing is slow. On a GPU (60 fps) this should not occur, but it has not been seen there. The browser test holds the simulated button 1.2 s for this reason.

### What was NOT tested (be aware)
* **A real Xbox controller and the Windows browser.** The browser tests use headless Chromium inside WSL with a *simulated* Gamepad object, so the
  mapping code and the whole command path are tested, but not real hardware, real stick feel, or whether your controller reports the "standard" layout
  (the page warns if it does not). WSL2 forwarding `localhost:8765` to Windows is the expected behaviour, but I have only used `localhost` from inside WSL.
* **Drawing speed on a real GPU.** This sandbox has no GPU: Chromium with software rendering ran 5.7 fps (1 pane), 7 fps (pane + minimap) and 1.3 fps
  (4 panes) at 1400x850. On a normal GPU this should be smooth, but I could not measure it. The page shows its fps and suggests layout "1" if it is slow.
* The "UAV camera (down)" view is a rendering of the elevation map from the UAV's pose, **not** the simulated sensor's image; world objects are drawn only
  from the map (in live mode an unscanned container does not exist on screen). That is deliberate: it shows exactly what the planner knows.
* Open the page as `http://localhost:8765`, not by the WSL IP address: `localhost` is a "secure context" and an IP is not (checked: `isSecureContext` is true for localhost, false
  for the IP). In the Chromium used here the Gamepad API function still exists on the IP address, but I cannot confirm whether a real pad would be reported there.
  If WSL2's localhost forwarding does not reach your Windows browser, `ros2 launch nav studio.launch.py host:=0.0.0.0` listens on all addresses (this exposes a server
  that can fly the UAV to the network; use it only on a trusted network).
* Not done: P5 (settings panel), the demo video, switching the map source without relaunching.
* Simulator limits still apply (see step 14): real-time factor 0.35-0.6 while the browser also runs here, doors 0.4 m narrower, a perfect camera.
* The test tooling (Playwright + Chromium, with permission) lives only in the session scratchpad, not in the repo or the system; to reuse it see the
  docstring of `studio_browser.py`.

## P2b: the viewer rework after the first real try (2026-11-23, same day)

**What the user saw** (first look at a live-scan start, 1862x1016 window): an almost empty checkered floor, two cone beacons, the UAV on the ground
with the e-stop engaged, a crammed bottom dock, instructions hidden behind a Help button, "options not working". My earlier 12/12 browser suite had
only ever started from a *saved* map, so it never saw this state. Reproduced, then fixed:

| Problem | Cause | Fix |
|---|---|---|
| "world not visible" | in live mode the page drew only scanned cells: at start 6 m2 of nothing | the server reads the world's `.world` file once (`nav/studio/scenery.py`: 36 box models + 31 cargo containers sized from their `mesh.obj`) and sends ~67 boxes in `hello` (about 12 KB). The page draws them as "the real site" (muted, outlined), the scanned map is drawn brighter over it (polygon offset so it wins on a scanned container roof), each can be switched off. The robots still only know what they scan. Checked against the ground-truth map: all 31 container centres sit on 2.6 m cells |
| "instructions not clear" | all guidance behind `? Help` | a **"next step" box** at the top of the side panel, driven by the state the server already streams: take off -> scan -> pick a goal -> planning -> driving -> arrived / no route / e-stop / manual / goal-tool, each with one button that does the step (Take off, Set goal, Cancel, Release). A short how-to opens once on the first visit. Cases that used to fail silently now say why |
| "options not working" | mostly the e-stop swallowing everything without saying so (keys, take-off, manual UGV are ignored while stopped), a clicked button keeping focus so Space also re-pressed it, no way to take off other than holding R | a red banner while stopped; the coach explains; every button blurs after a click and Space never reaches a button; **Take-off** command (`takeoff`, server-side climb to 6 m, any climb key or e-stop cancels it: `commands.takeoff_climb`, unit-tested) |
| UAV and UGV drawn as dots | the first camera framed the whole 56 m site | the free-orbit camera starts 24 m behind the UGV; the minimap pane shows the whole site; "Camera follows: nothing / UGV / UAV"; a dashed drop line from the UAV to the ground shows its height from any angle |
| "unprofessional" | bottom dock + chips + floating axes | one layout: slim top bar (site, map source, sim time, fps, link), a side panel in order of use (1 Fly & drive, 2 Send the UGV somewhere, View, Status), **the e-stop pinned at the bottom of the panel so it is always on screen**, a legend, calmer colours (flat ground green, tall things warm), no stray axes helper |

Server-safety semantics are unchanged (limits, dead-man, e-stop also cancels the goal). New: `takeoff` command, `scenery` in `hello`, `cmd.climb_to` in state.

**Verification of the rework**
| Check | Result |
|---|---|
| unit tests | `tests/nav/test_studio_scenery.py` (16: scenery from the shipped world incl. the ground-truth cross-check, include placement by mesh box + yaw, broken files, take-off command and climb rule) and launch wiring guards; whole nav suite after the rework: 593 passed, 14 skipped (was 575); emap unchanged |
| new browser test `studio_firstrun_browser.py` (live, empty map, 1862x1016, no saved preferences) | the first-visit how-to, the real site drawn with nothing scanned, camera near the robots, e-stop on screen, coach text in each state, take-off lifts / is cancelled by a climb key / finishes at 6 m, e-stop banner and refusal, Space does not re-press the last clicked button, **every side-panel control** (3 layouts, 3 colours, 3 follow modes, both layer switches, both sliders, UGV by hand and back, help open/close), goal click -> driving, no JS errors |
| old browser test `studio_browser.py` on the new page | 12/12, three fresh runs in a row at the end; first-run test 19/19 after the last code change |
| real stack, live scan (`studio.launch.py`) | hello carried 67 boxes, take-off to 5.9 m in 18 wall seconds (RTF 0.33), flying north scanned 57 m2 and the coach moved to "Pick a goal"; no JS errors |

**Found during the real-stack run, and what was done about it**
* A live scan flown quickly (speed 70 %, 6 m) is sparse and speckled: the first real try had only 333 of 3746 "drivable" scanned cells connected to the UGV, and a goal 3 m away got NO ROUTE. Part of this is real: **a container roof is scanned and flat, so it looks drivable, but nothing connects it to the ground**. The page now flood-fills from the UGV over scanned drivable cells (`updateReach`, 9 ms for a 600x600 map, once per map frame): in *Map colours > drivable* cut-off ground is steel blue, and clicking it warns before sending ("cut off from the UGV by scanned obstacles"). This is a necessary condition only (the planner also keeps a margin round obstacles). The NO-ROUTE advice in the coach now says what to do.
* The same stack with a **careful scan** (speed 35 %, hover over the UGV first, then a slow sweep north at ~5 m) gave 62 m2, 3454 of 5101 drivable cells connected, a goal 6 m away, and the UGV **arrived in 33 wall seconds**. So live-scan quality depends on how you fly; slow, low, and over the UGV first. (Not changed: the mapping parameters. Measuring scan density against speed would be a separate experiment.)
* The e-stop key/button now flips the page's own idea of the state at once (state messages already in flight are ignored for 0.7 s), because on a slow page two quick presses could both send "estop".
* The old browser suite had fixed sleeps that were too short for software rendering (cursor readout, gamepad, manual-mode switch); they now poll for the condition. Test timing, not product behaviour.

Remaining honesty: the real live map is speckled (one point in 16 is used and the camera is low-resolution), so a live scan does not look like the clean disc the pretend backend draws; this is the real map, not a rendering bug.


## P5 - the Settings card (2026-11-23)

**What it is.** A card in the side panel with a preset switch (gentle / normal / fast) and six sliders: UAV speed, climb, turn, flight ceiling, hand-driven UGV speed and turn. It changes the limits the *server* applies to hand-flying and hand-driving.

**How it works (newbie version).** The page never decides a limit. It sends `{"t":"settings","max_xy":2.0}`; the server (`commands.parse_command`) clamps every value into `HARD_CAPS` (`src/nav/nav/studio/commands.py`), swaps the limits in (`StudioNode._apply_settings`) and answers with a sticky `settings` event holding what it *really* applied. The sliders show only that answer, so they can never show a value the server refused. A new tab gets the current values in `hello`. Hard caps: UAV 0.5-5 m/s, climb 0.3-2.5 m/s, turn 0.3-2 rad/s, ceiling 3-14 m (never above the 14 m site limit; kept at least 2 m above the floor); UGV 0.2-1.0 m/s and 0.3-1.5 rad/s (the Jackal-class model tips over beyond that, step10).

**Scope limits (deliberate).** Only hand control is affected: the route-following UGV is limited by Nav2's own parameters, and the mapping/planner/safety settings are not exposed (changing those live is risky and needs a relaunch). Settings are not saved between server restarts.

**Verified.** 8 new unit tests (`tests/nav/test_studio_commands.py::TestSettings`: partial update, clamping, tip-over cap, NaN/bool/unknown/empty rejected); 3 new browser checks (11, 11b, 11c in `studio_firstrun_browser.py`: preset applies, one slider changes one limit, an absurd request is clamped to 5.0); on the real stack a websocket client asked `max_xy 99, ceiling 8, ugv_max_lin 0.5` and got back `5.0, 8.0, 0.5`. 601 nav tests pass.


## Auto-scan: the UAV maps the site by itself (2026-11-23)

**What it is.** The "Auto-scan the site" button (also offered by the coach on the ground) makes the server fly a lawnmower sweep: it climbs to 5 m, then flies parallel lanes 5 m apart over the site (the real site's boxes + 3 m, clipped to the flight fence; 52 x 52 m = 24 legs on W2). The same button then reads "Stop auto-scan".

**How it works.** Pure functions in `src/nav/nav/studio/scan_path.py`: `lawnmower(bounds, spacing)` makes the corner list; `autoscan_step(...)` gives one velocity step (climb first with no sideways motion, head for the corner at <= 1.2 m/s, slow down near it, hold height, hold yaw). `StudioNode._with_autoscan` adds it to the command pipeline *before* the flight fence and speed limits, so the Settings limits, the ceiling and the fence all still apply. Safety: any non-zero stick, the e-stop (also zeroes the UAV), or the Stop button ends it; it refuses while e-stopped; it finishes by hovering and the coach says to pick a goal.

**Verified.** 14 unit tests (`tests/nav/test_studio_scan_path.py`: lanes inside the area, edges covered, spacing, step direction, body-frame rotation, climb-first, command clamping); 3 browser checks (12, 12b, 12c: starts and moves, a flight key takes over, Stop button and e-stop end it; the pretend backend sweeps a small 12 m box); a real-simulator run: the UAV climbed to 5 m, flew the lanes by itself and the scanned area grew 51 -> 1265 m2 over 10 wall minutes (12 of 24 legs, RTF ~0.6).

**Honest limits.** A full sweep takes about **20 wall minutes** at the simulator's speed (about 12 sim minutes). The real run was stopped at the 10-minute cap, so **a finished auto-scan followed by a goal and a drive was not tested**; the careful hand scan that routed cleanly flew at a similar speed and height, but the equivalent claim for this sweep is not yet measured. The sweep covers the whole site; there is no corridor-only variant yet. The UAV has no obstacle sensing (5 m clears the containers, which are about 2.6 m tall).
