# emap Concepts: The Absolute Basics

This file explains the foundational ideas every `emap` step builds on. Read this once, first. Each `stepNN_*.md` file then only explains what's *new* in that step, and links back here for anything foundational.

If a word is unfamiliar anywhere else in `docs/work-docs/emap/`, it's probably explained here or in the glossary at the bottom.

---

## 1. Why do we simulate at all?

We're building software that will eventually fly a real drone. Testing new code directly on a real, expensive, crash-prone drone is slow and risky. Instead, we test in a **simulator**: a program that pretends to be the real world closely enough that code written against it also works on the real thing (with some tuning).

A simulator needs to fake two things convincingly:
- **Physics** — gravity, collisions, how a spinning propeller actually pushes a drone upward.
- **Sensors** — what a camera or GPS *would* see/measure if the drone were really there.

We use **Gazebo** (specifically "Ignition Gazebo Fortress", a newer rewrite of the older "Gazebo Classic") as our simulator.

## 2. What is ROS 2, in one paragraph?

ROS 2 (Robot Operating System 2) is not an operating system — it's a messaging framework for robotics. A robot's software is split into many small independent programs called **nodes** (e.g., "the camera driver", "the flight controller", "the mapping algorithm"). Nodes talk to each other by publishing and subscribing to **topics** — named channels carrying a stream of typed messages. A node doesn't need to know who's listening; it just publishes `Twist` messages on `/cmd_vel` and anyone interested subscribes to `/cmd_vel`. This is the same pattern radio broadcast uses: a station broadcasts, and any radio tuned to that frequency receives it, with neither side needing to know about the other.

A **launch file** is just a script that starts a group of nodes (and other programs) together with the right settings, instead of you typing several commands into several terminals by hand. Ours are Python files under `launch/`.

## 3. Gazebo and ROS 2 are two separate programs

This is the single most important thing to understand about everything in `src/emap/`:

**Gazebo and ROS 2 do not know about each other by default.** Gazebo has its own internal messaging system (called "Ignition Transport", with its own topics like `/model/iris_quad/odometry`) that is completely separate from ROS 2's topics (like `/odom`). They look similar (both are "publish a named, typed message") but they are two different systems that happen to use a similar idea.

To connect them, we run a translator program in between: `ros_gz_bridge`. You tell it "take everything published on Gazebo topic X and republish it as ROS 2 topic Y (and/or vice versa)", and it copies messages across, converting between the two systems' message formats. We call this **the bridge**. Step 1 introduces our bridge config (`config/bridge.yaml`) — see `step01_uav_gazebo_deployment.md` for the full walkthrough.

```
 ┌────────────┐   Ignition topics    ┌───────────────┐   ROS 2 topics   ┌────────────┐
 │   Gazebo    │ ───────────────────▶│  ros_gz_bridge │─────────────────▶│  ROS 2 node │
 │ (simulator) │◀─────────────────── │  (translator)  │◀─────────────────│ (our code)  │
 └────────────┘                      └───────────────┘                  └────────────┘
```

## 4. SDF: how you describe *what exists* in a simulation

**SDF** (Simulation Description Format) is an XML file format Gazebo uses to describe everything in a simulated world: the ground, the lighting, and every robot/object, down to their exact shape, mass, and how their parts connect. Think of it as a blueprint document, not a program — it doesn't run anything by itself; Gazebo reads it and builds the scene.

Two kinds of SDF file you'll see in `src/emap/`:
- A **world file** (`worlds/*.world`) — describes an entire scene: lighting, ground, physics settings, and which robot models are placed where.
- A **model file** (`models/<name>/model.sdf`) — describes one reusable object/robot, so it can be placed into any world via `<include>`.

### 4.1 The vocabulary, with a minimal example

Every physical object in SDF is a `<model>` made of one or more `<link>`s connected by `<joint>`s.

- **`<link>`** — one rigid, solid piece (nothing bends within a link). A simple box has one link; a robot arm has several links, one per segment.
- **`<joint>`** — connects two links and says how they're allowed to move relative to each other: `fixed` (bolted together, never moves), `revolute` (rotates around one axis, like a hinge or a wheel/propeller), `prismatic` (slides), etc.
- **`<inertial>`** — the link's mass and *how that mass is spread out* (its "moment of inertia", roughly: how hard it is to start/stop it spinning). Needed for physics to compute how forces move it.
- **`<collision>`** — the invisible shape the physics engine uses to detect this link hitting other things. Usually a simple shape (box, cylinder) for speed.
- **`<visual>`** — the shape actually drawn on screen. Can be a detailed 3D mesh even if the collision shape is a plain box — the physics doesn't care what it looks like, only what it collides as.
- **`<pose>`** — a position + orientation (`x y z roll pitch yaw`), used everywhere to say "this thing is located/oriented here, relative to its parent."

A tiny complete example — a link that's just a floating box, with no joints:

```xml
<model name="simple_box">
  <link name="box_link">
    <inertial>
      <mass>1.0</mass>
      <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
                <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="col"><geometry><box><size>1 1 1</size></box></geometry></collision>
    <visual name="vis"><geometry><box><size>1 1 1</size></box></geometry></visual>
  </link>
</model>
```

That's it — one link, a box-shaped collision and visual, and inertia numbers so physics can simulate it falling/bouncing. Our UAV model (`step01`) is the same idea repeated 5 times (1 body + 4 rotors) and connected with `revolute` joints.

### 4.2 Plugins: giving a model *behavior*

By itself, SDF only describes static shape and mass — it has no concept of "motor" or "sensor" or "spin this". A **`<plugin>`** is a small piece of compiled code, loaded by Gazebo at run time, that's given access to one part of the simulation and does something active with it every simulated time step — e.g. "read a commanded speed and push this joint accordingly" or "render a camera image every frame". Plugins are how anything *dynamic* happens in Gazebo; the SDF model itself stays passive geometry. `step01` covers the specific plugins that make our drone fly.

## 5. `colcon` and packages, briefly

ROS 2 code is organized into **packages** — self-contained folders with a `package.xml` (metadata) that group related nodes, launch files, and config. `colcon build` compiles/installs all packages it finds under `src/`. `colcon build --packages-select emap` builds only the `emap` package. After building, you must `source install/local_setup.bash` in your terminal so ROS 2 knows where to find what was just built — this is required after every `colcon build`, every new terminal.

---

## 6. Coordinate frames and TF (added in step 2)

A **coordinate frame** is just an origin point plus three perpendicular axes (X, Y, Z) that you measure positions relative to. "The camera is at (0, 0, -0.08)" is meaningless on its own — you have to say *relative to what*. Relative to the drone's body? Relative to the world? Relative to yesterday's starting point? Every measurement in robotics is relative to *some* frame, and a robot typically has many frames at once: one for the world, one for its own body, one for each sensor, sometimes one for each part of an arm.

Why not just use one single frame for everything? Because different things are naturally easiest to describe in different frames, and some of those frames move relative to each other. The camera's position *relative to the drone's body* never changes (it's bolted on) — that's easy and constant. The drone's position *relative to the world* changes constantly as it flies — that's a separate, moving relationship. Trying to force everything into one frame means recomputing the camera's absolute world position by hand every time the drone moves, for every sensor, by hand, in every piece of code that needs it. Instead, robotics software builds a **tree of frames**, each one only describing its relationship to its *immediate parent*:

```
iris_quad/odom  (≈ the fixed world origin)
  └── iris_quad/base_link            (moves: the drone's live position/orientation)
        └── iris_quad/camera_link/rgbd_camera   (fixed: bolted to the body, never changes)
```

Each arrow in that tree is one **transform**: a translation (x, y, z) plus a rotation, saying "here's where the child frame is, and how it's rotated, relative to its parent." Critically, each transform only needs to know about its *one direct neighbor* — nothing needs to know the whole tree. If you want to know "where is a camera measurement in the world," you don't need any single piece of code that understands the whole chain by hand; you just ask a library to walk the tree and combine the transforms for you. That library is **TF** (`tf2` in ROS 2): every node that knows a transform *broadcasts* it (onto the special topics `/tf` for transforms that change over time, or `/tf_static` for transforms that never change, like a bolted-on sensor), and any node that needs to convert between two frames asks `tf2`'s `Buffer`/`TransformListener` to look it up and do the combined math, however many links apart the two frames are.

This is exactly the tree `step02` builds: a moving `iris_quad/odom → iris_quad/base_link` transform (from the drone's live flight, published because it changes every instant) and a fixed `iris_quad/base_link → iris_quad/camera_link/rgbd_camera` transform (published once, because a bolted-on mount never moves relative to the body it's bolted to).

## 7. Point clouds (added in step 2)

A **point cloud** is simply a list of 3D points — `(x, y, z)` for each point a depth sensor measured, all expressed in that sensor's own coordinate frame (Section 6). In ROS 2 this is the `sensor_msgs/msg/PointCloud2` message type: conceptually a big list of points, though the message itself stores them as a flat binary blob (for speed — a real cloud can be hundreds of thousands of points, and re-parsing text for each one would be far too slow) alongside a description of how to decode that blob (which bytes are `x`, which are `y`, etc.) and a `frame_id` saying which coordinate frame every point in it is measured in. A depth/RGB-D camera like ours produces one of these every time it takes a "picture" — one 3D point per pixel, at the distance that pixel's ray hit something solid.

One easy point of confusion, worth flagging early: it's tempting to assume a camera's local axes are always "X = right, Y = down, Z = forward (the direction it's looking)" — this is a real, common convention (called the *optical frame* convention) used by many camera drivers. But it is a *convention*, not a law of physics, and step02 found that Ignition's simulated RGB-D sensor does **not** follow it — its point cloud instead has the viewing/depth direction along local **X**. The lesson isn't "memorize which axis is which" — it's: **when in doubt about a sensor's axis convention, don't guess or assume — publish real data and look at the actual numbers** (step02 does exactly this, and documents what it found).

## 8. 2.5D height grids and uncertainty (added in step 3)

An elevation map's job is to answer "how high is the ground at this (x, y) location?" for every location in some area. The most literal way to store that would be a full 3D grid: split the world into tiny cubes ("voxels") and mark which ones are solid ground. That works, but it's wasteful for this project's purpose — we don't care about overhangs, caves, or anything below the surface, only "what's the height of the ground here," which is a *single number* per (x, y) location, not a whole column of voxels. Storing one height value per (x, y) cell instead of a full 3D volume is dramatically cheaper in both memory and computation, and is exactly what "2.5D" means: a 2D grid (rows/columns of cells), each carrying one extra number (height) — "2D plus a bit," not full 3D.

Concretely, that means the map is one 2D array (`elevation` in the code) where `elevation[row, col]` is the terrain height at whatever real-world (x, y) that cell corresponds to (Section 4's coordinate-frame idea, applied to a grid instead of a robot part). Converting between "a real (x, y) position" and "a (row, col) index into that array" is ordinary arithmetic (scale by the cell size, shift so the map's center lands in the middle of the array) — `step03` walks through exactly that conversion with real numbers.

**Why also track a `variance` layer, not just `elevation`?** Because no sensor measurement is perfect — a depth camera's estimate of "the ground is 3.57m below me" always carries some amount of noise/uncertainty. If we only stored the latest height, a single noisy outlier measurement could wildly corrupt a cell that many earlier good measurements had already pinned down accurately. Instead, every cell keeps a second number, its **variance** — a statistical measure of *how uncertain* that cell's height estimate currently is (small variance = "we're pretty confident," large variance = "we barely have any idea yet"). A brand new cell starts with a deliberately huge variance ("no idea"); step 4 (Bayesian fusion) will show exactly how a real measurement's own uncertainty gets combined with a cell's existing variance so that confident, repeated measurements win out over noise, and a lone bad reading can't ruin an otherwise well-observed cell.

## 9. Combining two uncertain beliefs (added in step 4)

Imagine you have two thermometers reading the same room. One is a cheap thermometer you don't trust much (it's often off by a couple of degrees). The other is a lab-grade thermometer you trust closely (usually accurate to a tenth of a degree). If they disagree slightly, you wouldn't just average them 50/50 — you'd lean heavily toward the trustworthy one. If you had to combine them into a single best guess, the natural rule is: **weight each reading by how much you trust it, and let the more trustworthy one pull the combined answer closer to itself.**

That's exactly the situation every elevation-map cell is in, every time a new sensor measurement arrives for it. The cell already has a belief (its current `elevation` and `variance` from Section 8 — variance is "how much we trust this current number"). A new measurement arrives with its own height value and its own variance (a sensor is noisier for far-away points, so a measurement's variance depends on how far away it was taken from). Combining two independent, uncertain beliefs about the same true value has a clean mathematical answer:

```
combined_height   = (belief_height * new_variance + new_height * belief_variance) / (belief_variance + new_variance)
combined_variance = (belief_variance * new_variance) / (belief_variance + new_variance)
```

Look at what happens in the extremes: if the new measurement's variance is tiny (very trustworthy) compared to the belief's variance, `combined_height` ends up very close to `new_height` — exactly the "trust the lab thermometer" intuition. And `combined_variance` is always smaller than *either* input's variance — combining two independent observations can only make you more confident, never less (this is why a map cell's uncertainty shrinks as more measurements land on it, which `step04`'s tests confirm numerically).

One more piece: what if a measurement doesn't just disagree a little, but is wildly, suspiciously different from a belief we're already fairly confident about (e.g. a stray bad depth reading, or a bird flying through the camera's view)? Blindly fusing that in would corrupt a cell that many good measurements had already pinned down. So before fusing, every measurement is checked against how surprising it would be given the current belief; if it's too surprising, it's treated as an **outlier** — not fused into the height at all, though the cell's variance is still nudged up a little (something unexpected happened here, so maybe we should be a bit less sure than we were). `step04_bayesian_fusion.md` covers exactly how "too surprising" is decided, and works through the same formulas above with real numbers.

## 10. Why the map has to "follow" the robot (added in step 5)

Section 8 already decided the map is a fixed-size grid, not one that grows forever. But a UAV can fly a very long way from wherever it started — if the grid never moved, either it would have to be enormous (covering every place the UAV might ever go, mostly wasted memory) or it would quickly cover somewhere else entirely as the UAV flies away from its starting point, leaving the actual ground beneath the UAV unmapped.

The standard solution, used by both reference implementations in this project, is to keep the grid a fixed size but **periodically re-center it on the robot** — every so often, "forget" the patch of ground that's now far behind and "start fresh" on the patch that's newly nearby, while keeping whatever's still within the (fixed) window. Concretely, that means physically sliding the array's contents over so that cells still within view land back in the right position relative to the new center, and marking whatever edge just came into view as "never observed" (since the array literally has no old data for a place it's never covered before). `step05_map_shifting.md` covers exactly how this is done and, more importantly, exactly how it's proven correct — this is the single easiest place in the whole codebase to accidentally swap "which way is which" (row vs. column, or which edge is genuinely new vs. which edge should be left alone), so it gets its own dedicated, carefully-checked implementation rather than being folded into some other step.

## 11. GridMap: publishing a multi-layer map as one message (added in step 6)

Section 8 introduced the idea of a 2.5D grid with multiple named layers (`elevation`, `variance`, ...). `grid_map_msgs/GridMap` is the standard ROS 2 message type for sending exactly that kind of thing over a topic: one message carries the map's geometry (resolution, size, where it's centered - `info`) plus a list of layer names (`layers`) and, for each one, its actual data (`data`, one entry per layer). Anything that understands this message type - RViz's grid map plugin, or any other node - can then reconstruct the full multi-layer grid on the receiving end without needing separate messages (and separate synchronization) for every layer.

The one real gotcha (found and worked through in `step06_ros_node_integration.md`): each layer's data isn't just "the numbers in the obvious order." `GridMap` encodes each layer as a `Float32MultiArray` using a specific **column-major** layout (walk down each column fully before moving to the next), because that's the exact convention the receiving side (`grid_map_ros`, and therefore RViz) expects. Encoding it the "obvious" row-by-row way produces a message that's still technically valid ROS (right type, right number of floats) but decodes into a transposed/scrambled-looking map on the other end - a mistake that's easy to make and easy to miss without checking against something that actually decodes it correctly.

## 12. Traversability: from "how high" to "how easy to drive over" (added in step 7)

Everything through step 6 answers "how high is the ground here?" Traversability answers a different, more directly useful question for actually planning a route: "would it be a problem to drive/fly over this cell?" It's still stored as a per-cell number (Section 8's 2.5D idea), but it's *derived* from the map's other layers rather than measured directly by any sensor.

Three intuitive properties matter for that judgment, and step 7 computes each from things the map already tracks: how **steep** the ground is at a cell (a slope, computed from how much neighboring cells' heights differ), whether there's a sharp **ledge** nearby (a big height jump within a small neighborhood, which a smooth slope calculation can under-react to), and how **rough**/inconsistent the ground has measured out to be (reusing the cell's own `variance` from Section 9 - though see step07's docs for an honest caveat about that specific choice). Combining simple, named checks like these - rather than a single opaque score - keeps every decision explainable: a cell is flagged risky *because* it's steep, or *because* there's a ledge, not for a reason nobody can point to.

## 13. Local vs. global maps (added in step 8)

Step 5 gave the map a fixed size that re-centers on the robot ("rolling window") so it never has to grow forever. That's genuinely the right choice for one purpose - a small, fast, always-nearby picture of the immediate surroundings, useful for quick reactions - but it's the wrong choice for a different, equally real purpose: **planning a route across an area larger than what's currently in view**. A path planner has to be able to say "go around that hill" even after the robot has flown past it and it's no longer anywhere in the current sensor view - if the map forgets everything outside a small moving window, the planner has no memory of the hill at all once it scrolls out of range.

The standard answer (this is genuinely how real navigation stacks like Nav2 are built, not a one-off idea for this project) is to keep **both**, side by side, fed by the same sensor data: a small **local** map that keeps re-centering (for fast, nearby reactions) and a larger **global** map that never re-centers and never forgets (for planning across the whole known area). Nothing about the underlying `ElevationMap`/fusion/traversability code needs to be different between the two - the only difference is whether something ever calls `move_to` on a given map instance. A map that's never told to move just keeps quietly accumulating everything it's shown, forever, at a fixed spot - exactly what a persistent record of "everywhere I've ever looked" needs to do.

## 14. GPU acceleration: same math, different processor (added in step 9)

A CPU is good at doing many DIFFERENT things quickly, one after another. A GPU is built the opposite way: thousands of small, simple processing units that are only good at doing the SAME operation many times over, all at once. That mismatch usually makes a GPU useless for most code - but `fuse_points` (Section 9) does exactly the same handful of arithmetic steps to every point in a batch, completely independently of every other point. That's precisely the shape of work a GPU is good at, so running the identical algorithm there instead can make it faster.

CuPy is a library that makes this almost free to try: it re-implements NumPy's own functions (`zeros`, `add.at`, elementwise math) so that they run on the GPU instead, with nearly the same code. `fuse_points_gpu` (step 9) is literally `fuse_points` with `cp.` in place of `np.` in the handful of places the fusion math happens - the algorithm itself, and everything it was already unit-tested against, is unchanged.

One real cost this introduces: a GPU has its OWN separate memory, so data has to be copied there and back (host↔device transfer) - that copy isn't free, and for a small enough batch of data it can cost more time than the GPU saves. This project measured that honestly rather than assuming "GPU = faster" (see step09_gpu_acceleration.md for the actual numbers) instead of just turning it on and hoping.

## 15. Pose drift and how to correct it (added in step 10)

A real robot never knows its own exact position - it only has an ESTIMATE, built by continuously integrating noisy sensor readings (wheel rotation, IMU acceleration) over time. Every tiny error in those readings adds up the longer the robot runs, so the estimate slowly wanders away from the truth - this is called **drift**. It's not a malfunction; it's an unavoidable property of estimating position by accumulation instead of measuring it directly.

Why this matters for a map: every point this project fuses is placed using the CURRENT pose estimate (Section 6's TF lookups). If that estimate has drifted - say the robot believes it's 30cm higher than it truly is - then every point measured right now gets stamped 30cm higher than it should be, even though the sensor itself measured correctly. Fly over the same hill before and after some drift has accumulated, and the map doesn't get one clean hill - it gets two overlapping, slightly-offset copies of it.

The fix used here treats the MAP as the source of truth once it's confident about something: a cell that's been fused many times and agrees with itself has low `variance` (Section 9) - the map trusts it. If a fresh batch of points lands on those same trusted cells and reads a systematically different height, the simplest, most likely explanation isn't "the ground changed" - it's "the pose estimate is currently wrong", and that disagreement can be used to correct the pose going forward, without ever overwriting the trusted cell itself. This project only corrects the VERTICAL (Z) part of that drift, since "expected height at this (x, y)" is exactly what the map already stores - correcting horizontal (X/Y) drift properly needs matching the whole shape of a new scan against the map to find a 2D offset (scan-matching), a materially bigger algorithm this project didn't need to build to demonstrate the core idea.

The correction is also applied gradually (a small fraction of each new estimate, not the whole thing at once) rather than snapping instantly - a single noisy measurement shouldn't be allowed to yank the pose around; nudging it a little every time evidence comes in converges to the right answer over several updates while staying robust to any one bad reading.

## Glossary

| Term | Meaning |
|---|---|
| Node | One running ROS 2 program, usually doing one job |
| Topic | A named, typed stream that nodes publish to / subscribe from |
| Launch file | A script that starts several nodes/programs together |
| Gazebo / `gz sim` | The physics + sensor simulator we use |
| Ignition Transport | Gazebo's own internal messaging system (separate from ROS 2 topics) |
| Bridge (`ros_gz_bridge`) | Translator program that copies messages between Gazebo topics and ROS 2 topics |
| SDF | XML format describing simulated worlds and models |
| Model | One object/robot in SDF, made of links + joints |
| Link | One rigid piece of a model |
| Joint | Connection between two links, defining how they can move relative to each other |
| Inertial | A link's mass + how that mass is distributed (affects how forces spin/move it) |
| Collision | The (usually simplified) shape physics uses for contact detection |
| Visual | The shape actually rendered on screen |
| Pose | Position + orientation (`x y z roll pitch yaw`) |
| Plugin | Compiled code loaded into a simulation to make something behave dynamically |
| Package | A folder of related ROS 2 code with a `package.xml` |
| `colcon build` | The command that compiles/installs ROS 2 packages |
| Coordinate frame | An origin + 3 axes that positions are measured relative to |
| Transform | A translation + rotation describing one frame relative to its parent |
| TF / `tf2` | ROS 2's system for broadcasting and looking up transforms between frames, published on `/tf` (moving) and `/tf_static` (fixed) |
| Point cloud / `PointCloud2` | A list of 3D points a depth sensor measured, in that sensor's own frame |
| Optical frame convention | A common (not universal) camera-axis convention: Z=forward, X=right, Y=down |
| 2.5D | A 2D grid where each cell also stores one height value, instead of a full 3D voxel grid |
| Variance | A statistical measure of uncertainty - how much a value might be off from the truth |
| Bayesian fusion | Combining two uncertain beliefs by weighting each by how much it's trusted |
| Outlier | A measurement too inconsistent with an existing confident belief to be trusted |
| Map shifting / re-centering | Sliding a fixed-size grid's contents so it keeps following the robot |
| `GridMap` message | The standard ROS 2 message for a multi-layer 2.5D grid map, geometry + all layers in one message |
| Traversability | A per-cell score for how easy/safe that terrain is to drive over |
| Heightmap (SDF) | A grayscale image used to build real 3D terrain in a Gazebo world |
| Local map | A small map that re-centers on the robot - fast, nearby, forgets far-away ground |
| Global map | A larger map that never re-centers - persistent memory of everywhere ever seen |
| GPU / CUDA kernel | A separate processor built for doing the same operation on many data points at once; a "kernel" is one such operation |
| CuPy | A library mirroring NumPy's API but running the same array operations on the GPU |
| Host / device (transfer) | "Host" = the CPU's memory, "device" = the GPU's own separate memory - moving data between them isn't free |
| Pose drift | The slow accumulation of small errors in a robot's estimate of its own position over time |
| Drift compensation | Detecting and correcting pose drift by comparing new measurements against already-confident map data |
| Global map | A large map that never re-centers - persistent memory of everywhere ever seen |
