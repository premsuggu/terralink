# Step 1: UAV Deployment in Gazebo

**Package**: `src/emap/`
**Goal**: A real (non-box) UAV spawns in Gazebo, is physically simulated, and is controllable and observable from ROS 2.
**Status**: ✅ Complete and verified.
**Read first**: [`00_concepts.md`](00_concepts.md) — explains ROS 2, Gazebo, SDF, and "the bridge" from scratch. This document assumes you've read it and builds on that vocabulary.

---

## 1. What are we actually building, in plain terms?

We want a drone we can tell "go up" (from ROS 2) and watch it actually rise in the simulator, under real physics (gravity, propeller thrust, momentum) — not a scripted animation. That requires three things working together:

1. A **model** — the drone's physical shape, mass, and moving parts (its 4 propellers).
2. **Plugins** — code that turns "spin propeller 0 at X speed" into an actual upward push, and turns "the pilot wants 0.5 m/s upward" into 4 individual propeller-speed commands.
3. A **bridge** — so a ROS 2 command can reach into the simulator, and the simulator's sense of "where is the drone now" can reach back out to ROS 2.

The rest of this document walks through exactly how each piece was built, starting from how a quadrotor flies at all, since that's new territory for this step.

---

## 2. How does a quadrotor actually fly? (from scratch)

A quadrotor has 4 propellers ("rotors") arranged in a square, each spinning around a vertical axis. Two basic facts explain almost everything:

1. **Spinning a propeller pushes air down, which pushes the drone up** (Newton's third law — same idea as a fan pushing air one way and feeling a slight push back the other way, just much stronger). Push all 4 rotors equally hard and the drone goes straight up; ease off equally and it descends.
2. **A spinning propeller also twists its own arm slightly** (like the recoil from a spinning toy top) — this is called reaction torque. If all 4 propellers spun the *same* direction, the whole drone body would slowly spin too, which is not useful. The fix: **two rotors spin clockwise and two spin counter-clockwise**, arranged diagonally opposite each other, so their twists cancel out when everything's balanced. Deliberately spinning the two clockwise rotors slightly faster than the counter-clockwise ones (or vice versa) is, in fact, how the drone turns left/right (yaws) on purpose.

Tilting is how a drone moves sideways: spin the rotors on one side a bit harder than the other side, and the drone tips slightly in that direction — some of what was "pure upward thrust" becomes "sideways thrust" instead, the same way leaning a fan slightly makes it blow at an angle instead of straight ahead. A flight controller does this tilting/balancing continuously and automatically; you just say "go this direction" and it figures out the 4 individual rotor speeds.

### 2.1 Two separate jobs, two separate plugins

Gazebo splits this into two independent pieces of code, matching the two facts above:

- **`MulticopterMotorModel`** (one instance per rotor, so 4 total): the "physics of one spinning propeller." Give it a commanded rotational speed, and every simulated instant it computes the upward thrust and the reaction torque that speed would produce, and applies both to that rotor's joint. It knows nothing about the other 3 rotors or about "going up" — it only knows about *its own* propeller.
- **`MulticopterVelocityControl`** (one instance for the whole drone): the "pilot's brain." It receives a single command like *"move at 0.5 m/s upward, 0 sideways, don't yaw"*, compares that to the drone's current actual motion, and works out what speed each of the 4 rotors needs to spin at to achieve it — then sends those 4 speeds to the 4 `MulticopterMotorModel` plugins above.

This split is exactly why the same drone body can be flown by different "brains" without touching its propellers: swap `MulticopterVelocityControl` for a different controller later (e.g. one that takes GPS waypoints instead of velocities) and the 4 motor plugins don't change at all.

### 2.2 Why we didn't have to invent the numbers

Both plugin types need physical constants: how much thrust does a given rotor speed produce, how quickly can a motor speed up, etc. These aren't guesses — they came from a real, published drone design (details in [Section 4](#4-where-the-model-came-from)), so the physics is realistic rather than arbitrary.

---

## 3. Walkthrough: `models/iris_quad/model.sdf` (the airframe)

This file only describes shape/mass — no flight behavior yet (that's added in the world file, Section 5). Structure, following the vocabulary from `00_concepts.md`:

```
model "iris_quad"
├── link "base_link"       ← the drone's main body
├── link "rotor_0"  + joint "rotor_0_joint"  (revolute, spins around Z)  ← front-right, spins CCW
├── link "rotor_1"  + joint "rotor_1_joint"                              ← back-left,   spins CCW
├── link "rotor_2"  + joint "rotor_2_joint"                              ← front-left,  spins CW
└── link "rotor_3"  + joint "rotor_3_joint"                              ← back-right,  spins CW
```

Every rotor joint is `type="revolute"` with axis `0 0 1` (spins around the vertical/Z axis) and an essentially unlimited rotation range (`lower/upper = -1e16/1e16` — i.e. "spin freely forever," as opposed to a door hinge which stops at 90°).

**`base_link`**: mass 1.5 kg, a box-shaped `<collision>` roughly the drone's real footprint (0.47 × 0.47 × 0.11 m) for cheap, fast physics, but a detailed 3D mesh (`iris.stl`) for the `<visual>` — remember from `00_concepts.md` that collision and visual shapes can differ; physics only ever sees the simple box.

**Each `rotor_N` link**: tiny mass (5 grams — realistic for just a propeller), positioned at the actual arm-tip offset (e.g. rotor 0 sits at `x=0.13, y=-0.22` relative to the body center — that's the X-shaped arm layout), a cylinder collision shape, and a propeller-shaped visual mesh (`iris_prop_ccw.dae` or `iris_prop_cw.dae` depending on spin direction, matching the CCW/CW pairing explained in Section 2).

Notice there is **no plugin anywhere in this file.** That's deliberate — see Section 2.1: a model file only describes shape/mass; the *behavior* (turning a commanded speed into thrust) is attached separately, in whichever world includes this model. That happens next.

---

## 4. Where the model came from

Ignition Gazebo's own official multicopter example uses a drone called the "X3 UAV," downloaded on demand from Open Robotics' online model library ("Fuel"). We tried this first — but in this sandbox, connecting to Fuel's server succeeds (the network connection opens fine) and then the download simply never progresses, indefinitely. GitHub, by contrast, works normally here. Rather than depend on a fetch that never completes, we used an equivalent open-source drone whose files we *could* actually download: the **3DR Iris quadrotor**, published by the PX4 flight-controller project in their [`PX4/PX4-SITL_gazebo-classic`](https://github.com/PX4/PX4-SITL_gazebo-classic) GitHub repository (`models/iris/iris.sdf.jinja` — despite the `.jinja` template extension, everything we needed — links, joints, masses, meshes — is plain, already-filled-in SDF; only an unrelated section at the very end, for connecting to PX4's own flight-control software over the network, uses template placeholders, and we didn't need or use that section at all).

We copied over:
- the body + 4 rotors' shape, mass, and inertia,
- the 3 mesh files (`iris.stl`, `iris_prop_ccw.dae`, `iris_prop_cw.dae`),
- and the Iris's own published motor constants (how much thrust its motors produce, how fast they can spin, etc.) — used in Section 5 below, so the flight physics matches this specific airframe rather than being borrowed from an unrelated drone.

We did **not** copy the original file's flight-control plugins — those depend on Gazebo Classic (not installed here) and PX4's own software, neither of which we're using. We wrote our own plugin block instead, using Ignition Gazebo's built-in flight plugins from Section 2 — that's the entire content of Section 5.

Full provenance details are also recorded in `src/emap/models/iris_quad/model.config`.

---

## 5. Walkthrough: `worlds/uav_test.world` (the scene + the flight behavior)

A world file, per `00_concepts.md`, describes an entire scene. Ours has 4 parts, in order:

### 5.1 Simulation-wide setup
```xml
<physics name="4ms" type="ignored">
  <max_step_size>0.004</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```
This says: advance the physics simulation in steps of 4 milliseconds of simulated time each, and try to run at `1.0` = real-world speed (not fast-forwarded or slow-motion). Smaller steps are more physically accurate but slower to compute — 4 ms is the same value Ignition's own multicopter demo uses, a proven balance for this kind of vehicle.

Below that are four `<plugin>` lines (`Physics`, `SceneBroadcaster`, `UserCommands`, `Sensors`) — these are Gazebo's own always-needed internal systems (actually run the physics, publish what the world looks like, allow spawning/deleting things, and handle sensors respectively). Every Ignition Gazebo world needs these four; they're boilerplate, not specific to our drone.

### 5.2 The scene itself
A `<light type="directional">` (the sun) and a flat `<model name="ground_plane">` (a simple 100×100 m plane, both `<collision>` and `<visual>`, so the drone has something to rest on and we can see it). Nothing drone-specific here either — this is just "an empty field."

### 5.3 Placing our drone
```xml
<include>
  <name>iris_quad</name>
  <uri>model://iris_quad</uri>
  <pose>0 0 0.1 0 0 0</pose>
  ... plugins (5.4) ...
</include>
```
`<include>` is how a world *uses* a model file without repeating its contents — `model://iris_quad` tells Gazebo "look up a model folder named `iris_quad`" (found via the `GZ_SIM_RESOURCE_PATH` environment variable, set in our launch file — Section 7). `<name>iris_quad</name>` fixes what this specific copy is called in the running simulation (its "entity name") — important, because that name shows up literally in topic names like `/model/iris_quad/odometry`. `<pose>0 0 0.1 ...</pose>` places it 0.1 m above the ground at the world's center, so it starts just barely above the floor rather than starting inside it.

### 5.4 The flight plugins (the part that makes it actually fly)

Everything inside the `<include>` after the pose is a `<plugin>` — recall from `00_concepts.md` that a plugin is what gives a passive model actual behavior. We attach exactly the two plugin types from Section 2:

**Four `MulticopterMotorModel` blocks**, one per rotor. Each says "this joint is a rotor; here's how it converts a commanded speed into thrust and torque":
```xml
<plugin filename="ignition-gazebo-multicopter-motor-model-system"
  name="gz::sim::systems::MulticopterMotorModel">
  <robotNamespace>iris_quad</robotNamespace>
  <jointName>rotor_0_joint</jointName>
  <linkName>rotor_0</linkName>
  <turningDirection>ccw</turningDirection>
  <maxRotVelocity>1100</maxRotVelocity>
  <motorConstant>5.84e-06</motorConstant>
  ...
</plugin>
```
- `jointName`/`linkName` say *which* rotor this instance controls (matching the names from `model.sdf`, Section 3).
- `turningDirection` (`ccw`/`cw`) implements the "two spin one way, two the other" balancing from Section 2.
- `motorConstant` is the number that converts "rotor speed" into "thrust force" (physically: thrust ≈ `motorConstant × speed²`) — one of the Iris's own published values from Section 4, not something we invented.
- `maxRotVelocity` (1100) caps how fast the propeller can spin, same as a real motor has a top speed.
- `commandSubTopic` (`gazebo/command/motor_speed`) is the Gazebo-internal topic this plugin listens on for its target speed — but nothing publishes to it directly; that's the *next* plugin's job.

**One `MulticopterVelocityControl` block** — the "pilot's brain" from Section 2:
```xml
<plugin filename="ignition-gazebo-multicopter-control-system"
  name="gz::sim::systems::MulticopterVelocityControl">
  <robotNamespace>iris_quad</robotNamespace>
  <commandSubTopic>gazebo/command/twist</commandSubTopic>
  <comLinkName>base_link</comLinkName>
  <velocityGain>2.7 2.7 2.7</velocityGain>
  ...
  <rotorConfiguration>...</rotorConfiguration>
</plugin>
```
- `commandSubTopic` (`gazebo/command/twist`) is where it listens for the *single* "move like this" command — combined with `robotNamespace`, the full Gazebo topic name is `/iris_quad/gazebo/command/twist`. A "twist" is robotics terminology for "a linear velocity (x, y, z) plus an angular velocity (roll, pitch, yaw rate) bundled together" — the standard way to say "move like this" in one message.
- `velocityGain` / `attitudeGain` / `angularRateGain` are the controller's tuning knobs — roughly, "how aggressively do I correct when my actual speed/tilt doesn't match what was asked for." We reused Ignition's demo drone's tuned values as a reasonable starting point (they're a property of *how the controller reacts*, not of the airframe's shape, so borrowing them isn't inconsistent — though a follow-up note below flags that they may deserve retuning for this specific, heavier airframe).
- `rotorConfiguration` tells this plugin the same layout facts as the 4 motor-model blocks above (which joint is which, and which direction it spins) so it can work out the right 4-way split for any commanded motion.

**One `OdometryPublisher` block**: continuously publishes the drone's current position, orientation, and velocity onto `/model/iris_quad/odometry` — this is how anything outside the drone (including, later, our bridge) finds out where it currently is.

---

## 6. Walkthrough: `config/bridge.yaml` (crossing from Gazebo into ROS 2)

Recall from `00_concepts.md` that Gazebo and ROS 2 are two separate messaging systems, and `ros_gz_bridge` is the translator between them. Its config file is a list of "translate this topic" rules:

```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/iris_quad/gazebo/command/twist"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "ignition.msgs.Twist"
  direction: ROS_TO_GZ
```
Read as: *"Watch the ROS 2 topic `/cmd_vel`. Whenever a `Twist` message is published there, convert it to Gazebo's equivalent `Twist` type and republish it on the Gazebo topic `/iris_quad/gazebo/command/twist`."* `direction: ROS_TO_GZ` means one-way, ROS → Gazebo only (that's exactly the topic `MulticopterVelocityControl` is listening to, from Section 5.4 — so publishing to `/cmd_vel` in ROS 2 now drives the drone).

The other two rules go the opposite way (`GZ_TO_ROS`): Gazebo's `/model/iris_quad/odometry` (published by `OdometryPublisher`, Section 5.4) becomes ROS 2's `/odom`, and Gazebo's simulated clock becomes ROS 2's `/clock` — needed so any ROS 2 node that cares about time uses *simulated* time instead of the real wall clock (important once things run slower/faster than real time).

---

## 7. Walkthrough: `launch/uav_sim.launch.py` (starting everything together)

Recall from `00_concepts.md`: a launch file starts a group of programs together. Ours starts two: Gazebo itself, and the bridge. Three things worth calling out:

1. **`GZ_SIM_RESOURCE_PATH`** is set to point at our package's `models/` folder, *before* Gazebo starts. This is what makes `<uri>model://iris_quad</uri>` (Section 5.3) resolvable — without it, Gazebo has no idea where to look for a folder named `iris_quad`.
2. **`headless` launch argument** (default `true`): Gazebo can run with a visible 3D window (`-r <world>`) or purely as a background physics server with no window (`-r -s <world>`, the `-s` meaning "server only"). We default to headless, per this project's convention of running simulations headlessly unless a GUI is actually needed — everything about "is the drone actually flying" can be checked from the numbers in `/odom` alone, no window required. Pass `headless:=false` to pop the window when you do want to see it (this environment can show one — see Section 8).
3. **The bridge node** is started with our `config/bridge.yaml` (Section 6) passed in as a parameter, so all three topic translations start immediately alongside Gazebo.

---

## 8. How we verified it actually works

Verification here means: prove the drone is a real, physically-simulated, controllable vehicle — not a static prop sitting there for show. Each check below rules out a specific way that could be faked or broken:

- **`colcon build --packages-select emap` succeeds** — rules out basic file/packaging mistakes.
- **`ros2 launch emap uav_sim.launch.py` starts with no errors**, and the log confirms all 4 `MulticopterMotorModel` instances plus `MulticopterVelocityControl` and `OdometryPublisher` loaded — rules out a silently-missing plugin (which would leave the drone limp/uncontrollable even though the simulation "runs").
- **`ros2 topic list` shows `/clock`, `/cmd_vel`, `/odom`** — rules out the bridge not actually starting or misconfigured topic names.
- **At rest, `/odom` reports a stable pose** (settled at z≈0.055 m, having started at z=0.1 m) — shows gravity and ground contact are both working (it fell the small remaining distance and then stopped, rather than falling through the floor or floating).
- **The real test — commanding motion from ROS 2 itself**:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.6}}"
  ```
  This publishes one `Twist` message asking for 0.6 m/s upward. Over the next 4 seconds, `/odom` showed height rising from z≈0.05 m to z≈2.73 m, while x/y stayed essentially at zero. That rules out the biggest risk in this step: that the model might *look* right but not actually be flyable (wrong joint names, misconfigured plugin, wrong topic wiring). A command that entered through ROS 2, crossed the bridge, reached the Gazebo controller, and produced a real, physically consistent climb (straight up, no drift) is direct proof the whole pipeline — ROS 2 → bridge → Gazebo plugins → physics → back out through `/odom` — works end to end.
  - Sending a zero `Twist` afterward brought vertical velocity back to ~0 and held a steady hover (with some overshoot before settling, since the controller gains are the borrowed starting point noted in Section 5.4, not yet tuned for this airframe — not a concern for what this step set out to prove).

## Follow-ups for later steps

- Retune `velocityGain`/`attitudeGain`/`angularRateGain` for the Iris's actual mass/inertia if flight response needs to be tighter later (currently the borrowed X3-demo starting point from Section 5.4).
- If Fuel connectivity is ever available in this sandbox, the real X3 UAV mesh could replace `iris_quad`'s visuals without touching the flight-plugin block (same joint names throughout).
- Step 2 will add a downward-facing depth camera link + camera sensor plugin to this airframe, and introduce `PointCloud2` — see `00_concepts.md` for anything foundational and the step 2 doc for what's new there.
