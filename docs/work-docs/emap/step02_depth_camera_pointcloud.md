# Step 2: Depth Camera + Point Cloud Pipeline

**Package**: `src/emap/`
**Goal**: Mount a downward-facing depth/RGB-D camera on the UAV, get its point cloud into ROS 2, and have a correctly-published coordinate-frame (TF) tree so we — and later, an elevation-mapping node — can know exactly where in the world each point actually is.
**Status**: ✅ Complete and verified.
**Read first**: [`00_concepts.md`](00_concepts.md), especially the new Section 6 (Coordinate Frames & TF) and Section 7 (Point Clouds) added for this step. This document builds directly on both.

---

## 1. What are we adding, in plain terms?

Step 1 gave us a UAV that can fly. Flying is useless for mapping if the UAV can't *see* anything. This step adds one sensor — a depth camera pointed straight down — and gets its data all the way into ROS 2, correctly located in space. Concretely, three new things:

1. A **camera**, physically mounted on the drone, pointed down.
2. That camera's data (a stream of 3D points — Section 2 below) reaching ROS 2, not just Gazebo.
3. A **coordinate-frame tree** so that "point X was measured 3 meters below the camera" can be converted into "point X is at this location in the world" — the actual point of having a downward camera at all.

We treat all three as things to be *proven with real numbers*, not assumed — Section 5 is the most important part of this document for exactly that reason.

## 2. What is a depth / RGB-D camera actually measuring?

A normal camera tells you *what color* each pixel is. A **depth camera** additionally tells you *how far away* whatever that pixel is looking at is. An **RGB-D camera** does both at once: color and depth, pixel for pixel (real hardware examples: Intel RealSense, Microsoft Kinect). Combine the depth value for every pixel with that pixel's viewing angle (known from the camera's field of view and resolution), and you can compute an actual 3D point in space for every pixel — do that for the whole image, and the result is a **point cloud**: one 3D point per pixel, all bundled into one message. Section 7 of `00_concepts.md` covers the message format itself (`PointCloud2`).

Ignition Gazebo simulates this with a sensor type called `rgbd_camera` — you describe its resolution/field of view/range in SDF (Section 3 below), and Gazebo computes, every simulated frame, exactly what such a camera would see given the actual 3D scene and the camera's actual current position — the same way it computes real physics for the drone's rotors in step 1, just for optics instead of forces.

## 3. Walkthrough: adding the sensor to `models/iris_quad/model.sdf`

Recall from `00_concepts.md` Section 4 that a `<sensor>` lives inside a `<link>`, and a plugin isn't needed to make a sensor work (unlike the rotors in step 1) — Ignition's general-purpose `Sensors` system (already loaded in our world since step 1) automatically renders and publishes data for *any* sensor it finds on *any* link, no per-sensor plugin required.

We added one new link, one new joint, and the sensor itself:

```xml
<link name="camera_link">
  <pose>0 0 -0.08 0 1.5708 0</pose>
  ...
  <sensor name="rgbd_camera" type="rgbd_camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>320</width><height>240</height></image>
      <clip><near>0.1</near><far>20.0</far></clip>
    </camera>
    <update_rate>10</update_rate>
    <topic>iris_quad/rgbd_camera</topic>
  </sensor>
</link>
<joint name="camera_joint" type="fixed">
  <child>camera_link</child>
  <parent>base_link</parent>
</joint>
```

- **`type="fixed"` joint**: unlike the rotor joints from step 1 (`revolute`, free to spin), `fixed` means "these two links never move relative to each other, ever" — exactly what a bolted-on camera mount is. This single word is what later lets us treat the camera's position relative to the body as a constant (Section 4).
- **`camera_link`'s `<pose>0 0 -0.08 0 1.5708 0</pose>`**: position `(0, 0, -0.08)` places it 8 cm below the body's center (clear of the body box, which is 11 cm tall — see step01), and the last three numbers are its roll/pitch/yaw rotation — `pitch = 1.5708` radians (90°) tips the sensor from "looking forward" to "looking straight down." *How* we know this specific number is right, rather than just asserted, is the entire subject of Section 5 — we didn't stop at writing this line, we checked it.
- **`horizontal_fov` / `image` / `clip`**: how wide a view the camera has (1.047 rad ≈ 60°), its resolution (320×240 — modest on purpose, to keep each point cloud message small while we're just proving the pipeline works), and its usable range (0.1–20 m — matches flying a few meters to a couple dozen meters above terrain).
- **`<topic>iris_quad/rgbd_camera</topic>`**: without this, Gazebo picks a long auto-generated topic name from the model/link/sensor hierarchy. Setting it explicitly gives predictable topic names: Gazebo ends up publishing `/iris_quad/rgbd_camera/image`, `.../depth_image`, `.../points`, and `.../camera_info`.

## 4. Walkthrough: `config/bridge.yaml` additions

Recall from `00_concepts.md` Section 3 that Gazebo and ROS 2 need `ros_gz_bridge` to talk to each other at all. We added four more translation rules (on top of step 1's three):

```yaml
- ros_topic_name: "/camera/points"
  gz_topic_name: "/iris_quad/rgbd_camera/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "ignition.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```
...plus the equivalent for `/camera/image_raw` (the color image) and `/camera/camera_info` (the camera's calibration/resolution metadata — needed by many downstream tools even though we don't use it yet), all one-way Gazebo → ROS since nothing needs to command a camera.

The fourth rule is the interesting one:
```yaml
- ros_topic_name: "/tf"
  gz_topic_name: "/model/iris_quad/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "ignition.msgs.Pose_V"
  direction: GZ_TO_ROS
```
This bridges Gazebo's own pose information into ROS 2's `/tf` topic (Section 6 of `00_concepts.md`) — but read Section 5 below, because this rule alone turned out to give us *less* than we first assumed, and fixing that is the actual interesting part of this step.

## 5. The real lesson of this step: verify, don't assume

We went in with a plan-stage assumption: that bridging `/model/iris_quad/pose` would hand us *every* link's pose (body, all 4 rotors, the new camera) as one big TF tree, for free. Running it and actually reading `/tf` proved that assumption wrong:

```
$ ros2 topic echo /tf --once
transforms:
- frame_id: iris_quad/odom
  child_frame_id: iris_quad/base_footprint
  ...
```

Only **one** transform came through — the drone's own live position (from the `OdometryPublisher` plugin already in use since step 1), nothing about `camera_link` at all. In hindsight this makes sense (that specific Gazebo topic is that *specific plugin's* output, not a general "every link's pose" feed) — but the important habit here is that we found this out by *reading the actual topic*, not by re-reading the plugin documentation more carefully after the fact. This is precisely the discipline `00_concepts.md` Section 7 calls out: check real data before trusting an assumption, especially about geometry.

**The fix** turned out to be simpler than the original assumption anyway: since `camera_link` is bolted to `base_link` with a `fixed` joint, its position relative to the body *never changes* — there's no need to ask Gazebo for it every frame at all. We publish it **once**, as a **static** transform (`/tf_static` in `00_concepts.md` Section 6), using a standalone `tf2_ros` node added to the launch file:

```python
Node(
    package='tf2_ros', executable='static_transform_publisher', name='camera_static_tf',
    arguments=[
        '--x', '0', '--y', '0', '--z', '-0.08',
        '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
        '--frame-id', 'iris_quad/base_link',
        '--child-frame-id', 'iris_quad/camera_link/rgbd_camera',
    ],
)
```
The numbers are exactly `camera_link`'s `<pose>` from `model.sdf` (Section 3) — one mount, described once, in one place, kept consistent by eye since it's such a short file; a larger project would generate this from the same source instead of copying it by hand. The child frame name (`iris_quad/camera_link/rgbd_camera`) isn't something we chose — it's the exact `frame_id` Gazebo itself stamps on the camera's point cloud messages, found (again) by reading the actual message rather than guessing:
```
$ ros2 topic echo /camera/points --once --no-arr
header:
  frame_id: iris_quad/camera_link/rgbd_camera
```

We also made one small related fix: the `OdometryPublisher` plugin's *default* frame names are `iris_quad/odom` → `iris_quad/base_footprint` — a mismatch with the actual link name in our own model (`base_link`). We set `<robot_base_frame>iris_quad/base_link</robot_base_frame>` explicitly in the world file so the TF tree's names match the model's own link names, rather than leaving a confusing, easy-to-misread naming mismatch in place.

The result is exactly the tree described in `00_concepts.md` Section 6:
```
iris_quad/odom  (published on /tf, changes every instant - the drone's flight)
  └── iris_quad/base_link
        └── iris_quad/camera_link/rgbd_camera  (published once on /tf_static - a fixed mount)
```

### Did we also need to double-check the *rotation itself* (pitch = 1.5708)?

Yes — and we did, the same way: not by trusting the number, but by reading real point-cloud data (full walkthrough of exactly how in Section 6's Verification). It turned out to be right on the first try, but we didn't know that until we checked.

## 6. Verification

Every check below either builds confidence or would have caught a real mistake:

1. **Build and launch**: `colcon build --packages-select emap` succeeds; `ros2 launch emap uav_sim.launch.py` starts Gazebo, the bridge, and the new static-TF node with no errors.
   - **One real environment problem, found and fixed here**: on first attempt, `gz sim` **crashed** (`Ogre::UnimplementedException` in `GL3PlusTextureGpu::copyTo`) the moment it tried to render the new camera sensor. This machine's GPU driver, under WSL2, is Mesa's OpenGL-over-D3D12 translation layer (confirmed with `glxinfo`) — not a native Linux GPU driver — and it's missing a texture-copy operation Ignition's renderer needs. The fix: force Mesa's `llvmpipe` software rasterizer instead (`LIBGL_ALWAYS_SOFTWARE=1`, set automatically in the launch file, with a comment explaining exactly why). Slower, but correct — and this only matters for *rendering* (i.e. only because we added a camera); step 1 never hit this because nothing needed to render anything.
2. **Topics exist**: `ros2 topic list` shows `/camera/points`, `/camera/image_raw`, `/camera/camera_info`, `/tf`, and `/tf_static`.
3. **Climb first**: exactly like step 1's verification, we commanded the UAV up (`/cmd_vel` linear.z, then zero to hover) before looking at camera data — at rest on the ground, a downward camera would be almost touching the floor, telling us nothing useful.
4. **The camera data itself makes physical sense**: reading real points from `/camera/points` while hovering (using `sensor_msgs_py.point_cloud2` in a small throwaway script — not guesswork) showed the depth value constant at ≈3.57 m across the entire image, with only lateral (image-plane) values varying. This is *exactly* the expected signature of a flat surface viewed perpendicular to the camera — a plane directly below a straight-down-pointed camera has the same perpendicular distance everywhere, while the sideways spread grows with the field of view. This one number was the first real evidence the mount rotation was correct.
5. **The definitive test — transform real points through the real TF tree and check where they land**: using `tf2_ros`'s `Buffer`/`TransformListener` (Section 6 of `00_concepts.md`) to look up `iris_quad/odom → <camera frame>` (combining the dynamic and static transforms automatically, exactly what TF is for) and applying it to a live `/camera/points` message:
   ```
   transformed into iris_quad/odom frame:
     z range: -5.4e-07  to  3.9e-07   (expect near 0.0 - the ground plane)
     x range: -1.55 to 1.55
     y range: -2.07 to 2.07
   ```
   The resulting height is within a fraction of a millimeter of **exactly 0** — the real, known height of the ground plane in this world. Not "close enough," not "roughly the right sign" — the actual predicted ground-truth value, recovered purely by combining the camera's raw measurement with the TF tree we built. This is the concrete, numeric proof that the camera mount, the static transform, and the dynamic transform are all consistent with each other and with the real simulated world — precisely the class of bug (`terralink_elevation`'s unresolved downward-camera transform) this step set out to avoid repeating.
6. **GUI smoke test**: launched once with `headless:=false` to confirm the Gazebo window itself still starts cleanly under the software-rendering fix (it does — both the `gz sim server` and `gz sim gui` processes ran with no errors).

## Follow-ups for later steps

- Step 3 introduces the actual elevation-mapping algorithm, which will be the first real *consumer* of this TF tree + point cloud — subscribing to `/camera/points`, looking up `iris_quad/odom → iris_quad/camera_link/rgbd_camera` via `tf2_ros` exactly as this step's verification script did, and fusing the transformed points into a height map.
- A `map` frame (distinct from `iris_quad/odom`) will be introduced when it's actually needed — right now `iris_quad/odom` already coincides with the world origin at spawn, which is sufficient for a simulation-only step (see the step 1 plan's scope note).
- The RGB `/camera/image_raw` and `/camera/camera_info` topics are bridged and available but unused so far — kept for later visualization/debugging (e.g. an RViz camera view) and any future work that needs camera intrinsics.
