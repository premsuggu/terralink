"""Extra SDF for a world that is actually RUN (not just reviewed): the UGV and a fixed overhead camera.

`<world>.world` (review/geometry only) has no robot. `<world>_run.world` is the same scene plus
the blocks below - copied from `worlds/tunnel_test.world`, whose convention is proven live:

* `nav_ugv` with DiffDrive (cmd_vel in) and an OdometryPublisher that reports the chassis's TRUE
  world pose under `nav_ugv/odom` (wheel-integrated odometry drifts by metres - see that world's
  comments), so the launch file's `iris_quad/odom -> nav_ugv/odom` static transform is identity.
* an overhead camera that sees the whole map, for the run video: it looks straight down with image
  up = world +y and image right = world +x (pitch 90 deg, yaw 90 deg).
"""
from __future__ import annotations

import math
import re
from pathlib import Path

UGV_BLOCK = """
    <include>
      <name>nav_ugv</name>
      <uri>model://nav_ugv</uri>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <plugin filename="ignition-gazebo-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.05</wheel_radius>
        <topic>cmd_vel</topic>
        <odom_topic>odometry</odom_topic>
        <frame_id>nav_ugv/wheel_odom</frame_id>
        <child_frame_id>nav_ugv/wheel_odom_base</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
      </plugin>
      <plugin filename="ignition-gazebo-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>2</dimensions>
        <odom_frame>nav_ugv/odom</odom_frame>
        <robot_base_frame>nav_ugv/base_link</robot_base_frame>
        <odom_publish_frequency>30</odom_publish_frequency>
      </plugin>
    </include>
"""

CAMERA_BLOCK = """
    <model name="overhead_overview_camera">
      <static>true</static>
      <pose>{cx:.2f} {cy:.2f} {h:.1f} 0 1.5707963 1.5707963</pose>
      <link name="link">
        <sensor name="room_overview_camera" type="camera">
          <always_on>true</always_on>
          <update_rate>{rate:.1f}</update_rate>
          <topic>/room_overhead/image</topic>
          <camera>
            <horizontal_fov>{fov:.4f}</horizontal_fov>
            <image><width>{px}</width><height>{px}</height></image>
            <clip><near>0.5</near><far>400</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


# rate=1 Hz: the video only needs a few frames (a "clean plate" of the world; the robot is drawn from its 30 Hz pose),
# and rendering a 1440 px camera on the CPU was a large part of the simulation slowdown.
def run_blocks(spawn, ground_z, bounds, px=1440, height=150.0, rate=1.0) -> str:
    """UGV at `spawn=(x, y, yaw)` + a square overhead camera covering `bounds=(x0,x1,y0,y1)`."""
    x, y, yaw = spawn
    x0, x1, y0, y1 = bounds
    half = max(x1 - x0, y1 - y0) / 2 + 1.0
    fov = 2 * math.atan(half / height)
    cam = CAMERA_BLOCK.format(cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, h=height, fov=fov, px=px, rate=rate)
    return UGV_BLOCK.format(x=x, y=y, z=ground_z, yaw=yaw) + cam


_TUNNEL_WORLD = Path(__file__).resolve().parents[2] / "worlds" / "tunnel_test.world"


def uav_block(x: float, y: float) -> str:
    """The proven `iris_quad` include (motor model, velocity controller, ground-truth odometry) with its spawn pose
    replaced by (x, y, 0.1). Copied from `worlds/tunnel_test.world` at build time so the two can never drift apart.
    The UAV is what makes the planner map REAL: the live `emap` pipeline builds it from the UAV's depth camera."""
    txt = _TUNNEL_WORLD.read_text()
    name = txt.index("<name>iris_quad</name>")
    start = txt.rindex("<include>", 0, name)
    end = txt.index("</include>", name) + len("</include>")
    block = txt[start:end]
    block, n = re.subn(r"<pose>[^<]*</pose>", f"<pose>{x:.2f} {y:.2f} 0.1 0 0 0</pose>", block, count=1)
    assert n == 1, "iris_quad pose not found in tunnel_test.world"
    return "\n    " + block + "\n"


def lawnmower(x0: float, x1: float, y0: float, y1: float, z: float, lane: float, step: float = 5.0):
    """Boustrophedon sweep of the rectangle as (x, y, z) hover points: lanes along x, `lane` metres apart,
    a waypoint every `step` metres. The UAV camera fuses continuously in flight, so waypoints only steer."""
    ys = [y0 + i * lane for i in range(int(math.ceil((y1 - y0) / lane)) + 1)]
    xs = [x0 + i * step for i in range(int(math.ceil((x1 - x0) / step)) + 1)]
    pts = []
    for i, y in enumerate(ys):
        row = xs if i % 2 == 0 else xs[::-1]
        pts += [(min(x, x1), min(y, y1), z) for x in row]
    return pts
