#!/usr/bin/env python3
"""Live check of the UGV front depth camera (needs Gazebo + ROS): prints the floor strip it sees on flat ground.

    export LIBGL_ALWAYS_SOFTWARE=1 IGN_GAZEBO_RESOURCE_PATH=$PWD/src/nav/models
    ign gazebo -r -s tests/nav/integration/ugv_drive_test.world &
    ros2 run ros_gz_bridge parameter_bridge /nav_ugv/front_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image &
    python3 tests/nav/integration/ugv_camera_view.py          # then stop the sim and the bridge by PID

Prediction for pitch 0.35 rad, 0.155 m above the floor, hfov 1.047, 320 x 240 (see step11 doc, X-C5): nearest floor ~0.4 m ahead of the
base (0.27 m camera offset + 0.16 m), farthest hit where the top rays graze the floor, strip ~3-4 m wide at the far end.
"""
import math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PITCH, H, HFOV, W, HT, OFF = 0.35, 0.155, 1.047, 320, 240, 0.27


class N(Node):
    def __init__(s):
        super().__init__("cam_view"); s.img = None
        s.create_subscription(Image, "/nav_ugv/front_camera/depth_image", lambda m: setattr(s, "img", m), qos_profile_sensor_data)


rclpy.init(); n = N(); t0 = time.time()
while n.img is None and time.time() - t0 < 120: rclpy.spin_once(n, timeout_sec=0.2)
m = n.img
if m is None: print("NO IMAGE"); sys.exit(1)
d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
f = (W / 2) / math.tan(HFOV / 2)
fin = np.isfinite(d) & (d > 0.1)
print(f"depth image {m.width}x{m.height}: finite {fin.mean():.1%}")
# floor hit of every pixel: camera ray (forward, left, up) rotated by the pitch; ground distance along x from the base
ys, xs = np.mgrid[0:m.height, 0:m.width]
u, v = (xs + 0.5 - W / 2) / f, (ys + 0.5 - HT / 2) / f
ray = np.stack([np.ones_like(u), -u, -v], -1); ray /= np.linalg.norm(ray, axis=-1, keepdims=True)
cp, sp = math.cos(PITCH), math.sin(PITCH)
body = np.stack([cp * ray[..., 0] + sp * ray[..., 2], ray[..., 1], -sp * ray[..., 0] + cp * ray[..., 2]], -1)
pts_x = OFF + d * body[..., 0]; pts_y = d * body[..., 1]; pts_z = H - d * (-body[..., 2]) * -1  # z = H + d*body_z
pts_z = H + d * body[..., 2]
floor = fin & (np.abs(pts_z) < 0.03)
print(f"pixels on the floor (|z| < 3 cm): {floor.sum()} ({floor.mean():.1%} of the image)")
if floor.any():
    print(f"floor seen from x = {pts_x[floor].min():.2f} m to {pts_x[floor].max():.2f} m ahead of the base; "
          f"width at the far end {np.ptp(pts_y[floor & (pts_x > pts_x[floor].max() - 0.3)]):.2f} m, at x = 1 m "
          f"{np.ptp(pts_y[floor & (abs(pts_x - 1.0) < 0.1)]) if (floor & (abs(pts_x - 1.0) < 0.1)).any() else float('nan'):.2f} m")
