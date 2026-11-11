#!/usr/bin/env python3
"""Experiment C-07 driver: drive the Jackal-class UGV through each rubble lane of ugv_rough_test.world and report what happened,
next to the map-roughness (emap.terrain_analysis) of the same field.

    export LIBGL_ALWAYS_SOFTWARE=1 IGN_GAZEBO_RESOURCE_PATH=$PWD/src/nav/models
    ign gazebo -r -s tests/nav/integration/ugv_rough_test.world &
    ros2 run ros_gz_bridge parameter_bridge /model/nav_ugv/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
        /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
    PYTHONPATH=src/emap python3 tests/nav/integration/ugv_rough.py [speed=0.5]   # stop sim + bridge by PID afterwards

Per lane: bump height H, map roughness (median / 90th percentile over the field, 0.1 m map), time to cross 6 m (ideal = 12 s at 0.5 m/s),
max |roll| / |pitch| and a verdict (crossed = reached x > 9.3 m; BLOCKED; FLIPPED).
"""
import math, os, subprocess, sys
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_rough_world import HEIGHTS_CM, SIZE, X0, LENGTH, field          # one definition of the lanes
from emap.terrain_analysis import analyze_terrain, TerrainParams
SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
RES = 0.1


def map_roughness(i):
    """Rasterise lane i (max height of the boxes in a 0.1 m cell, floor = 0) and run the v2 classifier on it."""
    nx, ny = int((LENGTH + 2) / RES), 24
    el = np.zeros((ny, nx)); x_lo = X0 - 1.0
    for x, dy, h in field(i):
        c0, c1 = int((x - SIZE / 2 - x_lo) / RES), int((x + SIZE / 2 - x_lo) / RES)
        r0, r1 = int((dy - SIZE / 2) / RES) + ny // 2, int((dy + SIZE / 2) / RES) + ny // 2
        el[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = np.maximum(el[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1], h)
    ok = np.ones_like(el, bool)
    a = analyze_terrain(el, np.full_like(el, 0.01), ok, RES, TerrainParams())
    inner = a.roughness[6:18, 15:15 + int(LENGTH / RES) - 10]            # the middle of the field, away from its edges
    return float(np.median(inner)), float(np.percentile(inner, 90))


class N(Node):
    def __init__(s):
        super().__init__("rough"); s.o = None; s.t = 0.0
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(Odometry, "/model/nav_ugv/odometry", s.cb, 10)

    def cb(s, m):
        q = m.pose.pose.orientation; p = m.pose.pose.position
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        s.o = (p.x, p.y, p.z, roll, pitch); s.t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def teleport(y):
    req = f'name: "nav_ugv", position: {{x: 0.0, y: {y}, z: 0.06}}, orientation: {{w: 1.0}}'
    subprocess.run(["ign", "service", "-s", "/world/ugv_rough/set_pose", "--reqtype", "ignition.msgs.Pose",
                    "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True)


rclpy.init(); n = N()


def run_for(sim_s, vx):
    t0 = n.t; tw = Twist(); tw.linear.x = float(vx); samples = []
    while n.t - t0 < sim_s:
        n.pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
        if n.o: samples.append((n.t - t0,) + n.o)
    return samples


while n.o is None: rclpy.spin_once(n, timeout_sec=0.2)
print(f"speed {SPEED} m/s | H cm | map roughness med / p90 (m) | x_max | t_cross (s) | max roll / pitch (deg) | verdict")
for i, H in enumerate(HEIGHTS_CM):
    y = 3.0 * (i - 3.5)
    med, p90 = map_roughness(i)
    teleport(y); run_for(1.5, 0.0)
    s = run_for(40.0, SPEED)
    xmax = max(a[1] for a in s)
    tc = next((a[0] for a in s if a[1] > X0 + LENGTH + 0.3), float("nan"))
    roll = max(abs(a[4]) for a in s); pitch = max(abs(a[5]) for a in s)
    flipped = roll > math.radians(60) or pitch > math.radians(70)
    verdict = "FLIPPED" if flipped else ("crossed" if xmax > X0 + LENGTH + 0.3 else "BLOCKED")
    print(f"H {H:>2} cm | rough {med:.3f} / {p90:.3f} | x {xmax:5.2f} | t {tc:5.1f} | roll {math.degrees(roll):3.0f} pitch {math.degrees(pitch):3.0f} | {verdict}", flush=True)
run_for(0.5, 0.0)
