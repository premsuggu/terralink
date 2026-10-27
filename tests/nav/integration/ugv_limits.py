#!/usr/bin/env python3
"""Experiment X-A1 driver: teleport the UGV to each lane of ugv_limits_test.world, drive at the obstacle, report what happened.

    export LIBGL_ALWAYS_SOFTWARE=1 IGN_GAZEBO_RESOURCE_PATH=$PWD/src/nav/models
    ign gazebo -r -s tests/nav/integration/ugv_limits_test.world &
    ros2 run ros_gz_bridge parameter_bridge /model/nav_ugv/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
        /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
    python3 tests/nav/integration/ugv_limits.py [speed=0.5]       # stop the sim and the bridge by PID afterwards

Per lane: x_max, z_max (height gained), max |roll| / |pitch| (deg) and a verdict. Slope s = tan(angle); the planner's limits are
max_slope 0.35 (19.3 deg) and max_step 0.15 m.
"""
import math, subprocess, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_limits_world import RAMPS_DEG, STEPS_CM          # one definition of the lanes
SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5


class N(Node):
    def __init__(s):
        super().__init__("limits"); s.o = None; s.t = 0.0
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(Odometry, "/model/nav_ugv/odometry", s.cb, 10)

    def cb(s, m):
        q = m.pose.pose.orientation; p = m.pose.pose.position
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        s.o = (p.x, p.y, p.z, roll, pitch); s.t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def teleport(y):
    req = f'name: "nav_ugv", position: {{x: 0.0, y: {y}, z: 0.06}}, orientation: {{w: 1.0}}'
    subprocess.run(["ign", "service", "-s", "/world/ugv_limits/set_pose", "--reqtype", "ignition.msgs.Pose",
                    "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True)


rclpy.init(); n = N()


def run_for(sim_s, vx):
    t0 = n.t; tw = Twist(); tw.linear.x = float(vx); samples = []
    while n.t - t0 < sim_s:
        n.pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
        if n.o: samples.append(n.o)
    return samples


while n.o is None: rclpy.spin_once(n, timeout_sec=0.2)
print(f"speed {SPEED} m/s; lane: obstacle | x_max z_max(m) max|roll| max|pitch| (deg) | verdict")
for lane in range(12):
    y = 3.0 * (lane - 5.5)
    teleport(y); run_for(1.5, 0.0)                       # settle after the teleport
    s = run_for(26.0, SPEED)
    xmax = max(a[0] for a in s); zmax = max(a[2] for a in s)
    up = s[:max(range(len(s)), key=lambda k: s[k][2]) + 1]      # tilt only up to the highest point (after that: plateau / nothing to climb)
    roll = max(abs(a[3]) for a in up); pitch = max(abs(a[4]) for a in up)
    if lane < 6:
        d = RAMPS_DEG[lane]; name = f"ramp {d:>2} deg (slope {math.tan(math.radians(d)):.2f})"; need = 0.85 * 4.0 * math.sin(math.radians(d))
    else:
        h = STEPS_CM[lane - 6]; name = f"step {h:>2} cm"; need = 0.8 * h / 100
    flipped = roll > math.radians(60) or pitch > math.radians(70)
    verdict = "FLIPPED" if flipped else ("crossed" if zmax >= need else "BLOCKED")
    print(f"{name:<26}| x {xmax:5.2f}  z {zmax:5.3f}  roll {math.degrees(roll):4.0f}  pitch {math.degrees(pitch):4.0f} | {verdict}", flush=True)
run_for(0.5, 0.0)
