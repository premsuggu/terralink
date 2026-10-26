#!/usr/bin/env python3
"""Drive the UGV in a running test world and print what it did (live check of models/nav_ugv; needs Gazebo + ROS).

    export LIBGL_ALWAYS_SOFTWARE=1 IGN_GAZEBO_RESOURCE_PATH=$PWD/src/nav/models
    ign gazebo -r -s tests/nav/integration/ugv_drive_test.world &       # or ugv_terrain_test.world (8 cm curb + 20 deg ramp)
    ros2 run ros_gz_bridge parameter_bridge /chase/image@sensor_msgs/msg/Image[ignition.msgs.Image \
        /nav_ugv/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
        /model/nav_ugv/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
    python3 tests/nav/integration/ugv_drive.py /tmp/shots "[('spin',0,1.0,3.0),('arc',0.4,0.5,4.0),('straight',0.5,0,3.0)]"
    # each phase: (name, linear m/s, angular rad/s, SIM seconds); stop the sim and the bridge by PID afterwards

Expected with the shipped model (docs/work-docs/nav/step10_jackal_ugv_model.md): spin ~167 of 172 deg, arc ~118 of 115 deg,
straight ~1.5 m of 1.5 m; the first scan line must show no finite range shorter than the wall distance (no self-hits).
"""
import math, os, subprocess, sys, time
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
PLAN = eval(sys.argv[2])   # list of (name, vx, wz, sim_seconds)
class N(Node):
    def __init__(s):
        super().__init__("drive_test"); s.b = CvBridge(); s.pose = None; s.t = 0.0; s.scan = None; s.img = None; s.n = 0; s.lasts = -9
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(Odometry, "/model/nav_ugv/odometry", s.od, 10)
        s.create_subscription(LaserScan, "/nav_ugv/scan", lambda m: setattr(s, "scan", m), qos_profile_sensor_data)
        s.create_subscription(Image, "/chase/image", s.im, qos_profile_sensor_data)
    def od(s, m):
        q = m.pose.pose.orientation
        s.pose = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        s.t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    def im(s, m):
        s.img = s.b.imgmsg_to_cv2(m, "bgr8")
rclpy.init(); n = N()
def spin_until(cond, wall=240):
    t0 = time.time()
    while rclpy.ok() and not cond() and time.time()-t0 < wall: rclpy.spin_once(n, timeout_sec=0.05)
spin_until(lambda: n.pose is not None and n.scan is not None and n.img is not None)
print("first pose", n.pose, flush=True)
r = np.array(n.scan.ranges); fin = np.isfinite(r)
print(f"scan: {len(r)} rays, finite {fin.sum()}, min finite {r[fin].min():.2f} (range_min {n.scan.range_min}), -inf count {np.isneginf(r).sum()}, ray0={r[len(r)//2]:.2f}", flush=True)
def snap(tag):
    spin_until(lambda: n.img is not None and (setattr(n,'_x',1) or True)); cv2.imwrite(f"{OUT}/{tag}.png", n.img)
snap("00_start")
for name, vx, wz, dur in PLAN:
    p0, t0 = n.pose, n.t
    tw = Twist(); tw.linear.x = float(vx); tw.angular.z = float(wz)
    while n.t - t0 < dur:
        n.pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
    p1 = n.pose
    print(f"{name}: sim {n.t-t0:.1f}s  dx={p1[0]-p0[0]:.2f} dy={p1[1]-p0[1]:.2f} yaw {math.degrees(math.atan2(math.sin(p1[2]-p0[2]),math.cos(p1[2]-p0[2]))):.0f} deg  (cmd v={vx} w={wz}; ideal travel {vx*dur:.2f} m, ideal turn {math.degrees(wz*dur):.0f} deg)", flush=True)
    snap(name)
tw = Twist(); n.pub.publish(tw)
print("done", n.pose)
