#!/usr/bin/env python3
"""Record one live run of a stress world for the video (`make_run_video.py` composes it afterwards).

    python3 src/nav/scripts/record_world_run.py --out /tmp/w2_run --goal -12 13.8

Saves, stamped with SIMULATION time (the sensors are sim-driven, so playback speed is exact no
matter how slowly the software renderer runs in wall time):
    overhead/<stamp_ms>.jpg   fixed overhead camera (/room_overhead/image_raw)
    pov/<stamp_ms>.jpg        the UGV's own forward camera (/ugv/camera/image_raw)
    poses.csv                 t, x, y, yaw of the TRUE chassis pose (/ugv/odom_ground_truth)
    plans.jsonl               every plan the planner returned (`plan_trace` topic: waypoints, which stretches are uncertain,
                              exploration detour or not), stamped with the latest simulation time seen
Stops by itself a few seconds after the UGV is within --tol of --goal (or at --max-sim-sec).
Only subscribes: it never changes the simulation. Nothing but this recorder writes to disk during
the run, so the compositing/rendering cost stays out of the simulation's way.
"""
from __future__ import annotations

import argparse
import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self, a):
        super().__init__("record_world_run")
        self.a = a
        self.bridge = CvBridge()
        for sub in ("overhead", "pov"):
            os.makedirs(os.path.join(a.out, sub), exist_ok=True)
        self.poses = open(os.path.join(a.out, "poses.csv"), "w")
        self.poses.write("t,x,y,yaw\n")
        self.n = {"overhead": 0, "pov": 0}
        self.t0 = None                      # first pose stamp: the run's t = 0
        self.arrived_at = None              # sim time the goal was first reached
        self.done = False
        self.last_t = 0.0
        self.create_subscription(Image, a.overhead_topic, lambda m: self.on_image(m, "overhead"), qos_profile_sensor_data)
        self.create_subscription(Image, a.pov_topic, lambda m: self.on_image(m, "pov"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ugv/odom_ground_truth", self.on_odom, 50)
        self.plans = open(os.path.join(a.out, "plans.jsonl"), "w")
        self.create_subscription(String, "plan_trace", self.on_plan, 50)

    @staticmethod
    def stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_image(self, msg, kind):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t = self.stamp(msg)
        cv2.imwrite(os.path.join(self.a.out, kind, f"{int(round(t * 1000)):010d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 93])
        self.n[kind] += 1

    def on_plan(self, msg):
        # the planner's clock is not the recorder's: stamp with the newest simulation time seen (pose rate is 30 Hz)
        self.plans.write('{"t": %.3f, "plan": %s}\n' % (self.last_t, msg.data))
        self.plans.flush()

    def on_odom(self, msg):
        t = self.stamp(msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.poses.write(f"{t:.4f},{p.x:.4f},{p.y:.4f},{yaw:.4f}\n")
        self.last_t = t
        if self.t0 is None:
            self.t0 = t
        if self.a.goal and math.hypot(p.x - self.a.goal[0], p.y - self.a.goal[1]) < self.a.tol and self.arrived_at is None:
            self.arrived_at = t
            self.get_logger().info(f"goal reached at sim t={t:.1f}s; recording {self.a.tail:.0f} s more")
        if self.arrived_at is not None and t - self.arrived_at > self.a.tail:
            self.done = True
        if self.t0 is not None and t - self.t0 > self.a.max_sim_sec:
            self.get_logger().warn("max sim time reached without arriving")
            self.done = True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--goal", nargs=2, type=float)
    ap.add_argument("--tol", type=float, default=0.35)
    ap.add_argument("--tail", type=float, default=4.0, help="sim seconds to keep recording after arrival")
    ap.add_argument("--max-sim-sec", type=float, default=600.0)
    ap.add_argument("--overhead-topic", default="/room_overhead/image_raw")
    ap.add_argument("--pov-topic", default="/ugv/camera/image_raw")
    a = ap.parse_args(argv)
    rclpy.init()
    node = Recorder(a)
    t = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.poses.close()
    node.plans.close()
    print(f"recorded {node.n['overhead']} overhead + {node.n['pov']} pov frames over {node.last_t - (node.t0 or 0):.1f} sim s "
          f"({time.monotonic() - t:.0f} s wall) -> {a.out}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
