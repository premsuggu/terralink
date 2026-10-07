#!/usr/bin/env python3
"""Record WHERE the UGV really went and EVERY plan it was given, for one run.

Why this exists (user request, 2026-10-08): "an image where we can clearly see
the path taken by the UGV (how it investigates and checks) - basically all the
path plans it gets", for both planners, so they can be compared. Videos show
motion but not the sequence of plans; this logs both and
`nav/trace_render.py` / `render_run_trace.py` draw them.

What is recorded (all stamped with this process's own monotonic clock, so the
tracks and the plans share one time axis):

* the UGV's TRUE pose, from Gazebo's `/world/<world>/dynamic_pose/info`
  (world frame, ground truth). This script starts its own `ros_gz_bridge` for
  that one topic, so it needs no change to any world or bridge file and works in
  every world. (`/ugv/odom` is NOT ground truth - see docs.)
* every plan the planner returned - the planner's `plan_trace` topic (one JSON
  per `get_plan` request: waypoints, which stretches are uncertain, whether it
  was an exploration detour),
* every goal sent to Nav2 (`/goal_pose`),
* the 3D voxel verification verdicts (`/voxel_verification`),
* the final `/elevation_map` as a snapshot (the drawing's background).

It stops when the UGV has stayed within `--arrive-radius` of the goal for
`--hold` seconds, after `--duration` seconds, or on Ctrl+C, then writes
`<out-dir>/trace.json` and `<out-dir>/map.npz`.

    python3 src/nav/scripts/trace_run.py --out-dir /tmp/run1 --world tunnel_test --goal 3 0
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from emap.utils.gridmap_utils import decode_gridmap
from nav.replay import save_snapshot

_GT_TOPIC = "/trace_gt/pose"


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TraceRecorder(Node):
    def __init__(self, goal, arrive_radius: float, hold_sec: float, entity: str):
        super().__init__("trace_run")
        self.goal = goal
        self.arrive_radius = arrive_radius
        self.hold_sec = hold_sec
        self.entity = entity
        self.t0 = time.monotonic()
        self.poses: list[list[float]] = []  # [t, x, y, yaw]
        self.plans: list[dict] = []
        self.goals: list[list[float]] = []  # [t, x, y]
        self.verdicts: list[dict] = []
        self.arrive_t: float | None = None
        self._inside_since: float | None = None
        self._last_pose_t = -1.0
        self.finished = False
        self._last_map: GridMap | None = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, _GT_TOPIC, self._on_pose, sensor_qos)
        self.create_subscription(String, "/plan_trace", self._on_plan, 50)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 50)
        self.create_subscription(String, "/voxel_verification", self._on_verdict, 50)
        self.create_subscription(GridMap, "/elevation_map", self._on_map, 1)

    def now(self) -> float:
        return time.monotonic() - self.t0

    # -- callbacks ---------------------------------------------------------
    def _on_pose(self, msg: TFMessage) -> None:
        # Gazebo's Pose_V carries every entity; the model itself is named exactly
        # `entity` (links/sensors are "nav_ugv::something" - skipped).
        for tf in msg.transforms:
            if tf.child_frame_id != self.entity:
                continue
            t = self.now()
            if t - self._last_pose_t < 0.1:  # 10 Hz is plenty for a track
                return
            self._last_pose_t = t
            p = tf.transform.translation
            self.poses.append([t, p.x, p.y, _yaw_from_quat(tf.transform.rotation)])
            self._check_arrival(t, p.x, p.y)
            return

    def _check_arrival(self, t: float, x: float, y: float) -> None:
        if self.goal is None:
            return
        if math.hypot(x - self.goal[0], y - self.goal[1]) <= self.arrive_radius:
            if self._inside_since is None:
                self._inside_since = t
                if self.arrive_t is None:
                    self.arrive_t = t
            elif t - self._inside_since >= self.hold_sec:
                self.finished = True
        else:
            self._inside_since = None

    def _on_plan(self, msg: String) -> None:
        try:
            rec = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rec["t"] = self.now()
        self.plans.append(rec)

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goals.append([self.now(), msg.pose.position.x, msg.pose.position.y])

    def _on_verdict(self, msg: String) -> None:
        self.verdicts.append({"t": self.now(), "text": msg.data})

    def _on_map(self, msg: GridMap) -> None:
        self._last_map = msg  # decode only once, at the end

    # -- output --------------------------------------------------------------
    def save(self, out_dir: str, meta: dict) -> None:
        os.makedirs(out_dir, exist_ok=True)
        meta = dict(meta, duration=self.now(), arrive_t=self.arrive_t, goal=self.goal)
        with open(os.path.join(out_dir, "trace.json"), "w") as fh:
            json.dump(
                {"meta": meta, "poses": self.poses, "plans": self.plans, "goals": self.goals, "verdicts": self.verdicts},
                fh,
            )
        if self._last_map is not None:
            msg = self._last_map
            save_snapshot(
                os.path.join(out_dir, "map.npz"), decode_gridmap(msg), msg.info.resolution,
                msg.info.pose.position.x, msg.info.pose.position.y,
            )
        self.get_logger().info(
            f"saved {len(self.poses)} poses, {len(self.plans)} plans, {len(self.goals)} goals to {out_dir}"
            + ("" if self._last_map is not None else " (NO map received)")
        )


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--world", required=True, help="Gazebo world name (tunnel_test | room_maze)")
    ap.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), required=True)
    ap.add_argument("--entity", default="nav_ugv", help="model name of the UGV in Gazebo")
    ap.add_argument("--arrive-radius", type=float, default=0.3)
    ap.add_argument("--hold", type=float, default=3.0, help="seconds inside the radius before stopping")
    ap.add_argument("--duration", type=float, default=300.0, help="give up after this many seconds")
    ap.add_argument("--label", default="", help="free text stored in the trace (e.g. 'astar run 2')")
    ap.add_argument("--planner", default="", help="planner name stored in the trace")
    args = ap.parse_args(argv)

    gz_topic = f"/world/{args.world}/dynamic_pose/info"
    bridge = subprocess.Popen(
        [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{gz_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "--ros-args", "-r", f"{gz_topic}:={_GT_TOPIC}",
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    node = TraceRecorder(tuple(args.goal), args.arrive_radius, args.hold, args.entity)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    node.get_logger().info(f"trace_run: recording (max {args.duration:.0f}s) - stops on arrival at {args.goal}")
    try:
        while rclpy.ok() and not stop["flag"] and not node.finished and node.now() < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.save(args.out_dir, {"world": args.world, "planner": args.planner, "label": args.label})
        bridge.terminate()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
