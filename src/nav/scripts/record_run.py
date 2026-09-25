#!/usr/bin/env python3
"""User-requested capability (2026-09-24): "generate and save an mp4
recording of the UGV moving to reach from one target to another. It can be
from UGV's POV in the room as well."

Standalone script (same convention as `emap/scripts/synthetic_pointcloud_tf_
publisher.py` - run directly with `python3`, not a packaged ROS executable):
subscribes to two ALREADY-EXISTING camera image topics for the duration of a
run and writes each to its own .mp4 via OpenCV's own VideoWriter (no ffmpeg
binary needed - confirmed live in this environment that ffmpeg itself isn't
installed, but `cv2` bundles its own mp4 muxer/encoder and writes real,
playable .mp4 files without it):

- `/camera/image_raw` - the UAV's own downward-facing camera (bridged in
  emap/config/bridge.yaml, active in every launch this project has), giving
  a bird's-eye "watch the whole room" view. Because that camera looks
  straight down, the UGV will visibly vanish under the tunnel's roof and
  reappear on the other side - not a recording bug, the literal
  demonstration of the occlusion problem this whole project exists to solve
  (see step06_hybrid_3d_voxel_navigation.md's own opening paragraph).
- `/ugv/camera/image_raw` - the UGV's own front-facing camera (new bridge
  entry, nav/config/bridge.yaml), a first-person "POV" view.

Neither topic needs anything new launched - both cameras already exist and
publish in every `tunnel_demo.launch.py`/`nav_sim.launch.py` run; this script
only ever subscribes, it never adds sensors or changes simulation behavior.

Usage: start this BEFORE (or any time during) a run, let it run until the
UGV settles at the goal, then Ctrl+C (or let --duration elapse):

    python3 src/nav/scripts/record_run.py --out-dir /tmp/recordings --duration 200
"""
from __future__ import annotations

import argparse
import os
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class _CameraRecorder:
    """One image topic -> one .mp4 file. Frames are written as they arrive,
    at whatever rate the camera actually publishes - the FIRST frame's own
    size fixes the VideoWriter's frame size (OpenCV requires this upfront);
    every camera in this project publishes a fixed resolution for its whole
    lifetime, so this is never actually a problem in practice, just a real
    constraint worth naming.

    TWO REAL BUGS FOUND LIVE (2026-09-24, second pass, from a user report
    that a delivered recording "showed nothing"):

    1. `mp4v` fourcc is MPEG-4 Part 2, not H.264 - it decodes fine in
       OpenCV's own re-read (which is all the first pass's own
       verification checked) but is rejected outright by many browsers'
       and editors' built-in players, rendering as a blank/black player
       for anyone using one of those - a very plausible part of "shows
       nothing". `avc1` (real H.264, confirmed live: cv2's FFmpeg backend
       accepts and re-reads it cleanly in this environment) is universally
       playable instead.
    2. This project's cameras are declared at 10Hz in their SDF, but this
       sandbox's CPU-bound software rendering (LIBGL_ALWAYS_SOFTWARE=1 -
       see tunnel_demo.launch.py) actually delivers frames much slower
       under load - measured live: ~3.5-7Hz depending on camera and
       system load, not 10. Writing at a flat assumed `fps` (10) plays
       the result back 1.5-3x faster than the events actually happened -
       not literally "nothing", but a real, confusing distortion of what
       "the UGV reached the goal" actually looked like in time. Fixed by
       measuring the ACTUAL average arrival rate (wall-clock elapsed /
       frame count) and remuxing to that once recording stops, rather
       than trusting the sensor's own declared rate.
    """

    def __init__(self, node: Node, topic: str, out_path: str, fps: float, label: str):
        self._label = label
        self._out_path = out_path
        self._fps = fps
        self._bridge = CvBridge()
        self._writer: cv2.VideoWriter | None = None
        self._frame_count = 0
        self._first_frame_time: float | None = None
        node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        node.get_logger().info(f"record_run: recording {label} ({topic}) -> {out_path}")

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self._writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"avc1")
            self._writer = cv2.VideoWriter(self._out_path, fourcc, self._fps, (width, height))
            self._first_frame_time = time.monotonic()
        self._writer.write(frame)
        self._frame_count += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
        self._fix_playback_speed()

    def _fix_playback_speed(self) -> None:
        """Remux the just-written file at its ACTUALLY measured average
        fps if that differs meaningfully (>15%) from the fps it was
        written with - see this class's own docstring, bug 2."""
        if self._frame_count < 2 or self._first_frame_time is None:
            return
        elapsed = time.monotonic() - self._first_frame_time
        if elapsed <= 0:
            return
        measured_fps = self._frame_count / elapsed
        if abs(measured_fps - self._fps) / self._fps <= 0.15:
            return
        cap = cv2.VideoCapture(self._out_path)
        if not cap.isOpened():
            cap.release()
            return
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        tmp_path = self._out_path + ".remux.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        writer = cv2.VideoWriter(tmp_path, fourcc, measured_fps, (width, height))
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            writer.write(frame)
        writer.release()
        cap.release()
        os.replace(tmp_path, self._out_path)

    @property
    def frame_count(self) -> int:
        return self._frame_count


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", required=True, help="Directory to write overhead_view.mp4 and ugv_pov.mp4 into")
    parser.add_argument("--duration", type=float, default=240.0, help="Max seconds to record (default: 240)")
    parser.add_argument("--fps", type=float, default=10.0, help="Output video FPS - matches the cameras' own 10Hz")
    parser.add_argument(
        "--overhead-topic",
        default="/room_overhead/image_raw",
        help="Room overview camera topic capturing the full room (default: /room_overhead/image_raw)",
    )
    parser.add_argument(
        "--uav-topic",
        default="/camera/image_raw",
        help="UAV downward-camera image topic (default: /camera/image_raw)",
    )
    parser.add_argument(
        "--pov-topic", default="/ugv/camera/image_raw", help="UGV front-camera image topic (default: /ugv/camera/image_raw)"
    )
    args = parser.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    overhead_path = os.path.join(args.out_dir, "overhead_view.mp4")
    uav_path = os.path.join(args.out_dir, "uav_downward_view.mp4")
    pov_path = os.path.join(args.out_dir, "ugv_pov.mp4")

    rclpy.init()
    node = Node("record_run")
    overhead = _CameraRecorder(node, args.overhead_topic, overhead_path, args.fps, "full room overhead view")
    uav_cam = _CameraRecorder(node, args.uav_topic, uav_path, args.fps, "UAV downward camera")
    pov = _CameraRecorder(node, args.pov_topic, pov_path, args.fps, "UGV POV")

    stop = {"requested": False}

    def _on_sigint(signum, frame):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _on_sigint)

    node.get_logger().info(f"record_run: recording for up to {args.duration:.0f}s - Ctrl+C to stop early.")
    start = time.monotonic()
    try:
        while rclpy.ok() and not stop["requested"] and (time.monotonic() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        overhead.close()
        uav_cam.close()
        pov.close()
        node.get_logger().info(
            f"record_run: wrote {overhead.frame_count} frames to {overhead_path}, "
            f"{uav_cam.frame_count} frames to {uav_path}, "
            f"{pov.frame_count} frames to {pov_path}"
        )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
