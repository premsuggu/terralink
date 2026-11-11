#!/usr/bin/env python3
"""Real top-down pictures of the stress worlds, rendered by Gazebo itself (writes media/classification/topdown/<world>.png).

Every `<world>_run.world` already has a fixed overhead camera (`overhead_overview_camera`, 150 m up, looking straight down, its
field of view sized to the world - the camera the run videos use). Two pictures per world:

  <world>.png       THE UAV-VIEW PICTURE (used next to the class map). The camera is moved high above the map's centre with the
                    narrowest lens SDF allows (0.1 rad), at the height where that lens just covers the map (~10 x the map's width:
                    500 m for W4, 1200 m for W1), so the picture is nearly orthographic (a 5 m object at the map's edge shifts
                    outwards by ~1 %, at most 0.25 m). Its NEAR CLIP PLANE is put at the UAV ceiling (5 m above the ground, `UAV_CEILING` in
                    worldgen/worlds/*.py): everything higher - crane jibs, the upper storeys of W1's 58 m tower - is cut away,
                    exactly as the ground-truth maps drop it. Shadows are switched off (the cut-away tower would still cast one).
  <world>_full.png  the world as the run videos see it: the unchanged 150 m camera, with shadows. Tall things look bigger and lean
                    outwards (W1's tower top appears 1.6 x its size), so this one does NOT line up with the map.
Plus <world>.json: the picture's extent in world metres ([x0, x1, y0, y1], image up = +y), read by render_classification.py.

For each picture the script:
  1. copies the run world WITHOUT the UGV (so the picture shows only the site) and, for the UAV view, with the camera changed,
  2. starts the Gazebo server headless on that copy (software GL, as the launch files do on this machine),
  3. bridges the camera topic to ROS and saves one frame (after a few frames, so the scene has finished loading),
  4. stops everything again (~30-60 s per picture).

Usage (ROS sourced, nav built): python3 render_world_topdown.py [--worlds w1_house_site ...] [--out media/classification/topdown]
"""
from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import json
import math
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
WORLDS = ["w1_house_site", "w2_container_yard", "w3_excavation_slopes", "w4_structure_decks", "w5_door_gauntlet"]
GZ_TOPIC = "/room_overhead/image"           # the camera's Gazebo topic in every *_run.world
ROS_TOPIC = "/room_overhead/image_raw"
UAV_CEILING = 5.0                           # metres; the same in all five worlds (worldgen/worlds/*.py)
CAM_FOV = 0.1                               # rad; the narrowest horizontal_fov SDF accepts (smaller = "Error reading element")


def map_area(world: str):
    """(centre x, centre y, side length) in metres of the world's planner map (maps/<world>_uavview.npz)."""
    d = np.load(os.path.join(REPO, "src", "nav", "maps", f"{world}_uavview.npz"))
    return float(d["center_x"]), float(d["center_y"]), d["layer_elevation"].shape[0] * float(d["resolution"])


def cam_height(side: float) -> float:
    """Height at which a camera with the CAM_FOV lens sees exactly `side` metres across at the ground."""
    return side / 2 / math.tan(CAM_FOV / 2)


def world_without_robot(src: str, dst: str, uav_view_of=None):
    """Copy a run world, dropping the <include> block of the UGV (model://nav_ugv) and its plugins with it.
    uav_view_of = (cx, cy, side): also turn the overhead camera into the cut, nearly orthographic UAV-view camera (see top)."""
    txt = open(src).read()
    txt, n = re.subn(r"<include>(?:(?!</include>).)*?model://nav_ugv.*?</include>", "", txt, flags=re.S)
    if n != 1:
        raise RuntimeError(f"{src}: expected exactly one nav_ugv include, found {n}")
    if uav_view_of is not None:
        cx, cy, side = uav_view_of
        z = cam_height(side)
        i = txt.index('name="overhead_overview_camera"')
        head, cam = txt[:i], txt[i:]
        end = cam.index("</model>")
        cam, tail = cam[:end], cam[end:]
        # same orientation as before (looking straight down, image up = +y), only higher and centred on the map
        cam = re.sub(r"<pose>[^<]*</pose>", f"<pose>{cx:.4f} {cy:.4f} {z:.3f} 0 1.5707963 1.5707963</pose>", cam, count=1)
        cam = re.sub(r"<horizontal_fov>[^<]*</horizontal_fov>",
                     f"<horizontal_fov>{CAM_FOV}</horizontal_fov>", cam, count=1)
        # near plane = the UAV ceiling: a point is drawn only if it is more than (z - ceiling) below the camera
        cam = re.sub(r"<clip>.*?</clip>", f"<clip><near>{z - UAV_CEILING:.3f}</near><far>{z + 20:.1f}</far></clip>",
                     cam, count=1, flags=re.S)
        txt = head + cam + tail
        txt = txt.replace("<cast_shadows>true</cast_shadows>", "<cast_shadows>false</cast_shadows>")
    open(dst, "w").write(txt)


def grab_frame(timeout_s: float, skip: int = 3):
    """Wait for the overhead camera and return its `skip`-th frame as an RGB array (the first ones can be half-loaded)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node("topdown_grabber")
    got = {"n": 0, "img": None}

    def on_image(m):
        got["n"] += 1
        if got["n"] >= skip and got["img"] is None:
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            got["img"] = a[:, :, ::-1].copy() if m.encoding.lower().startswith("bgr") else a.copy()
    node.create_subscription(Image, ROS_TOPIC, on_image, qos_profile_sensor_data)
    t0 = time.time()
    while got["img"] is None and time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return got["img"]


def render(world: str, out_png: str, uav_view: bool, timeout_s: float = 240.0) -> bool:
    import cv2
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1")
    models = os.pathsep.join([os.path.join(REPO, "src", "nav", "models"), os.path.join(REPO, "src", "emap", "models"),
                              env.get("GZ_SIM_RESOURCE_PATH", "")])
    env["GZ_SIM_RESOURCE_PATH"] = env["IGN_GAZEBO_RESOURCE_PATH"] = models
    with tempfile.TemporaryDirectory() as tmp:
        wf = os.path.join(tmp, f"{world}_topdown.world")
        area = map_area(world)
        world_without_robot(os.path.join(REPO, "src", "nav", "worlds", f"{world}_run.world"), wf, area if uav_view else None)
        # own process groups, so stopping them never touches anything else on the machine
        log = open(os.path.join(tmp, "gz.log"), "w+")
        sim = subprocess.Popen(["ign", "gazebo", "-s", "-r", "-v", "1", wf], env=env, start_new_session=True,
                               stdout=log, stderr=subprocess.STDOUT)
        bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                   f"{GZ_TOPIC}@sensor_msgs/msg/Image[ignition.msgs.Image",
                                   "--ros-args", "-r", f"{GZ_TOPIC}:={ROS_TOPIC}"],
                                  env=env, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            img = grab_frame(timeout_s)
        finally:
            for p in (bridge, sim):
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if img is None:                         # say why (e.g. an SDF value Gazebo rejected)
            log.seek(0)
            errors = [ln.strip() for ln in log if "rror" in ln]
            print(f"{world}: no frame within {timeout_s:.0f} s; Gazebo errors: {errors[:3]}", flush=True)
            return False
    cv2.imwrite(out_png, img[:, :, ::-1])
    if uav_view:                                # the picture covers exactly the map's square
        cx, cy, side = area
        ext = [cx - side / 2, cx + side / 2, cy - side / 2, cy + side / 2]
    else:                                       # the unchanged 150 m camera (geometry from the world file)
        sys.path.insert(0, HERE)
        from make_run_video import camera_geometry
        from pathlib import Path
        ccx, ccy, mpp = camera_geometry(Path(os.path.join(REPO, "src", "nav", "worlds", f"{world}_run.world")), img.shape[1])
        half = img.shape[1] / 2 * mpp
        ext = [ccx - half, ccx + half, ccy - half, ccy + half]
    json.dump({"extent": ext, "uav_view": uav_view, "ceiling_m": UAV_CEILING if uav_view else None},
              open(out_png[:-4] + ".json", "w"))
    print(f"{world}: {img.shape[1]}x{img.shape[0]} -> {out_png}", flush=True)
    return True


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--worlds", nargs="*", default=WORLDS)
    a.add_argument("--out", default=os.path.join(REPO, "media", "classification", "topdown"))
    a.add_argument("--uav-only", action="store_true", help="only the cut UAV-view pictures")
    a.add_argument("--full-only", action="store_true", help="only the unchanged 150 m pictures")
    args = a.parse_args()
    os.makedirs(args.out, exist_ok=True)
    a_ok = []
    for w in args.worlds:
        if not args.full_only:
            a_ok.append(render(w, os.path.join(args.out, f"{w}.png"), uav_view=True))
        if not args.uav_only:
            a_ok.append(render(w, os.path.join(args.out, f"{w}_full.png"), uav_view=False))
    ok = a_ok
    sys.exit(0 if all(ok) else 1)


if __name__ == "__main__":
    main()
