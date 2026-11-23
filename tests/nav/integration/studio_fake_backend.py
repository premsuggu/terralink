#!/usr/bin/env python3
"""A stand-in for `studio_node` that needs NO simulator: real StudioServer + a saved map + two pretend robots.

Used to test the browser page quickly (tests/nav/integration/studio_browser.py drives it with a headless browser) and handy for trying the viewer
on a machine without Gazebo:

    python3 tests/nav/integration/studio_fake_backend.py --map src/nav/maps/w2_container_yard_uavview.npz --port 8799

What it pretends: the UAV obeys `uav_vel` commands (body frame, same limits as the real server), the UGV drives in a straight line at 1 m/s to
any goal it is sent (a two-point "plan" is published so the page draws something), and in manual mode it obeys `ugv_vel`. The map is the saved
snapshot, sent whole. Everything else (state fields, frames, events) uses the real protocol code.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import tornado.ioloop

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "nav"))

from nav.studio import protocol  # noqa: E402
from nav.studio.scan_path import autoscan_step, lawnmower  # noqa: E402
from nav.studio.commands import HARD_CAPS, UavBounds, UavCommandState, UavLimits, UgvCommandState, UgvLimits, apply_uav_bounds, takeoff_climb  # noqa: E402
from nav.studio.scenery import load_scenery  # noqa: E402
from nav.studio.server import StudioServer  # noqa: E402
from nav.studio.studio_node import default_web_dir  # noqa: E402
from nav.walkability import compute_walkable_mask  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--port", type=int, default=8799)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--ugv", type=float, nargs=2, default=(-3.0, -26.0))
    ap.add_argument("--live", action="store_true", help="start with an EMPTY map that fills in around the UAV as it flies (like a live scan)")
    ap.add_argument("--world", default=str(Path(__file__).resolve().parents[3] / "src/nav/worlds/w2_container_yard_studio.world"))
    args = ap.parse_args()

    d = np.load(args.map)
    elev, valid, trav = d["layer_elevation"], d["layer_is_valid"], d["layer_traversability"]
    walk = compute_walkable_mask(trav, valid)
    res, cx, cy = float(d["resolution"]), float(d["center_x"]), float(d["center_y"])
    full_valid = valid.astype(bool)
    shown = np.zeros_like(full_valid) if args.live else full_valid.copy()      # --live: cells revealed so far
    rr, cc = np.indices(full_valid.shape)
    version = [1]

    def make_frame():
        return protocol.pack_map(elev, shown, walk & shown, res, cx, cy, version[0])


    ioloop = tornado.ioloop.IOLoop.current()
    uav_cmd = UavCommandState(UavLimits(), 0.5)
    ugv_cmd = UgvCommandState(UgvLimits(), 0.5)
    bounds = UavBounds(fence=(cx - 28, cy - 28, cx + 28, cy + 28))
    world = {"uav": [args.ugv[0], args.ugv[1] + 2.0, 0.1, 0.0], "ugv": [args.ugv[0], args.ugv[1], 0.0], "goal": None, "mode": "auto", "plan": None, "climb": None}
    server: StudioServer | None = None

    def current_settings():
        u, g = uav_cmd.limits, ugv_cmd.limits
        return {"max_xy": u.max_xy, "max_z": u.max_z, "max_yaw": u.max_yaw, "ugv_max_lin": g.max_lin, "ugv_max_ang": g.max_ang,
                "ceiling": world.get("ceiling", 14.0)}

    def on_command(cmd):
        now = time.monotonic()
        if cmd.kind == "takeoff":
            if uav_cmd.estop:
                server.publish_event({"t": "log", "level": "warn", "msg": "The e-stop is on: release it (Space) before taking off."})
            else:
                world["climb"] = cmd.data["alt"]
        elif cmd.kind in ("uav_vel", "estop", "estop_release"):
            if cmd.kind == "estop":
                world["climb"] = None
                world["scan"] = None
            uav_cmd.handle(cmd, now)
            if cmd.kind == "estop" and world["goal"] is not None:          # like the real node: e-stop also cancels the UGV goal
                world["goal"] = None
                server.publish_event({"t": "plan", "plan": None}, sticky=True)
        elif cmd.kind == "autoscan":                                       # a SMALL sweep round the UAV (the real node sweeps the whole site)
            if uav_cmd.estop:
                server.publish_event({"t": "log", "level": "warn", "msg": "The e-stop is on: release it (Space) before starting an auto-scan."})
                return
            ux, uy = world["uav"][0], world["uav"][1]
            world["scan"] = {"wps": lawnmower((ux - 6, uy - 6, ux + 6, uy + 6), 5.0), "i": 0, "alt": cmd.data["alt"]}
            world["climb"] = None
        elif cmd.kind == "autoscan_cancel":
            world["scan"] = None
        elif cmd.kind == "settings":                                       # same rule as the real node: swap limits, answer with the truth
            d = cmd.data
            u, g = uav_cmd.limits, ugv_cmd.limits
            uav_cmd.limits = UavLimits(d.get("max_xy", u.max_xy), d.get("max_z", u.max_z), d.get("max_yaw", u.max_yaw))
            ugv_cmd.limits = UgvLimits(d.get("ugv_max_lin", g.max_lin), d.get("ugv_max_ang", g.max_ang))
            world["ceiling"] = d.get("ceiling", world.get("ceiling", 14.0))
            server.publish_event({"t": "settings", "settings": current_settings()}, sticky=True)
        elif cmd.kind == "ugv_vel" and world["mode"] == "manual":
            ugv_cmd.handle(cmd, now)
        elif cmd.kind == "mode":
            world["mode"] = cmd.data["mode"]
            if world["mode"] == "manual":
                world["goal"] = None
                server.publish_event({"t": "plan", "plan": None}, sticky=True)
            ugv_cmd.clear()
        elif cmd.kind == "cancel_goal":
            world["goal"] = None
            server.publish_event({"t": "plan", "plan": None}, sticky=True)
        elif cmd.kind == "goal":
            if world["mode"] != "auto":
                server.publish_event({"t": "log", "level": "warn", "msg": "Switch to AUTO mode first."})
                return
            world["goal"] = (cmd.data["x"], cmd.data["y"])
            gx, gy = world["goal"]
            world["plan"] = {"valid": True, "waypoints": [[world["ugv"][0], world["ugv"][1]], [gx, gy]], "kinds": ["free"],
                             "has_frontier": False, "has_anomaly": False, "goal": [gx, gy]}
            server.publish_event({"t": "plan", "plan": world["plan"]}, sticky=True)

    server = StudioServer(on_command, web_dir=default_web_dir(), host=args.host, port=args.port, ioloop=ioloop,
                          hello={"settings": current_settings(), "hard_caps": HARD_CAPS, "site": "w2_container_yard", "limits": {"max_xy": 3.0, "max_z": 1.5, "max_yaw": 1.0}, "deadman_sec": 0.5,
                                 "map_source": "live" if args.live else Path(args.map).stem, "ugv_limits": {"max_lin": 1.0, "max_ang": 1.5},
                                 "uav_bounds": {"min_z": 0.6, "max_z": 14.0}, "arrive_radius": 0.5,
                                 "scenery": load_scenery(args.world, Path(args.world).parents[1] / "models")})
    server.start()
    t0 = last = time.monotonic()

    def tick() -> None:
        nonlocal last
        now = time.monotonic()
        dt, last = now - last, now
        x, y, z, yaw = world["uav"]
        tw = list(uav_cmd.current(now))
        if world["climb"] is not None:          # the very same rule as the real node (nav.studio.commands.takeoff_climb)
            world["climb"], tw[2] = takeoff_climb(z, world["climb"], tw[2], 1.5)
        sc = world.get("scan")
        if sc is not None:                      # same rule as the real node: a non-zero stick ends it, a finished sweep ends it
            if any(tw) or sc["i"] >= len(sc["wps"]):
                world["scan"] = None
            else:
                done, v = autoscan_step((x, y, z, yaw), sc["wps"][sc["i"]], sc["alt"], 1.2, 1.5)
                sc["i"] += done
                tw = list(v)
        vx, vy, vz, wz = apply_uav_bounds(tuple(tw), (x, y, z, yaw), bounds)
        c, s = math.cos(yaw), math.sin(yaw)
        world["uav"] = [x + (c * vx - s * vy) * dt, y + (s * vx + c * vy) * dt, max(0.1, z + vz * dt), yaw + wz * dt]
        gx_, gy_, gyaw = world["ugv"]
        if world["mode"] == "manual":
            lin, ang = ugv_cmd.current(now)
            world["ugv"] = [gx_ + math.cos(gyaw) * lin * dt, gy_ + math.sin(gyaw) * lin * dt, gyaw + ang * dt]
        elif world["goal"] is not None:
            tx, ty = world["goal"]
            dist = math.hypot(tx - gx_, ty - gy_)
            if dist > 0.5:
                gyaw = math.atan2(ty - gy_, tx - gx_)
                world["ugv"] = [gx_ + (tx - gx_) / dist * dt, gy_ + (ty - gy_) / dist * dt, gyaw]
        ux, uy, _ = world["ugv"]
        nav = "none"
        if world["goal"] is not None:
            nav = "arrived" if math.hypot(world["goal"][0] - ux, world["goal"][1] - uy) <= 0.5 else "driving"
        if args.live and world["uav"][2] > 0.6:     # reveal a disc of ground under the "camera" (radius grows with height)
            ux_, uy_ = world["uav"][0], world["uav"][1]
            rad = 0.6 * world["uav"][2] + 1.0
            xs = (cc + 0.5 - full_valid.shape[1] / 2) * res + cx
            ys = (rr + 0.5 - full_valid.shape[0] / 2) * res + cy
            shown[:] |= full_valid & ((xs - ux_) ** 2 + (ys - uy_) ** 2 < rad * rad)
        n = int(shown.sum())
        server.publish_state({
            "sim_t": now - t0, "rtf": 0.6,
            "uav": dict(zip(("x", "y", "z", "yaw"), world["uav"])), "ugv": dict(zip(("x", "y", "yaw"), world["ugv"])),
            "goal": None if world["goal"] is None else {"x": world["goal"][0], "y": world["goal"][1]},
            "cmd": {"vx": vx, "vy": vy, "vz": vz, "wz": wz, "estop": uav_cmd.estop, "climb_to": world["climb"],
                    "autoscan": None if world.get("scan") is None else {"i": world["scan"]["i"], "n": len(world["scan"]["wps"])}},
            "map": {"version": 1, "observed_cells": n, "area_m2": n * res * res, "age_s": 0.0},
            "clients": len(server._clients), "mode": world["mode"], "nav": nav, "uav_limited": False,   # noqa: SLF001
        })

    tornado.ioloop.PeriodicCallback(tick, 100).start()
    def push_map() -> None:
        version[0] += 1
        server.publish_map(make_frame())

    tornado.ioloop.PeriodicCallback(push_map, 1000).start()
    print(f"fake studio backend on http://127.0.0.1:{server.port}", flush=True)
    try:
        ioloop.start()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
