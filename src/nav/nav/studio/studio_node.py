"""ROS 2 side of the studio: connects the simulation to the web server (`nav.studio.server`).

What it does
  * reads the UAV pose (/odom), the UGV pose (/ugv/odom_ground_truth), the simulation clock and the live /elevation_map;
  * turns the map into the compact binary frame of `nav.studio.protocol` (height, observed, walkable) at most `map_publish_hz` times a second;
  * publishes a small JSON `state` ten times a second (poses, real-time factor, how much ground has been scanned, what the UAV is being told);
  * receives validated browser commands and turns `uav_vel` into `/cmd_vel` (body frame) with the server-side limits, the dead-man timeout and
    the e-stop of `nav.studio.commands`.

Threads: tornado's IOLoop runs in the main thread, the ROS executor in a background thread. The only state shared between them is guarded by
`self._lock` (the UAV command state and the latest values for the `state` message).

Run it (after the simulation is up):  ros2 run nav studio_node   -> open http://localhost:8765
Parameters are listed in `__init__`. The UAV velocity is zero whenever the browser is silent for `deadman_sec` and on shutdown.
"""
from __future__ import annotations

import collections
import json
import math
import signal
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import tornado.ioloop
from geometry_msgs.msg import PoseStamped, Twist
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from emap.utils.gridmap_utils import decode_gridmap
from nav.studio import protocol
from nav.studio.commands import (Command, UavBounds, UavCommandState, UavLimits, UgvCommandState, UgvLimits, apply_uav_bounds, takeoff_climb)
from nav.studio.commands import HARD_CAPS
from nav.studio.scan_path import autoscan_step, lawnmower
from nav.studio.scenery import load_scenery
from nav.studio.server import StudioServer
from nav.walkability import compute_walkable_mask


def _yaw(q) -> float:
    """Heading (rad, counter-clockwise from +x) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_web_dir() -> Path | None:
    """The browser app: the installed share folder if present, else the source tree (so `ros2 run` works before and after `colcon build`)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        d = Path(get_package_share_directory("nav")) / "studio_web"
        if d.is_dir():
            return d
    except Exception:  # noqa: BLE001 - not installed (e.g. run from the source tree)
        pass
    src = Path(__file__).resolve().parents[2] / "studio_web"
    return src if src.is_dir() else None


Bounds4 = tuple[float, float, float, float]


class StudioNode(Node):
    def __init__(self):
        super().__init__("studio_node")
        g = self.declare_parameter
        g("host", "127.0.0.1")                       # 127.0.0.1 = this machine only (WSL2 forwards it to the Windows browser)
        g("port", 8765)
        g("site", "w2_container_yard")               # shown in the page title
        g("uav_odom_topic", "/odom")
        g("ugv_odom_topic", "/ugv/odom_ground_truth")
        g("map_topic", "/elevation_map")
        g("cmd_topic", "/cmd_vel")
        g("goal_topic", "/mission/goal")
        g("cancel_topic", "/mission/cancel")           # waypoint_follower(wait_for_goal) returns to idle when it hears this
        g("plan_topic", "/plan_trace")                 # planner_node's JSON record of every plan it answers
        g("ugv_cmd_topic", "/nav2_cmd_vel")            # the topic the bridge forwards to the UGV wheels (Nav2 publishes here too)
        g("ugv_max_lin", 1.0)                          # manual UGV limits
        g("ugv_max_ang", 1.5)
        g("uav_min_z", 0.6)                            # UAV flight limits (server side): floor, ceiling and the horizontal fence
        g("uav_max_z", 14.0)
        g("uav_fence", "")                             # "xmin,ymin,xmax,ymax"; empty = the extent of the elevation map once it is known
        g("map_source", "live")                        # shown in the page: live | a saved map's name (set by the launch file)
        g("world_file", "")                           # the simulator's .world file: its boxes/containers are sent to the page as "scenery"
        g("models_dir", "")                           # folder with the world's mesh models (needed to size the containers)
        g("arrive_radius", 0.5)                        # the follower's own waypoint_reached_radius_m: closer than this = goal reached
        g("max_xy", 3.0)                              # UAV limits, m/s
        g("max_z", 1.5)
        g("max_yaw", 1.0)                             # rad/s
        g("deadman_sec", 0.5)                         # no uav_vel message for this long -> zero velocity
        g("cmd_hz", 20.0)
        g("state_hz", 10.0)
        g("map_publish_hz", 1.0)
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        self.params = {n: v(n) for n in ("host", "port", "site", "max_xy", "max_z", "max_yaw", "deadman_sec", "map_source", "ugv_max_lin", "ugv_max_ang")}

        self._lock = threading.Lock()
        self._uav_cmd = UavCommandState(UavLimits(float(v("max_xy")), float(v("max_z")), float(v("max_yaw"))), float(v("deadman_sec")))
        self._ugv_cmd = UgvCommandState(UgvLimits(float(v("ugv_max_lin")), float(v("ugv_max_ang"))), float(v("deadman_sec")))
        self._mode = "auto"     # who drives the UGV: "auto" (planner + Nav2) or "manual" (the person)
        self._plan = None       # latest plan_trace record (dict) for the CURRENT goal, or None
        self._goal_seq = 0      # increments with each goal so an old plan is not mistaken for the new goal's
        self._plan_goal_seq = -1
        fence = str(v("uav_fence")).strip()
        self._fence_param = tuple(float(t) for t in fence.split(",")) if fence else None
        if self._fence_param is not None and len(self._fence_param) != 4:
            raise ValueError("uav_fence must be 'xmin,ymin,xmax,ymax'")
        self._map_fence = None  # derived from the first map (the area the map can describe, less a 2 m margin)
        self._min_z, self._max_z = float(v("uav_min_z")), float(v("uav_max_z"))
        self._arrive = float(v("arrive_radius"))
        self._scan: dict | None = None        # the running auto-scan: {"wps": [...], "i": next corner, "alt": height} (None = not scanning)
        self._pending_log = ""
        self._climb_to: float | None = None   # altitude the automatic take-off is climbing to (None = not climbing)
        wf, md = str(v("world_file")), str(v("models_dir"))
        self._scenery = load_scenery(wf, md) if wf else []
        self._uav = None        # (x, y, z, yaw)
        self._ugv = None        # (x, y, yaw)
        self._goal = None       # (x, y)
        self._sim_t = 0.0
        self._clock_samples: collections.deque = collections.deque(maxlen=64)   # (wall, sim) pairs for the real-time factor
        self._map_info = {"version": 0, "observed_cells": 0, "area_m2": 0.0, "stamp": None}
        self._map_version = 0
        self._last_map_wall = 0.0
        self._map_period = 1.0 / max(float(v("map_publish_hz")), 0.01)
        self.server: StudioServer | None = None

        self.create_subscription(Odometry, v("uav_odom_topic"), self._on_uav, 10)
        self.create_subscription(Odometry, v("ugv_odom_topic"), self._on_ugv, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(GridMap, v("map_topic"), self._on_map, 10)
        self._cmd_pub = self.create_publisher(Twist, v("cmd_topic"), 10)
        self._goal_pub = self.create_publisher(PoseStamped, v("goal_topic"), 10)
        self._cancel_pub = self.create_publisher(Empty, v("cancel_topic"), 10)
        self._ugv_cmd_pub = self.create_publisher(Twist, v("ugv_cmd_topic"), 10)
        self.create_subscription(String, v("plan_topic"), self._on_plan, 10)
        self.create_timer(1.0 / float(v("cmd_hz")), self._publish_cmd)
        self.create_timer(1.0 / float(v("state_hz")), self._publish_state)

    # ---- called by main() -------------------------------------------------------------------------------------------------------
    def attach(self, server: StudioServer) -> None:
        self.server = server

    def hello(self) -> dict:
        lim = self._uav_cmd.limits
        return {"settings": self.current_settings(), "hard_caps": HARD_CAPS, "site": self.params["site"], "limits": {"max_xy": lim.max_xy, "max_z": lim.max_z, "max_yaw": lim.max_yaw},
                "deadman_sec": self.params["deadman_sec"], "map_source": self.params["map_source"],
                "ugv_limits": {"max_lin": self.params["ugv_max_lin"], "max_ang": self.params["ugv_max_ang"]},
                "uav_bounds": {"min_z": self._min_z, "max_z": self._max_z}, "arrive_radius": self._arrive,
                "scenery": self._scenery}

    # ---- browser -> ROS (runs on the IOLoop thread) -----------------------------------------------------------------------------
    def on_command(self, cmd: Command) -> None:
        if cmd.kind == "takeoff":
            if self._uav_cmd.estop:
                self._log("warn", "The e-stop is on: release it (Space) before taking off.")
                return
            with self._lock:
                self._climb_to = cmd.data["alt"]
            self._log("info", f"Taking off: climbing to {cmd.data['alt']:.0f} m. Move the climb stick (R / F) to take over.")
        elif cmd.kind in ("uav_vel", "estop", "estop_release"):
            with self._lock:
                self._uav_cmd.handle(cmd, time.monotonic())
                if cmd.kind == "estop":
                    self._ugv_cmd.clear()                # the e-stop button stops the manual UGV too
                    self._climb_to = None                # ... and ends an automatic take-off
                    self._scan = None                    # ... and an auto-scan (the UAV then simply hovers)
            if cmd.kind == "estop":
                self._cmd_pub.publish(Twist())          # do not wait for the next timer tick
                if self._mode == "manual":
                    self._ugv_cmd_pub.publish(Twist())
                # An emergency stop that left a robot driving would surprise everyone: also cancel the UGV's goal (the follower stops it at its
                # current spot). The pilot can simply click the goal again afterwards.
                if self._goal is not None:
                    self._cancel_goal()
        elif cmd.kind == "settings":
            self._apply_settings(cmd.data)
        elif cmd.kind == "autoscan":
            self._start_autoscan(cmd.data["alt"])
        elif cmd.kind == "autoscan_cancel":
            self._end_autoscan("Auto-scan stopped.")
        elif cmd.kind == "ugv_vel":
            with self._lock:
                if self._mode == "manual" and not self._uav_cmd.estop:
                    self._ugv_cmd.handle(cmd, time.monotonic())
        elif cmd.kind == "mode":
            self._set_mode(cmd.data["mode"])
        elif cmd.kind == "cancel_goal":
            self._cancel_goal()
        elif cmd.kind == "goal":
            if self._mode != "auto":
                self._log("warn", "Switch to AUTO mode first: in MANUAL mode you drive the UGV yourself, so a goal is not used.")
                return
            with self._lock:
                self._goal_seq += 1
                self._plan = None
            if self.server is not None:
                self.server.publish_event({"t": "plan", "plan": None}, sticky=True)
            msg = PoseStamped()
            msg.header.frame_id = "iris_quad/odom"      # the map frame; the follower only reads x and y
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = cmd.data["x"], cmd.data["y"]
            msg.pose.orientation.w = 1.0
            self._goal_pub.publish(msg)
            with self._lock:
                self._goal = (cmd.data["x"], cmd.data["y"])

    def _scan_area(self) -> Bounds4 | None:
        """What to sweep: the real site's boxes (+3 m) when the page has scenery, else the whole fence; never outside the fence."""
        fence = self._bounds().fence
        if fence is None:
            return None
        xs = [b["x"] for b in self._scenery]
        if not xs:
            return fence
        ys = [b["y"] for b in self._scenery]
        m = 3.0
        return (max(fence[0], min(xs) - m), max(fence[1], min(ys) - m), min(fence[2], max(xs) + m), min(fence[3], max(ys) + m))

    def _start_autoscan(self, alt: float) -> None:
        if self._uav_cmd.estop:
            self._log("warn", "The e-stop is on: release it (Space) before starting an auto-scan.")
            return
        area = self._scan_area()
        if area is None or self._uav is None:
            self._log("warn", "Auto-scan needs the map and the UAV position first: wait a few seconds and try again.")
            return
        alt = min(alt, self._max_z - 0.5)
        wps = lawnmower(area, spacing=5.0)
        with self._lock:
            self._climb_to = None
            self._scan = {"wps": wps, "i": 0, "alt": alt}
        self._log("info", f"Auto-scan: sweeping {area[2] - area[0]:.0f} x {area[3] - area[1]:.0f} m at {alt:.0f} m ({len(wps)} legs). "
                          "This takes several minutes. Any stick input or the e-stop stops it.")

    def _end_autoscan(self, why: str) -> None:
        with self._lock:
            was = self._scan is not None
            self._scan = None
        if was:
            self._log("info", why)

    def _with_autoscan(self, twist):
        """Replace the pilot's command by the next sweep step. Caller holds the lock. The pilot always wins: any non-zero stick ends the scan."""
        if self._scan is None or self._uav is None:
            return twist
        if any(twist):
            self._scan = None
            self._pending_log = "Auto-scan stopped: you took over."
            return twist
        sc = self._scan
        if sc["i"] >= len(sc["wps"]):
            self._scan = None
            self._pending_log = "Auto-scan finished. Pick a goal for the UGV."
            return (0.0, 0.0, 0.0, 0.0)
        reached, vel = autoscan_step(self._uav, sc["wps"][sc["i"]], sc["alt"], min(1.2, self._uav_cmd.limits.max_xy), self._uav_cmd.limits.max_z)
        if reached:
            sc["i"] += 1
        return vel

    def current_settings(self) -> dict:
        """The run-time settings as the page shows them (same keys as `commands.HARD_CAPS`)."""
        u, g = self._uav_cmd.limits, self._ugv_cmd.limits
        return {"max_xy": u.max_xy, "max_z": u.max_z, "max_yaw": u.max_yaw, "ugv_max_lin": g.max_lin, "ugv_max_ang": g.max_ang, "ceiling": self._max_z}

    def _apply_settings(self, d: dict) -> None:
        """Swap in new limits (already clamped by parse_command). The ceiling may not be set below where the UAV is now: it would just hold it."""
        with self._lock:
            u, g = self._uav_cmd.limits, self._ugv_cmd.limits
            self._uav_cmd.limits = UavLimits(d.get("max_xy", u.max_xy), d.get("max_z", u.max_z), d.get("max_yaw", u.max_yaw))
            self._ugv_cmd.limits = UgvLimits(d.get("ugv_max_lin", g.max_lin), d.get("ugv_max_ang", g.max_ang))
            if "ceiling" in d:
                self._max_z = max(d["ceiling"], self._min_z + 2.0)
        if self.server is not None:
            self.server.publish_event({"t": "settings", "settings": self.current_settings()}, sticky=True)   # sticky: new tabs get it too
        self._log("info", "Settings changed.")

    def _log(self, level: str, text: str) -> None:
        """A message shown in the page's log, and in the terminal."""
        getattr(self.get_logger(), "warn" if level == "warn" else "info")(text)
        if self.server is not None:
            self.server.publish_event({"t": "log", "level": level, "msg": text})

    def _set_mode(self, mode: str) -> None:
        """Hand the UGV to the person ("manual") or back to the planner ("auto").
        Going manual first cancels the planner's goal, so Nav2 and the person never both command the wheels."""
        if mode == self._mode:
            return
        if mode == "manual":
            self._cancel_goal()
        with self._lock:
            self._mode = mode
            self._ugv_cmd.clear()
        self._ugv_cmd_pub.publish(Twist())              # stop the wheels at the hand-over, either direction
        self._log("info", "UGV is now driven by YOU (manual)." if mode == "manual" else "UGV is back under planner control (auto).")

    def _cancel_goal(self) -> None:
        with self._lock:
            had = self._goal is not None
            self._goal = None
            self._plan = None
            self._goal_seq += 1
        self._cancel_pub.publish(Empty())
        if self.server is not None:
            self.server.publish_event({"t": "plan", "plan": None}, sticky=True)
        if had:
            self._log("info", "Goal cancelled.")

    def _on_plan(self, m: String) -> None:
        """planner_node answered a plan request: keep it for the page (only while a goal is active)."""
        try:
            rec = json.loads(m.data)
        except ValueError:
            return
        with self._lock:
            if self._goal is None:
                return
            self._plan = {"valid": bool(rec.get("valid")), "waypoints": rec.get("waypoints", []), "kinds": rec.get("kinds", []),
                          "has_frontier": bool(rec.get("has_frontier")), "has_anomaly": bool(rec.get("has_anomaly")),
                          "goal": rec.get("goal")}
            plan = self._plan
        if self.server is not None:
            self.server.publish_event({"t": "plan", "plan": plan}, sticky=True)

    def _bounds(self) -> UavBounds:
        fence = self._fence_param or self._map_fence
        return UavBounds(self._min_z, self._max_z, fence)

    # ---- ROS callbacks ----------------------------------------------------------------------------------------------------------
    def _on_uav(self, m: Odometry) -> None:
        p = m.pose.pose
        with self._lock:
            self._uav = (p.position.x, p.position.y, p.position.z, _yaw(p.orientation))

    def _on_ugv(self, m: Odometry) -> None:
        p = m.pose.pose
        with self._lock:
            self._ugv = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_clock(self, m: Clock) -> None:
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        with self._lock:
            self._sim_t = sim
            self._clock_samples.append((time.monotonic(), sim))

    def _rtf(self) -> float | None:
        """Simulated seconds per wall second over the last few seconds (1.0 = real time). Caller holds the lock."""
        if len(self._clock_samples) < 2:
            return None
        (w0, s0), (w1, s1) = self._clock_samples[0], self._clock_samples[-1]
        return (s1 - s0) / (w1 - w0) if w1 > w0 else None

    def _on_map(self, msg: GridMap) -> None:
        now = time.monotonic()
        if now - self._last_map_wall < self._map_period or self.server is None:
            return                                       # throttle BEFORE decoding: decoding a 560 x 560 x 6 map is the costly part
        self._last_map_wall = now
        layers = decode_gridmap(msg)
        valid = np.asarray(layers["is_valid"]).astype(bool)
        walkable = compute_walkable_mask(layers["traversability"], layers["is_valid"])   # exactly what the planner will treat as drivable
        self._map_version += 1
        res = float(msg.info.resolution)
        if self._map_fence is None:
            # the map can only describe this square, so flying beyond it is pointless: keep the UAV 2 m inside
            h = 0.5 * res * max(valid.shape) - 2.0
            self._map_fence = (msg.info.pose.position.x - h, msg.info.pose.position.y - h, msg.info.pose.position.x + h, msg.info.pose.position.y + h)
        frame = protocol.pack_map(layers["elevation"], valid, walkable, res, msg.info.pose.position.x, msg.info.pose.position.y, self._map_version)
        n = int(valid.sum())
        with self._lock:
            self._map_info = {"version": self._map_version, "observed_cells": n, "area_m2": n * res * res, "stamp": now}
        self.server.publish_map(frame)

    # ---- ROS -> browser / UAV ---------------------------------------------------------------------------------------------------
    def _publish_cmd(self) -> None:
        now = time.monotonic()
        with self._lock:
            twist = self._with_autoscan(self._with_takeoff(self._uav_cmd.current(now)))
            vx, vy, vz, wz = apply_uav_bounds(twist, self._uav, self._bounds())
            manual = self._mode == "manual"
            lin, ang = self._ugv_cmd.current(now) if manual else (0.0, 0.0)
        if self._pending_log:          # the scan ended inside the locked section: say so out here (logging may call the network layer)
            msg, self._pending_log = self._pending_log, ""
            self._log("info", msg)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z, t.angular.z = float(vx), float(vy), float(vz), float(wz)
        self._cmd_pub.publish(t)
        if manual:          # only in manual mode: in auto mode Nav2 owns this topic and a stream of zeros from here would fight it
            u = Twist()
            u.linear.x, u.angular.z = float(lin), float(ang)
            self._ugv_cmd_pub.publish(u)

    def _with_takeoff(self, twist):
        """Add the automatic climb of a take-off to the pilot's command. Caller holds the lock.
        The pilot always wins: any climb/descend stick input ends the take-off. It also ends when the target height is reached."""
        if self._climb_to is None or self._uav is None:
            return twist
        vx, vy, vz, wz = twist
        self._climb_to, vz = takeoff_climb(self._uav[2], self._climb_to, vz, self._uav_cmd.limits.max_z)
        return (vx, vy, vz, wz)

    def _publish_state(self) -> None:
        if self.server is None:
            return
        now = time.monotonic()
        with self._lock:
            vx, vy, vz, wz = self._uav_cmd.current(now)
            mi = dict(self._map_info)
            state = {
                "sim_t": self._sim_t, "rtf": self._rtf(),
                "uav": None if self._uav is None else dict(zip(("x", "y", "z", "yaw"), self._uav)),
                "ugv": None if self._ugv is None else dict(zip(("x", "y", "yaw"), self._ugv)),
                "goal": None if self._goal is None else {"x": self._goal[0], "y": self._goal[1]},
                "cmd": {"vx": vx, "vy": vy, "vz": vz, "wz": wz, "estop": self._uav_cmd.estop, "climb_to": self._climb_to,
                        "autoscan": None if self._scan is None else {"i": self._scan["i"], "n": len(self._scan["wps"])}},
                "map": {"version": mi["version"], "observed_cells": mi["observed_cells"], "area_m2": mi["area_m2"],
                        "age_s": None if mi["stamp"] is None else now - mi["stamp"]},
                "clients": len(self.server._clients),    # noqa: SLF001
                "mode": self._mode, "nav": self._nav_status(),
                "uav_limited": apply_uav_bounds((vx, vy, vz, wz), self._uav, self._bounds()) != (vx, vy, vz, wz),   # a flight limit is holding the UAV back
            }
        self.server.publish_state(state)

    def _nav_status(self) -> str:
        """none | planning | no_route | driving | arrived. Caller holds the lock."""
        if self._goal is None:
            return "none"
        if self._ugv is not None and math.hypot(self._ugv[0] - self._goal[0], self._ugv[1] - self._goal[1]) <= self._arrive:
            return "arrived"
        if self._plan is None:
            return "planning"
        return "driving" if self._plan["valid"] else "no_route"

    def stop_uav(self) -> None:
        """Zero velocity, several times (a single message could be lost). Used on shutdown."""
        for _ in range(5):
            self._cmd_pub.publish(Twist())
            if self._mode == "manual":
                self._ugv_cmd_pub.publish(Twist())
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StudioNode()
    ioloop = tornado.ioloop.IOLoop.current()             # created in the main thread; the ROS thread only schedules callbacks on it
    server = StudioServer(
        node.on_command, web_dir=default_web_dir(), host=str(node.params["host"]), port=int(node.params["port"]),
        hello=node.hello(), ioloop=ioloop,
    )
    node.attach(server)
    server.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass            # rclpy was shut down from the main thread: the normal way to stop, not an error

    spin_thread = threading.Thread(target=spin, name="ros-spin")
    spin_thread.start()
    # Ctrl+C (SIGINT) and `kill` (SIGTERM) both end the loop cleanly, so the UAV is told to stop (see `finally`). Without this rclpy's own
    # signal handling swallowed SIGTERM and the process stayed alive after the launch was stopped.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: ioloop.add_callback_from_signal(ioloop.stop))
    node.get_logger().info(f"studio ready: open http://localhost:{server.port} in a browser")
    try:
        ioloop.start()
    finally:
        node.stop_uav()
        server.stop()
        # Order matters: stop the executor and WAIT for its thread before tearing rclpy down. Shutting rclpy down while another thread is
        # still inside the executor made the process abort with "terminate called without an active exception".
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
