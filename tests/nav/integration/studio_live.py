#!/usr/bin/env python3
"""LIVE check of the studio backend against a running simulation (needs the simulator + `ros2 run nav studio_node`; not part of pytest).

What it does, as a browser would: connect to the websocket, take off, fly, and check what comes back.

    # terminal 1 (headless W2 with the UAV under manual control)
    python3 src/nav/scripts/make_studio_world.py w2_container_yard        # once; writes worlds/w2_container_yard_studio.world
    ros2 launch nav tunnel_demo.launch.py world:=w2_container_yard_studio.world nav2_params:=nav2_params_large.yaml headless:=true \\
        autonomous_uav:=false start_follower:=false global_map_length:=56.0 emap_point_stride:=16 max_roughness:=0.5 drift_compensation:=false
    # terminal 2
    ros2 run nav studio_node
    # terminal 3
    python3 tests/nav/integration/studio_live.py [--port 8765]

Checks (each prints PASS/FAIL; exit code 1 if any fails):
  1 hello + state stream arrive (state at roughly 10 Hz)            2 the page itself is served over HTTP
  3 a streamed climb command raises the UAV                          4 map frames arrive and show newly scanned ground
  5 a frame decodes to the same observed area the state reports      6 dead-man: silence stops the UAV (commanded velocity -> 0)
  7 e-stop zeroes at once and ignores commands until released        8 a garbage message gets an error frame, the link stays up
Uses only tornado (installed with the ROS stack).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
import urllib.request

import tornado.websocket

sys.path.insert(0, "src/nav")
from nav.studio import protocol  # noqa: E402

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")


class Client:
    """Keeps the newest state, counts frames, and streams commands at 20 Hz like the browser's gamepad loop will."""

    def __init__(self):
        self.state = None
        self.states = 0
        self.maps = []                 # decoded header + frame size
        self.last_frame = None
        self.errors = []
        self.hello = None
        self.cmd = {}                  # what the sender loop sends
        self.send_enabled = True

    async def run(self, ws):
        while True:
            msg = await ws.read_message()
            if msg is None:
                return
            if isinstance(msg, bytes):
                self.last_frame = msg
                self.maps.append(protocol.unpack_map(msg).header)
            else:
                m = json.loads(msg)
                if m["t"] == "state":
                    self.state, self.states = m, self.states + 1
                elif m["t"] == "hello":
                    self.hello = m
                elif m["t"] == "error":
                    self.errors.append(m["msg"])

    async def sender(self, ws):
        while True:
            if self.send_enabled and self.cmd:
                await ws.write_message(json.dumps({"t": "uav_vel", **self.cmd}))
            await asyncio.sleep(0.05)


async def main(port: int) -> int:
    page = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5).read().decode()
    check("2 page is served", "TerraLink Studio" in page, f"({len(page)} bytes)")

    ws = await tornado.websocket.websocket_connect(f"ws://127.0.0.1:{port}/ws")
    c = Client()
    tasks = [asyncio.ensure_future(c.run(ws)), asyncio.ensure_future(c.sender(ws))]
    await asyncio.sleep(3.0)
    n0, t0 = c.states, time.monotonic()
    await asyncio.sleep(3.0)
    rate = (c.states - n0) / (time.monotonic() - t0)
    check("1 hello + state stream", c.hello is not None and c.state is not None and 5 <= rate <= 15,
          f"(hello site={c.hello and c.hello.get('site')}, {rate:.1f} states/s, rtf={c.state and c.state.get('rtf')})")
    z0 = c.state["uav"]["z"]
    maps0 = len(c.maps)

    # 3: climb at full stick for 8 simulated-ish seconds (wall time, the sim runs at about half speed)
    c.cmd = {"vz": 1.0, "scale": 1.0}
    await asyncio.sleep(14.0)
    z1 = c.state["uav"]["z"]
    check("3 streamed climb raises the UAV", z1 > z0 + 1.0, f"(z {z0:.2f} -> {z1:.2f}, commanded vz={c.state['cmd']['vz']:.2f})")

    # 4: fly forward a while so new ground is scanned
    c.cmd = {"vx": 0.6, "scale": 1.0}
    await asyncio.sleep(14.0)
    c.cmd = {}
    await asyncio.sleep(3.0)
    new_maps = len(c.maps) - maps0
    obs = [h for h in c.maps if h["h"] > 0]
    check("4 map frames arrive", new_maps >= 3 and len(obs) >= 1, f"({new_maps} frames while flying, {len(obs)} non-empty)")

    # 5: the newest frame decodes to the same observed area the state reports (within one frame of lag)
    pm = protocol.unpack_map(c.last_frame)
    decoded_cells = int(pm.valid.sum())
    reported = c.state["map"]["observed_cells"]
    check("5 frame matches reported area", decoded_cells > 0 and abs(decoded_cells - reported) <= 0.25 * max(reported, 1),
          f"(decoded {decoded_cells} cells, state says {reported}, {c.state['map']['area_m2']:.1f} m^2, v{pm.header['v']})")

    # 6: dead-man. Send one forward command, then go silent: the commanded velocity must fall back to zero by itself.
    c.cmd = {"vx": 0.5}
    await asyncio.sleep(1.0)
    moving = abs(c.state["cmd"]["vx"]) > 0.1
    c.send_enabled = False
    await asyncio.sleep(1.5)
    check("6 dead-man stops the UAV when the page goes silent", moving and abs(c.state["cmd"]["vx"]) < 1e-6,
          f"(vx while sending {1.5 if moving else 0}, after silence {c.state['cmd']['vx']})")

    # 7: e-stop
    c.send_enabled = True
    c.cmd = {"vx": 1.0}
    await asyncio.sleep(0.6)
    await ws.write_message(json.dumps({"t": "estop"}))
    await asyncio.sleep(0.8)                              # the sender keeps streaming vx=1 the whole time (a "stuck stick")
    stopped = c.state["cmd"]["estop"] and abs(c.state["cmd"]["vx"]) < 1e-6
    await ws.write_message(json.dumps({"t": "estop_release"}))
    c.send_enabled = False
    await asyncio.sleep(0.8)
    released_zero = (not c.state["cmd"]["estop"]) and abs(c.state["cmd"]["vx"]) < 1e-6
    check("7 e-stop holds against a stuck stick, release starts from zero", stopped and released_zero,
          f"(during: estop={stopped}, after release vx={c.state['cmd']['vx']})")

    # 8: garbage
    await ws.write_message("{broken")
    await asyncio.sleep(0.5)
    await ws.write_message(json.dumps({"t": "ping"}))
    await asyncio.sleep(0.5)
    check("8 garbage gets an error frame, link stays up", len(c.errors) >= 1 and not tasks[0].done(), f"(errors: {c.errors})")

    c.send_enabled = False
    for t in tasks:
        t.cancel()
    ws.close()
    bad = [n for n, ok, _ in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} checks passed" + (f"; FAILED: {bad}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    sys.exit(asyncio.run(main(ap.parse_args().port)))
