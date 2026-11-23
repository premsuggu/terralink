#!/usr/bin/env python3
"""BROWSER check of the studio page: load it in a headless Chromium, look at every view, and drive it with keys, a fake gamepad and mouse clicks.

Not part of pytest (needs Playwright + a Chromium, which are NOT project dependencies - install them yourself, ideally into a throwaway folder):

    pip install --target /tmp/pw playwright && PLAYWRIGHT_BROWSERS_PATH=/tmp/pwb PYTHONPATH=/tmp/pw python3 -m playwright install chromium

Run it against EITHER backend (it must already be listening):
    python3 tests/nav/integration/studio_fake_backend.py --map src/nav/maps/w2_container_yard_uavview.npz --port 8799   # no simulator needed
    PLAYWRIGHT_BROWSERS_PATH=/tmp/pwb PYTHONPATH=/tmp/pw python3 tests/nav/integration/studio_browser.py --url http://127.0.0.1:8799 --out /tmp/shots
  or the real stack (`ros2 launch nav studio.launch.py map:=uavview`) with `--real`, which skips the checks that only make sense for the
  pretend backend (exact UGV speed) and waits longer for the slow simulator.

Checks (PASS/FAIL, exit code 1 if any fails; screenshots of every view are written to --out for a human to look at):
  1 page loads with no JavaScript errors, WebGL works, link up, map decoded       2 each layout draws every pane (not blank)
  3 each of the 7 camera views draws something                                    4 keyboard: R climbs the UAV, Space e-stops and releases
  5 a fake Xbox controller: stick values reach the vehicle, B e-stops, A releases    6 clicking the top view sets the goal where the mouse points
  7 clicking a blocked (red) spot sends no goal                                     8 a plan arrives and is drawn; Cancel clears the goal
  9 manual UGV mode: W drives the UGV, and goals are refused in that mode           10 the page recovers its link after the server drops it (reconnect)
"""
from __future__ import annotations

import argparse
import io
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright

GAMEPAD_JS = """
window.__pad = {id: 'Xbox 360 Controller (STANDARD GAMEPAD Vendor: 045e Product: 028e)', index: 0, connected: true, mapping: 'standard',
  axes: [0, 0, 0, 0], buttons: Array.from({length: 17}, () => ({pressed: false, touched: false, value: 0})), timestamp: 0};
navigator.getGamepads = () => [window.__pad, null, null, null];
"""

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""), flush=True)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8799")
    ap.add_argument("--out", default="/tmp/studio_shots")
    ap.add_argument("--real", action="store_true", help="real simulator behind the page (slower, skips fake-only checks)")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    slow = 4.0 if args.real else 1.0
    errors: list[str] = []

    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
        # the real stack leaves little CPU for a software-rendered browser: smaller window, patient timeouts
        pg = browser.new_page(viewport={"width": 1000, "height": 640} if args.real else {"width": 1400, "height": 850})
        pg.set_default_timeout(150000 if args.real else 90000)
        pg.add_init_script(GAMEPAD_JS)
        pg.add_init_script("try { localStorage.setItem('terralink.prefs', JSON.stringify({helpSeen: true})); } catch (e) {}")   # no first-visit help sheet in the way
        pg.on("console", lambda m: errors.append(f"console.{m.type}: {m.text}") if m.type in ("error", "warning") else None)
        pg.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        pg.goto(args.url)
        try:
            pg.wait_for_function("window.__studio && window.__studio.S.grid && window.__studio.S.state && window.__studio.S.state.ugv", timeout=60000)
            loaded = True
        except Exception:  # noqa: BLE001
            loaded = False
        time.sleep(1.5)
        pg.screenshot(path=str(out / "00_first_look.png"))
        if not check("1 page loads, link up, map decoded", loaded and not errors,
                     "; ".join(errors[:3]) if errors else ("no map/state within 60 s" if not loaded else "")):
            browser.close()
            return 1
        info = pg.evaluate("({h: __studio.S.grid.h, w: __studio.S.grid.w, v: __studio.S.mapVersion})")
        print(f"      map block {info['h']}x{info['w']} cells, version {info['v']}")

        state = lambda: pg.evaluate("__studio.S.state")  # noqa: E731

        def wait_for(js: str, seconds: float = 8.0) -> bool:
            """Poll a JavaScript condition. Fixed sleeps are unreliable when the page draws at a few frames per second (software rendering)."""
            end = time.time() + seconds * slow
            while time.time() < end:
                if pg.evaluate(js):
                    return True
                time.sleep(0.2)
            return False

        # ---- 2/3: panes draw something -------------------------------------------------------------------------------------------
        def pane_stds() -> list[float]:
            img = np.asarray(Image.open(io.BytesIO(pg.screenshot())).convert("L"), dtype=float)
            boxes = pg.evaluate("[...document.querySelectorAll('.pane')].map(e => {const r = e.getBoundingClientRect(); return [r.left, r.top, r.width, r.height];})")
            return [float(img[int(t) + 4:int(t + h) - 4, int(l) + 4:int(l + w) - 4].std()) for l, t, w, h in boxes]

        ok_all, detail = True, []
        for layout in ("single", "inset", "quad"):
            pg.evaluate(f"__studio.setLayout('{layout}')")
            time.sleep(0.8)
            stds = pane_stds()
            pg.screenshot(path=str(out / f"layout_{layout}.png"))
            detail.append(f"{layout}: " + "/".join(f"{s:.0f}" for s in stds))
            ok_all &= all(s > 8 for s in stds)
        check("2 every layout draws all of its panes", ok_all, "pixel spread per pane " + "; ".join(detail))

        pg.evaluate("__studio.setLayout('single')")
        kinds = pg.evaluate("Object.keys(window.__studio.panes[0] ? {orbit:1,top:1,uav_chase:1,uav_pov:1,uav_down:1,ugv_chase:1,ugv_pov:1} : {})")
        bad = []
        for k in kinds:
            pg.evaluate(f"__studio.setMainView('{k}')")
            time.sleep(0.7)
            s = pane_stds()[0]
            pg.screenshot(path=str(out / f"view_{k}.png"))
            if s <= 8:
                bad.append(f"{k}({s:.0f})")
        check("3 each camera view draws something", not bad, "blank: " + ", ".join(bad) if bad else f"{len(kinds)} views")
        pg.evaluate("__studio.setMainView('orbit'); __studio.setLayout('inset')")
        time.sleep(0.5)

        # ---- 4: keyboard + e-stop --------------------------------------------------------------------------------------------------
        z0 = state()["uav"]["z"]
        pg.keyboard.down("r")
        time.sleep(1.5 * slow)
        mid = state()
        pg.keyboard.up("r")
        time.sleep(0.5)
        climbed = mid["uav"]["z"] - z0
        check("4a keyboard R climbs the UAV", climbed > 0.2 and mid["cmd"]["vz"] > 0, f"climbed {climbed:.2f} m, commanded vz {mid['cmd']['vz']:.2f}")
        pg.keyboard.press("Space")
        time.sleep(0.5)
        e1 = state()["cmd"]["estop"]
        pg.keyboard.down("r")
        time.sleep(0.6)
        held = state()["cmd"]["vz"]
        pg.keyboard.up("r")
        pg.keyboard.press("Space")
        time.sleep(0.5)
        e2 = state()["cmd"]["estop"]
        check("4b Space e-stops (a held key is ignored) and releases", e1 and held == 0 and not e2, f"estop {e1}->{e2}, vz while stopped {held}")

        # ---- 5: fake gamepad -----------------------------------------------------------------------------------------------------
        pg.evaluate("window.__pad.axes = [0, -1, 0, 0]")          # left stick fully up = climb
        wait_for("__studio.S.state.cmd.vz > 0")
        vz = state()["cmd"]["vz"]
        pg.evaluate("window.__pad.axes = [0, 0, 0, -1]")          # right stick up = forward
        wait_for("__studio.S.state.cmd.vx > 0")
        vx = state()["cmd"]["vx"]
        pg.evaluate("window.__pad.axes = [0, 0, 0, 0]")
        time.sleep(0.5)
        press = lambda i, on: pg.evaluate(f"window.__pad.buttons[{i}].pressed = {str(on).lower()}; window.__pad.buttons[{i}].value = {1 if on else 0}")  # noqa: E731
        # Buttons are polled by a 50 ms timer. With software rendering in this sandbox one frame can take 0.2-0.8 s and delays that timer, so the
        # simulated press is held 1.2 s (a real tap is shorter but real machines draw at 60 fps). Keyboard keys are events and are never missed.
        press(1, True)
        time.sleep(1.2)
        press(1, False)
        time.sleep(0.6)
        eb = state()["cmd"]["estop"]
        press(0, True)
        time.sleep(1.2)
        press(0, False)
        time.sleep(0.6)
        ea = state()["cmd"]["estop"]
        check("5 gamepad sticks reach the vehicle; B e-stops, A releases", vz > 0.5 and vx > 0.5 and eb and not ea,
              f"vz {vz:.2f}, vx {vx:.2f}, after B estop={eb}, after A estop={ea}")

        # ---- 6/7: picking with the mouse -----------------------------------------------------------------------------------------
        pg.evaluate("__studio.setLayout('single'); __studio.setMainView('top')")
        time.sleep(0.8)
        spots = pg.evaluate("""(() => {
          // the drivable cell and the tall blocked cell NEAREST THE MAP CENTRE: the middle of the top view is clear of the HUD panels
          const g = __studio.S.grid; let good = null, blocked = null, dg = 1e9, db = 1e9;
          const mx = (g.bounds.xmin + g.bounds.xmax) / 2, my = (g.bounds.ymin + g.bounds.ymax) / 2;
          for (let i = 0; i < g.h * g.w; i++) {
            const c = i % g.w, r = (i - c) / g.w, x = g.x0 + c * g.res, y = g.y0 + r * g.res, d = Math.hypot(x - mx, y - my);
            if (g.isObserved(i) && g.isWalkable(i) && d < dg) { dg = d; good = [x, y]; }
            if (g.isObserved(i) && !g.isWalkable(i) && g.height(i) > 2 && d < db) { db = d; blocked = [x, y]; }
          }
          return {good, blocked};
        })()""")
        to_px = lambda wx, wy: pg.evaluate(  # noqa: E731
            f"(() => {{ const p = __studio.panes[0], v = new THREE.Vector3({wx}, {wy}, 0).project(p.camera), r = document.querySelector('.pane .hit').getBoundingClientRect();"
            f" return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height]; }})()")
        if spots["blocked"]:
            pg.keyboard.press("g")
            px = to_px(*spots["blocked"])
            pg.mouse.move(px[0], px[1])
            wait_for("document.getElementById('hud-cursor').textContent !== '–'")          # the readout is refreshed by the draw loop
            readout = pg.inner_text("#hud-cursor")          # proves the mouse really is over that spot (not under a panel)
            pg.mouse.click(px[0], px[1])
            time.sleep(1.0)
            check("7 clicking a blocked spot sends no goal", state()["goal"] is None and "blocked" in readout,
                  f"blocked spot {[round(v, 1) for v in spots['blocked']]}, cursor readout '{readout}'")
            pg.keyboard.press("Escape")
            time.sleep(0.3)
        else:
            check("7 clicking a blocked spot sends no goal", False, "no blocked test cell found in the map")
        gx, gy = spots["good"]
        pg.keyboard.press("g")
        px = to_px(gx, gy)
        pg.mouse.move(px[0], px[1])
        # the readout still shows the PREVIOUS (blocked) spot until the draw loop refreshes it: wait for it to change, or the click can
        # land before the page has registered the new hover (a flake seen under slow software rendering)
        wait_for("(t => t !== '–' && !t.includes('blocked'))(document.getElementById('hud-cursor').textContent)")
        hover_text = pg.inner_text("#hud-cursor")
        pg.mouse.click(px[0], px[1])
        time.sleep(1.5 * slow)
        goal = state()["goal"]
        err = float("inf") if goal is None else float(np.hypot(goal["x"] - gx, goal["y"] - gy))
        check("6 clicking the top view sets the goal where the mouse points", goal is not None and err < 0.3, f"asked ({gx:.2f},{gy:.2f}) got {goal}, error {err:.2f} m; cursor readout '{hover_text}'")

        # ---- 8: plan, e-stop cancels the goal, the SAME goal can be sent again, Cancel clears it ----------------------------------------
        def wait_nav(want: tuple[str, ...], seconds: float) -> str:
            end, nav = time.time() + seconds, ""
            while time.time() < end:
                nav = state()["nav"]
                if nav in want:
                    break
                time.sleep(0.5)
            return nav

        nav1 = wait_nav(("driving", "arrived"), 25 * slow)
        time.sleep(1.0)
        pg.screenshot(path=str(out / "goal_and_plan.png"))
        pg.keyboard.press("Space")                                   # e-stop must also cancel the UGV's goal
        time.sleep(1.0 * slow)
        s_e = state()
        stop_cancelled = s_e["cmd"]["estop"] and s_e["goal"] is None
        pg.keyboard.press("Space")                                   # release
        time.sleep(0.8)
        # the same spot again: a follower that remembered the cancelled goal would ignore it and stay idle ("planning" forever)
        pg.keyboard.press("g")
        pg.mouse.click(px[0], px[1])
        nav2 = wait_nav(("driving", "arrived"), 25 * slow)
        pg.click("#btn-cancel")
        time.sleep(1.0)
        check("8 plan arrives; e-stop cancels the goal; the same goal can be re-sent; Cancel clears it",
              nav1 in ("driving", "arrived") and stop_cancelled and nav2 in ("driving", "arrived") and state()["goal"] is None,
              f"first goal {nav1}; after e-stop goal cancelled={stop_cancelled}; same goal again {nav2}; goal after Cancel {state()['goal']}")

        # ---- 9: manual UGV ---------------------------------------------------------------------------------------------------------
        pg.keyboard.press("m")
        wait_for("__studio.S.state.mode === 'manual'")
        time.sleep(0.5)
        s0 = state()
        manual = s0["mode"] == "manual"
        pg.keyboard.down("w")
        time.sleep(2.0 * slow)
        s1 = state()
        pg.keyboard.up("w")
        moved = float(np.hypot(s1["ugv"]["x"] - s0["ugv"]["x"], s1["ugv"]["y"] - s0["ugv"]["y"]))
        pg.keyboard.press("g")
        gpx = to_px(gx, gy)
        pg.mouse.click(gpx[0], gpx[1])
        time.sleep(0.8)
        refused = state()["goal"] is None
        pg.keyboard.press("Escape")
        pg.keyboard.press("m")
        time.sleep(0.8)
        back = state()["mode"] == "auto"
        pg.screenshot(path=str(out / "manual_mode.png"))
        check("9 manual mode: W drives the UGV, goals refused, M returns to auto", manual and moved > 0.3 and refused and back,
              f"manual={manual}, moved {moved:.2f} m, goal refused={refused}, back to auto={back}")

        # ---- 10: reconnect ---------------------------------------------------------------------------------------------------------
        pg.evaluate("window.__ws0 = __studio.S.ws; __studio.S.ws.close()")
        end = time.time() + 15
        while time.time() < end and not pg.evaluate("__studio.S.ws !== window.__ws0 && __studio.S.connected"):
            time.sleep(0.3)
        down = pg.evaluate("__studio.S.ws !== window.__ws0")               # a NEW socket exists = the old one really was dropped
        up = pg.evaluate("__studio.S.connected")
        check("10 the page reconnects after the link drops", down and up, f"dropped={down}, reconnected={up}")

        late = [e for e in errors if "favicon" not in e]
        check("11 no JavaScript errors during the whole session", not late, "; ".join(late[:3]))
        browser.close()

    failed = [n for n, ok, _ in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed" + (f"; FAILED: {', '.join(failed)}" if failed else ""))
    print(f"screenshots in {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
