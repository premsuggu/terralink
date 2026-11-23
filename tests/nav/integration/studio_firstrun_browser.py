#!/usr/bin/env python3
"""BROWSER check of the FIRST-RUN experience of the studio: a live-scan start (empty map, UAV on the ground) in a big window.

The older `studio_browser.py` starts from a saved map and never looked at what a person sees when they open the page for the first time with a
live scan - exactly the state in which the page once looked empty and confusing. This script reproduces that state and checks that
  * the real site (containers, fences) is DRAWN even though nothing has been scanned
  * the "next step" box tells the truth in each state (take off -> scan -> goal; e-stop; goal tool)
  * the Take-off button really lifts the UAV, and the pilot's climb key takes over from it
  * the e-stop shows a banner, blocks take-off with a message, and Space does not also re-trigger the button last clicked
  * every control in the side panel visibly does something (layouts, camera follow, colours, the two layer switches, sliders, help)
  * the emergency-stop button is on screen without scrolling

Run (needs Playwright, see studio_browser.py), against the pretend backend started in --live mode:
    python3 tests/nav/integration/studio_fake_backend.py --map src/nav/maps/w2_container_yard_uavview.npz --port 8798 --live
    PLAYWRIGHT_BROWSERS_PATH=/tmp/pwb PYTHONPATH=/tmp/pw python3 tests/nav/integration/studio_firstrun_browser.py --url http://127.0.0.1:8798
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

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""), flush=True)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8798")
    ap.add_argument("--out", default="/tmp/studio_firstrun_shots")
    ap.add_argument("--width", type=int, default=1862)          # the window size of the person who reported the empty-looking page
    ap.add_argument("--height", type=int, default=1016)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []

    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
        pg = browser.new_page(viewport={"width": args.width, "height": args.height})
        pg.set_default_timeout(90000)
        pg.on("console", lambda m: errors.append(f"console.{m.type}: {m.text}") if m.type == "error" else None)
        pg.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        pg.goto(args.url)                                          # NO saved preferences: a genuinely first visit
        pg.wait_for_function("window.__studio && window.__studio.S.state && window.__studio.S.state.ugv && window.__studio.S.hello", timeout=60000)
        time.sleep(1.0)
        st = lambda: pg.evaluate("__studio.S.state")  # noqa: E731
        coach = lambda: pg.evaluate("[document.getElementById('coach-eyebrow').textContent, document.getElementById('coach-title').textContent]")  # noqa: E731

        # ---- the how-to sheet appears once, on a first visit ----------------------------------------------------------------------
        shown = pg.evaluate("!document.getElementById('help').hidden")
        pg.screenshot(path=str(out / "01_first_visit_help.png"))
        pg.click("#btn-help-close")
        time.sleep(0.4)
        check("1 first visit shows the short how-to; 'Got it' closes it", shown and pg.evaluate("document.getElementById('help').hidden"))
        pg.reload()
        pg.wait_for_function("window.__studio && window.__studio.S.state && window.__studio.S.hello", timeout=60000)
        check("1b ...and it does not come back on the next visit", pg.evaluate("document.getElementById('help').hidden"))
        time.sleep(1.5)
        pg.screenshot(path=str(out / "02_first_look.png"))

        # ---- the world is visible although nothing has been scanned ------------------------------------------------------------------
        info = pg.evaluate("({boxes: __studio.scenery.count, scanned: __studio.S.state.map.area_m2, source: __studio.S.hello.map_source})")
        img = np.asarray(Image.open(io.BytesIO(pg.screenshot())).convert("RGB"), dtype=float)
        box = pg.evaluate("(() => { const r = document.querySelector('.pane').getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; })()")
        l, t, w, h = (int(v) for v in box)
        region = img[t + 40:t + h - 8, l + 8:l + w - 8]
        # container colours are saturated blue/white: count clearly blue pixels (the floor and sky are grey-blue and much less saturated)
        blue = ((region[..., 2] > 150) & (region[..., 2] - region[..., 0] > 70)).mean()
        check("2 the real site is drawn from the first frame (live mode, nothing scanned)",
              info["source"] == "live" and info["scanned"] < 5 and info["boxes"] > 20 and blue > 0.01,
              f"{info['boxes']} boxes, {info['scanned']:.0f} m2 scanned, {100 * blue:.1f}% of the main view is container-blue")

        # the robots are close enough to be seen: the orbit camera starts near the UGV, not framing the whole site
        dist = pg.evaluate("__studio.panes[0].orbit.dist")
        check("3 the camera starts near the robots (they are not dots)", dist < 40, f"orbit distance {dist:.0f} m")
        check("3b the emergency-stop button is on screen without scrolling",
              pg.evaluate("(() => { const r = document.getElementById('btn-estop').getBoundingClientRect(); return r.bottom <= innerHeight && r.top >= 0; })()"))

        # ---- the coach says the right thing in each state ------------------------------------------------------------------------
        e0 = coach()
        check("4 on the ground, the coach says to take off", "TAKE OFF" in e0[0], str(e0))
        z0 = st()["uav"]["z"]
        pg.click("#btn-takeoff")
        time.sleep(2.0)
        mid = coach()
        climbing = st()["cmd"]["climb_to"]
        z1 = st()["uav"]["z"]
        check("5 the Take-off button lifts the UAV by itself", z1 > z0 + 0.5 and climbing is not None, f"z {z0:.1f} -> {z1:.1f} m, target {climbing}")
        check("5b the coach reports the climb", "TAKING OFF" in mid[0], str(mid))
        pg.keyboard.down("f")                                    # the pilot pushes the descend key: manual input must win over the climb
        time.sleep(0.8)
        pg.keyboard.up("f")
        check("5c the pilot's climb/descend key cancels the automatic take-off", st()["cmd"]["climb_to"] is None)
        time.sleep(0.5)                                          # let the released key's zero stick reach the server (else a stale descend cancels the next take-off)
        pg.click("#btn-takeoff")
        deadline = time.time() + 5
        while time.time() < deadline and st()["cmd"]["climb_to"] is None:          # state updates lag the click by up to ~0.1 s: wait for the climb to START
            time.sleep(0.1)
        deadline = time.time() + 60
        while time.time() < deadline and st()["cmd"]["climb_to"] is not None:      # wait for the climb to finish by itself
            time.sleep(0.5)
        time.sleep(0.5)
        zt = st()["uav"]["z"]
        check("5d the take-off reaches its height and stops climbing", zt >= 5.7 and st()["cmd"]["climb_to"] is None, f"z = {zt:.1f} m")

        # flying scans ground (the pretend backend reveals a disc under the UAV; the real one maps what the camera sees)
        pg.keyboard.down("w")
        time.sleep(4.0)
        pg.keyboard.up("w")
        time.sleep(1.5)
        scanned = st()["map"]["area_m2"]
        e1 = coach()
        check("6 flying scans ground and the coach moves on to choosing a goal", scanned > 40 and "GOAL" in e1[0], f"{scanned:.0f} m2, {e1}")
        pg.screenshot(path=str(out / "03_flying.png"))

        # ---- e-stop ------------------------------------------------------------------------------------------------------------
        pg.click("[data-layout=single]")                         # leave keyboard focus on a clicked button, then press Space
        pg.keyboard.press("Space")
        time.sleep(0.8)
        s_ = st()
        banner = pg.evaluate("!document.getElementById('banner').hidden")
        layout_now = pg.evaluate("__studio.prefs.layout")
        check("7 Space e-stops, shows a banner, and does not re-press the last clicked button", s_["cmd"]["estop"] and banner and layout_now == "single",
              f"estop={s_['cmd']['estop']} banner={banner} layout={layout_now}")
        check("7b the coach explains the stop and offers to release it", "STOPPED" in coach()[0], str(coach()))
        pg.click("#btn-takeoff", force=True)
        time.sleep(0.5)
        check("7c take-off is refused while stopped", st()["cmd"]["climb_to"] is None)
        pg.screenshot(path=str(out / "04_estopped.png"))
        pg.click("#coach-action")                                # the coach's own button releases it
        time.sleep(0.8)
        check("7d the coach button releases the stop", not st()["cmd"]["estop"] and not pg.evaluate("!document.getElementById('banner').hidden"))

        # ---- every side-panel control does something ---------------------------------------------------------------------------------
        bad: list[str] = []
        for lay, n in (("single", 1), ("inset", 2), ("quad", 4)):
            pg.click(f"[data-layout={lay}]")
            time.sleep(0.4)
            if pg.evaluate("document.querySelectorAll('.pane').length") != n:
                bad.append(f"layout {lay}")
        for colour in ("drivable", "plain", "height"):
            pg.click(f"[data-colour={colour}]")
            if pg.evaluate("__studio.terrain.mode") != colour:
                bad.append(f"colour {colour}")
        pg.click("[data-layout=inset]")
        for f in ("uav", "ugv", "none"):
            pg.click(f"[data-follow={f}]")
            if pg.evaluate("__studio.S.follow") != f:
                bad.append(f"follow {f}")
        pg.click("#opt-world")
        if pg.evaluate("__studio.scenery.group.visible"):
            bad.append("real-site switch (hide)")
        pg.click("#opt-world")
        if not pg.evaluate("__studio.scenery.group.visible"):
            bad.append("real-site switch (show)")
        pg.click("#opt-scanned")
        if pg.evaluate("__studio.terrain.mesh && __studio.terrain.mesh.visible"):
            bad.append("scanned-map switch (hide)")
        pg.click("#opt-scanned")
        if not pg.evaluate("__studio.terrain.mesh && __studio.terrain.mesh.visible"):
            bad.append("scanned-map switch (show)")
        pg.fill("#rng-scale", "6")
        pg.dispatch_event("#rng-scale", "input")
        if pg.evaluate("__studio.prefs.scale") != 6 or "6" not in pg.inner_text("#val-scale"):
            bad.append("robot size slider")
        pg.fill("#rng-speed", "0.4")
        pg.dispatch_event("#rng-speed", "input")
        if abs(pg.evaluate("__studio.prefs.speed") - 0.4) > 1e-6 or "40" not in pg.inner_text("#val-speed"):
            bad.append("speed slider")
        pg.click("#seg-target [data-target=ugv]")
        time.sleep(0.5)
        if st()["mode"] != "manual" or pg.evaluate("document.getElementById('keys-ugv').hidden"):
            bad.append("UGV by hand")
        pg.click("#seg-target [data-target=uav]")
        time.sleep(0.5)
        if st()["mode"] != "auto":
            bad.append("back to UAV")
        pg.click("#btn-help")
        if pg.evaluate("document.getElementById('help').hidden"):
            bad.append("help button")
        pg.keyboard.press("Escape")
        if not pg.evaluate("document.getElementById('help').hidden"):
            bad.append("Esc closes help")
        check("8 every side-panel control visibly does what it says", not bad, "broken: " + ", ".join(bad) if bad else "layouts, colours, follow, 2 layer switches, 2 sliders, UGV/UAV, help")

        # ---- goal flow from the first-run state -----------------------------------------------------------------------------------
        pg.click("[data-layout=single]")
        pg.evaluate("__studio.setMainView('top')")
        pg.click("#btn-goal")
        time.sleep(0.5)
        goal_coach = coach()
        # click on scanned, drivable ground: the cell of the map nearest the UAV (revealed by the flight above)
        pt = pg.evaluate("""(() => {
            const S = __studio.S, g = S.grid; if (!g) return null;
            const u = S.state.uav; let best = null;
            for (let r = 0; r < g.h; r++) for (let c = 0; c < g.w; c++) { const i = r * g.w + c;
              if (g.isObserved(i) && g.isWalkable(i)) { const x = g.x0 + c * g.res, y = g.y0 + r * g.res, d = Math.hypot(x - u.x, y - u.y);
                if (!best || d < best.d) best = {x, y, d}; } }
            if (!best) return null;
            const pane = __studio.panes[0], el = document.querySelector('.pane .hit').getBoundingClientRect(), t = pane.top;
            const a = el.width / el.height;
            return {px: el.left + el.width / 2 + (best.x - t.cx) / (t.half * a) * (el.width / 2), py: el.top + el.height / 2 - (best.y - t.cy) / t.half * (el.height / 2), x: best.x, y: best.y};
        })()""")
        check("9 the goal tool changes the coach text", "SET GOAL" in goal_coach[0], str(goal_coach))
        if pt:
            pg.mouse.click(pt["px"], pt["py"])
            time.sleep(1.5)
            goal = st()["goal"]
            check("9b clicking scanned ground sets the goal and the coach follows the mission",
                  goal is not None and ("DRIVING" in coach()[0] or "DONE" in coach()[0] or "PLANNING" in coach()[0]), f"goal={goal}, coach={coach()}")
        else:
            check("9b clicking scanned ground sets the goal", False, "no drivable scanned cell found")
        pg.screenshot(path=str(out / "05_goal.png"))

        # P5 settings card: a preset changes the limits on the SERVER (the page only shows what the server answers with)
        pg.click("#seg-preset button[data-preset=gentle]")
        try:
            pg.wait_for_function("__studio.S.settings && __studio.S.settings.max_xy === 1.5", timeout=30000)
        except Exception:                                          # say WHAT the page held, not just "timeout"
            print("settings now:", pg.evaluate("[JSON.stringify(__studio.S.settings), __studio.S.connected]"))
            raise
        sl = pg.evaluate("[+document.getElementById('set-max_xy').value, document.getElementById('val-max_xy').textContent, "
                         "document.querySelector('#seg-preset button.on').dataset.preset]")
        check("11 the 'gentle' preset lowers the limits and the card shows them", sl[0] == 1.5 and sl[2] == "gentle" and "1.5" in sl[1], str(sl))
        pg.evaluate("const e=document.getElementById('set-max_z'); e.value=0.5; e.dispatchEvent(new Event('change'))")
        pg.wait_for_function("__studio.S.settings.max_z === 0.5", timeout=30000)
        check("11b one slider changes one limit; no preset matches any more",
              pg.evaluate("!document.querySelector('#seg-preset button.on')"))
        pg.evaluate("__studio.S.ws.send(JSON.stringify({t:'settings', max_xy: 99}))")        # a hostile/buggy client asks for far too much
        pg.wait_for_function("__studio.S.settings.max_xy > 1.5", timeout=30000)
        check("11c the server clamps an absurd request to its hard cap", pg.evaluate("__studio.S.settings.max_xy") == 5.0)
        pg.click("#seg-preset button[data-preset=normal]")
        pg.screenshot(path=str(out / "06_settings.png"))

        # auto-scan: the button starts a sweep by itself, the coach shows progress, a flight key takes over, the e-stop stops it
        pg.keyboard.press("Escape"); time.sleep(0.5)
        pg.click("#btn-autoscan")
        pg.wait_for_function("__studio.S.state.cmd.autoscan !== null", timeout=30000)
        x0 = st()["uav"]["x"], st()["uav"]["y"]
        time.sleep(4)
        moved = ((st()["uav"]["x"] - x0[0]) ** 2 + (st()["uav"]["y"] - x0[1]) ** 2) ** 0.5
        c12 = coach()
        check("12 Auto-scan flies the UAV by itself and the coach shows the progress", moved > 0.5 and "AUTO-SCAN" in c12[0] and "leg" in c12[1],
              f"moved {moved:.1f} m, {c12}")
        pg.keyboard.down("r"); time.sleep(0.6); pg.keyboard.up("r")
        pg.wait_for_function("__studio.S.state.cmd.autoscan === null", timeout=30000)
        check("12b a flight key takes over from the auto-scan", True)
        pg.click("#btn-autoscan")
        pg.wait_for_function("__studio.S.state.cmd.autoscan !== null", timeout=30000)
        pg.click("#btn-autoscan")                                  # the same button now says "Stop" - and does
        pg.wait_for_function("__studio.S.state.cmd.autoscan === null", timeout=30000)
        pg.click("#btn-autoscan")
        pg.wait_for_function("__studio.S.state.cmd.autoscan !== null", timeout=30000)
        pg.keyboard.press("Space")
        pg.wait_for_function("__studio.S.state.cmd.autoscan === null && __studio.S.state.cmd.estop", timeout=30000)
        check("12c the Stop button and the e-stop both end it", True)
        pg.keyboard.press("Space"); time.sleep(0.5)
        pg.screenshot(path=str(out / "07_autoscan.png"))

        check("10 no JavaScript errors during the whole run", not errors, "; ".join(errors[:3]))
        browser.close()

    failed = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
