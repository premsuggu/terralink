#!/usr/bin/env python3
"""Compose the run video from a `record_world_run.py` recording (offline - no simulator needed).

    python3 src/nav/scripts/make_run_video.py --run /tmp/w2_run --world w2_container_yard \\
        --start -3 -26 --goal -12 13.8 --out media/runs/w2_container_yard_run.mp4

Three panels, all from the same simulation run:
  * left   - the whole world from the fixed overhead camera, with the A* route (yellow), the path
             actually driven (cyan), start / goal and the UGV's TRUE pose from the physics
             ground truth (30 Hz, so motion is smooth even though the software renderer only
             delivers ~2 overhead frames per simulated second). The UGV is only ~9 px wide at
             this scale, so it is drawn to scale on a "clean plate": the median of the overhead
             frames with the robot masked out, i.e. the real rendered world without the robot.
  * top right    - a 14 m window that follows the UGV (same plate, 1.8x enlarged).
  * bottom right - the UGV's own forward camera (real footage, latest frame at each instant).
Playback is exact simulation time (frames are stamped with sim time, not wall time).
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
for _p in (_ROOT / "nav", _ROOT / "emap"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

FPS = 20
MAIN, COLW, COLH = 900, 600, 450
YELLOW, CYAN, GREEN, RED, ORANGE = (0, 215, 255), (255, 220, 40), (60, 200, 60), (40, 40, 230), (30, 120, 255)


def read_frames(folder: Path):
    files = sorted(folder.glob("*.jpg"))
    return np.array([int(f.stem) / 1000.0 for f in files]), files


def camera_geometry(world_file: Path, px: int):
    """(cx, cy, metres per pixel at the ground) of the overhead camera in a *_run.world file."""
    txt = world_file.read_text()
    blk = txt[txt.index('name="overhead_overview_camera"'):]
    cx, cy, h = (float(v) for v in re.search(r"<pose>([-\d.]+) ([-\d.]+) ([-\d.]+) ", blk).groups())
    fov = float(re.search(r"<horizontal_fov>([\d.]+)", blk).group(1))
    return cx, cy, 2 * h * math.tan(fov / 2) / px


def clean_plate(files, times, pose_at, to_px, px, n=24, mask_m=1.3, mpp=0.042):
    """Median of overhead frames, ignoring pixels within `mask_m` of the robot in each frame."""
    idx = np.linspace(0, len(files) - 1, min(n, len(files))).astype(int)
    frames = [cv2.imread(str(files[i])) for i in idx]
    H, W = frames[0].shape[:2]
    stack_valid = []
    yy, xx = np.mgrid[0:H, 0:W]
    for i, f in zip(idx, frames):
        x, y, _ = pose_at(times[i])
        u, v = to_px(x, y)
        stack_valid.append(((xx - u) ** 2 + (yy - v) ** 2) > (mask_m / mpp) ** 2)
    out = np.empty_like(frames[0])
    for r0 in range(0, H, 120):
        sl = slice(r0, min(H, r0 + 120))
        data = np.stack([f[sl].astype(np.float32) for f in frames])                 # (n, rows, W, 3)
        valid = np.stack([m[sl] for m in stack_valid])[..., None]
        data = np.where(valid, data, np.nan)
        with np.errstate(all="ignore"):
            med = np.nanmedian(data, axis=0)
        fallback = np.median(np.stack([f[sl] for f in frames]), axis=0)
        out[sl] = np.where(np.isnan(med), fallback, med).astype(np.uint8)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--world", required=True, help="e.g. w2_container_yard")
    ap.add_argument("--start", nargs=2, type=float, required=True)
    ap.add_argument("--goal", nargs=2, type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--map", default="uavview", help="map variant the planner was given (uavview | groundtruth)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hold the last frame")
    ap.add_argument("--fps", type=int, default=FPS, help="output frame rate (10 halves the composing time of a long run; playback speed stays exact)")
    a = ap.parse_args(argv)
    run = Path(a.run)

    from nav.replay import build_masks, load_snapshot, run_plan
    snap = load_snapshot(_ROOT / "nav" / "maps" / f"{a.world}_{a.map}.npz")
    plans = []                  # [(t, waypoints, kinds, exploration)] - what the planner actually returned during the run
    pf = run / "plans.jsonl"
    if pf.exists():
        import json
        for line in pf.read_text().splitlines():
            r = json.loads(line)["plan"] if line.strip() else None
            if r and r.get("waypoints"):
                plans.append((json.loads(line)["t"], np.array(r["waypoints"]), r.get("kinds", []), r.get("exploration", "")))
    if plans:       # a live mission: the plans are whatever the planner returned during the run
        route = np.zeros((0, 2))
    else:           # a run on a known map: the offline A* route on the same map is what the planner was given
        plan = run_plan(snap, build_masks(snap, use_frontier=False, use_anomaly=False, resolved=[], frontier_min_width_m=0.0),
                        tuple(a.start), tuple(a.goal), planner="astar", explore=False)
        route = np.array(plan.waypoints) if plan.valid else np.zeros((0, 2))
    plan_times = np.array([p[0] for p in plans])

    P = np.loadtxt(run / "poses.csv", delimiter=",", skiprows=1)
    pt, pxs, pys, pyaw = P[:, 0], P[:, 1], P[:, 2], np.unwrap(P[:, 3])
    pose_at = lambda t: (np.interp(t, pt, pxs), np.interp(t, pt, pys), np.interp(t, pt, pyaw))      # noqa: E731
    dist = np.r_[0, np.cumsum(np.hypot(np.diff(pxs), np.diff(pys)))]

    ot, ofiles = read_frames(run / "overhead")
    vt, vfiles = read_frames(run / "pov")
    full = cv2.imread(str(ofiles[0])).shape[0]
    cx, cy, mpp = camera_geometry(_ROOT / "nav" / "worlds" / f"{a.world}_run.world", full)
    to_px = lambda x, y: (full / 2 + (x - cx) / mpp, full / 2 - (y - cy) / mpp)                     # noqa: E731
    print(f"overhead {full}px, {mpp * 100:.2f} cm/px; {len(ofiles)} overhead / {len(vfiles)} pov frames; "
          f"{pt[-1] - pt[0]:.1f} s of poses; driven {dist[-1]:.2f} m")
    plate = clean_plate(ofiles, ot, pose_at, to_px, full, mpp=mpp)

    def draw_world(img, scale, off, t_idx, x, y, yaw, thick, t=None):
        """Draw route, trail, markers and the robot on `img`. world -> pixel: to_px*scale - off."""
        tf = lambda X, Y: (int(round(to_px(X, Y)[0] * scale - off[0])), int(round(to_px(X, Y)[1] * scale - off[1])))   # noqa: E731
        if len(route) > 1:
            cv2.polylines(img, [np.array([tf(*p) for p in route], np.int32)], False, YELLOW, thick, cv2.LINE_AA)
        if plans and t is not None:
            k = int(np.searchsorted(plan_times, t, side="right")) - 1
            if k >= 0:
                _, wps, kinds, expl = plans[k]
                for i in range(len(wps) - 1):        # confirmed stretches yellow, tentative (frontier / anomaly) orange, exploration detours magenta
                    kind = kinds[i] if i < len(kinds) else "free"
                    col = (255, 0, 255) if expl else (YELLOW if kind == "free" else (0, 140, 255))
                    cv2.line(img, tf(*wps[i]), tf(*wps[i + 1]), col, thick, cv2.LINE_AA)
        if t_idx > 1:
            cv2.polylines(img, [np.array([tf(X, Y) for X, Y in zip(pxs[:t_idx], pys[:t_idx])], np.int32)], False, CYAN, thick, cv2.LINE_AA)
        s = tf(*a.start)
        cv2.rectangle(img, (s[0] - 6 * thick // 2, s[1] - 6 * thick // 2), (s[0] + 6 * thick // 2, s[1] + 6 * thick // 2), GREEN, -1)
        g = tf(*a.goal)
        cv2.drawMarker(img, g, RED, cv2.MARKER_STAR, 12 * thick, thick, cv2.LINE_AA)
        c = tf(x, y)
        r = max(3, int(round(0.22 / mpp * scale)))                                                    # true robot radius
        cv2.circle(img, c, int(1.0 / mpp * scale), (255, 255, 255), max(1, thick - 1), cv2.LINE_AA)      # 1 m ring: easy to spot
        cv2.circle(img, c, r, ORANGE, -1, cv2.LINE_AA)
        tip = (int(c[0] + math.cos(yaw) * 2.2 * r), int(c[1] - math.sin(yaw) * 2.2 * r))                 # image y points down
        cv2.arrowedLine(img, c, tip, (0, 0, 0), max(1, thick - 1), cv2.LINE_AA, tipLength=0.5)

    fps = a.fps
    t_first, t_last = max(pt[0], vt[0]), pt[-1]
    n_frames = int((t_last - t_first) * fps) + 1 + int(a.hold * fps)
    W, H = MAIN + COLW, MAIN
    vw = None
    for fourcc in ("avc1", "mp4v"):
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*fourcc), fps, (W, H))
        if vw.isOpened():
            break
    sc_main = MAIN / full
    plate_main = cv2.resize(plate, (MAIN, MAIN), interpolation=cv2.INTER_AREA)
    zoom, crop_m = 1.8, 14.0
    title = a.title or a.world
    for k in range(n_frames):
        t = min(t_first + k / fps, t_last)
        x, y, yaw = pose_at(t)
        ti = int(np.searchsorted(pt, t))
        canvas = np.zeros((H, W, 3), np.uint8)
        # --- main overhead panel
        m = plate_main.copy()
        draw_world(m, sc_main, (0, 0), ti, x, y, yaw, 2, t)
        canvas[:, :MAIN] = m
        # --- follow window (enlarged crop of the full-res plate, overlays drawn at the enlarged scale)
        cwpx, chpx = int(crop_m / mpp), int(crop_m / mpp * COLH / COLW)
        u, v = to_px(x, y)
        x0 = int(np.clip(u - cwpx / 2, 0, full - cwpx)); y0 = int(np.clip(v - chpx / 2, 0, full - chpx))
        crop = cv2.resize(plate[y0:y0 + chpx, x0:x0 + cwpx], (COLW, COLH), interpolation=cv2.INTER_CUBIC)
        s2 = COLW / cwpx
        draw_world(crop, s2, (x0 * s2, y0 * s2), ti, x, y, yaw, 3, t)
        canvas[:COLH, MAIN:] = crop
        # --- UGV forward camera (latest real frame)
        vi = max(0, int(np.searchsorted(vt, t, side="right")) - 1)
        canvas[COLH:, MAIN:] = cv2.resize(cv2.imread(str(vfiles[vi])), (COLW, COLH), interpolation=cv2.INTER_LINEAR)
        # --- labels
        def label(img, text, org, scale=0.6):
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
        label(canvas, title, (14, 26), 0.7)
        d_now = float(np.interp(t, pt, dist))
        if plans:
            k = int(np.searchsorted(plan_times, t, side="right")) - 1
            info = f"plan #{k + 1} of {len(plans)}" + (f" ({plans[k][3]} detour)" if k >= 0 and plans[k][3] else "") if k >= 0 else "no plan yet"
            label(canvas, f"t = {t - pt[0]:5.1f} s   driven {d_now:5.1f} m   {info}", (14, 52))
            label(canvas, "yellow: confirmed route   orange: tentative stretch   magenta: exploration detour   cyan: path driven", (14, MAIN - 14), 0.5)
        else:
            label(canvas, f"t = {t - pt[0]:5.1f} s   driven {d_now:5.1f} m   A* route {plan_len(route):.1f} m", (14, 52))
            label(canvas, "yellow: A* route   cyan: path driven   green: start   red star: goal", (14, MAIN - 14), 0.55)
        label(canvas, "UGV follow view", (MAIN + 10, 24)); label(canvas, "UGV camera", (MAIN + 10, COLH + 24))
        cv2.line(canvas, (MAIN, 0), (MAIN, H), (30, 30, 30), 2); cv2.line(canvas, (MAIN, COLH), (W, COLH), (30, 30, 30), 2)
        vw.write(canvas)
    vw.release()
    print(f"wrote {a.out}: {n_frames} frames, {n_frames / fps:.1f} s at {fps} fps")
    return 0


def plan_len(route) -> float:
    return float(np.hypot(*np.diff(route, axis=0).T).sum()) if len(route) > 1 else 0.0


if __name__ == "__main__":
    sys.exit(main())
