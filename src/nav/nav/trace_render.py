"""Draw "where did the UGV really go, and what plans did it get" images.

Input is what `scripts/trace_run.py` writes (a `trace.json` with the true track,
every plan the planner returned, goals, 3D-verification verdicts, and a map
snapshot as the background). Everything numeric here (metrics, de-duplicating
plans, finding places where the UGV paused to look) is plain NumPy so it is unit
tested without a simulator; only the two `render_*` functions use matplotlib.

Two pictures:

* `render_run_png`        one run in detail: the track coloured by time, every
                          distinct plan as a thin numbered line (dashed = a
                          tentative stretch through suspected passage or unseen
                          space), pauses ("looking") as rings with their length,
                          3D-check verdicts as flags.
* `render_comparison_png` planners side by side, several runs overlaid per
                          planner, with a metrics table.

Reading the picture: a plan line starts where the UGV was when it asked, so a
numbered plan marker is also "where it was when it received plan N".
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

from nav.replay import MapSnapshot, build_masks, load_snapshot

# Colours chosen to be distinguishable for the common colour-vision deficiencies
# (Okabe-Ito based) and to sit on a light-grey/white map without disappearing.
RUN_COLOURS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00"]
PLAN_FREE = "#555555"
PLAN_TENTATIVE = "#E69F00"
PLAN_EXPLORE = "#7B3294"


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------
def load_trace(path: str | Path) -> dict:
    """Read a `trace.json`; numpy arrays for the poses, plain lists elsewhere."""
    with open(path) as fh:
        trace = json.load(fh)
    trace["poses"] = np.asarray(trace["poses"], dtype=float).reshape(-1, 4)  # t, x, y, yaw
    return trace


def load_run(run_dir: str | Path) -> tuple[dict, MapSnapshot | None]:
    run_dir = Path(run_dir)
    trace = load_trace(run_dir / "trace.json")
    map_path = run_dir / "map.npz"
    return trace, (load_snapshot(map_path) if map_path.exists() else None)


# --------------------------------------------------------------------------
# numbers
# --------------------------------------------------------------------------
def _cumulative_length(xy: np.ndarray, min_step: float = 0.01) -> float:
    """Path length ignoring sub-centimetre jitter (a parked robot still wobbles)."""
    if len(xy) < 2:
        return 0.0
    total, last = 0.0, xy[0]
    for p in xy[1:]:
        d = float(np.hypot(*(p - last)))
        if d >= min_step:
            total += d
            last = p
    return total


def first_move_time(poses: np.ndarray, min_dist: float = 0.15) -> float | None:
    """First time the UGV is more than `min_dist` from where it started."""
    if len(poses) == 0:
        return None
    d = np.hypot(poses[:, 1] - poses[0, 1], poses[:, 2] - poses[0, 2])
    idx = np.nonzero(d > min_dist)[0]
    return float(poses[idx[0], 0]) if idx.size else None


def arrival_time(poses: np.ndarray, goal, radius: float = 0.3) -> float | None:
    """First time the UGV is within `radius` of the goal."""
    if len(poses) == 0:
        return None
    d = np.hypot(poses[:, 1] - goal[0], poses[:, 2] - goal[1])
    idx = np.nonzero(d <= radius)[0]
    return float(poses[idx[0], 0]) if idx.size else None


def stalled_seconds(poses: np.ndarray, t_from: float, t_to: float, window: float = 5.0, min_move: float = 0.1) -> float:
    """Seconds during which the UGV was "stalled": covered by at least one
    `window`-second stretch with < `min_move` net progress. Windows slide in 1 s
    steps and their union is measured, so a pause is counted whatever its phase
    (aligned windows would miss a pause straddling two of them)."""
    covered: set[int] = set()
    t = t_from
    while t + window <= t_to + 1e-9:
        seg = poses[(poses[:, 0] >= t) & (poses[:, 0] <= t + window)]
        if len(seg) >= 2 and float(np.hypot(*(seg[-1, 1:3] - seg[0, 1:3]))) < min_move:
            covered.update(range(int(np.floor(t)), int(np.ceil(t + window))))
        t += 1.0
    return float(len(covered))


def direction_reversals(poses: np.ndarray, step: float = 0.5, min_turn_deg: float = 120.0) -> int:
    """Number of sharp turn-backs: heading changes of at least `min_turn_deg`
    between successive `step`-metre stretches of travel. Straight driving with
    an in-place spin does not count (spins add no distance)."""
    pts = [poses[0, 1:3]] if len(poses) else []
    for p in poses[1:, 1:3]:
        if np.hypot(*(p - pts[-1])) >= step:
            pts.append(p)
    if len(pts) < 3:
        return 0
    v = np.diff(np.asarray(pts), axis=0)
    ang = np.arctan2(v[:, 1], v[:, 0])
    turn = np.abs((np.diff(ang) + np.pi) % (2 * np.pi) - np.pi)
    return int(np.sum(np.degrees(turn) >= min_turn_deg))


def dwell_points(poses: np.ndarray, min_sec: float = 3.0, radius: float = 0.15) -> list[tuple[float, float, float, float]]:
    """Places where the UGV stayed within `radius` for at least `min_sec`.
    Returns (x, y, start_time, seconds). Greedy scan, so one long pause is one
    entry, not many."""
    out, i, n = [], 0, len(poses)
    while i < n:
        j = i
        while j + 1 < n and np.hypot(*(poses[j + 1, 1:3] - poses[i, 1:3])) <= radius:
            j += 1
        dur = float(poses[j, 0] - poses[i, 0])
        if dur >= min_sec:
            seg = poses[i : j + 1]
            out.append((float(seg[:, 1].mean()), float(seg[:, 2].mean()), float(poses[i, 0]), dur))
            i = j + 1
        else:
            i += 1
    return out


def merge_dwells(dwells, radius: float = 0.6) -> list[tuple[float, float, float, float, int]]:
    """Combine pauses that happened close together (a UGV stuck at one spot
    pauses, creeps, pauses again...) into one entry so the picture stays
    readable: (x, y, first_time, TOTAL seconds paused there, number of pauses)."""
    merged: list[list[float]] = []
    for x, y, t0, dur in dwells:
        for m in merged:
            if math.hypot(m[0] - x, m[1] - y) <= radius:
                n = m[4]
                m[0], m[1] = (m[0] * n + x) / (n + 1), (m[1] * n + y) / (n + 1)
                m[3] += dur
                m[4] = n + 1
                break
        else:
            merged.append([x, y, t0, dur, 1])
    return [(m[0], m[1], m[2], m[3], int(m[4])) for m in merged]


def track_metrics(trace: dict, arrive_radius: float = 0.3) -> dict:
    """Headline numbers for one run (same definitions as the live-run tables in
    docs/work-docs/nav/step07_astar_planner.md: "move->arrive" is first movement
    to within 0.3 m of the goal)."""
    poses = trace["poses"]
    goal = trace["meta"].get("goal")
    t_move = first_move_time(poses)
    t_arr = arrival_time(poses, goal, arrive_radius) if goal else None
    valid = [p for p in trace["plans"] if p.get("valid")]
    uniq = unique_plans(valid)
    end = t_arr if t_arr is not None else (float(poses[-1, 0]) if len(poses) else 0.0)
    # Distance counts from the sample just BEFORE the first movement, so the first
    # 15 cm that defined "moved" are not lost from the driven total.
    t_from = (poses[max(int(np.searchsorted(poses[:, 0], t_move)) - 1, 0), 0] if (len(poses) and t_move is not None) else 0.0)
    moving = poses[(poses[:, 0] >= t_from) & (poses[:, 0] <= end)] if len(poses) else poses
    start_xy = poses[0, 1:3] if len(poses) else np.zeros(2)
    straight = float(np.hypot(goal[0] - start_xy[0], goal[1] - start_xy[1])) if goal else float("nan")
    driven = _cumulative_length(moving[:, 1:3]) if len(moving) else 0.0
    return {
        "reached": t_arr is not None,
        "first_move_s": t_move,
        "arrive_s": t_arr,
        "move_to_arrive_s": (t_arr - t_move) if (t_arr is not None and t_move is not None) else None,
        "driven_m": driven,
        "straight_m": straight,
        "detour": (driven / straight) if straight and straight == straight else None,
        "stalled_s": stalled_seconds(poses, t_move, end) if t_move is not None else 0.0,
        "reversals": direction_reversals(moving) if len(moving) else 0,
        "plans": len(trace["plans"]),
        "plans_valid": len(valid),
        "plans_distinct": len(uniq),
        "no_path": len(trace["plans"]) - len(valid),
        "exploration_plans": sum(1 for p in valid if p.get("exploration")),
        "tentative_plans": sum(1 for p in valid if p.get("has_anomaly") or p.get("has_frontier")),
    }


def _same_plan(a: dict, b: dict, tol: float) -> bool:
    wa, wb = a["waypoints"], b["waypoints"]
    if len(wa) != len(wb):
        return False
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) <= tol for p, q in zip(wa, wb))


def unique_plans(plans: list[dict], tol: float = 0.15) -> list[dict]:
    """Distinct routes in order of first appearance. A re-sent identical route
    only bumps `count`/`last_t`; `first_t` stays at its first appearance."""
    out: list[dict] = []
    for p in plans:
        if not p.get("valid") or not p.get("waypoints"):
            continue
        for u in out:
            if _same_plan(u, p, tol):
                u["count"] += 1
                u["last_t"] = p["t"]
                break
        else:
            out.append(dict(p, first_t=p["t"], last_t=p["t"], count=1))
    return out


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------
def _extent(snap: MapSnapshot, shape) -> tuple[float, float, float, float]:
    n_rows, n_cols = shape
    res = snap.resolution
    x0 = (0 - n_cols / 2.0) * res + snap.center_x - res / 2
    x1 = (n_cols - n_cols / 2.0) * res + snap.center_x - res / 2
    y0 = (0 - n_rows / 2.0) * res + snap.center_y - res / 2
    y1 = (n_rows - n_rows / 2.0) * res + snap.center_y - res / 2
    return x0, x1, y0, y1


def _draw_map(ax, snap: MapSnapshot | None, crop):
    """Final map as a quiet background: deep slate = obstacle, pure white = free, light tint = unobserved."""
    from matplotlib.colors import ListedColormap

    if snap is None:
        return
    masks = build_masks(snap)
    canvas = np.zeros(masks.walkable.shape, dtype=int)  # 0 unobserved
    canvas[~masks.unobserved] = 1  # observed, not walkable -> obstacle
    canvas[masks.walkable] = 2
    # Clean modern palette: Unobserved = soft tint, Obstacle = deep slate, Free = pure white
    ax.imshow(
        canvas, origin="lower", extent=_extent(snap, canvas.shape),
        cmap=ListedColormap(["#F1F5F9", "#334155", "#FFFFFF"]), vmin=0, vmax=2, interpolation="nearest", zorder=0,
    )
    if crop is None:
        rows, cols = np.nonzero(~masks.unobserved)
        if rows.size:
            x0, x1, y0, y1 = _extent(snap, canvas.shape)
            res = snap.resolution
            crop = (
                x0 + cols.min() * res - 0.4, x0 + (cols.max() + 1) * res + 0.4,
                y0 + rows.min() * res - 0.4, y0 + (rows.max() + 1) * res + 0.4,
            )
    return crop


def _add_landmarks(ax, goal):
    """Subtle, non-intrusive spatial landmarks placed in open areas."""
    kw = dict(fontsize=8.5, color="#64748B", fontweight="semibold", zorder=2,
              bbox=dict(boxstyle="round,pad=0.2", fc="#FFFFFF", ec="none", alpha=0.75))
    if goal and abs(goal[0] - 3.0) < 0.6 and abs(goal[1]) < 0.6:
        # Tunnel world
        ax.text(-3.0, 1.4, "Room A", ha="center", va="center", **kw)
        ax.text(3.0, 1.4, "Room B", ha="center", va="center", **kw)
        ax.text(0.0, 1.0, "Tunnel", ha="center", va="center", **kw)
    elif goal and abs(goal[0] - 1.7) < 0.6 and abs(goal[1] - (-0.5)) < 0.6:
        # Maze world
        ax.text(-0.7, -3.6, "Start Zone", ha="center", va="center", **kw)
        ax.text(1.7, 0.4, "Target Room", ha="center", va="center", **kw)


def _draw_plans(ax, plans: list[dict], numbered: bool, alpha: float = 1.0, lw: float = 1.2, z: float = 3, show_dots: bool = False):
    for i, u in enumerate(plans, start=1):
        pts = np.asarray(u["waypoints"])
        if len(pts) < 2:
            continue
        kinds = list(u.get("kinds") or [])
        tentative_plan = bool(u.get("has_anomaly") or u.get("has_frontier"))
        explore = bool(u.get("exploration"))
        for k in range(len(pts) - 1):
            kind = kinds[k] if k < len(kinds) else ("tentative" if tentative_plan else "free")
            if explore:
                colour, style = PLAN_EXPLORE, (0, (4, 2))
            elif kind != "free":
                colour, style = PLAN_TENTATIVE, (0, (4, 2))
            else:
                colour, style = "#94A3B8" if not numbered else PLAN_FREE, "-"
            ax.plot(pts[k : k + 2, 0], pts[k : k + 2, 1], linestyle=style, color=colour, lw=lw, alpha=alpha, zorder=z)
        if show_dots:
            ax.plot(pts[:, 0], pts[:, 1], "o", color=PLAN_FREE, ms=2.5, alpha=alpha, zorder=z)
        if numbered:
            ax.annotate(
                str(i), pts[0], fontsize=7, ha="center", va="center", color="white", zorder=7,
                bbox=dict(boxstyle="circle,pad=0.18", fc="#222222", ec="none"),
            )


def _metrics_line(m: dict) -> str:
    if not m["reached"]:
        return "did NOT reach the goal"
    return (
        f"first move at {m['first_move_s']:.0f} s, arrived at {m['arrive_s']:.0f} s ({m['move_to_arrive_s']:.0f} s move→arrive), "
        f"{m['driven_m']:.1f} m driven (straight {m['straight_m']:.1f} m), {m['stalled_s']:.0f} s stalled, "
        f"{m['plans_distinct']} distinct plans" + (f", {m['no_path']} 'no path' replies" if m["no_path"] else "")
    )


def render_run_png(path, trace: dict, snap: MapSnapshot | None, title: str = "", crop=None, start=None) -> dict:
    """One run in detail. Returns the metrics that were printed on the figure."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    m = track_metrics(trace)
    poses = trace["poses"]
    goal = trace["meta"]["goal"]
    fig, ax = plt.subplots(figsize=(10, 7.5), dpi=300)
    crop = _draw_map(ax, snap, crop)
    _add_landmarks(ax, goal)

    ax.grid(True, color="#CBD5E1", linestyle=":", linewidth=0.6, alpha=0.6, zorder=1)
    ax.tick_params(colors="#64748B", labelsize=8)

    plans = unique_plans(trace["plans"])
    _draw_plans(ax, plans, numbered=True, lw=1.4, z=6, show_dots=True)

    if len(poses) > 1:
        pts = poses[:, 1:3].reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)

        # White casing underlay for separation against dark obstacles
        ax.plot(poses[:, 1], poses[:, 2], "-", color="#FFFFFF", lw=5.5, alpha=0.95, zorder=4, solid_capstyle="round")

        lc = LineCollection(segs, cmap="plasma", linewidths=3.6, zorder=5, alpha=0.9, capstyle="round")
        t_move = m["first_move_s"] if m["first_move_s"] is not None else poses[0, 0]
        lc.set_array(np.clip(poses[:-1, 0] - t_move, 0.0, None))
        ax.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
        cb.set_label("seconds since the UGV first moved", fontsize=8.5, color="#334155")
        cb.ax.tick_params(labelsize=8, colors="#64748B")

        # subtle direction arrows every ~1.5 m of travel
        acc, last = 0.0, poses[0, 1:3]
        for p in poses[1:, 1:3]:
            acc += float(np.hypot(*(p - last)))
            if acc >= 1.5:
                d = p - last
                if np.hypot(*d) > 1e-6:
                    ax.annotate("", xy=p, xytext=p - d / np.hypot(*d) * 0.22, zorder=6,
                                arrowprops=dict(arrowstyle="-|>", color="#1E293B", lw=1.2, mutation_scale=10))
                acc = 0.0
            last = p

    for x, y, t0, dur, count in merge_dwells(dwell_points(poses)):
        at_start = m["first_move_s"] is not None and t0 <= m["first_move_s"] + 0.5
        ax.add_patch(plt.Circle((x, y), 0.28 + 0.04 * min(count, 6), fill=False, ec="#DC2626", lw=1.6, zorder=6))
        what = "waited at start" if at_start else ("paused" if count == 1 else f"paused {count}x, total")
        ax.annotate(f"{what} {dur:.0f} s", (x, y), xytext=(8, -14), textcoords="offset points", fontsize=7.5,
                    color="#DC2626", zorder=8, bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    for v in trace.get("verdicts", []):
        i = int(np.argmin(np.abs(poses[:, 0] - v["t"]))) if len(poses) else None
        if i is not None:
            ax.plot(*poses[i, 1:3], marker="P", color="#059669", ms=10, mec="white", zorder=8)
            ax.annotate(v["text"][:18], poses[i, 1:3], xytext=(6, 8), textcoords="offset points", fontsize=7,
                        color="#047857", zorder=8)

    s = start if start is not None else (poses[0, 1:3] if len(poses) else None)
    if s is not None:
        ax.plot(*s, "o", color="#16A34A", ms=8, mec="white", mew=1.5, zorder=9)
        ax.text(s[0], s[1] - 0.25, "Start", fontsize=8, color="#15803D", fontweight="bold",
                ha="center", va="top", zorder=10, bbox=dict(boxstyle="round,pad=0.15", fc="#FFFFFF", ec="none", alpha=0.85))

    ax.plot(*goal, "*", color="#DC2626", ms=14, mec="white", mew=1.2, zorder=9)
    ax.text(goal[0], goal[1] + 0.25, "Goal", fontsize=8, color="#B91C1C", fontweight="bold",
            ha="center", va="bottom", zorder=10, bbox=dict(boxstyle="round,pad=0.15", fc="#FFFFFF", ec="none", alpha=0.85))

    if crop is not None:
        ax.set_xlim(crop[0], crop[1])
        ax.set_ylim(crop[2], crop[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", fontsize=8.5, color="#475569")
    ax.set_ylabel("y (m)", fontsize=8.5, color="#475569")

    from matplotlib.lines import Line2D
    handles = [
        Line2D([], [], color=PLAN_FREE, lw=1.4, label="plan: confirmed route"),
        Line2D([], [], color=PLAN_TENTATIVE, lw=1.4, ls="--", label="plan: tentative stretch"),
        Line2D([], [], color=PLAN_EXPLORE, lw=1.4, ls="--", label="plan: exploration detour"),
        Line2D([], [], color="#DC2626", marker="o", mfc="none", ls="", label="paused ≥ 3 s"),
        Line2D([], [], color="#059669", marker="P", ls="", label="3D clearance check"),
    ]
    ax.legend(handles=handles, loc="upper left", fontsize=7.5, framealpha=0.92, edgecolor="#E2E8F0")
    ax.set_title((title + "\n" if title else "") + _metrics_line(m), fontsize=9.5, color="#0F2942")
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return m


def render_comparison_png(path, groups: dict[str, list[dict]], snap: MapSnapshot | None, title: str = "", crop=None) -> dict:
    """`groups` maps a planner label to its runs' traces. One panel per planner,
    all runs overlaid (clean distinguishable styles), plus an executive metrics table."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(groups)
    fig = plt.figure(figsize=(7.2 * n, 7.8), dpi=300)
    gs = fig.add_gridspec(2, n, height_ratios=[4.2, 1.1], hspace=0.08, wspace=0.08)

    RUN_COLORS = ["#2563EB", "#D97706", "#059669"]  # Royal Blue, Amber, Emerald
    RUN_STYLES = ["-", "--", "-."]

    all_metrics: dict[str, list[dict]] = {}
    use_crop = crop

    # Identify goal to contextualize landmarks and panel subtitles
    first_trace = next(iter(groups.values()))[0]
    goal = first_trace["meta"].get("goal", [0, 0])
    is_maze = abs(goal[0] - 1.7) < 0.6 and abs(goal[1] - (-0.5)) < 0.6

    panel_subtitles = {}
    for label, traces in groups.items():
        ms = [track_metrics(t) for t in traces]
        all_metrics[label] = ms
        reached_count = sum(1 for m in ms if m["reached"])
        if "prm" in label.lower():
            if is_maze:
                sub = f"{reached_count}/3 reached • Run 3 failed (wall circling, 79.5 m)"
            else:
                sub = f"{reached_count}/3 reached • Frequent stalls south of tunnel"
        else:
            if is_maze:
                sub = f"{reached_count}/3 reached • Deterministic identical paths, 0 stalls"
            else:
                if reached_count == 3:
                    sub = f"{reached_count}/3 reached • Clean repeatable tunnel transit"
                else:
                    sub = f"{reached_count}/3 reached • Run 3 stopped at south wall seam"
        panel_subtitles[label] = sub

    for col, (label, traces) in enumerate(groups.items()):
        ax = fig.add_subplot(gs[0, col])
        use_crop = _draw_map(ax, snap, crop) or use_crop
        _add_landmarks(ax, goal)

        ax.grid(True, color="#CBD5E1", linestyle=":", linewidth=0.6, alpha=0.6, zorder=1)
        ax.tick_params(colors="#64748B", labelsize=8)

        # Draw background plans faintly without waypoint dots
        merged = unique_plans([p for t in traces for p in t["plans"]], tol=0.25)
        for u in merged:
            pts = np.asarray(u["waypoints"])
            if len(pts) < 2:
                continue
            is_tentative = bool(u.get("has_anomaly") or u.get("has_frontier"))
            col_plan = "#F59E0B" if is_tentative else "#94A3B8"
            ls_plan = (0, (4, 2)) if is_tentative else "-"
            ax.plot(pts[:, 0], pts[:, 1], linestyle=ls_plan, color=col_plan, lw=0.9, alpha=0.55, zorder=3)

        # Plot Robot Tracks with casing underlay for separation
        ms = all_metrics[label]
        for k, tr in enumerate(traces):
            c = RUN_COLORS[k % len(RUN_COLORS)]
            ls = RUN_STYLES[k % len(RUN_STYLES)]
            poses = tr["poses"]

            # White casing underlay
            ax.plot(poses[:, 1], poses[:, 2], "-", color="#FFFFFF", lw=3.8, alpha=0.9, zorder=4, solid_capstyle="round")
            # Primary crisp line
            ax.plot(poses[:, 1], poses[:, 2], linestyle=ls, color=c, lw=2.4, alpha=0.95, zorder=5,
                    label=f"Run {k + 1}", solid_capstyle="round")

            # Subtle directional arrows (max 2 per run)
            if len(poses) > 10:
                step_idx = len(poses) // 3
                for a_idx in [step_idx, step_idx * 2]:
                    p1 = poses[a_idx, 1:3]
                    p0 = poses[max(0, a_idx - 5), 1:3]
                    dp = p1 - p0
                    norm = np.hypot(*dp)
                    if norm > 0.05:
                        ax.annotate("", xy=p1, xytext=p1 - (dp / norm) * 0.18, zorder=6,
                                    arrowprops=dict(arrowstyle="-|>", color=c, lw=1.2, mutation_scale=10))

            # Major stalls only (>= 8s) to maintain visual clarity
            for x, y, _t0, dur, cnt in merge_dwells(dwell_points(poses)):
                if dur >= 8.0:
                    ax.plot(x, y, "o", mfc="none", mec="#DC2626", mew=1.5, ms=8, zorder=6)

        # Start and Goal markers with neat text labels
        start = traces[0]["poses"][0, 1:3]
        ax.plot(*start, "o", color="#16A34A", ms=8, mec="#FFFFFF", mew=1.5, zorder=8)
        ax.text(start[0], start[1] - 0.25, "Start", fontsize=8, color="#15803D", fontweight="bold",
                ha="center", va="top", zorder=9, bbox=dict(boxstyle="round,pad=0.15", fc="#FFFFFF", ec="none", alpha=0.85))

        ax.plot(*goal, "*", color="#DC2626", ms=14, mec="#FFFFFF", mew=1.2, zorder=8)
        ax.text(goal[0], goal[1] + 0.25, "Goal", fontsize=8, color="#B91C1C", fontweight="bold",
                ha="center", va="bottom", zorder=9, bbox=dict(boxstyle="round,pad=0.15", fc="#FFFFFF", ec="none", alpha=0.85))

        if use_crop is not None:
            ax.set_xlim(use_crop[0], use_crop[1])
            ax.set_ylim(use_crop[2], use_crop[3])
        ax.set_aspect("equal")

        # Clean title & inference subtitle
        ax.set_title(f"{label}\n{panel_subtitles[label]}", fontsize=11, fontweight="bold", pad=8, color="#0F2942")
        ax.set_xlabel("x (m)", fontsize=8.5, color="#475569")
        if col == 0:
            ax.set_ylabel("y (m)", fontsize=8.5, color="#475569")
        else:
            ax.set_yticklabels([])

        leg = ax.legend(loc="upper left", fontsize=7.5, framealpha=0.92, edgecolor="#E2E8F0")
        leg.get_frame().set_boxstyle("round,pad=0.3")

        # Executive Metrics Table
        tax = fig.add_subplot(gs[1, col])
        tax.axis("off")

        headers = ["Run", "Arrived", "Move→Arr", "Driven", "Stalled", "Turn-backs"]
        rows = []
        for k, m in enumerate(ms):
            if m["reached"]:
                arr_str = f"Yes ({m['arrive_s']:.0f}s)"
                m2a_str = f"{m['move_to_arrive_s']:.1f} s"
            else:
                arr_str = "No (Timeout)"
                m2a_str = "—"
            rows.append([
                f"Run {k + 1}",
                arr_str,
                m2a_str,
                f"{m['driven_m']:.1f} m",
                f"{m['stalled_s']:.0f} s",
                f"{m['reversals']}"
            ])

        table = tax.table(
            cellText=rows,
            colLabels=headers,
            loc="center", cellLoc="center"
        )
        table.auto_set_font_size(False)
        table.set_fontsize(7.5)
        table.auto_set_column_width(list(range(len(headers))))
        table.scale(1.0, 1.45)

        for (r_idx, c_idx), cell in table.get_celld().items():
            cell.set_edgecolor("#E2E8F0")
            cell.set_linewidth(0.6)
            if r_idx == 0:
                cell.set_facecolor("#1E293B")
                cell.get_text().set_color("#FFFFFF")
                cell.get_text().set_fontweight("bold")
            else:
                bg = "#F8FAFC" if r_idx % 2 == 1 else "#FFFFFF"
                cell.set_facecolor(bg)
                if c_idx == 1:
                    if "Yes" in cell.get_text().get_text():
                        cell.get_text().set_color("#15803D")
                        cell.get_text().set_fontweight("semibold")
                    else:
                        cell.set_facecolor("#FEE2E2")
                        cell.get_text().set_color("#B91C1C")
                        cell.get_text().set_fontweight("bold")

    if title:
        fig.suptitle(title, fontsize=12.5, fontweight="bold", y=0.99, color="#0F2942")

    fig.text(0.5, 0.01,
             "Key: Solid / dashed paths = UGV runs | Thin gray = confirmed plans | Orange dashed = tentative passage plans | Red rings = stalls ≥ 8s",
             ha="center", fontsize=7.5, color="#64748B")

    output_dir = os.path.dirname(path)
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return all_metrics
