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
    """Final map as a quiet background: dark = obstacle, white = free, grey = never seen."""
    from matplotlib.colors import ListedColormap

    if snap is None:
        return
    masks = build_masks(snap)
    canvas = np.zeros(masks.walkable.shape, dtype=int)  # 0 unobserved
    canvas[~masks.unobserved] = 1  # observed, not walkable -> obstacle
    canvas[masks.walkable] = 2
    ax.imshow(
        canvas, origin="lower", extent=_extent(snap, canvas.shape),
        cmap=ListedColormap(["#e4e4e4", "#4a4a4a", "#ffffff"]), vmin=0, vmax=2, interpolation="nearest", zorder=0,
    )
    if crop is None:
        rows, cols = np.nonzero(~masks.unobserved)
        if rows.size:
            x0, x1, y0, y1 = _extent(snap, canvas.shape)
            res = snap.resolution
            crop = (
                x0 + cols.min() * res - 0.5, x0 + (cols.max() + 1) * res + 0.5,
                y0 + rows.min() * res - 0.5, y0 + (rows.max() + 1) * res + 0.5,
            )
    return crop


def _draw_plans(ax, plans: list[dict], numbered: bool, alpha: float = 1.0, lw: float = 1.4, z: float = 3):
    for i, u in enumerate(plans, start=1):
        pts = np.asarray(u["waypoints"])
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
                colour, style = PLAN_FREE, "-"
            ax.plot(pts[k : k + 2, 0], pts[k : k + 2, 1], linestyle=style, color=colour, lw=lw, alpha=alpha, zorder=z)
        ax.plot(pts[:, 0], pts[:, 1], "o", color=PLAN_FREE, ms=3, alpha=alpha, zorder=z)
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
    fig, ax = plt.subplots(figsize=(11, 8))
    crop = _draw_map(ax, snap, crop)

    plans = unique_plans(trace["plans"])
    _draw_plans(ax, plans, numbered=True, lw=1.8, z=6)

    if len(poses) > 1:
        pts = poses[:, 1:3].reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap="plasma", linewidths=6.0, zorder=5, alpha=0.85)
        t_move = m["first_move_s"] if m["first_move_s"] is not None else poses[0, 0]
        lc.set_array(np.clip(poses[:-1, 0] - t_move, 0.0, None))  # waiting at the start = darkest
        ax.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
        cb.set_label("seconds since the UGV first moved")
        # direction arrows every ~1.5 m of travel
        acc, last = 0.0, poses[0, 1:3]
        for p in poses[1:, 1:3]:
            acc += float(np.hypot(*(p - last)))
            if acc >= 1.5:
                d = p - last
                if np.hypot(*d) > 1e-6:
                    ax.annotate("", xy=p, xytext=p - d / np.hypot(*d) * 0.25, zorder=6,
                                arrowprops=dict(arrowstyle="-|>", color="black", lw=1.2))
                acc = 0.0
            last = p

    for x, y, t0, dur, count in merge_dwells(dwell_points(poses)):
        at_start = m["first_move_s"] is not None and t0 <= m["first_move_s"] + 0.5
        ax.add_patch(plt.Circle((x, y), 0.28 + 0.04 * min(count, 6), fill=False, ec="#C00000", lw=1.6, zorder=6))
        what = "waited at start" if at_start else ("paused" if count == 1 else f"paused {count}x, total")
        ax.annotate(f"{what} {dur:.0f} s", (x, y), xytext=(10, -16), textcoords="offset points", fontsize=8,
                    color="#C00000", zorder=8, bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.75))
    for v in trace.get("verdicts", []):
        i = int(np.argmin(np.abs(poses[:, 0] - v["t"]))) if len(poses) else None
        if i is not None:
            ax.plot(*poses[i, 1:3], marker="P", color="#00A000", ms=11, mec="white", zorder=8)
            ax.annotate(v["text"][:18], poses[i, 1:3], xytext=(6, 8), textcoords="offset points", fontsize=7,
                        color="#006400", zorder=8)

    s = start if start is not None else (poses[0, 1:3] if len(poses) else None)
    if s is not None:
        ax.plot(*s, "s", color="#238b45", ms=12, mec="white", zorder=9, label="start")
    ax.plot(*goal, "*", color="#cb181d", ms=18, mec="white", zorder=9, label="goal")
    if crop is not None:
        ax.set_xlim(crop[0], crop[1])
        ax.set_ylim(crop[2], crop[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    from matplotlib.lines import Line2D
    handles = [
        Line2D([], [], color=PLAN_FREE, lw=1.6, label="plan: confirmed route"),
        Line2D([], [], color=PLAN_TENTATIVE, lw=1.6, ls="--", label="plan: tentative stretch"),
        Line2D([], [], color=PLAN_EXPLORE, lw=1.6, ls="--", label="plan: exploration detour"),
        Line2D([], [], color="#C00000", marker="o", mfc="none", ls="", label="paused ≥ 3 s"),
        Line2D([], [], color="#00A000", marker="P", ls="", label="3D clearance check"),
        Line2D([], [], color="#238b45", marker="s", ls="", label="start"),
        Line2D([], [], color="#cb181d", marker="*", ls="", ms=11, label="goal"),
    ]
    ax.legend(handles=handles, loc="upper left", fontsize=7, framealpha=0.9)
    ax.set_title((title + "\n" if title else "") + _metrics_line(m), fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return m


def render_comparison_png(path, groups: dict[str, list[dict]], snap: MapSnapshot | None, title: str = "", crop=None) -> dict:
    """`groups` maps a planner label to its runs' traces. One panel per planner,
    all runs overlaid (a colour per run), plus a table of the per-run numbers."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(groups)
    fig = plt.figure(figsize=(7.0 * n, 7.4))
    gs = fig.add_gridspec(2, n, height_ratios=[4.0, 1.0], hspace=0.04, wspace=0.06)
    all_metrics: dict[str, list[dict]] = {}
    use_crop = crop
    for col, (label, traces) in enumerate(groups.items()):
        ax = fig.add_subplot(gs[0, col])
        use_crop = _draw_map(ax, snap, crop) or use_crop
        ms = []
        # every distinct plan from every run, drawn faintly and without numbers
        merged = unique_plans([p for t in traces for p in t["plans"]], tol=0.25)
        _draw_plans(ax, merged, numbered=False, alpha=0.9, lw=1.1, z=6)
        for k, tr in enumerate(traces):
            c = RUN_COLOURS[k % len(RUN_COLOURS)]
            poses = tr["poses"]
            m = track_metrics(tr)
            ms.append(m)
            ax.plot(poses[:, 1], poses[:, 2], "-", color=c, lw=5.0, alpha=0.55, zorder=5, label=f"run {k + 1}", solid_capstyle="round")
            for x, y, _t0, _d, cnt in merge_dwells(dwell_points(poses)):
                ax.add_patch(plt.Circle((x, y), 0.25 + 0.04 * min(cnt, 6), fill=False, ec=c, lw=1.6, zorder=6))
        goal = traces[0]["meta"]["goal"]
        start = traces[0]["poses"][0, 1:3]
        ax.plot(*start, "s", color="#238b45", ms=12, mec="white", zorder=9)
        ax.plot(*goal, "*", color="#cb181d", ms=18, mec="white", zorder=9)
        if use_crop is not None:
            ax.set_xlim(use_crop[0], use_crop[1])
            ax.set_ylim(use_crop[2], use_crop[3])
        ax.set_aspect("equal")
        ax.set_title(label, fontsize=13, fontweight="bold")
        ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
        if col:
            ax.set_yticklabels([])
        all_metrics[label] = ms

        tax = fig.add_subplot(gs[1, col])
        tax.axis("off")
        rows = []
        for k, m in enumerate(ms):
            if m["reached"]:
                rows.append([f"run {k + 1}", f"{m['first_move_s']:.0f}", f"{m['arrive_s']:.0f}", f"{m['move_to_arrive_s']:.0f}",
                             f"{m['driven_m']:.1f}", f"{m['stalled_s']:.0f}", f"{m['plans_distinct']}", f"{m['no_path']}", f"{m['reversals']}"])
            else:
                fm = f"{m['first_move_s']:.0f}" if m["first_move_s"] is not None else "-"
                rows.append([f"run {k + 1}", fm, "no", "-", f"{m['driven_m']:.1f}", f"{m['stalled_s']:.0f}",
                             f"{m['plans_distinct']}", f"{m['no_path']}", f"{m['reversals']}"])
        table = tax.table(
            cellText=rows,
            colLabels=["", "first move\n(s)", "arrived\n(s)", "move→\narrive (s)", "driven\n(m)", "stalled\n(s)", "distinct\nplans", "'no path'\nreplies", "turn-\nbacks"],
            loc="center", cellLoc="center",
        )
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.auto_set_column_width(list(range(9)))
        table.scale(1, 1.7)
    if title:
        fig.suptitle(title, fontsize=14, y=0.995)
    fig.text(0.5, 0.005, "thin grey = confirmed plan, orange dashed = tentative stretch, purple dashed = exploration detour; "
             "rings = pauses ≥ 3 s; square = start, star = goal. Simulation, few runs: shows behaviour, not statistics.",
             ha="center", fontsize=8, color="#444444")
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return all_metrics
