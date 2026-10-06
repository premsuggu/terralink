"""Offline replay of the planning step - no Gazebo, no UAV flight, no ROS.

WHY THIS EXISTS
---------------
Testing a planner change by launching the whole simulation means waiting for
the UAV to fly around and scan the room every single time (minutes), even
though the thing being tested - path planning - only needs the finished map.
This module lets you capture that finished map ONCE from a live run
(`scripts/save_map_snapshot.py`) and then re-run planning on it as often as
you like in a fraction of a second (`scripts/replay_plan.py`), looking at the
result as a picture.

A snapshot is just the layers `/elevation_map` publishes (elevation,
traversability, is_valid, ...) plus the map's resolution and centre, stored in
one `.npz` file. Everything downstream of that - walkable mask, frontier,
anomaly, resolved regions, inflation, A*, route memory, exploration - is the
real production code, not a copy.

This file is pure NumPy (+ matplotlib only inside `render_png`).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from nav import astar_planner
from nav.anomaly import compute_anomaly_mask
from nav.global_planner import GlobalPlanner, MapContext
from nav.map_growth import MapGrowthTracker
from nav.mask_latch import MaskLatch
from nav.prm_planner import plan as prm_plan
from nav.resolved_regions import ResolvedRegion, ResolvedRegionStore
from nav.walkability import compute_frontier_mask, compute_walkable_mask

# Layers every snapshot must contain (the ones the planner actually uses).
_REQUIRED = ("traversability", "is_valid")


@dataclass
class MapSnapshot:
    """A saved `/elevation_map`: named layers + grid geometry."""

    layers: dict[str, np.ndarray]
    resolution: float
    center_x: float
    center_y: float

    @property
    def shape(self) -> tuple[int, int]:
        return self.layers["traversability"].shape


def save_snapshot(path: str | Path, layers: dict[str, np.ndarray], resolution: float, center_x: float, center_y: float) -> None:
    """Write a snapshot. `layers` is what `emap.utils.gridmap_utils.decode_gridmap`
    returns for a received `/elevation_map` message."""
    missing = [k for k in _REQUIRED if k not in layers]
    if missing:
        raise ValueError(f"snapshot is missing required layers: {missing}")
    np.savez_compressed(
        path,
        resolution=np.float64(resolution),
        center_x=np.float64(center_x),
        center_y=np.float64(center_y),
        **{f"layer_{name}": np.asarray(arr) for name, arr in layers.items()},
    )


def load_snapshot(path: str | Path) -> MapSnapshot:
    data = np.load(path)
    layers = {k[len("layer_") :]: data[k] for k in data.files if k.startswith("layer_")}
    missing = [k for k in _REQUIRED if k not in layers]
    if missing:
        raise ValueError(f"{path}: not a valid map snapshot (missing layers {missing})")
    return MapSnapshot(
        layers=layers,
        resolution=float(data["resolution"]),
        center_x=float(data["center_x"]),
        center_y=float(data["center_y"]),
    )


@dataclass
class Masks:
    """Everything the planner is given, derived from a snapshot the same way
    `planner_node._map_callback` derives it."""

    walkable: np.ndarray
    traversability: np.ndarray
    unobserved: np.ndarray
    frontier: np.ndarray | None
    anomaly: np.ndarray | None
    raw_anomaly: np.ndarray | None
    force_free: np.ndarray | None


def build_masks(
    snap: MapSnapshot,
    *,
    use_frontier: bool = False,
    use_anomaly: bool = False,
    resolved: list[ResolvedRegion] | None = None,
    latch: MaskLatch | None = None,
    now: float = 0.0,
    frontier_min_width_m: float = 0.0,
) -> Masks:
    """Derive the planner's masks from a snapshot, as `planner_node` does.

    `latch`/`now`: optionally hold flickering anomaly cells (see
    `nav.mask_latch`) - pass the SAME latch object across successive
    snapshots to reproduce what the node does over time.
    """
    layers = snap.layers
    walkable = compute_walkable_mask(layers["traversability"], layers["is_valid"])
    frontier = (
        compute_frontier_mask(
            walkable, layers["is_valid"], min_unobserved_width_m=frontier_min_width_m, resolution=snap.resolution
        )
        if use_frontier
        else None
    )
    anomaly = raw_anomaly = force_free = None
    if use_anomaly:
        if "elevation" not in layers:
            raise ValueError("anomaly detection needs an 'elevation' layer in the snapshot")
        raw_anomaly = compute_anomaly_mask(layers["elevation"], walkable, layers["is_valid"], snap.resolution)
        held = latch.update(raw_anomaly, now) if latch is not None else raw_anomaly
        store = ResolvedRegionStore()
        for region in resolved or []:
            store.add(region)
        walkable, anomaly = store.apply(walkable, held, snap.resolution, snap.center_x, snap.center_y)
        force_free = store.passable_mask(walkable.shape, snap.resolution, snap.center_x, snap.center_y)
    return Masks(
        walkable=walkable,
        traversability=layers["traversability"],
        unobserved=~np.asarray(layers["is_valid"]).astype(bool),
        frontier=frontier,
        anomaly=anomaly,
        raw_anomaly=raw_anomaly,
        force_free=force_free,
    )


def run_plan(
    snap: MapSnapshot,
    masks: Masks,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *,
    planner: str = "astar",
    explore: bool = False,
    params: astar_planner.PlannerParams | None = None,
):
    """Plan once on a snapshot, exactly as `planner_node` would."""
    if planner == "prm":
        return prm_plan(
            masks.walkable, snap.resolution, snap.center_x, snap.center_y, start_xy, goal_xy,
            rng=np.random.default_rng(20260924),
            frontier_mask=masks.frontier, allow_frontier=masks.frontier is not None,
            anomaly_mask=masks.anomaly, allow_anomaly=masks.anomaly is not None,
        )  # fmt: skip
    return GlobalPlanner(params, enable_memory=False, enable_exploration=explore).plan(
        _context(snap, masks), start_xy, goal_xy
    )


def _context(snap: MapSnapshot, masks: Masks, mapping_active: bool = False) -> MapContext:
    """The planner's input, built from a snapshot's masks."""
    return MapContext(
        walkable=masks.walkable, resolution=snap.resolution, center_x=snap.center_x, center_y=snap.center_y,
        traversability=masks.traversability, frontier=masks.frontier, anomaly=masks.anomaly,
        unobserved=masks.unobserved, force_free=masks.force_free,
        allow_frontier=masks.frontier is not None, allow_anomaly=masks.anomaly is not None,
        mapping_active=mapping_active,
    )  # fmt: skip


@dataclass
class PlanMetrics:
    """Numbers that describe how GOOD a route is, for comparing planners."""

    length_m: float
    straight_line_m: float
    detour_ratio: float
    n_waypoints: int
    total_turn_deg: float
    sharpest_turn_deg: float
    min_clearance_m: float
    kinds: list[str] = field(default_factory=list)


def path_metrics(waypoints, snap: MapSnapshot, masks: Masks, kinds=None) -> PlanMetrics:
    """Length, detour, turning and clearance of a waypoint list."""
    import math

    pts = np.asarray(waypoints, dtype=float)
    segs = np.diff(pts, axis=0)
    seg_len = np.hypot(segs[:, 0], segs[:, 1])
    length = float(seg_len.sum())
    straight = float(np.hypot(*(pts[-1] - pts[0])))
    headings = np.arctan2(segs[:, 1], segs[:, 0])
    turns = np.degrees(np.abs((np.diff(headings) + np.pi) % (2 * np.pi) - np.pi)) if len(headings) > 1 else np.zeros(1)

    clear = astar_planner.clearance_m(~masks.walkable, snap.resolution)
    n = masks.walkable.shape[0]
    worst = math.inf
    for p, q in zip(pts[:-1], pts[1:]):
        for t in np.linspace(0, 1, 40):
            x, y = p + (q - p) * t
            r = int(round((y - snap.center_y) / snap.resolution + n / 2))
            c = int(round((x - snap.center_x) / snap.resolution + n / 2))
            if 0 <= r < n and 0 <= c < n:
                worst = min(worst, float(clear[r, c]))
    return PlanMetrics(
        length_m=length,
        straight_line_m=straight,
        detour_ratio=length / straight if straight > 0 else 1.0,
        n_waypoints=len(pts),
        total_turn_deg=float(turns.sum()),
        sharpest_turn_deg=float(turns.max()),
        min_clearance_m=worst,
        kinds=list(kinds or []),
    )


@dataclass
class SequenceStep:
    """What the planner would have answered at one point of a recorded run."""

    index: int
    t: float
    start_xy: tuple[float, float]
    valid: bool
    tentative: bool
    n_waypoints: int
    waypoints: list[tuple[float, float]]
    route_changed: bool  # differs from the previous valid answer
    note: str = ""


def _routes_differ(a, b, tol_m: float = 0.3) -> bool:
    """Same idea as the follower's own test, with a looser tolerance: a
    different waypoint count, or any waypoint more than `tol_m` apart."""
    if a is None or b is None or len(a) != len(b):
        return True
    return any(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5 > tol_m for (ax, ay), (bx, by) in zip(a, b))


def replay_sequence(
    snapshots: list[MapSnapshot],
    times: list[float],
    starts: list[tuple[float, float]],
    goal_xy: tuple[float, float],
    *,
    planner: str = "astar",
    use_frontier: bool = True,
    use_anomaly: bool = True,
    hold_sec: float = 0.0,
    memory: bool = True,
    explore: bool = False,
    wait_for_mapping: bool = False,
    mapping_max_wait_sec: float = 150.0,
    stuck_feedback: bool = False,
    frontier_min_width_m: float = 0.0,
    params: astar_planner.PlannerParams | None = None,
) -> list[SequenceStep]:
    """Feed a recorded series of maps (and where the robot was at each) through
    the planner in order - what `planner_node` would have answered over a run,
    with route memory and the anomaly hold carried across steps.

    The count of `route_changed` steps is the number to compare between
    planners: every change is a moment the UGV may turn around.
    """
    latch = MaskLatch(hold_sec)
    planner_obj = (
        GlobalPlanner(
            params, enable_memory=memory, enable_exploration=explore,
            wait_for_mapping=wait_for_mapping, stuck_feedback=stuck_feedback,
        )  # fmt: skip
        if planner == "astar"
        else None
    )
    growth = MapGrowthTracker()
    t_first = times[0] if times else 0.0
    out: list[SequenceStep] = []
    prev_route = None
    for i, (snap, t, start) in enumerate(zip(snapshots, times, starts)):
        masks = build_masks(
            snap, use_frontier=use_frontier, use_anomaly=use_anomaly, latch=latch, now=t,
            frontier_min_width_m=frontier_min_width_m,
        )  # fmt: skip
        growth.update(int(np.asarray(snap.layers["is_valid"]).astype(bool).sum()), t)
        mapping_active = growth.growing(t) and (t - t_first) < mapping_max_wait_sec
        if planner_obj is not None:
            result = planner_obj.plan(_context(snap, masks, mapping_active), start, goal_xy, now=t)
            note = planner_obj.last_note
        else:
            result = run_plan(snap, masks, start, goal_xy, planner=planner)
            note = ""
        valid = bool(result.valid)
        route = list(result.waypoints) if valid else None
        changed = valid and _routes_differ(prev_route, route)
        if valid:
            prev_route = route
        out.append(
            SequenceStep(
                index=i, t=t, start_xy=start, valid=valid,
                tentative=valid and (result.has_frontier_segments or result.has_anomaly_segments),
                n_waypoints=len(route) if route else 0, waypoints=route or [], route_changed=changed, note=note,
            )
        )  # fmt: skip
    return out


def render_png(
    path: str | Path,
    snap: MapSnapshot,
    masks: Masks,
    start_xy,
    goal_xy,
    result,
    title: str = "",
    crop: tuple[float, float, float, float] | None = None,
) -> None:
    """Draw the map, the uncertain cells and the planned route to a PNG.

    `crop` = (x_min, x_max, y_min, y_max) in meters; by default the view is
    cropped to the observed part of the map (plus a 1 m margin) so a small
    scanned area on a big grid is not drawn as a speck.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    n_rows, n_cols = masks.walkable.shape
    res = snap.resolution
    x0 = (0 - n_cols / 2.0) * res + snap.center_x - res / 2
    x1 = (n_cols - n_cols / 2.0) * res + snap.center_x - res / 2
    y0 = (0 - n_rows / 2.0) * res + snap.center_y - res / 2
    y1 = (n_rows - n_rows / 2.0) * res + snap.center_y - res / 2

    # 0 unobserved / 1 obstacle / 2 free / 3 frontier / 4 anomaly
    canvas = np.zeros(masks.walkable.shape, dtype=int)
    canvas[~masks.unobserved] = 1
    canvas[masks.walkable] = 2
    if masks.frontier is not None:
        canvas[masks.frontier] = 3
    if masks.anomaly is not None:
        canvas[masks.anomaly] = 4
    cmap = ListedColormap(["#d9d9d9", "#3b3b3b", "#ffffff", "#9ecae1", "#fdae6b"])

    fig, ax = plt.subplots(figsize=(9, 9 * n_rows / n_cols))
    ax.imshow(canvas, origin="lower", extent=(x0, x1, y0, y1), cmap=cmap, vmin=0, vmax=4, interpolation="nearest")
    if result is not None and result.valid:
        pts = np.asarray(result.waypoints)
        kinds = list(getattr(result, "segment_kinds", [])) or ["free"] * (len(pts) - 1)
        colours = {"free": "#08519c", "frontier": "#2171b5", "anomaly": "#d94801"}
        for (p, q), kind in zip(zip(pts[:-1], pts[1:]), kinds):
            ax.plot([p[0], q[0]], [p[1], q[1]], "-", color=colours.get(kind, "#08519c"), lw=2.5)
        ax.plot(pts[:, 0], pts[:, 1], "o", color="#08306b", ms=5)
    ax.plot(*start_xy, "s", color="#238b45", ms=11, label="start")
    ax.plot(*goal_xy, "*", color="#cb181d", ms=16, label="goal")
    if crop is None:
        rows, cols = np.nonzero(~masks.unobserved)
        if rows.size:
            cx0 = (cols.min() - n_cols / 2.0) * res + snap.center_x - 1.0
            cx1 = (cols.max() - n_cols / 2.0) * res + snap.center_x + 1.0
            cy0 = (rows.min() - n_rows / 2.0) * res + snap.center_y - 1.0
            cy1 = (rows.max() - n_rows / 2.0) * res + snap.center_y + 1.0
            crop = (cx0, cx1, cy0, cy1)
    if crop is not None:
        ax.set_xlim(crop[0], crop[1])
        ax.set_ylim(crop[2], crop[3])
    ax.set_title(title or ("no path" if result is None or not result.valid else "planned route"))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
