"""Deterministic grid A* path planner - the replacement for `prm_planner`.

WHY THIS EXISTS
---------------
`prm_planner` samples random cells and links them into a roadmap. That has
three problems for this project (see docs/work-docs/nav/step07_astar_planner.md
for the full discussion):

1. It is random, so two plans for nearly the same map can pick different
   routes - one source of the UGV's "useless, random-looking motion".
2. It can miss narrow passages entirely (a tunnel with no sample in it).
3. It never inflates obstacles: its line-of-sight check is zero-width, so
   robot clearance was left entirely to Nav2's costmap.

This module plans on the grid directly, so it is complete up to the grid's
resolution (if a path exists on the grid, it is found), deterministic (same
input -> same output), and aware of the robot's size.

THE PIPELINE (one call to `plan()`)
-----------------------------------
1. INFLATE: every hard-blocked cell is grown outward by the robot radius (+ a
   small margin) so a path planned for a single point can never graze a wall.
2. COST GRID: free cells cost 1; cells near the inflation boundary, "difficult"
   terrain, and uncertain (anomaly/frontier) cells cost more.
3. SEARCH, in passes, so a confirmed route is ALWAYS preferred over a guess:
       strict (free cells only)        -> found? done
       optimistic (+ anomaly/frontier) -> found? it is a TENTATIVE route
   each pass tried first at the full inflation, then at the bare robot radius
   (a relaxed fallback for very narrow gaps).
4. STRAIGHTEN: line-of-sight shortening removes the grid "staircase".
   Shortening is done per STRETCH (a run of free cells, or a run of uncertain
   cells), never across a boundary, so the points where the path ENTERS and
   LEAVES an uncertain region survive as waypoints.
5. STANDOFF: a waypoint is pinned about `standoff_m` before each uncertain
   stretch - the spot where the UGV should stop and look (build its 3D voxel
   map) before committing to the uncertain part.

Grid conventions are the same as the rest of `nav`/`emap`: arrays are
(row, col) with row <-> world Y and col <-> world X; conversion goes through
`emap.utils.coord_transform` so it matches how the map itself was built.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field, replace

import numpy as np
from scipy.ndimage import distance_transform_edt

from emap.traversability import DIFFICULT
from emap.utils.coord_transform import grid_to_world, world_to_grid
from nav.prm_planner import PlanResult

_SQRT2 = math.sqrt(2.0)

# (d_row, d_col, step_length_in_cells) for the 8-connected neighbourhood.
# The ORDER matters: it is part of what makes tie-breaking deterministic.
_MOVES = (
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, 0, 1.0),
    (-1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (1, 1, _SQRT2),
)

# Path-cell classes, used while post-processing a found path.
_FREE, _FRONTIER, _ANOMALY = 0, 1, 2

# Tolerance when comparing a cell's clearance (meters) to the inflation
# radius. Cell-center distances in a 0.1 m grid are often EXACT multiples of
# the radius we compare against (e.g. 0.3 m), so without a tolerance a
# floating-point wobble decides whether a corridor is open or closed.
_CLEARANCE_EPS = 1e-6


@dataclass
class AStarResult(PlanResult):
    """`PlanResult` (so every existing consumer keeps working) plus a few
    A*-specific extras.

    `segment_kinds[i]` describes the straight segment from `waypoints[i]` to
    `waypoints[i+1]`: "free", "frontier" or "anomaly". A downstream consumer
    uses it to know exactly WHICH segment is the uncertain one, rather than
    only that "some segment somewhere" is.

    `cost`: total search cost of the (pre-shortening) cell path under the cost
    grid used - comparable between two plans made with the same parameters,
    which is what route-stability logic needs.

    `inflation_m`: the inflation radius actually used (the full one, or the
    relaxed bare-robot-radius fallback).

    `goal_xy`: the goal actually planned to - differs from the requested goal
    only when it had to be snapped to the nearest passable cell.

    `mode`: "strict" or "optimistic" - which pass found this path (so a later
    re-evaluation can use the identical grids).

    `is_exploration`: True when this is NOT a route to the requested goal but
    to a frontier viewpoint chosen to reveal more of the map (see
    `nav.exploration`). The flags above are then set so a caller treats the
    plan as tentative and keeps re-planning.
    """

    segment_kinds: list[str] = field(default_factory=list)
    cost: float = math.inf
    inflation_m: float = 0.0
    goal_xy: tuple[float, float] | None = None
    mode: str = ""
    is_exploration: bool = False
    exploration_kind: str = ""  # "anomaly" (look at a suspected passage) or "frontier"


# ---------------------------------------------------------------------------
# Grid preparation: inflation and cost
# ---------------------------------------------------------------------------


def clearance_m(blocked: np.ndarray, resolution: float) -> np.ndarray:
    """Distance (meters) from each cell's center to the nearest `blocked`
    cell's center. Blocked cells themselves get 0.

    If NOTHING is blocked, every cell is infinitely far from an obstacle.
    (scipy's EDT has no sensible answer for an all-"foreground" input, so that
    case is handled explicitly instead of trusting whatever it returns.)
    """
    if not blocked.any():
        return np.full(blocked.shape, np.inf, dtype=np.float64)
    return distance_transform_edt(~blocked) * resolution


def build_cost_grid(
    clearance: np.ndarray,
    inflation_m: float,
    traversability: np.ndarray | None,
    anomaly: np.ndarray | None,
    frontier: np.ndarray | None,
    clearance_weight: float,
    clearance_range_m: float,
    difficult_weight: float,
    anomaly_penalty: float,
    frontier_penalty: float,
) -> np.ndarray:
    """Per-cell cost of ENTERING a cell. Always >= 1, which is what keeps the
    straight-line heuristic in `_astar` admissible (it never over-estimates).

    cost = (1
            + clearance_weight * how-close-to-the-inflation-boundary   [0..1]
            + difficult_weight * (cell is DIFFICULT terrain))
           * (anomaly_penalty if anomaly) * (frontier_penalty if frontier)

    The clearance term is a soft preference to keep away from walls (so paths
    drift toward the middle of open space/corridors) on top of the hard
    inflation rule.
    """
    # 0 right at the inflation boundary -> 1 at `clearance_range_m` beyond it.
    headroom = np.clip((clearance - inflation_m) / max(clearance_range_m, 1e-9), 0.0, 1.0)
    # Infinite clearance (no obstacles at all) -> headroom 1 -> no penalty.
    headroom = np.where(np.isfinite(clearance), headroom, 1.0)
    cost = 1.0 + clearance_weight * (1.0 - headroom)
    if traversability is not None:
        cost = cost + difficult_weight * (np.isclose(traversability, DIFFICULT))
    if anomaly is not None:
        cost = np.where(anomaly, cost * anomaly_penalty, cost)
    if frontier is not None:
        cost = np.where(frontier, cost * frontier_penalty, cost)
    return cost


# ---------------------------------------------------------------------------
# Line of sight (supercover)
# ---------------------------------------------------------------------------


def line_is_clear(passable: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> bool:
    """True if EVERY cell the segment (r0,c0)->(r1,c1) touches is passable.

    This is a CONSERVATIVE "supercover" test: a cell counts as touched if the
    segment passes through its closed square - including just its corner.
    That matters at 0.1 m resolution: the plain Bresenham line `prm_planner`
    uses can slip diagonally between two blocked cells that only touch at a
    corner (a real collision for a robot). Here, squeezing through such a
    crack is rejected.

    Cell centers sit at integer coordinates, so a cell i spans [i-0.5, i+0.5].
    The cells touched change only where the line crosses a half-integer
    coordinate; we evaluate the line at every such crossing (plus both
    endpoints) and collect all cells whose closed square contains that point
    (two cells when the point sits exactly on a boundary, which is how the
    corner case is caught).
    """
    dr, dc = r1 - r0, c1 - c0
    ts = [0.0, 1.0]
    for start, delta in ((r0, dr), (c0, dc)):
        if delta == 0:
            continue
        lo, hi = min(start, start + delta), max(start, start + delta)
        # half-integer boundaries strictly between the endpoints
        first = math.ceil(lo - 0.5)
        last = math.ceil(hi - 0.5) - 1
        if last >= first:
            bounds = np.arange(first, last + 1) + 0.5
            ts.extend(((bounds - start) / delta).tolist())
    t = np.asarray(ts)
    rr = r0 + dr * t
    cc = c0 + dc * t
    eps = 1e-9
    # Cells whose closed square [i-0.5, i+0.5] contains the coordinate.
    r_lo = np.ceil(rr - 0.5 - eps).astype(np.int64)
    r_hi = np.floor(rr + 0.5 + eps).astype(np.int64)
    c_lo = np.ceil(cc - 0.5 - eps).astype(np.int64)
    c_hi = np.floor(cc + 0.5 + eps).astype(np.int64)
    n_rows, n_cols = passable.shape
    for r_idx, c_idx in ((r_lo, c_lo), (r_lo, c_hi), (r_hi, c_lo), (r_hi, c_hi)):
        if r_idx.min() < 0 or c_idx.min() < 0 or r_idx.max() >= n_rows or c_idx.max() >= n_cols:
            return False
        if not passable[r_idx, c_idx].all():
            return False
    return True


# ---------------------------------------------------------------------------
# The search itself
# ---------------------------------------------------------------------------


def _astar(
    passable: np.ndarray,
    cost: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> tuple[list[tuple[int, int]], float] | None:
    """A* over an 8-connected grid. Returns (cells start->goal, total cost),
    or None if the goal cannot be reached.

    HOW IT WORKS (the short version - see the work doc for the long one):
    every cell gets g = cheapest known cost from the start, h = a guess of the
    cost still to go, and f = g + h. We repeatedly expand the open cell with
    the lowest f. Because h NEVER over-estimates (it is the straight-line
    octile distance and every step costs at least its own length), the first
    time the goal is taken off the queue the path is the cheapest possible.
    If the queue empties first, no path exists on this grid.

    DETERMINISM: ties on f are broken by smaller h (prefer the cell closer to
    the goal), then by insertion order. Neighbours are always visited in the
    fixed order of `_MOVES`. No randomness anywhere, so the same input always
    yields the same path.

    SPEED: the grid is padded with a blocked border so neighbour lookups never
    need bounds checks, and everything is flattened to plain Python lists
    (list indexing beats NumPy scalar indexing inside a hot loop).

    DIAGONAL RULE: a diagonal step is only allowed if BOTH orthogonal cells it
    passes between are passable - no cutting corners through wall corners.
    """
    n_rows, n_cols = passable.shape
    width = n_cols + 2
    pad_pass = np.zeros((n_rows + 2, width), dtype=bool)
    pad_pass[1:-1, 1:-1] = passable
    pad_cost = np.ones((n_rows + 2, width), dtype=np.float64)
    pad_cost[1:-1, 1:-1] = cost

    ok = pad_pass.ravel().tolist()
    step_cost = pad_cost.ravel().tolist()

    # Octile distance to the goal for every cell, precomputed vectorised.
    gr, gc = goal[0] + 1, goal[1] + 1
    rr, cc = np.indices((n_rows + 2, width))
    dy, dx = np.abs(rr - gr), np.abs(cc - gc)
    h_grid = (dx + dy) + (_SQRT2 - 2.0) * np.minimum(dx, dy)
    heur = h_grid.ravel().tolist()

    start_i = (start[0] + 1) * width + (start[1] + 1)
    goal_i = gr * width + gc
    # (d_flat_index, step_len, d_row_flat, d_col_flat) - the last two are the
    # two orthogonal cells a diagonal step squeezes between.
    moves = [(dr * width + dc, ln, dr * width, dc) for dr, dc, ln in _MOVES]

    n = len(ok)
    g = [math.inf] * n
    parent = [-1] * n
    closed = [False] * n
    g[start_i] = 0.0
    counter = 0
    heap = [(heur[start_i], heur[start_i], counter, start_i)]

    while heap:
        _, _, _, cur = heapq.heappop(heap)
        if closed[cur]:
            continue  # a stale, superseded queue entry
        closed[cur] = True
        if cur == goal_i:
            break
        g_cur = g[cur]
        for d_flat, length, d_r, d_c in moves:
            nxt = cur + d_flat
            if closed[nxt] or not ok[nxt]:
                continue
            if d_r and d_c and not (ok[cur + d_r] and ok[cur + d_c]):
                continue  # diagonal would cut a corner
            g_new = g_cur + length * step_cost[nxt]
            if g_new < g[nxt]:
                g[nxt] = g_new
                parent[nxt] = cur
                counter += 1
                h = heur[nxt]
                heapq.heappush(heap, (g_new + h, h, counter, nxt))

    if not closed[goal_i]:
        return None

    cells: list[tuple[int, int]] = []
    node = goal_i
    while node != -1:
        cells.append((node // width - 1, node % width - 1))
        node = parent[node]
    cells.reverse()
    return cells, g[goal_i]


def dijkstra_all(
    passable: np.ndarray, cost: np.ndarray, start: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray]:
    """Cheapest cost from `start` to EVERY reachable cell, plus a parent map.

    Same graph, move set, corner rule and cost model as `_astar` (it is the
    same search without a goal and without a heuristic), so a cost read from
    here equals the cost `_astar` would report for that cell. Used by frontier
    exploration, which needs "how expensive is it to reach each candidate
    viewpoint" for many cells at once.

    Returns (g, parent): g is (rows, cols) float with inf where unreachable;
    parent is (rows, cols) int32 of flat indices (-1 = none) for
    `path_from_parents`.
    """
    n_rows, n_cols = passable.shape
    width = n_cols + 2
    pad_pass = np.zeros((n_rows + 2, width), dtype=bool)
    pad_pass[1:-1, 1:-1] = passable
    pad_cost = np.ones((n_rows + 2, width), dtype=np.float64)
    pad_cost[1:-1, 1:-1] = cost
    ok = pad_pass.ravel().tolist()
    step_cost = pad_cost.ravel().tolist()
    moves = [(dr * width + dc, ln, dr * width, dc) for dr, dc, ln in _MOVES]

    n = len(ok)
    g = [math.inf] * n
    parent = [-1] * n
    closed = [False] * n
    start_i = (start[0] + 1) * width + (start[1] + 1)
    g[start_i] = 0.0
    counter = 0
    heap = [(0.0, counter, start_i)]
    while heap:
        g_cur, _, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        closed[cur] = True
        for d_flat, length, d_r, d_c in moves:
            nxt = cur + d_flat
            if closed[nxt] or not ok[nxt]:
                continue
            if d_r and d_c and not (ok[cur + d_r] and ok[cur + d_c]):
                continue
            g_new = g_cur + length * step_cost[nxt]
            if g_new < g[nxt]:
                g[nxt] = g_new
                parent[nxt] = cur
                counter += 1
                heapq.heappush(heap, (g_new, counter, nxt))
    g_arr = np.array(g).reshape(n_rows + 2, width)[1:-1, 1:-1]
    parent_arr = np.array(parent, dtype=np.int64).reshape(n_rows + 2, width)[1:-1, 1:-1]
    return g_arr, parent_arr


def path_from_parents(parent: np.ndarray, target: tuple[int, int]) -> list[tuple[int, int]]:
    """Rebuild start->target cells from `dijkstra_all`'s parent map."""
    n_cols = parent.shape[1]
    width = n_cols + 2
    cells = []
    node = (target[0] + 1) * width + (target[1] + 1)
    while True:
        cells.append((node // width - 1, node % width - 1))
        nxt = parent[cells[-1][0], cells[-1][1]]
        if nxt == -1:
            break
        node = int(nxt)
    cells.reverse()
    return cells


# ---------------------------------------------------------------------------
# Post-processing: straighten, mark uncertain stretches, add standoff points
# ---------------------------------------------------------------------------


def _line_cost(cost: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> float:
    """Approximate cost of driving the straight segment, on the same scale as
    the search's own `g` (step length x cost of the cell being entered).
    The segment is sampled once per cell of its longer axis.
    """
    n_steps = max(abs(r1 - r0), abs(c1 - c0))
    if n_steps == 0:
        return 0.0
    rows = np.round(np.linspace(r0, r1, n_steps + 1)[1:]).astype(np.int64)
    cols = np.round(np.linspace(c0, c1, n_steps + 1)[1:]).astype(np.int64)
    return float(cost[rows, cols].sum()) * math.hypot(r1 - r0, c1 - c0) / n_steps


def _shorten(
    cells: list[tuple[int, int]],
    passable: np.ndarray,
    cost: np.ndarray,
    lookahead: int,
    cost_tolerance: float,
) -> list[tuple[int, int]]:
    """Greedy, COST-AWARE line-of-sight shortening ("string pulling").

    From the current anchor, look for the farthest later cell that is directly
    visible AND whose straight line is not meaningfully more expensive than
    the cell path it would replace. The cost test is what keeps shortening
    honest: without it, straightening happily drags a path back across rough
    ground the search deliberately went around, or slides it against a wall
    the search deliberately kept away from. In open, uniform space the
    straight line is always cheaper than the staircase, so it still pulls taut.

    Visibility along a path can be lost and regained (behind a pillar), so
    after the first rejected cell we keep looking `lookahead` more cells before
    giving up - a bounded compromise between always finding the farthest cell
    (quadratic) and stopping at the first failure (misses easy shortcuts).
    """
    if len(cells) <= 2:
        return list(cells)
    # cum[k] = search cost of walking cells[0] -> cells[k] along the path.
    cum = [0.0]
    for k in range(1, len(cells)):
        (ra, ca), (rb, cb) = cells[k - 1], cells[k]
        cum.append(cum[-1] + math.hypot(rb - ra, cb - ca) * cost[rb, cb])

    kept = [cells[0]]
    i = 0
    last = len(cells) - 1
    while i < last:
        best = i + 1
        misses = 0
        j = i + 1
        while j <= last:
            ri, ci = cells[i]
            rj, cj = cells[j]
            ok = line_is_clear(passable, ri, ci, rj, cj) and (
                _line_cost(cost, ri, ci, rj, cj) <= (cum[j] - cum[i]) * (1.0 + cost_tolerance) + 1e-9
            )
            if ok:
                best = j
                misses = 0
            else:
                misses += 1
                if misses > lookahead:
                    break
            j += 1
        kept.append(cells[best])
        i = best
    return kept


def _classify_cells(
    cells: list[tuple[int, int]],
    anomaly: np.ndarray | None,
    frontier: np.ndarray | None,
    near_kind: np.ndarray | None = None,
) -> list[int]:
    out = []
    for r, c in cells:
        if anomaly is not None and anomaly[r, c]:
            out.append(_ANOMALY)
        elif frontier is not None and frontier[r, c]:
            out.append(_FRONTIER)
        elif near_kind is not None and near_kind[r, c]:
            out.append(int(near_kind[r, c]))
        else:
            out.append(_FREE)
    return out


def _uncertain_runs(classes: list[int], max_gap_cells: int) -> list[tuple[int, int]]:
    """Index ranges [first, last] of uncertain cells along the path, with
    short free gaps between two uncertain runs MERGED into one.

    The merge matters for a real tunnel: the 2.5D map flags the two mouths
    (anomaly) but the tunnel's roof between them reads as ordinary walkable
    ground. Left alone that would be anomaly / free / anomaly - three stretches
    for one physical thing. Merged, it is one uncertain stretch from entry to
    exit, which is what the voxel check is asked about.
    """
    runs: list[list[int]] = []
    i = 0
    n = len(classes)
    while i < n:
        if classes[i] == _FREE:
            i += 1
            continue
        j = i
        while j + 1 < n and classes[j + 1] != _FREE:
            j += 1
        if runs and (i - runs[-1][1] - 1) <= max_gap_cells:
            runs[-1][1] = j
        else:
            runs.append([i, j])
        i = j + 1
    return [(a, b) for a, b in runs]


def _postprocess(
    cells: list[tuple[int, int]],
    pass_strict: np.ndarray,
    pass_optimistic: np.ndarray,
    anomaly: np.ndarray | None,
    frontier: np.ndarray | None,
    cell_cost: np.ndarray,
    resolution: float,
    standoff_m: float,
    merge_gap_m: float,
    lookahead: int,
    cost_tolerance: float,
    near_kind: np.ndarray | None = None,
) -> tuple[list[tuple[int, int]], list[str]]:
    """Turn the raw cell path into final waypoint cells + per-segment kinds."""
    classes = _classify_cells(cells, anomaly, frontier, near_kind)
    last = len(cells) - 1
    runs = _uncertain_runs(classes, max_gap_cells=int(round(merge_gap_m / resolution)))

    # Stretch boundaries (indices into `cells`). Each stretch is shortened on
    # its own, and consecutive stretches SHARE their boundary cell.
    stretches: list[tuple[int, int, str]] = []  # (from_idx, to_idx, kind)
    cursor = 0
    for first, run_last in runs:
        entry = max(first - 1, 0)  # last free cell before the uncertain run
        exit_ = min(run_last + 1, last)  # first free cell after it

        # STANDOFF: walk back from the entry cell along the path until we have
        # covered `standoff_m`, and pin a waypoint there.
        if entry > cursor and standoff_m > 0:
            walked = 0.0
            pin = entry
            while pin > cursor and walked < standoff_m:
                (r_a, c_a), (r_b, c_b) = cells[pin], cells[pin - 1]
                walked += math.hypot(r_a - r_b, c_a - c_b) * resolution
                pin -= 1
            if pin > cursor:
                stretches.append((cursor, pin, "free"))
                cursor = pin
        if entry > cursor:
            stretches.append((cursor, entry, "free"))
        kind = "anomaly" if any(classes[k] == _ANOMALY for k in range(first, run_last + 1)) else "frontier"
        if exit_ > entry:
            stretches.append((entry, exit_, kind))
        cursor = max(cursor, exit_)
    if cursor < last:
        stretches.append((cursor, last, "free"))
    if not stretches:  # a one-cell path (start == goal)
        return [cells[0], cells[-1]], ["free"]

    waypoints: list[tuple[int, int]] = []
    kinds: list[str] = []
    for a, b, kind in stretches:
        mask = pass_strict if kind == "free" else pass_optimistic
        # Inside an UNCERTAIN stretch the penalty multipliers dominate the cost
        # and would block every shortcut (leaving tiny zig-zags); the route
        # through it was already chosen by the search, so straighten it by line
        # of sight alone. Free stretches keep the cost test (see `_shorten`).
        tol = cost_tolerance if kind == "free" else math.inf
        short = _shorten(cells[a : b + 1], mask, cell_cost, lookahead, tol)
        if waypoints:
            short = short[1:]  # shared boundary cell is already there
        else:
            waypoints.append(short[0])
            short = short[1:]
        for cell in short:
            waypoints.append(cell)
            kinds.append(kind)
    return waypoints, kinds


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def _nearest_passable(passable: np.ndarray, cell: tuple[int, int], max_cells: float):
    """Nearest passable cell to `cell` within `max_cells`, or None."""
    if not passable.any():
        return None
    dist, (ri, ci) = distance_transform_edt(~passable, return_indices=True)
    if dist[cell] > max_cells:
        return None
    return int(ri[cell]), int(ci[cell])


@dataclass(frozen=True)
class PlannerParams:
    """Every tunable of the planner in one place (so a node can declare them
    once as ROS parameters and hand the same object to `plan`,
    `evaluate_path` and the exploration code - they must all agree).

    robot_radius_m, inflation_margin_m: obstacles are inflated by their sum;
        if no path exists, the bare `robot_radius_m` is tried too.
    mask_rim_m: how far the map's lethal cells already extend BEYOND a real
        obstacle's surface. `emap`'s traversability marks a step's whole 3x3
        neighbourhood lethal, so a real 0.15 m wall shows up as a 0.3 m lethal
        band (measured on a real saved map). Inflating by the full robot
        radius on top of that double-counts about one cell of clearance and
        closes doorways the robot can really use, so this amount is subtracted
        from the inflation. Set 0 for a map whose lethal cells hug the true
        surface.
    footprint_radius_m: area around the START always treated as free (the
        robot is physically there; its own chassis can pollute the map).
    clearance_weight, clearance_range_m: soft keep-away-from-walls cost.
    difficult_weight: extra cost of DIFFICULT terrain.
    anomaly_penalty, frontier_penalty: cost multipliers for uncertain cells
        (anomaly > frontier: it is a bet AGAINST the map's own reading).
    standoff_m: how far before an uncertain stretch to pin a waypoint.
    merge_gap_m: free gaps shorter than this between two uncertain runs are
        merged into one stretch (see `_uncertain_runs`). The default is the
        longest tunnel interior the anomaly detector accepts as an "island"
        (3 m^2 area, ~1 m wide -> ~3 m long).
    goal_snap_m: if the goal cell itself is not a hard obstacle but sits
        inside the inflation band, move it to the nearest passable cell within
        this distance. A goal INSIDE a hard obstacle is rejected.
    shorten_lookahead, shorten_cost_tolerance: see `_shorten`. A shortcut is
        accepted only if it costs at most (1 + tolerance) x the path segment
        it replaces.
    uncertain_waive_m: in the OPTIMISTIC pass, uncertain cells and the free
        cells within this distance of one are exempt from the clearance test.
        Why: the 2.5D map's clearance numbers are meaningless inside a
        suspected passage - the cells there are lethal readings that are
        probably artifacts, and the detector often flags only a few of them
        (3x3 cells = 0.3 m seen on the live tunnel, narrower than the robot).
        Demanding robot clearance there made the planner say "no path" through
        a tunnel the zero-width PRM crossed fine. The real check on such a
        stretch is the 3D voxel headroom test and Nav2's own costmap; the
        route is flagged tentative and gets a standoff point for exactly that.
    """

    robot_radius_m: float = 0.22
    inflation_margin_m: float = 0.05
    mask_rim_m: float = 0.1
    footprint_radius_m: float = 0.3
    clearance_weight: float = 1.0
    clearance_range_m: float = 0.5
    difficult_weight: float = 1.0
    anomaly_penalty: float = 20.0
    frontier_penalty: float = 2.0
    standoff_m: float = 1.0
    merge_gap_m: float = 3.0
    goal_snap_m: float = 0.4
    shorten_lookahead: int = 12
    shorten_cost_tolerance: float = 0.02
    uncertain_waive_m: float = 0.5


@dataclass
class _Grids:
    """Everything derived for ONE (mode, inflation) combination."""

    passable: np.ndarray  # cells the search may use in this mode
    pass_strict: np.ndarray  # confirmed-free, roomy enough
    pass_optimistic: np.ndarray  # strict + uncertain cells that are roomy
    cost: np.ndarray
    anomaly: np.ndarray | None  # only set in optimistic mode
    frontier: np.ndarray | None  # only set in optimistic mode
    # Optimistic mode only: for every cell closer to an uncertain cell than the
    # robot's inflation, which kind it leans on (0 none, 1 frontier, 2
    # anomaly). A route that merely RUNS ALONG unobserved/suspect cells is
    # only valid if those cells really are clear, so it is just as tentative
    # as one that crosses them - see `Scene.grids`.
    near_kind: np.ndarray | None = None


class _Scene:
    """The map + options a planning query runs against. Built once per query
    and asked for the grids of each (mode, inflation) pass - shared by
    `plan`, `evaluate_path` and the exploration code so they can never
    disagree about what is passable or what anything costs."""

    def __init__(
        self,
        walkable_mask,
        resolution,
        center_x,
        center_y,
        start_xy,
        *,
        traversability,
        frontier_mask,
        allow_frontier,
        anomaly_mask,
        allow_anomaly,
        force_free_mask,
        params: PlannerParams,
        cost_multiplier=None,
    ):
        self.params = params
        # Optional (rows, cols) factor applied to the whole cost grid - used to
        # make an area less attractive (e.g. where the robot got stuck).
        self.cost_multiplier = None if cost_multiplier is None else np.asarray(cost_multiplier, dtype=np.float64)
        self.resolution = resolution
        self.center_x, self.center_y = center_x, center_y
        self.walkable = np.asarray(walkable_mask, dtype=bool)
        self.shape = self.walkable.shape
        self.n = self.shape[0]  # maps are square (see module docstring)
        self.traversability = traversability

        self.use_anomaly = bool(allow_anomaly and anomaly_mask is not None)
        self.use_frontier = bool(allow_frontier and frontier_mask is not None)
        self.anomaly = np.asarray(anomaly_mask, dtype=bool) if self.use_anomaly else None
        self.frontier = np.asarray(frontier_mask, dtype=bool) if self.use_frontier else None
        self.uncertain = np.zeros(self.shape, dtype=bool)
        if self.use_anomaly:
            self.uncertain |= self.anomaly
        if self.use_frontier:
            self.uncertain |= self.frontier & ~self.walkable

        force_free = (
            np.asarray(force_free_mask, dtype=bool)
            if force_free_mask is not None
            else np.zeros(self.shape, dtype=bool)
        )
        sr, sc = world_to_grid(start_xy[0], start_xy[1], center_x, center_y, resolution, self.n)
        self.start = (int(sr), int(sc))
        self.start_in_grid = self.in_grid(self.start)
        if self.start_in_grid:
            # The robot's own footprint is free by definition.
            rows_i, cols_i = np.indices(self.shape)
            near = np.hypot(rows_i - self.start[0], cols_i - self.start[1]) * resolution <= params.footprint_radius_m
            near[self.start] = True
            force_free = force_free | near
        self.force_free = force_free
        self.base_walkable = self.walkable | force_free

    def in_grid(self, cell) -> bool:
        return 0 <= cell[0] < self.shape[0] and 0 <= cell[1] < self.shape[1]

    def cell_of(self, xy) -> tuple[int, int]:
        r, c = world_to_grid(xy[0], xy[1], self.center_x, self.center_y, self.resolution, self.n)
        return int(r), int(c)

    def xy_of(self, cell) -> tuple[float, float]:
        x, y = grid_to_world(cell[0], cell[1], self.center_x, self.center_y, self.resolution, self.n)
        return float(x), float(y)

    def levels(self) -> list[tuple[str, float]]:
        """The (mode, inflation) passes to try, in order of preference:
        confirmed routes before tentative ones, full clearance before relaxed.
        Effective inflation = what the robot needs minus the clearance the
        map's own lethal rim already provides (never below zero)."""
        p = self.params
        full = max(p.robot_radius_m + p.inflation_margin_m - p.mask_rim_m, 0.0)
        relaxed = max(p.robot_radius_m - p.mask_rim_m, 0.0)
        infls = [full] if p.inflation_margin_m <= 0 else [full, relaxed]
        out = [("strict", i) for i in infls]
        if self.use_anomaly or self.use_frontier:
            out += [("optimistic", i) for i in infls]
        return out

    def grids(self, mode: str, inflation_m: float) -> _Grids:
        p = self.params
        optimistic = mode == "optimistic"
        # Obstacle sources for inflation. In the STRICT pass uncertain cells
        # count as obstacles (we are not entering them, and they might well be
        # walls); in the OPTIMISTIC pass they do not (we plan to cross them).
        blocked = ~(self.base_walkable | (self.uncertain if optimistic else False))
        blocked = blocked & ~self.force_free
        clear = clearance_m(blocked, self.resolution)
        roomy = clear >= inflation_m - _CLEARANCE_EPS
        pass_strict = (self.base_walkable & roomy) | self.force_free
        near_uncertain = None
        if optimistic and self.uncertain.any():
            dist, (ri, ci) = distance_transform_edt(~self.uncertain, return_indices=True)
            dist_m = dist * self.resolution
            # cells whose clearance does not count: the uncertain cells
            # themselves, and free cells close enough to approach them
            waived = self.uncertain | (self.base_walkable & (dist_m <= p.uncertain_waive_m + _CLEARANCE_EPS))
            pass_optimistic = pass_strict | waived
            near_uncertain = (dist_m, ri, ci)
        else:
            pass_optimistic = pass_strict
        anomaly = self.anomaly if optimistic else None
        frontier = self.frontier if optimistic else None
        cost = build_cost_grid(
            clear,
            inflation_m,
            self.traversability,
            anomaly,
            frontier,
            p.clearance_weight,
            p.clearance_range_m,
            p.difficult_weight,
            p.anomaly_penalty,
            p.frontier_penalty,
        )
        if self.cost_multiplier is not None:
            cost = cost * self.cost_multiplier
        near_kind = None
        if near_uncertain is not None:
            # In the optimistic pass uncertain cells are NOT obstacles for
            # clearance and the cells approaching them skip the clearance
            # test, so a path may run right alongside them. That path is only
            # valid if those cells turn out clear - mark every cell within the
            # waive distance (or the inflation, if larger) of one as leaning
            # on that uncertainty, so the route is reported tentative.
            dist_m, ri, ci = near_uncertain
            near = dist_m < max(inflation_m, p.uncertain_waive_m) - _CLEARANCE_EPS
            nearest_is_anomaly = (
                self.anomaly[ri, ci] if self.anomaly is not None else np.zeros(self.shape, dtype=bool)
            )
            near_kind = np.where(near, np.where(nearest_is_anomaly, _ANOMALY, _FRONTIER), _FREE)
        return _Grids(
            passable=pass_optimistic if optimistic else pass_strict,
            pass_strict=pass_strict,
            pass_optimistic=pass_optimistic if optimistic else pass_strict,
            cost=cost,
            anomaly=anomaly,
            frontier=frontier,
            near_kind=near_kind,
        )


def _make_scene(
    walkable_mask,
    resolution,
    center_x,
    center_y,
    start_xy,
    traversability,
    frontier_mask,
    allow_frontier,
    anomaly_mask,
    allow_anomaly,
    force_free_mask,
    params,
    overrides,
    cost_multiplier=None,
) -> _Scene:
    params = params if params is not None else PlannerParams()
    if overrides:
        params = replace(params, **overrides)  # unknown names raise TypeError
    return _Scene(
        walkable_mask,
        resolution,
        center_x,
        center_y,
        start_xy,
        traversability=traversability,
        frontier_mask=frontier_mask,
        allow_frontier=allow_frontier,
        anomaly_mask=anomaly_mask,
        allow_anomaly=allow_anomaly,
        force_free_mask=force_free_mask,
        params=params,
        cost_multiplier=cost_multiplier,
    )


def plan(
    walkable_mask: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *,
    traversability: np.ndarray | None = None,
    frontier_mask: np.ndarray | None = None,
    allow_frontier: bool = False,
    anomaly_mask: np.ndarray | None = None,
    allow_anomaly: bool = False,
    force_free_mask: np.ndarray | None = None,
    cost_multiplier: np.ndarray | None = None,
    params: PlannerParams | None = None,
    **overrides,
) -> AStarResult:
    """Plan a path from `start_xy` to `goal_xy` (world meters).

    Same first six arguments as `prm_planner.plan`, so a caller can swap
    planners. Tunables live in `PlannerParams` (pass `params=`, and/or any
    single field by name as an override, e.g. `standoff_m=0.0`).

    Args:
        walkable_mask: (rows, cols) bool, True = confirmed free ground.
        traversability: optional (rows, cols) {LETHAL, DIFFICULT, EASY} layer;
            only used to make DIFFICULT cells cost more.
        frontier_mask / anomaly_mask, allow_frontier / allow_anomaly: the
            uncertain-cell categories, same meaning as in `prm_planner`.
            Only usable when the matching `allow_*` flag is set.
        force_free_mask: cells that are ALWAYS passable and never inflated
            from - used for regions the 3D voxel check confirmed passable, so
            inflation from nearby real walls cannot shut them again.
        cost_multiplier: optional (rows, cols) factor applied to every cell's
            cost (>= 1 makes an area less attractive).
    """
    scene = _make_scene(
        walkable_mask, resolution, center_x, center_y, start_xy, traversability,
        frontier_mask, allow_frontier, anomaly_mask, allow_anomaly, force_free_mask,
        params, overrides, cost_multiplier,
    )  # fmt: skip
    p = scene.params
    goal = scene.cell_of(goal_xy)
    if not (scene.start_in_grid and scene.in_grid(goal)):
        return AStarResult(valid=False)
    # A goal that is itself a hard obstacle (known-lethal / unobserved and not
    # an uncertain category) is rejected outright, like prm_planner does.
    if not (scene.base_walkable[goal] or scene.uncertain[goal]):
        return AStarResult(valid=False)

    for mode, infl in scene.levels():
        g = scene.grids(mode, infl)
        goal_cell = goal
        if not g.passable[goal_cell]:
            # Only snap a goal that is genuinely free ground sitting inside
            # the inflation band. A goal INSIDE an uncertain region must not be
            # silently moved out of it by the strict pass - that would answer
            # a different question than the one asked.
            if not scene.base_walkable[goal_cell]:
                continue
            snapped = _nearest_passable(g.passable, goal_cell, p.goal_snap_m / scene.resolution)
            if snapped is None:
                continue
            goal_cell = snapped
        found = _astar(g.passable, g.cost, scene.start, goal_cell)
        if found is None:
            continue
        cells, total = found

        # Shortening is checked against the SAME masks the search used, so a
        # shortened segment can never cut through anything the search avoided.
        wp_cells, kinds = _postprocess(
            cells, g.pass_strict, g.pass_optimistic, g.anomaly, g.frontier, g.cost,
            scene.resolution, p.standoff_m, p.merge_gap_m, p.shorten_lookahead,
            p.shorten_cost_tolerance, g.near_kind,
        )  # fmt: skip
        return AStarResult(
            waypoints=[scene.xy_of(cell) for cell in wp_cells],
            valid=True,
            has_frontier_segments=any(k == "frontier" for k in kinds),
            has_anomaly_segments=any(k == "anomaly" for k in kinds),
            segment_kinds=kinds,
            cost=float(total),
            inflation_m=infl,
            goal_xy=scene.xy_of(goal_cell),
            mode=mode,
        )
    return AStarResult(valid=False)


def evaluate_path(
    waypoints: list[tuple[float, float]],
    walkable_mask: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    mode: str,
    inflation_m: float,
    *,
    traversability: np.ndarray | None = None,
    frontier_mask: np.ndarray | None = None,
    allow_frontier: bool = False,
    anomaly_mask: np.ndarray | None = None,
    allow_anomaly: bool = False,
    force_free_mask: np.ndarray | None = None,
    cost_multiplier: np.ndarray | None = None,
    params: PlannerParams | None = None,
    **overrides,
) -> tuple[bool, float, list[str]]:
    """Re-judge an EXISTING polyline against the CURRENT map: returns
    (still_valid, cost, per-segment kinds).

    This is what lets route memory ask "is the route I am already driving
    still OK, and what would it cost from here?" using exactly the grids a
    fresh `plan` with the same `mode`/`inflation_m` would use - so an old and
    a new path are always compared on the same scale.

    A segment is valid if its whole supercover line is passable in the mode's
    mask; its kind is "anomaly"/"frontier" if the line touches such a cell,
    else "free". Cost uses the same per-cell cost as the search.
    """
    scene = _make_scene(
        walkable_mask, resolution, center_x, center_y, start_xy, traversability,
        frontier_mask, allow_frontier, anomaly_mask, allow_anomaly, force_free_mask,
        params, overrides, cost_multiplier,
    )  # fmt: skip
    if len(waypoints) < 2 or not scene.start_in_grid:
        return False, math.inf, []
    g = scene.grids(mode, inflation_m)
    cells = [scene.cell_of(w) for w in waypoints]
    if not all(scene.in_grid(c) for c in cells):
        return False, math.inf, []
    total = 0.0
    kinds: list[str] = []
    for (r0, c0), (r1, c1) in zip(cells[:-1], cells[1:]):
        if not line_is_clear(g.passable, r0, c0, r1, c1):
            return False, math.inf, []
        total += _line_cost(g.cost, r0, c0, r1, c1)
        n_steps = max(abs(r1 - r0), abs(c1 - c0), 1)
        rows = np.round(np.linspace(r0, r1, n_steps + 1)).astype(np.int64)
        cols = np.round(np.linspace(c0, c1, n_steps + 1)).astype(np.int64)
        near = g.near_kind[rows, cols] if g.near_kind is not None else None
        if (g.anomaly is not None and g.anomaly[rows, cols].any()) or (near is not None and (near == _ANOMALY).any()):
            kinds.append("anomaly")
        elif (g.frontier is not None and g.frontier[rows, cols].any()) or (
            near is not None and (near == _FRONTIER).any()
        ):
            kinds.append("frontier")
        else:
            kinds.append("free")
    return True, total, kinds
