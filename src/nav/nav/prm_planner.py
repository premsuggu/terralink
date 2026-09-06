"""Probabilistic Roadmap (PRM) path planner - a NumPy/SciPy port of
`src/d3/my_bot/includes/processImage.cpp`'s `GridSpace` class, kept as the
same algorithm on purpose (random sampling restricted to free space, a
line-of-sight check to connect nearby samples, shortest-path search over the
resulting graph) rather than reinvented, per the project's instruction to
reuse d3's planner and only change how "free space" is determined.

Two real differences from the C++ original, both because `emap`'s published
map already carries information d3's planner never had access to (a live
camera image has neither), not because the algorithm itself changed:

1. **No camera projection.** d3's `coordToPixel`/`pixelToCoord` assumed a
   fixed-altitude, fixed-FOV downward camera to convert between world
   coordinates and image pixels - baking that assumption directly into the
   planner. `emap`'s `GridMap` already carries an exact `resolution` and
   world-frame center, so world<->grid conversion here is exact grid math
   (`emap.utils.coord_transform.world_to_grid`/`grid_to_world`, the same
   functions the map itself was built and published with) - no camera or
   altitude assumption needed at all.
2. **Start/goal are exact roadmap nodes, not "nearest random sample".**
   d3's `getWaypoints` always snaps both endpoints to whichever randomly
   sampled node happens to be closest (`findNearest`) - a real source of
   imprecision purely from sampling luck. Since we know the query points
   up front, they're added as guaranteed nodes 0 and 1 before sampling
   begins (still no different in kind - PRM has always allowed adding
   arbitrary free-space configurations as nodes; this just always adds
   these two).

REAL BUG FOUND LIVE, fixed by a third difference: a parked `nav_ugv` is
itself a physical object sitting in the room, and `emap`'s depth camera has
no notion of "that's the robot, not terrain" - it fuses the UGV's own
chassis into the elevation map like any other obstacle whenever the UAV
flies back over it. That corrupted the map exactly at the UGV's own
position, and since `plan()` originally rejected any query whose START cell
wasn't walkable, the UGV could get permanently locked out of ever planning
a route away from wherever it happened to be parked - it isn't "stuck next
to an obstacle", it silently BECAME the obstacle, at its own location, the
longer it sat there. The fix (`_clear_footprint_around`) mirrors standard
costmap practice (Nav2's own costmap always clears the robot's own
footprint before planning): a small disk around the START point is forced
walkable before anything else runs, on the principle that wherever the
robot currently, physically is, must be treated as reachable by
definition. The GOAL is NOT given this exemption - rejecting a query whose
goal is genuinely blocked is still correct; only "trust that I'm not
sitting inside a wall right now" makes sense for a live position report.

See docs/work-docs/nav/00_concepts.md for the from-scratch PRM explanation
this module's tests are built against.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import lil_matrix
from scipy.sparse.csgraph import dijkstra

from emap.utils.coord_transform import grid_to_world, world_to_grid


@dataclass
class PlanResult:
    """Mirrors d3's `Path` struct: a sequence of world-frame (x, y)
    waypoints (empty if `valid` is False), start-to-goal order.
    """
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    valid: bool = False


def _cell_is_walkable(walkable_mask: np.ndarray, row: int, col: int) -> bool:
    n_rows, n_cols = walkable_mask.shape
    if not (0 <= row < n_rows and 0 <= col < n_cols):
        return False
    return bool(walkable_mask[row, col])


def _clear_footprint_around(walkable_mask: np.ndarray, row: int, col: int, radius_cells: int) -> np.ndarray:
    """Return a COPY of `walkable_mask` with a small disk around (row, col)
    forced to True - "wherever the robot's own body currently is, treat as
    passable", the same footprint-clearing standard costmaps (Nav2's
    included) apply before planning. See this module's docstring for the
    real bug this fixes: without it, a parked UGV's own chassis getting
    fused into the elevation map as if it were terrain could permanently
    lock the UGV out of planning any route away from wherever it's sitting.
    """
    mask = walkable_mask.copy()
    n_rows, n_cols = mask.shape
    r0, r1 = max(0, row - radius_cells), min(n_rows, row + radius_cells + 1)
    c0, c1 = max(0, col - radius_cells), min(n_cols, col + radius_cells + 1)
    # A square block, not a true circle - simpler, and radius_cells is small
    # enough (a robot's own footprint) that the difference is negligible.
    mask[r0:r1, c0:c1] = True
    return mask


def _line_is_walkable(walkable_mask: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> bool:
    """Rasterized line-of-sight check: every cell the straight line from
    (r0, c0) to (r1, c1) passes through must be walkable, or the two nodes
    can't be connected by an edge - the NumPy equivalent of d3's
    `checkLine`, which walked a `cv::LineIterator` over the same segment.

    `n_steps` is chosen from the longer of the row/col span so every
    grid cell the line crosses gets its own sample point (no gaps a
    thin single-cell obstacle could hide in), then each sampled point is
    rounded to its nearest cell - the same "one sample per cell along the
    line" guarantee `cv::LineIterator`'s Bresenham stepping provides.
    """
    n_steps = int(max(abs(r1 - r0), abs(c1 - c0))) + 1
    rows = np.round(np.linspace(r0, r1, n_steps)).astype(np.int64)
    cols = np.round(np.linspace(c0, c1, n_steps)).astype(np.int64)
    return bool(np.all(walkable_mask[rows, cols]))


def plan(
    walkable_mask: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    num_samples: int = 300,
    connect_radius_m: float = 3.0,
    footprint_radius_m: float = 0.3,
    rng: np.random.Generator | None = None,
) -> PlanResult:
    """Plan a path from `start_xy` to `goal_xy` (both world-frame meters)
    through `walkable_mask` (True = free space).

    Args:
        walkable_mask: (rows, cols) boolean array - see `nav.walkability`.
        resolution, center_x, center_y: the map's own grid geometry, taken
            straight from the decoded GridMap message (`msg.info.resolution`,
            `msg.info.pose.position.x/y`) - same convention `ElevationMap`
            itself uses, so no conversion is needed on the caller's side.
        start_xy, goal_xy: world-frame (x, y) meters.
        num_samples: how many random walkable cells to add as roadmap nodes
            (mirrors d3's `runPRM(..., n)` sample count - d3's own default
            was 100 samples per camera frame, repeated across frames up to
            a 1000-node cap; here the whole roadmap is (re)built in one call
            from a single request, so this is the total node budget).
        connect_radius_m: only node pairs within this Euclidean distance are
            even considered for an edge (mirrors d3's pixel-radius cutoff in
            `connectNewNode`, expressed in meters instead of pixels since
            there's no camera scale to divide by here).
        footprint_radius_m: radius (meters) around `start_xy` ALWAYS treated
            as walkable, regardless of what the map says there - see this
            module's docstring for the real bug this fixes (a parked UGV
            getting fused into its own elevation map as an "obstacle").
            Not applied to `goal_xy` - a genuinely blocked goal should still
            be rejected. Default ~ a small ground robot's own footprint.
        rng: inject a seeded `np.random.default_rng(seed)` for deterministic
            tests; defaults to a fresh, unseeded generator for real use.

    Returns:
        A `PlanResult`. `valid=False` (empty waypoints) if the goal cell
        isn't walkable, or if no path connects start to goal through the
        sampled roadmap (a real possibility with random sampling - a caller
        needing a guarantee should retry with a higher `num_samples`, same
        caveat that applies to every PRM planner, including d3's).
    """
    if rng is None:
        rng = np.random.default_rng()

    n_rows, n_cols = walkable_mask.shape
    cell_n = n_rows  # ElevationMap's grid is always square (see elevation_map.py)

    start_row, start_col = world_to_grid(start_xy[0], start_xy[1], center_x, center_y, resolution, cell_n)
    goal_row, goal_col = world_to_grid(goal_xy[0], goal_xy[1], center_x, center_y, resolution, cell_n)
    start_row, start_col, goal_row, goal_col = int(start_row), int(start_col), int(goal_row), int(goal_col)

    footprint_radius_cells = max(1, int(round(footprint_radius_m / resolution)))
    walkable_mask = _clear_footprint_around(walkable_mask, start_row, start_col, footprint_radius_cells)

    if not _cell_is_walkable(walkable_mask, goal_row, goal_col):
        return PlanResult(valid=False)

    # --- Sample roadmap nodes: node 0 = start, node 1 = goal, the rest
    # random walkable cells (vectorized: pick directly from the set of
    # already-walkable cell coordinates rather than d3's reject-and-retry
    # loop over uniformly random pixels - same "restrict sampling to free
    # space" idea, just without wasting draws on cells we already know are
    # blocked). ---
    walkable_rows, walkable_cols = np.nonzero(walkable_mask)
    if walkable_rows.size == 0:
        return PlanResult(valid=False)  # nothing walkable at all - unreachable by definition
    n_samples = min(num_samples, walkable_rows.size)
    sample_idx = rng.choice(walkable_rows.size, size=n_samples, replace=False)

    node_rows = np.concatenate(([start_row, goal_row], walkable_rows[sample_idx])).astype(np.float64)
    node_cols = np.concatenate(([start_col, goal_col], walkable_cols[sample_idx])).astype(np.float64)
    n_nodes = node_rows.size
    START_IDX, GOAL_IDX = 0, 1

    # --- Candidate edges: only pairs within connect_radius_m even get a
    # line-of-sight check (mirrors d3's radius pre-filter before the more
    # expensive checkLine call). Distances computed once, vectorized, over
    # every pair - fine at this node count (num_samples is a few hundred,
    # so n_nodes^2 is at most ~10^5, not a bottleneck). ---
    connect_radius_cells = connect_radius_m / resolution
    dr = node_rows[:, None] - node_rows[None, :]
    dc = node_cols[:, None] - node_cols[None, :]
    dist_cells = np.sqrt(dr**2 + dc**2)

    graph = lil_matrix((n_nodes, n_nodes), dtype=np.float64)
    for i in range(n_nodes):
        # j > i: each undirected edge only needs to be found and
        # line-checked once; lil_matrix assignment below sets both (i, j)
        # and (j, i) so the graph stays symmetric for dijkstra.
        for j in np.nonzero(dist_cells[i, i + 1:] < connect_radius_cells)[0] + (i + 1):
            if _line_is_walkable(walkable_mask, int(node_rows[i]), int(node_cols[i]), int(node_rows[j]), int(node_cols[j])):
                cost = dist_cells[i, j] * resolution  # edge cost in real meters
                graph[i, j] = cost
                graph[j, i] = cost

    distances, predecessors = dijkstra(
        graph.tocsr(), directed=False, indices=START_IDX, return_predecessors=True
    )

    if not np.isfinite(distances[GOAL_IDX]):
        return PlanResult(valid=False)  # sampled roadmap never connected start to goal

    # Walk the predecessor chain from goal back to start (dijkstra's
    # standard path-reconstruction pattern), then reverse it into
    # start-to-goal order.
    path_node_indices = [GOAL_IDX]
    node = GOAL_IDX
    while node != START_IDX:
        node = int(predecessors[node])
        path_node_indices.append(node)
    path_node_indices.reverse()

    waypoints = []
    for idx in path_node_indices:
        wx, wy = grid_to_world(node_rows[idx], node_cols[idx], center_x, center_y, resolution, cell_n)
        waypoints.append((float(wx), float(wy)))

    return PlanResult(waypoints=waypoints, valid=True)
