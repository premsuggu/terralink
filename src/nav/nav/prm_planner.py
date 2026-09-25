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

A fourth, later addition, OFF BY DEFAULT (`allow_frontier=False`): frontier-
aware planning. `nav.walkability.compute_frontier_mask` identifies cells the
UAV's overhead camera can never observe in principle (e.g. the interior of a
tunnel/culvert) but that border known-walkable space. Before this, such a
cell was indistinguishable from a solid wall - a real, structural limitation,
not a bug: if the tunnel is the ONLY route to a goal, `plan()` reported no
path at all, forever, no matter how much of the surrounding area got
scanned. With `allow_frontier=True` and a `frontier_mask` supplied, a path
that can only be completed by crossing such cells is now returned as a
TENTATIVE result (`PlanResult.has_frontier_segments=True`) instead of
failing - see docs/work-docs/nav/step05_frontier_tunnel_navigation.md for the
full scenario, the literature this design follows (frontier-based
exploration, Yamauchi 1997), and how a caller (`waypoint_follower.py`) is
expected to react to a tentative path (drive it cautiously, then re-plan
once the UGV's own sensor has resolved those cells with real data).

A fifth addition, OFF BY DEFAULT (`allow_anomaly=False`), step06 Phase 3:
anomaly-aware planning. Step05 found live that the frontier mechanism above
does NOT solve the literal "opaque overhang" case (a tunnel roof) - a solid
overhang produces a *confidently wrong* elevation reading, not an absence of
data, so `compute_frontier_mask` (built entirely around `is_valid=False`)
correctly never fires there. `nav.anomaly.compute_anomaly_mask` instead
flags cells whose LETHAL reading looks like a step-discontinuity DATA
ARTIFACT (two real surfaces competing for one grid cell) rather than a
genuinely solid obstacle - see that module's docstring for the full
reasoning and the live evidence it was tuned against
(docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md, Phase 2). With
`allow_anomaly=True` and an `anomaly_mask` supplied, a path that can only be
completed by crossing such cells is likewise returned as tentative
(`PlanResult.has_anomaly_segments=True`).

Anomaly cells get different treatment from frontier cells, deliberately: a
frontier cell is *merely unmeasured* (the literature-standard, relatively
low-risk case - it might turn out to be anything), while an anomaly cell is
*measured and scored LETHAL*, just possibly for the wrong reason - crossing
one is a real bet against the map's own explicit "this is a step" reading,
not just an absence of information. So unlike frontier edges (left at their
plain distance cost - step05 never needed to weigh a frontier route against
a confirmed one, since nothing about "unmeasured" is more or less trustworthy
than another unmeasured patch), any edge that requires an anomaly cell is
charged `_ANOMALY_EDGE_PENALTY` extra cost. This makes Dijkstra strongly
prefer a genuinely confirmed-safe route whenever ANY exists (even a much
longer one), while still finding and returning the anomaly route - as a
clearly-flagged last resort - when it's the only way to connect start to
goal at all. Roadmap NODES still only ever come from confirmed-`walkable_mask`
cells, exactly like frontier cells (see above) - an anomaly cell is a
corridor to pass through under suspicion, never a place to plan as if it
were solid ground.

See docs/work-docs/nav/00_concepts.md for the from-scratch PRM explanation
this module's tests are built against.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import lil_matrix
from scipy.sparse.csgraph import dijkstra

from emap.utils.coord_transform import grid_to_world, world_to_grid

# Cost multiplier applied to any PRM edge that can only be connected by
# passing through an "anomaly" cell (see nav.anomaly / this module's
# docstring). Large enough that Dijkstra always prefers a genuinely
# confirmed-safe route when one exists at all - even a much longer one - but
# still finite, so an anomaly route is found and returned (flagged tentative)
# when it's the only way to connect start to goal. Not applied to frontier
# edges: an unmeasured cell carries no signal one way or the other about
# being more or less trustworthy than another unmeasured cell, so there's
# nothing to weight there - only an anomaly cell carries an explicit "the map
# thinks this is LETHAL" reading that a route through it is knowingly betting
# against.
_ANOMALY_EDGE_PENALTY = 20.0


@dataclass
class PlanResult:
    """Mirrors d3's `Path` struct: a sequence of world-frame (x, y)
    waypoints (empty if `valid` is False), start-to-goal order.

    `has_frontier_segments`: True only when `allow_frontier=True` was passed
    to `plan()` AND the returned path actually needed to cross at least one
    unobserved-but-reachable "frontier" cell to connect start to goal (see
    `nav.walkability.compute_frontier_mask`) - i.e. this path is a tentative
    GUESS through territory nothing has actually measured yet, not a
    confirmed-safe route. Always False for a plan found without frontier
    cells at all (including every call that doesn't pass `allow_frontier`),
    so existing callers see no behavior change.

    `has_anomaly_segments`: same idea, but for `allow_anomaly=True` and cells
    from `nav.anomaly.compute_anomaly_mask` - a path that could only be
    completed by crossing a cell the map scored LETHAL but suspects is a
    step-discontinuity data artifact (e.g. a tunnel roof), not a genuine
    obstacle. Kept as a separate field from `has_frontier_segments`
    deliberately - a downstream consumer needs to know WHICH resolution
    mechanism a tentative segment needs (more UAV scanning, for frontier; a
    3D voxel-map check, for anomaly - see step06 Phase 4), and a single path
    can in principle need both kinds in different places.
    """
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    valid: bool = False
    has_frontier_segments: bool = False
    has_anomaly_segments: bool = False


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
    frontier_mask: np.ndarray | None = None,
    allow_frontier: bool = False,
    anomaly_mask: np.ndarray | None = None,
    allow_anomaly: bool = False,
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
        frontier_mask: (rows, cols) boolean array from
            `nav.walkability.compute_frontier_mask` - cells that are
            currently unobserved but reachable (e.g. the interior of a
            tunnel the UAV can never see into from overhead). Ignored unless
            `allow_frontier=True`. See `PlanResult.has_frontier_segments`.
        allow_frontier: if True AND `frontier_mask` is given, a path that
            can only be completed by crossing frontier cells is returned as
            a TENTATIVE result instead of failing outright (default False -
            existing callers that never pass this get byte-for-byte the same
            behavior as before this parameter existed). Roadmap NODES are
            still only ever sampled from confirmed-`walkable_mask` cells
            (frontier cells are just corridors a straight-line edge is
            allowed to pass THROUGH, never a place to land/stop - we have no
            actual measurement of what's there, so treating one as a
            trustworthy waypoint would be a stronger claim than "reachable
            unknown" warrants).
        anomaly_mask: (rows, cols) boolean array from
            `nav.anomaly.compute_anomaly_mask` - cells scored LETHAL but
            suspected of being a step-discontinuity data artifact (e.g. a
            tunnel roof) rather than a genuine obstacle. Ignored unless
            `allow_anomaly=True`. See `PlanResult.has_anomaly_segments` and
            this module's docstring for why anomaly edges are cost-penalized
            (`_ANOMALY_EDGE_PENALTY`) while frontier edges are not - a
            confirmed-safe alternate route is always preferred when one
            exists, and an anomaly route is only ever returned as a
            last-resort tentative guess.
        allow_anomaly: if True AND `anomaly_mask` is given, a path that can
            only be completed by crossing anomaly cells is returned as a
            TENTATIVE result instead of failing outright (default False -
            same byte-for-byte-unchanged guarantee as `allow_frontier` for
            any caller that doesn't pass this). Roadmap NODES are still only
            ever sampled from confirmed-`walkable_mask` cells, same
            restriction as frontier cells.

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

    # The mask actually used for LINE-OF-SIGHT checks below. Only differs
    # from walkable_mask when a caller opts in to frontier and/or anomaly
    # mode - every existing call site (both flags default to False) gets
    # exactly the same traverse_mask == walkable_mask as before either
    # feature existed, so nothing about the plain-walkable path below
    # changed. See PlanResult's docstring for why frontier/anomaly cells are
    # eligible as PASS-THROUGH corridor cells but never as sampled roadmap
    # nodes (that part is unchanged below).
    #
    # `walkable_or_frontier` is kept as its own intermediate (rather than
    # only ever looking at the final traverse_mask) because it's also
    # exactly the mask needed to answer "could this edge be connected WITHOUT
    # anomaly cells" - both for the cost-penalty decision below and for the
    # post-hoc has_frontier_segments/has_anomaly_segments split at the end of
    # this function. Reusing it in both places keeps those two answers
    # consistent with each other by construction, rather than two separately
    # written checks that could quietly drift apart.
    walkable_or_frontier = walkable_mask
    if allow_frontier and frontier_mask is not None:
        walkable_or_frontier = walkable_mask | np.asarray(frontier_mask, dtype=bool)

    traverse_mask = walkable_or_frontier
    if allow_anomaly and anomaly_mask is not None:
        traverse_mask = walkable_or_frontier | np.asarray(anomaly_mask, dtype=bool)

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

    anomaly_active = allow_anomaly and anomaly_mask is not None

    graph = lil_matrix((n_nodes, n_nodes), dtype=np.float64)
    for i in range(n_nodes):
        # j > i: each undirected edge only needs to be found and
        # line-checked once; lil_matrix assignment below sets both (i, j)
        # and (j, i) so the graph stays symmetric for dijkstra.
        for j in np.nonzero(dist_cells[i, i + 1:] < connect_radius_cells)[0] + (i + 1):
            ri, ci, rj, cj = int(node_rows[i]), int(node_cols[i]), int(node_rows[j]), int(node_cols[j])
            if _line_is_walkable(traverse_mask, ri, ci, rj, cj):
                cost = dist_cells[i, j] * resolution  # edge cost in real meters
                # Only pay for a second line check when anomaly mode is even
                # on - the common (default) case does exactly one check per
                # edge, same as before this feature existed. An edge that
                # needs traverse_mask to connect but NOT walkable_or_frontier
                # is, by construction, one that specifically requires an
                # anomaly cell - see this module's docstring for why that's
                # penalized while a frontier-only edge is not.
                if anomaly_active and not _line_is_walkable(walkable_or_frontier, ri, ci, rj, cj):
                    cost *= _ANOMALY_EDGE_PENALTY
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

    # Did this path actually NEED a frontier and/or anomaly cell, or would it
    # have been found even without them? Re-check each edge in the ACTUAL
    # returned path (not every candidate edge considered above) against
    # progressively looser masks - the same walkable_or_frontier
    # intermediate used for the cost-penalty decision above, reused here so
    # the two answers can't drift apart. Only bothered with at all when the
    # corresponding mode was even on, since otherwise the relevant mask
    # equals walkable_mask and the answer is always False anyway (same
    # early-exit discipline as before either feature existed).
    has_frontier_segments = False
    has_anomaly_segments = False
    if allow_frontier or anomaly_active:
        for a, b in zip(path_node_indices[:-1], path_node_indices[1:]):
            ra, ca, rb, cb = int(node_rows[a]), int(node_cols[a]), int(node_rows[b]), int(node_cols[b])
            if _line_is_walkable(walkable_mask, ra, ca, rb, cb):
                continue  # this edge never needed frontier OR anomaly cells
            if allow_frontier and _line_is_walkable(walkable_or_frontier, ra, ca, rb, cb):
                # Satisfiable by adding frontier cells alone - this edge's
                # "extra" cells are unmeasured space, not a suspected
                # data artifact, regardless of whether anomaly mode is
                # also on.
                has_frontier_segments = True
            elif anomaly_active:
                # Not satisfiable by walkable+frontier alone -> whatever
                # extra cells this edge needed must include at least one
                # anomaly cell (traverse_mask, which DID connect this edge,
                # only ever adds frontier and/or anomaly cells on top of
                # walkable_mask).
                has_anomaly_segments = True

    return PlanResult(
        waypoints=waypoints,
        valid=True,
        has_frontier_segments=has_frontier_segments,
        has_anomaly_segments=has_anomaly_segments,
    )
