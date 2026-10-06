"""Frontier exploration: where should the UGV go to LEARN something, when no
route to the goal exists yet - not even a tentative one?

WHEN THIS RUNS
--------------
`nav.astar_planner.plan` first looks for a confirmed route, then a tentative
one through suspicious (anomaly/frontier) cells. If BOTH fail, the goal is cut
off by space nobody has looked at, or by walls nobody flagged. Declaring "no
path" and sitting still would be the wrong answer - the UGV carries a camera
and builds a 3D map as it drives, so the right answer is "go look".

THE IDEA (frontier-based exploration, Yamauchi 1997)
----------------------------------------------------
A FRONTIER is the boundary between known free ground and unobserved cells.
Standing next to one reveals what is behind it. So:

1. Find every cell the UGV can reach (confirmed ground, full clearance) and
   how expensive each is to reach (`astar_planner.dijkstra_all`).
2. Candidate VIEWPOINTS = reachable cells right next to a frontier cell.
   Neighbouring candidates are grouped into clusters (one opening in a wall is
   one cluster, not fifty candidates).
3. Score each cluster's best cell:

       score = cost to get there
             + straight-line distance from there to the goal
             - gain_weight * sqrt(cluster size)

   i.e. prefer viewpoints that are cheap to reach AND point toward the goal AND
   open onto a lot of unknown. (The straight-line term is the same optimistic
   guess A* uses: unknown space might be open.)
4. Drive to the best one. Once there the map has changed; the caller asks
   again and gets the next best viewpoint, until a real route appears.

AVOIDING USELESS MOTION
-----------------------
Two things would make exploration wander, and `ExplorationMemory` handles both:

* DITHERING between two similar viewpoints as scores wobble -> the previous
  target is kept unless a new one is better by `switch_margin`.
* RE-VISITING a viewpoint that turned out to reveal nothing (the frontier
  there never resolves, e.g. unobserved cells that are really solid wall) ->
  once the robot has arrived at a target, that spot is blacklisted.

Pure NumPy/SciPy, no ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label

from nav import astar_planner as ap


@dataclass
class Viewpoint:
    """One candidate place to stand and look."""

    cell: tuple[int, int]
    xy: tuple[float, float]
    score: float  # lower is better
    reach_cost: float
    cluster_size: int


@dataclass
class ExplorationMemory:
    """Remembers the current exploration target and the ones already visited.

    arrive_radius_m: within this distance of the target, it counts as visited.
    blacklist_radius_m: candidates this close to a visited spot are ignored.
    switch_margin: a new target must beat the current one's score by this
        fraction to replace it.
    """

    arrive_radius_m: float = 0.6
    blacklist_radius_m: float = 1.0
    switch_margin: float = 0.2
    target_xy: tuple[float, float] | None = None
    visited: list[tuple[float, float]] = field(default_factory=list)

    def clear(self) -> None:
        self.target_xy = None
        self.visited.clear()

    def note_robot_at(self, robot_xy: tuple[float, float]) -> None:
        """Call on every request: if the robot has reached the target, retire
        it (blacklist it) so the next choice is somewhere new."""
        if self.target_xy is not None and math.dist(robot_xy, self.target_xy) <= self.arrive_radius_m:
            self.visited.append(self.target_xy)
            self.target_xy = None

    def is_blacklisted(self, xy: tuple[float, float]) -> bool:
        return any(math.dist(xy, v) <= self.blacklist_radius_m for v in self.visited)

    def choose(self, candidates: list[Viewpoint]) -> Viewpoint | None:
        """Best non-blacklisted candidate, keeping the current target unless a
        clearly better one exists."""
        live = [c for c in candidates if not self.is_blacklisted(c.xy)]
        if not live:
            self.target_xy = None
            return None
        best = min(live, key=lambda c: (c.score, c.cell))  # cell: deterministic tie-break
        if self.target_xy is not None:
            # the candidate that is (still) the current target, if any
            current = [c for c in live if math.dist(c.xy, self.target_xy) <= self.arrive_radius_m]
            if current:
                cur = min(current, key=lambda c: (c.score, c.cell))
                if best.score >= cur.score * (1.0 - self.switch_margin):
                    best = cur
        self.target_xy = best.xy
        return best


def find_viewpoints(
    walkable_mask: np.ndarray,
    frontier_mask: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *,
    unobserved_mask: np.ndarray | None = None,
    traversability: np.ndarray | None = None,
    force_free_mask: np.ndarray | None = None,
    cost_multiplier: np.ndarray | None = None,
    params: ap.PlannerParams | None = None,
    gain_weight: float = 1.0,
    min_target_dist_m: float = 0.5,
    **overrides,
) -> tuple[list[Viewpoint], "ap._Scene", "ap._Grids", np.ndarray] | None:
    """All candidate viewpoints, scored. Returns (viewpoints, scene, grids,
    parent_map) - the extras let `plan_exploration` rebuild a path without
    searching again - or None if the robot itself is off the map.

    `unobserved_mask`: every cell the map has never observed (the inverse of
    `is_valid`). Needed because the frontier itself is only a one-cell-deep
    ring; the unobserved cells just beyond it must not count as obstacles
    either, or the robot could never stand next to a frontier. If omitted,
    only the frontier ring is exempted (less accurate).

    `min_target_dist_m`: viewpoints closer than this to the robot are skipped
    (we are already there; a plan to the spot we stand on moves nowhere).
    """
    frontier = np.asarray(frontier_mask, dtype=bool)
    walkable = np.asarray(walkable_mask, dtype=bool)
    unknown = frontier if unobserved_mask is None else (np.asarray(unobserved_mask, dtype=bool) | frontier)
    # Unobserved cells are handed to the scene as "uncertain" ONLY so that they
    # are not treated as obstacles when working out clearance - unknown is not
    # a wall. We still only ever drive on `pass_strict` (confirmed ground), so
    # exploration never gambles.
    scene = ap._make_scene(
        walkable, resolution, center_x, center_y, start_xy, traversability,
        unknown & ~walkable, True, None, False, force_free_mask, params, overrides, cost_multiplier,
    )  # fmt: skip
    if not scene.start_in_grid:
        return None
    infl = scene.levels()[0][1]
    grids = scene.grids("optimistic", infl)
    g, parent = ap.dijkstra_all(grids.pass_strict, grids.cost, scene.start)

    # reachable cells touching (8-neighbourhood) a frontier cell
    touching = binary_dilation(frontier, structure=np.ones((3, 3), dtype=bool))
    candidates = touching & np.isfinite(g) & grids.pass_strict
    labels, n_clusters = label(candidates, structure=np.ones((3, 3), dtype=int))

    viewpoints: list[Viewpoint] = []
    goal = np.asarray(goal_xy, dtype=float)
    for k in range(1, n_clusters + 1):
        rows, cols = np.nonzero(labels == k)
        size = rows.size
        best: Viewpoint | None = None
        for r, c in zip(rows.tolist(), cols.tolist()):
            xy = scene.xy_of((r, c))
            if math.dist(xy, start_xy) < min_target_dist_m:
                continue
            reach = float(g[r, c]) * resolution
            score = reach + float(np.hypot(*(np.asarray(xy) - goal))) - gain_weight * math.sqrt(size) * resolution
            vp = Viewpoint(cell=(r, c), xy=xy, score=score, reach_cost=reach, cluster_size=size)
            if best is None or (vp.score, vp.cell) < (best.score, best.cell):
                best = vp
        if best is not None:
            viewpoints.append(best)
    return viewpoints, scene, grids, parent


def find_anomaly_viewpoints(
    walkable_mask: np.ndarray,
    anomaly_mask: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    *,
    traversability: np.ndarray | None = None,
    force_free_mask: np.ndarray | None = None,
    cost_multiplier: np.ndarray | None = None,
    params: ap.PlannerParams | None = None,
    view_radius_m: float = 1.2,
    min_target_dist_m: float = 0.5,
    **overrides,
) -> tuple[list[Viewpoint], "ap._Scene", "ap._Grids", np.ndarray] | None:
    """Places to stand and LOOK AT a suspected passage (an anomaly cluster).

    The anomaly detector flags the mouths of something that looks like a
    tunnel or culvert from above, but the 2.5D map cannot see into it. The
    UGV's own camera can - if it goes there. Even when the flagged cells are
    too incomplete to route THROUGH yet (the detector's output builds up over
    time), they are a strong, specific signal that something passable is
    here, far stronger than a random edge of unobserved space. So these
    viewpoints are offered separately and ranked ahead of plain frontiers.

    One viewpoint per cluster (flagged cells within ~0.2 m of each other are
    one cluster, so the two mouths of one tunnel stay separate clusters but
    the cells of one mouth are not). The viewpoint is the reachable
    confirmed-ground cell within `view_radius_m` of the cluster that is
    CHEAPEST TO REACH from the robot - near enough to see in, no closer than
    needed.
    """
    anomaly = np.asarray(anomaly_mask, dtype=bool)
    scene = ap._make_scene(
        walkable_mask, resolution, center_x, center_y, start_xy, traversability,
        None, False, None, False, force_free_mask, params, overrides, cost_multiplier,
    )  # fmt: skip
    if not scene.start_in_grid:
        return None
    infl = scene.levels()[0][1]
    grids = scene.grids("strict", infl)
    if not anomaly.any():
        # (scipy's distance transform has no meaningful answer when there is
        # nothing to measure distance TO, so do not call it.)
        return [], scene, grids, None
    g, parent = ap.dijkstra_all(grids.pass_strict, grids.cost, scene.start)

    dist, (near_r, near_c) = distance_transform_edt(~anomaly, return_indices=True)
    close_enough = dist * resolution <= view_radius_m
    # cluster ids over the anomaly cells themselves (joined across tiny gaps)
    joined = binary_dilation(anomaly, structure=np.ones((3, 3), dtype=bool), iterations=2)
    cluster_of, n_clusters = label(joined, structure=np.ones((3, 3), dtype=int))
    candidates = grids.pass_strict & np.isfinite(g) & close_enough

    rows, cols = np.nonzero(candidates)
    best_by_cluster: dict[int, Viewpoint] = {}
    for r, c in zip(rows.tolist(), cols.tolist()):
        xy = scene.xy_of((r, c))
        if math.dist(xy, start_xy) < min_target_dist_m:
            continue
        k = int(cluster_of[near_r[r, c], near_c[r, c]])
        reach = float(g[r, c]) * resolution
        size = int(anomaly[cluster_of == k].sum())
        vp = Viewpoint(cell=(r, c), xy=xy, score=reach, reach_cost=reach, cluster_size=size)
        if k not in best_by_cluster or (vp.score, vp.cell) < (best_by_cluster[k].score, best_by_cluster[k].cell):
            best_by_cluster[k] = vp
    return list(best_by_cluster.values()), scene, grids, parent


def _result_for(choice: Viewpoint, scene, grids, parent, kind: str) -> ap.AStarResult:
    """Turn a chosen viewpoint into a path result (shared by both kinds)."""
    cells = ap.path_from_parents(parent, choice.cell)
    p = scene.params
    wp_cells, kinds = ap._postprocess(
        cells, grids.pass_strict, grids.pass_strict, None, None, grids.cost,
        scene.resolution, 0.0, p.merge_gap_m, p.shorten_lookahead, p.shorten_cost_tolerance,
    )  # fmt: skip
    return ap.AStarResult(
        waypoints=[scene.xy_of(c) for c in wp_cells],
        valid=True,
        # Flagged tentative on purpose: this does not reach the goal, so the
        # follower must keep re-planning once the UGV has looked around.
        has_frontier_segments=True,
        has_anomaly_segments=False,
        segment_kinds=kinds,
        cost=choice.reach_cost / scene.resolution,
        inflation_m=scene.levels()[0][1],
        goal_xy=choice.xy,
        mode="strict",
        is_exploration=True,
        exploration_kind=kind,
    )


def plan_exploration(
    walkable_mask: np.ndarray,
    frontier_mask: np.ndarray | None,
    resolution: float,
    center_x: float,
    center_y: float,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    memory: ExplorationMemory,
    *,
    unobserved_mask: np.ndarray | None = None,
    anomaly_mask: np.ndarray | None = None,
    traversability: np.ndarray | None = None,
    force_free_mask: np.ndarray | None = None,
    cost_multiplier: np.ndarray | None = None,
    params: ap.PlannerParams | None = None,
    gain_weight: float = 1.0,
    view_radius_m: float = 1.2,
    frontier_viewpoints: bool = True,
    **overrides,
) -> ap.AStarResult:
    """A path to the best place to go and LOOK, as an `AStarResult` with
    `is_exploration=True` (invalid if there is nothing left worth visiting).

    Order of preference:
      1. a viewpoint on a suspected passage (anomaly cluster) - a specific
         signal that something passable is there;
      2. a viewpoint on a frontier (edge of unobserved space), scored by cost
         to reach + distance to the goal.
    Spots already visited (`memory`) are skipped in both.

    `frontier_viewpoints=False` disables kind 2 (used while the map is still
    being built, when edges of the known map are about to move anyway).
    """
    memory.note_robot_at(start_xy)

    if anomaly_mask is not None and np.asarray(anomaly_mask).any():
        found = find_anomaly_viewpoints(
            walkable_mask, anomaly_mask, resolution, center_x, center_y, start_xy,
            traversability=traversability, force_free_mask=force_free_mask, cost_multiplier=cost_multiplier,
            params=params, view_radius_m=view_radius_m, **overrides,
        )  # fmt: skip
        if found is not None and found[0]:
            choice = memory.choose(found[0])
            if choice is not None:
                return _result_for(choice, found[1], found[2], found[3], "anomaly")

    if frontier_mask is None or not frontier_viewpoints:
        return ap.AStarResult(valid=False)
    found = find_viewpoints(
        walkable_mask, frontier_mask, resolution, center_x, center_y, start_xy, goal_xy,
        unobserved_mask=unobserved_mask, traversability=traversability, force_free_mask=force_free_mask,
        cost_multiplier=cost_multiplier, params=params, gain_weight=gain_weight, **overrides,
    )  # fmt: skip
    if found is None:
        return ap.AStarResult(valid=False)
    choice = memory.choose(found[0])
    if choice is None:
        return ap.AStarResult(valid=False)
    return _result_for(choice, found[1], found[2], found[3], "frontier")
