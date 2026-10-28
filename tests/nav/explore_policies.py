"""Ports of EXISTING exploration / unknown-space navigation strategies, for comparison in the offline simulator.

These are not our planner. Each class re-implements, in a few dozen lines, the DECISION RULE of a published, widely used
system, so it can be compared with ours (`nav.global_planner` + `nav.exploration`) under identical sensing, motion and map
(`tests/nav/explore_sim.py`, driven by `src/nav/scripts/eval_exploration.py`). The point is to find out whether an
existing solution can be adopted instead of growing our own - see docs/work-docs/nav/step12_exploration.md.

Each class here is an INNER planner with `plan(ctx, start_xy, goal_xy, now) -> AStarResult` and `last_note`. They are
wrapped in `nav.overhead.GoalIslandPlanner`, exactly like ours, so all candidates see the same map: the UAV map, with a
roofed island that holds the goal (or the robot) hidden as "unknown" until the UGV's lidar has seen it.

1. `FreespacePlanner` - the FREESPACE ASSUMPTION (Koenig & Likhachev, D* Lite, 2002; Nav2's NavFn/Smac with
   `allow_unknown: true`): plan straight to the goal, treating cells nobody has observed as free (optionally at a higher
   cost), re-plan as the lidar reveals walls. `unknown_cost=1` is pure D* Lite; ~5 is what NavFn does (an unknown cell
   costs 253 against a free cell's 50).
2. `ExploreLitePlanner` - m-explore / explore_lite (Horner 2016; ROS 2 port `robo-friends/m-explore-ros2`): frontier
   clusters reachable from the robot, cost = potential_scale * distance * res - gain_scale * size * res (its own formula,
   including the extra `* res` on the distance), go to the best cluster's centroid with Nav2 (= `FreespacePlanner` with
   NavFn's unknown cost), blacklist a goal Nav2 cannot reach or one with no progress for `progress_timeout`.
   explore_lite has no goal of its own: here it explores until a route to the goal over OBSERVED ground exists, then
   drives that route (the fairest goal-directed use of it).
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage as ndi

from nav import astar_planner as ap


def _cell(xy, ctx):
    n = ctx.walkable.shape[0]
    return (int(round((xy[1] - ctx.center_y) / ctx.resolution + n / 2)),
            int(round((xy[0] - ctx.center_x) / ctx.resolution + n / 2)))


def _xy(cell, ctx):
    n = ctx.walkable.shape[0]
    return (ctx.center_x + (cell[1] - n / 2) * ctx.resolution, ctx.center_y + (cell[0] - n / 2) * ctx.resolution)


def _snap(passable, cell, max_cells):
    """`cell` if passable, else the nearest passable cell within `max_cells` (None if there is none)."""
    if 0 <= cell[0] < passable.shape[0] and 0 <= cell[1] < passable.shape[1] and passable[cell]:
        return cell
    r = int(max_cells)
    r0, c0 = max(0, cell[0] - r), max(0, cell[1] - r)
    win = passable[r0:cell[0] + r + 1, c0:cell[1] + r + 1]
    ys, xs = np.nonzero(win)
    if len(ys) == 0:
        return None
    k = int(np.argmin((ys + r0 - cell[0]) ** 2 + (xs + c0 - cell[1]) ** 2))
    return (int(ys[k] + r0), int(xs[k] + c0))


def _waypoints(cells, ctx, every=5):
    """Cell path -> waypoints every `every` cells (0.5 m at 0.1 m/cell) plus the last one."""
    keep = cells[::every] + ([cells[-1]] if (len(cells) - 1) % every else [])
    return [_xy(c, ctx) for c in keep]


class FreespacePlanner:
    """Plan to the goal as if unobserved cells were free; known obstacles are inflated by the robot radius.

    unknown_cost: cost multiplier of an unobserved cell (1 = pure freespace assumption, ~5 = Nav2 NavFn).
    """

    def __init__(self, unknown_cost: float = 1.0, inflation_m: float = 0.27, footprint_m: float = 0.3, goal_snap_m: float = 0.4):
        self.unknown_cost, self.inflation_m, self.footprint_m, self.goal_snap_m = unknown_cost, inflation_m, footprint_m, goal_snap_m
        self.last_note = ""

    def grids(self, ctx, start_xy):
        unknown = np.asarray(ctx.unobserved, bool)
        walkable = np.asarray(ctx.walkable, bool)
        # Our A*'s scene clears the robot's own footprint and its "self-pollution" blob (the UAV scan saw the parked UGV as
        # an obstacle); every candidate gets the same start, so reuse it rather than copy it.
        scene = ap._make_scene(walkable, ctx.resolution, ctx.center_x, ctx.center_y, start_xy, ctx.traversability,
                               None, False, None, False, ctx.force_free, ap.PlannerParams(), {}, None)
        known_blocked = ~scene.base_walkable & ~unknown
        clear = ap.clearance_m(known_blocked, ctx.resolution)
        passable = (clear >= self.inflation_m - 1e-6) | scene.force_free
        s = _cell(start_xy, ctx)
        cost = ap.build_cost_grid(clear, self.inflation_m, ctx.traversability, None, unknown,
                                  1.0, 0.5, 1.0, 1.0, self.unknown_cost)
        return passable, cost, unknown, s

    def route(self, ctx, start_xy, target_xy):
        """(cells, cost) of the cheapest route to `target_xy`, or None."""
        passable, cost, unknown, s = self.grids(ctx, start_xy)
        g = _snap(passable, _cell(target_xy, ctx), self.goal_snap_m / ctx.resolution)
        if g is None:
            return None, unknown
        found = ap._astar(passable, cost, s, g)
        return found, unknown

    def plan(self, ctx, start_xy, goal_xy, now=None):
        found, unknown = self.route(ctx, start_xy, goal_xy)
        if found is None:
            self.last_note = "freespace: no route even through unknown space"
            return ap.AStarResult(valid=False)
        cells, cost = found
        through_unknown = any(unknown[c] for c in cells)
        self.last_note = "freespace: route " + ("through unknown space" if through_unknown else "over observed ground")
        return ap.AStarResult(waypoints=_waypoints(cells, ctx), valid=True, has_frontier_segments=through_unknown,
                              cost=cost, goal_xy=_xy(cells[-1], ctx), mode="freespace")


class ExploreLitePlanner:
    """explore_lite's frontier choice (defaults from its `params.yaml`), driven by a NavFn-like planner."""

    def __init__(self, potential_scale=3.0, gain_scale=1.0, min_frontier_size=0.75, progress_timeout=30.0,
                 blacklist_cells=5, nav_unknown_cost=5.0):
        self.potential_scale, self.gain_scale, self.min_frontier_size = potential_scale, gain_scale, min_frontier_size
        self.progress_timeout, self.blacklist_cells = progress_timeout, blacklist_cells
        self.nav = FreespacePlanner(unknown_cost=nav_unknown_cost)        # Nav2 with allow_unknown, as explore_lite uses it
        self.known = FreespacePlanner(unknown_cost=1.0)
        self.blacklist: list[tuple[float, float]] = []
        self.prev_goal = None
        self.prev_distance = math.inf
        self.last_progress = 0.0
        self.last_note = ""

    def frontiers(self, ctx, start_xy):
        """explore_lite's frontier search: unknown cells 4-adjacent to free space reachable from the robot, grouped 8-connected."""
        res = ctx.resolution
        walkable = np.asarray(ctx.walkable, bool)
        unknown = np.asarray(ctx.unobserved, bool)
        s = _cell(start_xy, ctx)
        lab, _ = ndi.label(walkable)                                     # 4-connected free space
        start_lab = lab[s] if lab[s] else lab[_snap(walkable, s, 5) or s]
        if not start_lab:
            return []
        free = lab == start_lab
        frontier = unknown & ndi.binary_dilation(free)                    # 4-neighbourhood (scipy's default structure)
        flab, n = ndi.label(frontier, structure=np.ones((3, 3), int))
        out = []
        for k in range(1, n + 1):
            rows, cols = np.nonzero(flab == k)
            if len(rows) * res < self.min_frontier_size:
                continue
            xs = ctx.center_x + (cols - walkable.shape[1] / 2) * res
            ys = ctx.center_y + (rows - walkable.shape[0] / 2) * res
            centroid = (float(xs.mean()), float(ys.mean()))
            min_d = float(np.min(np.hypot(xs - start_xy[0], ys - start_xy[1])))
            cost = self.potential_scale * min_d * res - self.gain_scale * len(rows) * res
            out.append((cost, centroid, min_d))
        return sorted(out)

    def _blacklisted(self, xy, res):
        tol = self.blacklist_cells * res
        return any(abs(xy[0] - b[0]) < tol and abs(xy[1] - b[1]) < tol for b in self.blacklist)

    def plan(self, ctx, start_xy, goal_xy, now=None):
        now = 0.0 if now is None else now
        # 1. a route to the goal over OBSERVED ground only? Then exploration is over: drive it.
        unknown = np.asarray(ctx.unobserved, bool)
        found, _ = self.known.route(_KnownOnly(ctx), start_xy, goal_xy)
        if found is not None and not any(unknown[c] for c in found[0]):
            self.last_note = "explore_lite: a route over observed ground exists - driving to the goal"
            return ap.AStarResult(waypoints=_waypoints(found[0], ctx), valid=True, cost=found[1], goal_xy=goal_xy, mode="known")
        # 2. the best non-blacklisted frontier Nav2 can plan to
        for cost, centroid, min_d in self.frontiers(ctx, start_xy):
            if self._blacklisted(centroid, ctx.resolution):
                continue
            same_goal = self.prev_goal is not None and math.dist(self.prev_goal, centroid) < 1e-6
            if not same_goal or self.prev_distance > min_d:
                self.last_progress, self.prev_distance = now, min_d
            if same_goal and now - self.last_progress > self.progress_timeout:
                self.blacklist.append(centroid)                            # no progress: give up on this frontier
                continue
            route, _ = self.nav.route(ctx, start_xy, centroid)
            if route is None:
                self.blacklist.append(centroid)                            # Nav2 would abort: blacklist (explore_lite does)
                continue
            self.prev_goal = centroid
            self.last_note = f"explore_lite: frontier at ({centroid[0]:.1f},{centroid[1]:.1f}) cost {cost:.1f}"
            return ap.AStarResult(waypoints=_waypoints(route[0], ctx), valid=True, has_frontier_segments=True,
                                  cost=route[1], goal_xy=centroid, mode="frontier", is_exploration=True,
                                  exploration_kind="frontier")
        self.last_note = "explore_lite: no frontiers left"
        return ap.AStarResult(valid=False)


class IslandFreespacePlanner:
    """Freespace ONLY where it is needed: inside a hidden island (the goal or the robot under a roof) plan with the freespace
    assumption; everywhere else use our GlobalPlanner unchanged. Assuming unobserved cells are free is fine under a roof (the
    UAV saw nothing there at all) but risky in a UAV gap in the open, which may be a pit the 2D lidar cannot see.

    Inner planner of a `GoalIslandPlanner`; `outer` is that planner (its `last_zone` is set before the inner plan is called).
    """

    def __init__(self, global_planner, freespace: FreespacePlanner | None = None):
        self.gp, self.fs, self.outer = global_planner, freespace or FreespacePlanner(), None
        self.last_note = ""

    def plan(self, ctx, start_xy, goal_xy, now=None):
        use = self.fs if self.outer is not None and self.outer.last_zone is not None else self.gp
        r = use.plan(ctx, start_xy, goal_xy, now=now)
        self.last_note = use.last_note
        return r


def island_freespace(params=None, **gp_kwargs):
    """A GoalIslandPlanner whose inner planner is `IslandFreespacePlanner` (GlobalPlanner configured like ExploreSim's)."""
    from nav.global_planner import GlobalPlanner
    from nav.overhead import GoalIslandPlanner
    gp = GlobalPlanner(params or ap.PlannerParams(), enable_memory=True, enable_exploration=True, stuck_feedback=True,
                       viewpoint_standoff_m=0.8, viewpoint_block_m=3.0, **gp_kwargs)
    inner = IslandFreespacePlanner(gp)
    outer = GoalIslandPlanner(inner)
    inner.outer = outer
    return outer


class _KnownOnly:
    """A view of a MapContext in which nothing is unobserved (unknown cells count as obstacles)."""

    def __init__(self, ctx):
        self.__dict__.update(ctx.__dict__)
        self.unobserved = np.zeros_like(np.asarray(ctx.unobserved, bool))
