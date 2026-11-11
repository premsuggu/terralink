"""Wall-following and exit-list exploration strategies, for the offline simulator (tests/nav/explore_sim.py).

The question (user, 2026-10-29): in the live runs the UGV slides along a wall, reaches the next wall, turns back and tries
somewhere else - a lot of useless driving. Would a SYSTEMATIC search do better: follow the walls, notice when you are back
where you started (= no way through here), go back and try the next passage?

Three strategies are implemented here, all as INNER planners (`plan(ctx, start_xy, goal_xy, now) -> AStarResult` and
`last_note`), so they are wrapped in `nav.overhead.GoalIslandPlanner` exactly like ours and see the same map:

1. `BugPlanner(variant="bug1")` - Bug1 (Lumelsky & Stepanov, 1987). The user's idea taken literally:
     motion-to-goal: drive straight at the goal until a wall blocks the way (the HIT point);
     wall-following: keep the wall on the left and follow it ALL the way round, until back at the hit point
                     (= the whole obstacle has been seen), remembering the point of it closest to the goal (the LEAVE point);
     then go back to the leave point (the shorter way round) and drive at the goal again from there.
     If the way to the goal is blocked right at the leave point, the goal cannot be reached.
   Complete (finds a way if there is one) but slow: every obstacle met is circled completely.
2. `BugPlanner(variant="bug2")` - Bug2 (same paper): follow the wall only until back on the straight line start -> goal
   (the "m-line") at a point closer to the goal than the hit point, then leave. Usually much shorter, but can loop.
3. `ExitsPlanner` - the lidar version proposed in chat: the robot's 360 deg lidar already shows a room's walls and its
   openings from the doorway, so it does not need to TOUCH the walls. Each plan lists the EXITS (frontier clusters: places
   where known floor reachable from the robot meets unseen space), scores each by
       known driving distance to it + straight-line distance from it to the goal   (optimistic, goal-directed)
   and COMMITS to the chosen exit until it is reached or disappears (seen to be closed) - no flip-flopping between
   exits as estimates change. A closed exit vanishes from the list by itself, so dead ends are remembered for free.
     order="best": always the best-scoring exit (best-first search);
     order="dfs":  the exits discovered MOST RECENTLY first (best score among them) - a depth-first search: go through
                   the newest opening, and when everything new is closed, fall back to the openings found before
                   (backtracking), which is the second half of the user's idea.
   As soon as a route to the goal over OBSERVED ground exists, it is driven.

Wall following works on the planner's grid: the "wall" is the boundary of the cells the robot's centre may occupy
(known obstacles inflated by robot radius + margin; unseen cells count as free, as for any sensor-based Bug). The tracer is
Moore-neighbour contour tracing (8-connected): from the last blocked neighbour, turn clockwise to the first free one, which
keeps the wall on the robot's left.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage as ndi
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from nav import astar_planner as ap

from explore_policies import FreespacePlanner, _KnownOnly, _cell, _snap, _waypoints, _xy

# 8 neighbours in counter-clockwise order, as (d_row, d_col) with rows = +y: E, NE, N, NW, W, SW, S, SE
N8 = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]


def _cheb(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


class _Bumper:
    """Bump memory: if the robot has not moved since the last plan although that plan had somewhere to go, something the
    sensors cannot see (or a spot too tight for the body) is in the way - remember the point just ahead as an obstacle
    (added to the FreespacePlanner's `extra_blocked`). Our GlobalPlanner has the same idea (its stuck tracker)."""

    def __init__(self, fs, ahead_m: float = 0.3, expire_plans: int = 15):
        self.fs, self.ahead_m, self.expire_plans = fs, ahead_m, expire_plans
        self.prev_start = None
        self.prev_wps = None
        self.bumps: list[tuple[tuple[float, float], int]] = []      # (x, y), plan number
        self.n = 0

    def note(self, start_xy):
        # A bump mark EXPIRES (found in W4: one scrape of B's door frame, remembered for ever, sealed the only door into
        # the goal room - the same failure as a false camera obstacle live). Retrying a door later costs a few metres.
        self.n += 1
        self.bumps = [b for b in self.bumps if self.n - b[1] <= self.expire_plans]
        if self.prev_start is not None and self.prev_wps and math.dist(start_xy, self.prev_start) < 0.05:
            # the point `ahead_m` along the previous plan
            left, here = self.ahead_m, tuple(start_xy)
            for wp in self.prev_wps[1:]:
                d = math.dist(here, wp)
                if d >= left:
                    t = left / d
                    self.bumps.append(((here[0] + (wp[0] - here[0]) * t, here[1] + (wp[1] - here[1]) * t), self.n))
                    break
                left, here = left - d, wp
        self.fs.extra_blocked = [xy for xy, _ in self.bumps]
        self.prev_start = tuple(start_xy)

    def result(self, r):
        self.prev_wps = list(r.waypoints) if r.valid and r.waypoints else None
        return r


class BugPlanner:
    """Bug1 / Bug2 on the planner grid (see the module docstring). Stateful: one object per run."""

    def __init__(self, variant: str = "bug1", step_cells: int = 30, inflation_m: float = 0.35):
        if variant not in ("bug1", "bug2"):
            raise ValueError(variant)
        self.variant, self.step_cells = variant, step_cells
        # inflation 0.35 (not 0.27): a wall follower hugs the very edge of the free cells, and at 0.27 the body does not
        # really fit there everywhere (grid discretisation, walls not yet seen on the far side); real Bug robots keep a margin
        self.fs = FreespacePlanner(unknown_cost=1.0, inflation_m=inflation_m)
        self.bumper = _Bumper(self.fs)
        self.st: dict | None = None          # the algorithm's state (mode, hit point, best point, ...)
        self.trail: list = []                # (cell, state) for every step of the last plan, to resync with the robot
        self.last_note = ""

    # -------------------------------------------------------------- helpers
    def _free(self, c):
        return 0 <= c[0] < self.passable.shape[0] and 0 <= c[1] < self.passable.shape[1] and bool(self.passable[c])

    def _trace(self, p, bi, cw=True):
        """One contour-following step from cell `p` whose neighbour N8[bi] is blocked. cw=True keeps the wall on the LEFT,
        cw=False on the right (used to go back the shorter way). Returns (next cell, its blocked-neighbour index) or None."""
        for k in range(1, 9):
            i = (bi - k) % 8 if cw else (bi + k) % 8
            q = (p[0] + N8[i][0], p[1] + N8[i][1])
            if self._free(q):
                j = (i + 1) % 8 if cw else (i - 1) % 8          # the neighbour checked just before q: blocked
                prev = (p[0] + N8[j][0], p[1] + N8[j][1])
                return q, N8.index((prev[0] - q[0], prev[1] - q[1]))
        return None

    def _reacquire(self, p, prefer):
        """Index of a blocked neighbour of `p` to trace from (nearest to `prefer` around the ring), or None (no wall here)."""
        blocked = [i for i in range(8) if not self._free((p[0] + N8[i][0], p[1] + N8[i][1]))]
        if not blocked:
            return None
        return min(blocked, key=lambda i: min((i - prefer) % 8, (prefer - i) % 8))

    def _nearest_edge(self, p, max_cells):
        """The nearest free cell with a blocked 8-neighbour (a cell on the wall's edge) within `max_cells`, or None."""
        r0, c0 = max(0, p[0] - max_cells), max(0, p[1] - max_cells)
        win = self.passable[r0:p[0] + max_cells + 1, c0:p[1] + max_cells + 1]
        edge = win & ~ndi.binary_erosion(win, structure=np.ones((3, 3), bool), border_value=1)
        ys, xs = np.nonzero(edge)
        if len(ys) == 0:
            return None
        k = int(np.argmin((ys + r0 - p[0]) ** 2 + (xs + c0 - p[1]) ** 2))
        return (int(ys[k] + r0), int(xs[k] + c0))

    @staticmethod
    def _toward(p, g):
        return int(round(math.atan2(g[0] - p[0], g[1] - p[1]) / (math.pi / 4))) % 8

    def _on_mline(self, p, g):
        a, b = np.array(self.st["mstart"], float), np.array(g, float)
        ab, ap_ = b - a, np.array(p, float) - a
        n = np.linalg.norm(ab)
        if n < 1e-9:
            return True
        return abs(ab[0] * ap_[1] - ab[1] * ap_[0]) / n <= 1.0 and 0.0 <= np.dot(ap_, ab) / n ** 2 <= 1.0

    # ----------------------------------------------------------------- plan
    def plan(self, ctx, start_xy, goal_xy, now=None):
        self.bumper.note(start_xy)
        return self.bumper.result(self._plan(ctx, start_xy, goal_xy))

    def _plan(self, ctx, start_xy, goal_xy):
        self.passable, _, _, s = self.fs.grids(ctx, start_xy)
        g = _cell(goal_xy, ctx)
        if self.st is None:
            self.st = {"mode": "goal", "mstart": s, "bi": 0, "hit": None, "hit_d": math.inf, "best": None, "best_d": math.inf,
                       "best_k": 0, "traced": 0, "loop_len": 0, "cw": True, "last_leave": None}
        else:
            # Resync with where the robot really is (the simulator drives only ~2 m of each plan, or stops at a new wall).
            near = [(_cheb(c, s), k) for k, (c, _) in enumerate(self.trail)]
            if near and min(near)[0] <= 2:
                k = min(near)[1]
                self.st = dict(self.trail[k][1])
        p = self.st.get("p", s) if _cheb(self.st.get("p", s), s) <= 2 else s
        if not self._free(p):
            p = _snap(self.passable, p, 6)
            if p is None:
                self.last_note = f"{self.variant}: boxed in"
                return ap.AStarResult(valid=False)
        st = self.st
        cells, self.trail = [p], []
        if st["mode"] != "goal" and self._free((p[0] + N8[st["bi"]][0], p[1] + N8[st["bi"]][1])):
            bi = self._reacquire(p, st["bi"])          # the wall moved (map update) or the robot drifted off it
            if bi is None:
                # Lost contact with the wall (the robot was stopped off the traced line, or a bump changed the edge): step
                # to the nearest wall cell and keep following - the hit point and the loop bookkeeping are kept.
                q = self._nearest_edge(p, 15)
                if q is None:
                    st["mode"] = "goal"
                else:
                    p = q
                    cells.append(p)
                    st["bi"] = self._reacquire(p, st["bi"])
        for _ in range(self.step_cells):
            if _cheb(p, g) <= 2:
                break
            if st["mode"] == "goal":
                d = self._toward(p, g)
                q = (p[0] + N8[d][0], p[1] + N8[d][1])
                if self._free(q):
                    p = q
                else:                                     # HIT: start following the wall, wall on the left
                    if self.variant == "bug1" and st["last_leave"] is not None and _cheb(p, st["last_leave"]) <= 2:
                        self.last_note = "bug1: blocked again at the leave point - goal unreachable"
                        return ap.AStarResult(valid=False)
                    dg = math.dist(p, g)
                    st.update(mode="follow", bi=d, hit=p, hit_d=dg, best=p, best_d=dg, best_k=0, traced=0, cw=True)
                    continue
            else:
                step = self._trace(p, st["bi"], st["cw"])
                if step is None:
                    self.last_note = f"{self.variant}: no free neighbour"
                    return ap.AStarResult(valid=False)
                p, st["bi"] = step
                st["traced"] += 1
                dg = math.dist(p, g)
                if st["mode"] == "follow":
                    if self.variant == "bug1":
                        if dg < st["best_d"] - 1e-9:
                            st.update(best=p, best_d=dg, best_k=st["traced"])
                        if st["traced"] > 20 and _cheb(p, st["hit"]) <= 2:      # back at the hit point: obstacle circled
                            st["loop_len"] = st["traced"]
                            back = st["loop_len"] - st["best_k"]
                            st.update(mode="to_best", traced=0, cw=st["best_k"] <= back)   # the shorter way to the best point
                            if not st["cw"]:
                                st["bi"] = self._reacquire(p, st["bi"]) or st["bi"]
                    else:
                        d = self._toward(p, g)
                        if (self._on_mline(p, g) and dg < st["hit_d"] - 1.0
                                and self._free((p[0] + N8[d][0], p[1] + N8[d][1]))):
                            st["mode"] = "goal"                                # Bug2: back on the m-line, closer: leave
                        elif st["traced"] > 20 and _cheb(p, st["hit"]) <= 1:
                            self.last_note = "bug2: back at the hit point - goal unreachable"
                            return ap.AStarResult(valid=False)
                elif st["mode"] == "to_best" and _cheb(p, st["best"]) <= 1:
                    st.update(mode="goal", last_leave=p)                       # Bug1: leave from the closest point
            cells.append(p)
            st["p"] = p
            self.trail.append((p, dict(st)))
        if len(cells) < 2:
            if _cheb(p, g) <= 2:
                return ap.AStarResult(waypoints=[start_xy, goal_xy], valid=True, goal_xy=goal_xy, mode=self.variant)
            self.last_note = f"{self.variant}: could not move"
            return ap.AStarResult(valid=False)
        self.last_note = f"{self.variant}: {st['mode']}"
        wps = _waypoints(cells, ctx, every=3)
        if _cheb(cells[-1], g) <= 2:
            wps.append(tuple(goal_xy))
        return ap.AStarResult(waypoints=wps, valid=True, has_frontier_segments=True, goal_xy=goal_xy, mode=self.variant)


class ExitsPlanner:
    """Goal-directed exit list with commitment and backtracking (see the module docstring). One object per run."""

    # min_exit_cells = 1: the inflation already guarantees the robot fits, and a door that is only just wide enough shows only
    # 1-2 passable cells (found in W5: with 3 the 0.9 m door was never listed and the planner wrongly gave up)
    def __init__(self, order: str = "best", min_exit_cells: int = 1, reach_m: float = 0.8, alive_m: float = 1.0,
                 inflation_m: float = 0.27):
        if order not in ("best", "dfs"):
            raise ValueError(order)
        self.order, self.min_exit_cells, self.reach_m, self.alive_m = order, min_exit_cells, reach_m, alive_m
        self.fs = FreespacePlanner(unknown_cost=1.0, inflation_m=inflation_m)
        self.bumper = _Bumper(self.fs)
        self.target: tuple[float, float] | None = None
        self.seen: list[list] = []          # [x, y, first-seen plan number] of every exit ever listed (for "dfs")
        self.n_plans = 0
        self.last_note = ""

    def exits(self, ctx, start_xy, goal_xy):
        """[(score, (x, y), first_seen)] of every exit reachable from the robot over known ground."""
        passable, _, unknown, s = self.fs.grids(ctx, start_xy)
        known_free = passable & ~unknown
        if not known_free[s]:
            s2 = _snap(known_free, s, 6)
            if s2 is None:
                return []
            s = s2
        lab, _ = ndi.label(known_free, structure=np.ones((3, 3), int))
        reach = lab == lab[s]
        # geodesic (driving) distance from the robot over the reachable known floor: Dijkstra on the 8-connected grid
        idx = -np.ones(reach.shape, int)
        rr, cc = np.nonzero(reach)
        idx[rr, cc] = np.arange(len(rr))
        rows, cols, w = [], [], []
        for dr, dc in N8[:4]:                                     # each undirected edge once
            r2, c2 = rr + dr, cc + dc
            ok = (r2 >= 0) & (r2 < reach.shape[0]) & (c2 >= 0) & (c2 < reach.shape[1])
            ok[ok] &= reach[r2[ok], c2[ok]]
            rows.append(idx[rr[ok], cc[ok]])
            cols.append(idx[r2[ok], c2[ok]])
            w.append(np.full(ok.sum(), math.hypot(dr, dc) * ctx.resolution))
        n = len(rr)
        G = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
        dist = np.full(reach.shape, np.inf)
        dist[rr, cc] = dijkstra(G, directed=False, indices=idx[s])
        # exit cells: reachable known floor touching unseen space - but not within reach of the robot: the lidar is blind
        # closer than 0.3 m, so there is always a ring of "unseen" cells round the robot itself (found: with no UAV map the
        # robot chose that ring as its exit, "reached" it at once, chose it again, and never moved)
        front = reach & ndi.binary_dilation(unknown) & (dist > self.reach_m + 0.2)
        flab, nf = ndi.label(front, structure=np.ones((3, 3), int))
        out = []
        for k in range(1, nf + 1):
            fr, fc = np.nonzero(flab == k)
            if len(fr) < self.min_exit_cells:
                continue
            xs = ctx.center_x + (fc - reach.shape[1] / 2) * ctx.resolution
            ys = ctx.center_y + (fr - reach.shape[0] / 2) * ctx.resolution
            score = dist[fr, fc] + np.hypot(xs - goal_xy[0], ys - goal_xy[1])
            j = int(np.argmin(score))
            out.append([float(score[j]), (float(xs[j]), float(ys[j]))])
        # remember when each exit was first listed (an exit moves as the robot approaches it: match within 2 m)
        for e in out:
            m = min(self.seen, key=lambda q: math.dist(q[:2], e[1]), default=None)
            if m is not None and math.dist(m[:2], e[1]) < 2.0:
                m[0], m[1] = e[1]
                e.append(m[2])
            else:
                self.seen.append([e[1][0], e[1][1], self.n_plans])
                e.append(self.n_plans)
        return out

    def _route(self, ctx, start_xy, xy, kind):
        found, unknown = self.fs.route(ctx, start_xy, xy)
        if found is None:
            return None
        cells, cost = found
        return ap.AStarResult(waypoints=_waypoints(cells, ctx), valid=True, has_frontier_segments=kind != "goal",
                              cost=cost, goal_xy=_xy(cells[-1], ctx), mode=kind,
                              is_exploration=kind != "goal", exploration_kind=None if kind == "goal" else "frontier")

    def plan(self, ctx, start_xy, goal_xy, now=None):
        self.bumper.note(start_xy)
        return self.bumper.result(self._plan(ctx, start_xy, goal_xy))

    def _order(self, ex, ctx, start_xy, goal_xy):
        """The exits in the order to try them. best: lowest score first. dfs: newest first, lowest score among equally new
        ones (older ones = backtracking)."""
        key = (lambda e: (-e[2], e[0])) if self.order == "dfs" else (lambda e: e[0])
        return sorted(ex, key=key)

    def _plan(self, ctx, start_xy, goal_xy):
        self.n_plans += 1
        # 1. a route over OBSERVED ground to the goal: drive it
        unknown = np.asarray(ctx.unobserved, bool)
        found, _ = self.fs.route(_KnownOnly(ctx), start_xy, goal_xy)
        if found is not None and not any(unknown[c] for c in found[0]):
            self.target = None
            self.last_note = f"exits-{self.order}: route over observed ground to the goal"
            return ap.AStarResult(waypoints=_waypoints(found[0], ctx), valid=True, cost=found[1], goal_xy=goal_xy, mode="known")
        ex = self.exits(ctx, start_xy, goal_xy)
        if not ex:
            self.target = None
            self.last_note = f"exits-{self.order}: no exits left - the goal cannot be reached"
            return ap.AStarResult(valid=False)
        # 2. keep the committed exit while it is still open and not reached
        if self.target is not None:
            alive = min(math.dist(self.target, e[1]) for e in ex) <= self.alive_m
            reached = math.dist(self.target, start_xy) <= self.reach_m
            if alive and not reached:
                r = self._route(ctx, start_xy, self.target, "exit")
                if r is not None:
                    self.last_note = f"exits-{self.order}: committed exit ({self.target[0]:.1f},{self.target[1]:.1f})"
                    return r
        # 3. choose a new exit (in the order `_order` gives; the first one A* can route to wins)
        for score, xy, _ in self._order(ex, ctx, start_xy, goal_xy):
            r = self._route(ctx, start_xy, xy, "exit")
            if r is not None:
                self.target = xy
                self.last_note = f"exits-{self.order}: new exit ({xy[0]:.1f},{xy[1]:.1f}) score {score:.1f} m of {len(ex)}"
                return r
        self.target = None
        self.last_note = f"exits-{self.order}: no exit can be routed to"
        return ap.AStarResult(valid=False)
