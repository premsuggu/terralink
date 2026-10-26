"""Offline simulator for exploration logic: no Gazebo, runs in seconds.

The planner is given the map a real run has (the UAV scan: `<world>_uavscan.npz`, or any map), and a
simulated UGV that SEES the true floor (a ground-truth map) only near itself, with line of sight blocked by
walls. It then does what the live stack does: plan -> drive a couple of metres -> look -> re-plan, using the
same `GlobalPlanner` / `GoalIslandPlanner` the ROS node uses.

What it is for: iterating on exploration quickly (a live W4 mission is ~40 min of wall time). What it is NOT:
a proof the live stack works - the sensor model is simpler than the real camera, motion is kinematic, and
there is no Nav2. Live runs stay the final check.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage as ndi

from nav.floor_map_node import integrate_scan, to_layers
from nav.astar_planner import PlannerParams
from nav.global_planner import GlobalPlanner
from nav.overhead import GoalIslandPlanner
from nav.replay import MapSnapshot, _context, build_masks

LETHAL_LEVEL = 0.15


def resample(layer: np.ndarray, src: MapSnapshot, dst: MapSnapshot, fill: float) -> np.ndarray:
    """`layer` (on `src`'s grid) sampled at the centre of every `dst` cell (nearest neighbour)."""
    n_r, n_c = dst.shape
    rows, cols = np.mgrid[0:n_r, 0:n_c]
    x = dst.center_x + (cols - n_c / 2) * dst.resolution
    y = dst.center_y + (rows - n_r / 2) * dst.resolution
    sc = np.round((x - src.center_x) / src.resolution + src.shape[1] / 2).astype(int)
    sr = np.round((y - src.center_y) / src.resolution + src.shape[0] / 2).astype(int)
    ok = (sr >= 0) & (sr < src.shape[0]) & (sc >= 0) & (sc < src.shape[1])
    out = np.full((n_r, n_c), fill, dtype=layer.dtype)
    out[ok] = layer[sr[ok], sc[ok]]
    return out


@dataclass
class SimResult:
    reached: bool
    travelled_m: float
    plans: int
    seconds: float
    path: list = field(default_factory=list)
    notes: list = field(default_factory=list)
    revealed_frac: float = 0.0
    targets: list = field(default_factory=list)      # (plan index, x, y, kind) of every plan's final waypoint


class ExploreSim:
    def __init__(self, base: MapSnapshot, truth: MapSnapshot, start, goal, *, robot_radius=0.22,
                 cone_range_m=12.0, cone_half_deg=180.0, around_m=12.0, speed=0.5, replan_every_m=2.0,
                 frontier_min_width_m=0.44, use_island=True, standoff_m=0.8, block_m=3.0, plan_overhead_s=10.0, waive_m=0.5, body_radius=0.27):
        self.base, self.start, self.goal = base, tuple(start), tuple(goal)
        self.res = base.resolution
        self.t_trav = resample(truth.layers["traversability"], truth, base, 0.0)           # outside the truth map = wall
        self.t_elev = resample(truth.layers["elevation"], truth, base, 0.0)
        self.lethal = self.t_trav < LETHAL_LEVEL
        self.clear = ndi.distance_transform_edt(~self.lethal) * self.res                    # distance to the nearest wall
        self.revealed = np.zeros(base.shape, bool)           # = the floor map's `observed`
        self.logodds = np.zeros(base.shape, np.float32)        # the floor map's lidar log-odds (nav/floor_map_node.py)
        self.robot_radius = body_radius                         # Nav2 will not drive a gap narrower than ~2 x this (0.22 radius + inflation)
        self.cone_range, self.cone_half = cone_range_m, math.radians(cone_half_deg)
        self.around, self.speed, self.replan_every = around_m, speed, replan_every_m
        self.plan_overhead_s = plan_overhead_s                  # live: each new goal costs ~10 s (stop, turn, accelerate; measured in W4)
        self.frontier_min_width_m = frontier_min_width_m
        self.gp = GlobalPlanner(PlannerParams(uncertain_waive_m=waive_m), enable_memory=True, enable_exploration=True, stuck_feedback=True, viewpoint_standoff_m=standoff_m, viewpoint_block_m=block_m)   # as the live mission runs it
        self.planner = GoalIslandPlanner(self.gp) if use_island else None

    # ----------------------------------------------------------------- geometry
    def cell(self, x, y):
        return (int(round((y - self.base.center_y) / self.res + self.base.shape[0] / 2)),
                int(round((x - self.base.center_x) / self.res + self.base.shape[1] / 2)))

    def look(self, x, y, yaw):
        """One lidar scan from (x, y): 360 rays (1 deg), 0.3-12 m, stopped by walls, integrated with the SAME code the live
        `floor_map_node` runs (`integrate_scan`). The depth camera is not used: it is tilted down and sees only a
        0.1-0.9 m strip of floor ahead (models/nav_ugv)."""
        n_r, n_c = self.base.shape
        angs = np.radians(np.arange(0, 360, 1.0)) - math.pi               # LaserScan angle_min = -pi, increment 1 deg
        ranges = np.full(len(angs), 13.0)                                # above range_max = "no return"
        # A wall closer than the lidar's 0.3 m minimum range gives NO return (the sensor is blind there): keep the 13.0 marker.
        alive = np.ones(len(angs), bool)
        bearing = yaw + angs
        for s_ in np.arange(0.3, 12.0, self.res * 0.5):
            r = np.round((y + s_ * np.sin(bearing) - self.base.center_y) / self.res + n_r / 2).astype(int)
            c = np.round((x + s_ * np.cos(bearing) - self.base.center_x) / self.res + n_c / 2).astype(int)
            inside = (r >= 0) & (r < n_r) & (c >= 0) & (c < n_c)
            blocked = np.zeros(len(angs), bool)
            blocked[inside] = self.lethal[r[inside], c[inside]]
            blocked |= ~inside
            newly = alive & blocked
            ranges[newly] = s_ if s_ > 0.3 + 1e-9 else -np.inf       # blocked at the very first sample = inside the 0.3 m blind zone
            alive &= ~blocked
            if not alive.any():
                break
        integrate_scan(self.logodds, self.revealed, self.res, self.base.center_x, self.base.center_y, (x, y), yaw, ranges,
                       -math.pi, math.radians(1.0), 0.3, 12.0)

    def floor_layers(self):
        return to_layers(self.logodds, self.revealed, 0.0)

    def make_ctx(self, layers, use_anomaly=True):
        snap = MapSnapshot(layers=layers, resolution=self.res, center_x=self.base.center_x, center_y=self.base.center_y)
        return _context(snap, build_masks(snap, use_frontier=True, use_anomaly=use_anomaly,
                                          frontier_min_width_m=self.frontier_min_width_m))

    def _same_spot(self, res, plans) -> bool:
        """True when the robot has not moved at all while the last 8 plans were made."""
        self._plan_pos = getattr(self, '_plan_pos', [])
        self._plan_pos.append(res.path[-1])
        last = self._plan_pos[-8:]
        return len(last) == 8 and max(math.dist(last[0], p) for p in last) < 0.05

    @staticmethod
    def _ahead(waypoints, x, y):
        """The route's legs from the one nearest the robot on (like the live follower, which skips waypoints it has already
        passed instead of driving back to them)."""
        pts = np.asarray(waypoints, float)
        best, k = 1e18, 0
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            t = 0.0 if not ab.any() else float(np.clip(np.dot([x, y] - a, ab) / np.dot(ab, ab), 0, 1))
            d = math.dist((x, y), a + t * ab)
            if d < best - 1e-9:
                best, k = d, i
        return list(zip(map(tuple, pts[k:-1]), map(tuple, pts[k + 1:])))

    # ---------------------------------------------------------------- the loop
    def run(self, max_travel_m=600.0, max_plans=400, verbose=False) -> SimResult:
        x, y = self.start
        yaw, travelled, plans, t = 0.0, 0.0, 0, 0.0
        res = SimResult(False, 0.0, 0, 0.0, path=[(x, y)])
        while plans < max_plans and travelled < max_travel_m:
            if math.dist((x, y), self.goal) < 0.6:
                res.reached = True
                break
            self.look(x, y, yaw)
            plans += 1
            layers = {k: np.array(v, copy=True) for k, v in self.base.layers.items()}
            if self.planner is not None:
                r = self.planner.plan(layers, self.floor_layers(), self.make_ctx, (x, y), self.goal,
                                      self.res, self.base.center_x, self.base.center_y, now=t)
                note = self.planner.note + "; " + self.gp.last_note
            else:
                r = self.gp.plan(self.make_ctx(layers), (x, y), self.goal, now=t)
                note = self.gp.last_note
            res.notes.append(note)
            self.last_plan = r
            t += self.plan_overhead_s
            if r.valid and r.waypoints:
                res.targets.append((plans, round(r.waypoints[-1][0], 1), round(r.waypoints[-1][1], 1),
                                    str(r.exploration_kind) if getattr(r, 'is_exploration', False) else 'goal'))
            if verbose:
                print(f"plan {plans:3d} at ({x:6.1f},{y:6.1f}) travelled {travelled:6.1f}: valid={r.valid} "
                      f"{'explore:' + str(r.exploration_kind) if getattr(r, 'is_exploration', False) else ''} {note[:90]}")
            if plans > 10 and self._same_spot(res, plans):
                res.notes.append('STUCK: no movement over the last plans')
                break
            if not r.valid or len(r.waypoints) < 2:
                t += 5.0                                    # nothing to do: wait for the map to change (nothing will)
                if plans > 5 and not r.valid and len(set(res.notes[-5:])) == 1:
                    break
                continue
            moved = 0.0
            for (ax, ay), (bx, by) in self._ahead(r.waypoints, x, y):
                yaw = math.atan2(by - ay, bx - ax)
                blocked = False
                # walk straight toward (bx, by) in 0.25 m steps
                dist = math.dist((x, y), (bx, by))
                while dist > 0.05 and moved < self.replan_every:
                    step = min(0.25, dist)
                    nx, ny = x + (bx - x) / dist * step, y + (by - y) / dist * step
                    rr, cc = self.cell(nx, ny)
                    if self.clear[rr, cc] < self.robot_radius:
                        blocked = True
                        break
                    x, y, moved, travelled, t = nx, ny, moved + step, travelled + step, t + step / self.speed
                    res.path.append((x, y))
                    self.look(x, y, yaw)
                    dist = math.dist((x, y), (bx, by))
                if blocked or moved >= self.replan_every:
                    break
            if moved == 0.0:                                # boxed in on the first step: let the map update and re-plan
                t += 2.0
        res.travelled_m, res.plans, res.seconds = travelled, plans, t
        if math.dist((x, y), self.goal) < 0.6:
            res.reached = True
        res.revealed_frac = float(self.revealed.mean())
        return res
