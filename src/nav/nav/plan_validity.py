"""Is the plan the UGV is driving still valid on the CURRENT map? (event-driven re-planning, step 11b)

A plan is made on the map of the moment. When the UGV's own camera later finds a wall the UAV could not see (the ground layer,
`nav.ground_layer`), the old plan may run straight through it; the UGV then drives up to the wall and Nav2 reports "failed to make
progress" minutes later (seen live: W4, the UGV stalled 4 m before goal B). Cheap fix: after every map update, sample the plan
every 10 cm and ask whether it crosses cells that are now KNOWN to be blocked. `planner_node` publishes the answer on `plan_blocked`
and `waypoint_follower` asks for a fresh plan - only when the plan is really affected, not on a timer.

Unobserved cells are not "blocked" here: a tentative plan through the frontier is legitimate until the cells are seen.
"""
from __future__ import annotations

import numpy as np


def blocked_samples(waypoints, walkable: np.ndarray, is_valid: np.ndarray, resolution: float, center_x: float, center_y: float,
                    step_m: float = 0.1, skip_m: float = 0.0) -> int:
    """Number of samples (every `step_m`) along the polyline that lie on an OBSERVED, non-walkable cell.

    `skip_m`: ignore the first `skip_m` metres of the plan (the robot's own footprint at the start is forced free by the planner).
    Samples outside the grid are ignored."""
    pts = np.asarray(waypoints, dtype=float)
    if len(pts) < 2:
        return 0
    n_r, n_c = walkable.shape
    bad = 0
    travelled = 0.0
    for p, q in zip(pts[:-1], pts[1:]):
        seg = float(np.hypot(*(q - p)))
        k = max(int(seg / step_m), 1)
        for t in np.linspace(0.0, 1.0, k, endpoint=False):
            if travelled + t * seg < skip_m:
                continue
            x, y = p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t
            r = int(round((y - center_y) / resolution + n_r / 2.0))
            c = int(round((x - center_x) / resolution + n_c / 2.0))
            if 0 <= r < n_r and 0 <= c < n_c and is_valid[r, c] and not walkable[r, c]:
                bad += 1
        travelled += seg
    return bad


def plan_is_blocked(waypoints, walkable, is_valid, resolution, center_x, center_y, min_samples: int = 3, skip_m: float = 0.4) -> bool:
    """True when at least `min_samples` samples (0.3 m of path) are on known-blocked cells; the first `skip_m` m are ignored."""
    return blocked_samples(waypoints, walkable, is_valid, resolution, center_x, center_y, skip_m=skip_m) >= min_samples
