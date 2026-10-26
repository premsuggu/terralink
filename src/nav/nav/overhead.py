"""Goals that sit under (or inside) something the UAV can only see the TOP of.

THE PROBLEM (found by the first real-pipeline run in stress world W4, see
docs/work-docs/nav/step09_stress_worlds.md)
---------------------------------------------------------------------------
A UAV's depth camera looks straight down. A building with a deck or a roof is seen as a big flat
plateau at roof height; the rooms, corridors and doorways under it are never measured. In the 2.5D
map that plateau is a large, enclosed, flat, "walkable" island fenced by a lethal ring (the roof edge
is a 3.5 m step). Nothing in the map says "unknown" there, so:

* frontier exploration (built around `is_valid = False`) sees nothing to explore;
* anomaly detection (built for SMALL islands - a culvert roof) ignores an 800 m^2 plateau;
* a goal under the roof is simply unreachable ("no path"), and the planner chased meaningless
  frontiers on the map edge for 15 minutes.

THE IDEA
--------
Whatever stands on the plateau is not what the UGV will drive on. So when the goal lies on such an
island and no confirmed route reaches it, treat the island as UNKNOWN: replace its cells with what the
UGV's OWN sensors have seen there (the floor map), and "not seen yet" everywhere else. The existing
frontier exploration then does what it was built for: drive to the boundary between known ground and
unknown cells, look, repeat - and the unknown region shrinks as the UGV finds doors and corridors.

WHY ONLY THE GOAL'S ISLAND
--------------------------
From above, the top of a hollow structure and the top of a solid box (a shipping container, a truck)
look identical. Treating every elevated island as unknown would send the UGV to inspect every
container in the yard. Only the island that contains the goal matters, and only when no confirmed
route exists - so ordinary routes are untouched.

An elevated island is: a connected component of walkable cells whose elevation is at least
`min_height_m` above the walkable ground around it, large enough to be more than a post or a crate.
The zone to hide is the island plus the lethal ring around it (the step at its edge).
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage as ndi


def goal_island_zone(
    elevation: np.ndarray,
    walkable: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    goal_xy: tuple[float, float],
    *,
    min_height_m: float = 1.5,
    min_area_m2: float = 3.0,
    ring_m: float = 0.8,
    surround_m: float = 4.0,
    goal_snap_m: float = 1.0,
) -> np.ndarray | None:
    """Boolean mask of the elevated island holding the goal (plus its lethal ring), or None.

    None means "the goal is not on an elevated island": ordinary ground, nothing to reinterpret.
    `walkable` is the planner's walkable mask for the UAV map (observed and not lethal).
    """
    n_rows, n_cols = walkable.shape
    col = int(round((goal_xy[0] - center_x) / resolution + n_cols / 2))
    row = int(round((goal_xy[1] - center_y) / resolution + n_rows / 2))
    if not (0 <= row < n_rows and 0 <= col < n_cols):
        return None
    labels, _ = ndi.label(walkable)
    lab = labels[row, col]
    if lab == 0:                                     # goal on a lethal/unobserved cell: snap to the nearest walkable one
        r = int(goal_snap_m / resolution)
        r0, r1, c0, c1 = max(0, row - r), min(n_rows, row + r + 1), max(0, col - r), min(n_cols, col + r + 1)
        win = labels[r0:r1, c0:c1]
        ys, xs = np.nonzero(win)
        if len(ys) == 0:
            return None
        k = np.argmin((ys + r0 - row) ** 2 + (xs + c0 - col) ** 2)
        lab = win[ys[k], xs[k]]
    comp = labels == lab
    if comp.sum() * resolution ** 2 < min_area_m2:
        return None
    # Ground around the island: walkable cells of OTHER components in a band just outside it.
    outer = ndi.binary_dilation(comp, iterations=max(1, int(surround_m / resolution)))
    inner = ndi.binary_dilation(comp, iterations=max(1, int(ring_m / resolution)))
    around = outer & ~inner & walkable & ~comp
    if not around.any():
        return None                                  # no ground around it (it IS the ground): not an island
    if np.median(elevation[comp]) - np.median(elevation[around]) < min_height_m:
        return None
    return inner                                     # island + the step ring at its edge


def compose_with_floor(
    layers: dict[str, np.ndarray],
    zone: np.ndarray | None,
    floor_layers: dict[str, np.ndarray] | None,
) -> dict[str, np.ndarray]:
    """The planning map: the UAV map, completed and corrected by what the UGV has seen.

    * Cells the UAV never observed take the UGV's observation when it has one (the UGV's own camera sees
      what the patrol missed). This always applies.
    * Inside `zone` (an elevated island the UAV only saw the top of) the UAV's values are not trusted at all:
      every cell is "not observed" unless the UGV has seen it, and then it takes the UGV's elevation / variance /
      traversability.
    The input dict is not modified. `floor_layers` = the UGV's own elevation map on the same grid.
    """
    out = {k: np.array(v, copy=True) for k, v in layers.items()}
    zone_mask = np.zeros(out["is_valid"].shape, bool) if zone is None else zone
    seen = np.zeros_like(zone_mask)
    if floor_layers is not None:
        floor_valid = np.asarray(floor_layers["is_valid"]).astype(bool)
        seen = (zone_mask | ~np.asarray(out["is_valid"]).astype(bool)) & floor_valid
        for k in ("elevation", "variance", "traversability"):
            if k in out and k in floor_layers:
                out[k][seen] = np.asarray(floor_layers[k])[seen]
        out["is_valid"][seen] = 1
    unseen = zone_mask & ~seen
    out["is_valid"][unseen] = 0
    if "variance" in out:
        out["variance"][unseen] = np.maximum(out["variance"][unseen], 1.0)
    return out


class GoalIslandPlanner:
    """`GlobalPlanner` + the rule above: plan on the UAV map; if the goal is on an elevated island AND the UAV
    map has no confirmed route to it, plan on the map where that island is unknown / UGV-observed instead.

    Shared by `planner_node` and the offline simulator (`tests/nav/explore_sim.py`), like `GlobalPlanner` itself.
    `make_ctx(layers, use_anomaly=True) -> MapContext` is how the caller turns a layer dict into planner masks (frontier,
    anomaly, latches, resolved regions...), so this class does not duplicate that logic.
    """

    def __init__(self, planner, **island_kwargs):
        self.planner = planner
        self.island_kwargs = island_kwargs
        self.last_zone: np.ndarray | None = None
        self.note = ""

    def plan(self, layers, floor_layers, make_ctx, start_xy, goal_xy, resolution, center_x, center_y, now=None):
        from emap.traversability import LETHAL  # noqa: F401  (documented dependency: walkable = observed and not lethal)

        from .walkability import compute_walkable_mask

        # What the UGV has seen fills the UAV's gaps everywhere (outside islands too); the UAV map alone is `layers`.
        walkable0 = compute_walkable_mask(layers["traversability"], layers["is_valid"])
        zone = goal_island_zone(layers["elevation"], walkable0, resolution, center_x, center_y, goal_xy, **self.island_kwargs)
        # The ROBOT can be on the island too (once it has driven in under the roof, the UAV map says it is standing on a
        # huge open plateau). Then the UAV map is meaningless around it: its "route across the plateau" goes through walls
        # nobody has seen. Hide that island as well.
        start_zone = goal_island_zone(layers["elevation"], walkable0, resolution, center_x, center_y, start_xy, **self.island_kwargs)
        inside = start_zone is not None
        if start_zone is not None:
            zone = start_zone if zone is None else (zone | start_zone)
        self.last_zone = zone
        if zone is None:
            self.note = "goal on ordinary ground"
            return self.planner.plan(make_ctx(compose_with_floor(layers, None, floor_layers)), start_xy, goal_xy, now=now)
        # No "is there a confirmed route on the UAV map?" shortcut: an island is a component that is NOT connected to the
        # ground, so any UAV-map route into it squeezes across a 1-cell lethal ring (found in simulation: A* hopped the
        # 3.5 m deck edge at the SE corner, the real wall stopped the UGV, and it never recovered).
        self.note = ("robot is on" if inside else "goal is on") + " an elevated island: island treated as unknown / UGV-observed"
        # Anomaly viewpoints ("suspected passages", e.g. a truck parked next to the building) are switched off here: with the
        # goal under a roof the search is for a way INTO the island, and in the live run the UGV was pulled away from the
        # open door by an anomaly target on the dumper truck and never returned (step09 doc).
        return self.planner.plan(make_ctx(compose_with_floor(layers, zone, floor_layers), use_anomaly=False), start_xy, goal_xy, now=now)
