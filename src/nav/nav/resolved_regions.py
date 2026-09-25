"""step06 Phase 5: the bidirectional coordination loop's shared memory - what
`waypoint_follower.py`'s local 3D voxel checks (step06 Phase 4) have actually
settled, kept around so `planner_node.py` doesn't have to re-derive the same
answer from a freshly-decoded `/elevation_map` every time it arrives.

Why this needs to exist as its own store, not be "handled downstream":
`nav.anomaly.compute_anomaly_mask` and `nav.walkability.compute_walkable_mask`
are both PURE functions of the CURRENT `/elevation_map` message - by design
(see their own docstrings, e.g. `compute_anomaly_mask`'s "this module only
identifies candidate cells; nothing here changes planning behavior by
itself"), neither has any memory of its own. Without something remembering
resolutions across messages, `planner_node` would ask the exact same anomaly
question about the exact same cells on every single future `get_plan` call,
even long after step06 Phase 4 already spent a real 3D sensor check settling
it - and a "confirmed blocked" verdict has nowhere else to live at all:
nothing about the 2.5D `/elevation_map` data itself ever changes just because
the UGV drove up and looked, so `compute_anomaly_mask` will keep flagging the
exact same tunnel-mouth shape forever, correctly, from its own (2.5D-only)
point of view - it simply isn't the most current information available
anymore. This module is where that better information gets remembered and
re-applied on top of every future mask computation.

Only two verdicts are ever stored here - "unknown" (see
`nav.voxel_map.check_region_headroom`'s three-way return) is never a
resolution at all, so there is nothing useful to remember about a question
that hasn't actually been answered yet; see `waypoint_follower.py`'s own
`_new_resolution_to_report` for where that filtering happens before anything
ever reaches this module.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from emap.utils.coord_transform import world_to_grid


@dataclass(frozen=True)
class ResolvedRegion:
    """One settled step06 Phase 4 verdict, in world-frame meters - the exact
    shape `waypoint_follower.py`'s `_segment_bounding_box` already produces,
    unchanged going out over `resolved_region_report`'s
    `std_msgs/Float64MultiArray` wire encoding (`[x_min, y_min, x_max, y_max,
    verdict]`, `verdict` 1.0=passable/0.0=blocked - see
    `planner_node.py`'s `_resolved_region_callback`, the only place that
    encoding is decoded back into one of these).

    `frozen=True` (and therefore hashable/equality-comparable by value, not
    identity) is deliberate, not incidental - `ResolvedRegionStore.add`
    relies on structural equality to skip an exact-duplicate report (see its
    own docstring) rather than growing without bound over a long mission
    every time the same already-settled segment gets re-checked.
    """
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    passable: bool


class ResolvedRegionStore:
    """Accumulates `ResolvedRegion` reports over a mission and re-applies all
    of them, in order, on top of a freshly-decoded walkable/anomaly mask
    pair every time `planner_node.py`'s `_map_callback` runs. Pure
    Python/NumPy, no ROS dependency - see that module for the thin wrapper
    (subscribes to `resolved_region_report`, decodes it into a
    `ResolvedRegion`, calls `add()`, calls `apply()` every `/elevation_map`
    message) - same "pure core + thin ROS wrapper" split every other
    algorithm module in this project already follows.
    """

    def __init__(self) -> None:
        self._regions: list[ResolvedRegion] = []

    def add(self, region: ResolvedRegion) -> None:
        """Record one settled verdict. Exact-duplicate reports are expected
        and harmless in normal operation (e.g. `waypoint_follower.py` may
        legitimately report the same already-resolved segment again on a
        later leg of the same mission) - skipped here rather than growing
        the list forever for no benefit; `apply()` is idempotent either way,
        this is purely a memory-growth guard, not a correctness requirement.
        """
        if region in self._regions:
            return
        self._regions.append(region)

    def apply(
        self,
        walkable_mask: np.ndarray,
        anomaly_mask: np.ndarray | None,
        resolution: float,
        center_x: float,
        center_y: float,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """Return COPIES of `walkable_mask`/`anomaly_mask` with every stored
        region's resolution overlaid on top, applied in the order they were
        added (so a later report about a region overlapping an earlier one
        wins - the freshest ground-truth check should always take
        precedence over a stale one). Both masks must come from the SAME
        `/elevation_map` message (same `resolution`/`center_x`/`center_y`/
        shape) this store's regions were originally reported against - same
        "caller's responsibility to pass matching arrays" convention
        `nav.anomaly`/`nav.walkability` already use.

        A PASSABLE region: forced `walkable=True` (the UGV's own sensor
        confirmed real ground/headroom there - it's ordinary safe terrain
        from now on, not merely a suspected artifact) and, when an
        `anomaly_mask` was supplied, forced `anomaly=False` there too (no
        longer tentative - a plan through it should cost the same as any
        other confirmed-walkable edge, not still carry `prm_planner`'s
        anomaly-edge penalty for a question that's already been settled).

        A BLOCKED region: forced `walkable=False` (ground truth said no,
        regardless of what the 2.5D reading looked like) and `anomaly=False`
        too - not because it's safe, but because "anomalous" specifically
        means "uncertain, worth a 3D check" (see `nav.anomaly`'s own
        docstring), and once that check has actually happened and come back
        negative, this cell is just ordinary confirmed-LETHAL now, not a
        tentative pass-through corridor `prm_planner` should keep offering
        at a penalty.

        If the store is empty, returns `walkable_mask`/`anomaly_mask`
        UNCHANGED (the exact same object, not even copied) - the common case
        (no resolutions reported yet, e.g. anomaly mode never turned on, or
        step06 Phase 4/5 never enabled) pays zero extra cost per message.
        """
        if not self._regions:
            return walkable_mask, anomaly_mask

        walkable_mask = np.array(walkable_mask, dtype=bool, copy=True)
        anomaly_mask = None if anomaly_mask is None else np.array(anomaly_mask, dtype=bool, copy=True)
        n_rows, n_cols = walkable_mask.shape
        cell_n = n_rows  # ElevationMap's grid is always square (same assumption prm_planner.py makes)

        for region in self._regions:
            r0, c0 = world_to_grid(region.x_min, region.y_min, center_x, center_y, resolution, cell_n)
            r1, c1 = world_to_grid(region.x_max, region.y_max, center_x, center_y, resolution, cell_n)
            # world_to_grid has no notion of which corner maps to the larger
            # index (row increases with Y, col with X - see that function's
            # own docstring) - min/max the CONVERTED indices, not the world
            # coordinates, to build a well-formed, always-non-inverted slice.
            row_lo, row_hi = int(min(r0, r1)), int(max(r0, r1)) + 1
            col_lo, col_hi = int(min(c0, c1)), int(max(c0, c1)) + 1
            # Clip to the grid's real bounds - a region reported near the
            # map's edge could otherwise ask for an out-of-range slice.
            # NumPy would clip this silently on its own; doing it explicitly
            # here documents that it's expected, not an oversight.
            row_lo, row_hi = max(0, row_lo), min(n_rows, row_hi)
            col_lo, col_hi = max(0, col_lo), min(n_cols, col_hi)
            if row_lo >= row_hi or col_lo >= col_hi:
                continue  # region fell entirely outside the current map extent

            walkable_mask[row_lo:row_hi, col_lo:col_hi] = region.passable
            if anomaly_mask is not None:
                anomaly_mask[row_lo:row_hi, col_lo:col_hi] = False

        return walkable_mask, anomaly_mask
