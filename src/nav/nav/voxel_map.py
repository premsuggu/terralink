"""step06 Phase 1: a bounded, persistent 3D occupancy map - the piece step05
proved the 2.5D `/elevation_map` structurally cannot be: at the same (x, y),
it can hold a floor voxel AND a roof voxel as two distinct things, with free
space in between, instead of one number overwriting the other.

Why OctoMap (via the `octomap` PyPI binding) instead of a hand-rolled sparse
voxel structure: it already solves the hard parts correctly (probabilistic
per-voxel occupancy fusion via log-odds, ray casting so a "no return" beam
correctly marks the space it passed through as free, and octree sparsity so
empty/unknown regions don't cost memory) - this project's own precedent is
"reuse a proven algorithm rather than reinvent it" (see `prm_planner.py`'s
own docstring re: d3's PRM, or `emap`'s "reference `src/d1`, don't rebuild
its ideas from scratch").

REAL ENVIRONMENT CONSTRAINT FOUND LIVE while building this: the plan
(`docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md`) originally called
for the ROS package `ros-humble-octomap-server` (a separate C++ node,
consulted over its own `clear_bbx` service). Installing it requires
`apt-get install`, which requires root - and this sandboxed environment has
no passwordless sudo (confirmed live: `sudo -n true` fails, `apt-get
install` as this user fails with a dpkg lock permission error). The PyPI
package `octomap` (a Python binding to the SAME underlying `liboctomap`
C++ library, which the base `liboctomap-dev` is already installed here) has
no such requirement - `pip install --user octomap` works with no elevated
privileges. This module uses that binding directly and does the
insertion/eviction bookkeeping in-process instead of over a ROS service,
which is a BETTER fit for this codebase's own style anyway: every other
`emap`/`nav` algorithm module (see `fusion.py`, `prm_planner.py`,
`walkability.py`) is a pure-Python/NumPy core wrapped by a thin ROS node,
not a call out to an external node written by someone else with its own
service contract to track.
"""
from __future__ import annotations

import numpy as np
import octomap


def voxels_outside_radius(voxel_centers: np.ndarray, center: np.ndarray, radius_m: float) -> np.ndarray:
    """Pure eviction-decision logic, deliberately kept free of any OctoMap
    (or ROS) dependency so it's trivially unit-testable in isolation - the
    same "small pure function, thin wrapper around it" split every other
    algorithm module in this project uses.

    `voxel_centers`: (N, 3) array of voxel center coordinates (e.g. from
    iterating an OcTree's leaves). `center`: (3,) the map's current
    reference point (this project centers it on the UGV's live position -
    see `BoundedVoxelMap.evict_outside_radius`'s own docstring for why).

    Returns a boolean mask, True where a voxel is farther than `radius_m`
    from `center` (Euclidean, all three axes - a tunnel's ceiling is exactly
    as real a piece of "not local anymore" as anything on the ground plane,
    so this deliberately does NOT flatten to an XY-only radius).
    """
    distances = np.linalg.norm(voxel_centers - np.asarray(center), axis=1)
    return distances > radius_m


class BoundedVoxelMap:
    """A thin, stateful wrapper around `octomap.OcTree` that adds the one
    thing the raw library doesn't provide: a hard cap on how much space it's
    allowed to remember, so a mission that runs for an arbitrarily long time
    or distance has bounded (not ever-growing) memory/compute cost - the
    same "cheap because it's bounded" property `emap`'s own local rolling
    map already has (step08), applied here to 3D instead of 2.5D.

    Deliberately recentered on the UGV, not the UAV: this map exists to give
    the ground vehicle a genuinely local, full-3D ground-truth check right
    before it commits to a tentative/anomalous segment (step06 phases 3-4,
    not yet built) - the UAV keeps planning globally on the cheap 2.5D map,
    so it has no need for this map to follow it around instead.
    """

    def __init__(self, resolution_m: float = 0.05, max_radius_m: float = 12.0):
        self.resolution_m = resolution_m
        self.max_radius_m = max_radius_m
        self._tree = octomap.OcTree(resolution_m)

    def insert_points(self, points_xyz: np.ndarray, sensor_origin_xyz: np.ndarray) -> None:
        """Fuse one sensor frame's worth of points (already transformed into
        the shared map frame - see `emap.utils.tf_utils.transform_points`,
        reused as-is by `voxel_map_node.py` rather than re-derived here) into
        the octree. `sensor_origin_xyz` matters as much as the points
        themselves: OctoMap ray-casts from origin to each point, which is
        what correctly marks the tunnel's INTERIOR as free (the ray passes
        through it on the way to the floor) rather than merely "unknown" -
        this is the mechanism, not an incidental detail.
        """
        if points_xyz.size == 0:
            return
        self._tree.insertPointCloud(np.asarray(points_xyz, dtype=np.float64), np.asarray(sensor_origin_xyz, dtype=np.float64))
        self._tree.updateInnerOccupancy()

    def query(self, point_xyz) -> str:
        """Returns "occupied", "free", or "unknown" for one point - the
        three-way answer this whole map exists to provide, since that's
        exactly what step05 found the 2.5D map's `is_valid`-based two-way
        answer (valid/invalid) cannot: a point can be genuinely, confidently
        FREE (the tunnel's interior) even though it sits directly below
        something genuinely, confidently OCCUPIED (the roof) at the same
        (x, y) - a distinction `is_valid` has no way to express.
        """
        # REAL BUG FOUND LIVE in the `octomap` PyPI binding (v1.10.0.0):
        # `search()` on a point with no data does NOT return Python `None`
        # (confirmed live: it always returns a real `OcTreeNode` wrapper
        # object, even on a completely empty tree) - checking `is None` here
        # would silently never trigger. The actual failure mode is that
        # USING that wrapper (`isNodeOccupied`/`getOccupancy`) raises
        # `octomap.NullPointerException` - that's the real, only reliable
        # "this point has no data" signal this binding provides.
        node = self._tree.search(np.asarray(point_xyz, dtype=np.float64))
        try:
            return "occupied" if self._tree.isNodeOccupied(node) else "free"
        except octomap.NullPointerException:
            return "unknown"

    def leaf_centers(self) -> np.ndarray:
        """All current leaf-voxel center coordinates as an (N, 3) array -
        the raw material `evict_outside_radius` feeds to the pure
        `voxels_outside_radius` decision function above.
        """
        centers = [leaf.getCoordinate() for leaf in self._tree.begin_leafs()]
        if not centers:
            return np.empty((0, 3))
        return np.asarray(centers)

    def occupied_leaf_centers(self) -> np.ndarray:
        """Like `leaf_centers` but OCCUPIED voxels only (filters out the
        much larger number of merely `free` leaves an octree also tracks) -
        built for `scripts/export_voxel_map.py`'s "show me the actual 3D
        solid structure" use case (the tunnel's roof/floor as distinct
        voxels - the exact thing this whole map exists to represent, see
        this module's own docstring), where free-space leaves would just be
        visual clutter. Uses the SAME iteration pass to get the coordinate
        and its occupancy together, rather than iterating twice (twice
        would also risk a subtly inconsistent snapshot if a callback
        mutates the tree in between - `insert_points` runs on this same
        node's own executor thread, so that's not a real risk today, but
        there's no reason to rely on it).
        """
        centers = [
            leaf.getCoordinate() for leaf in self._tree.begin_leafs() if self._tree.isNodeOccupied(leaf)
        ]
        if not centers:
            return np.empty((0, 3))
        return np.asarray(centers)

    def evict_outside_radius(self, center_xyz) -> int:
        """Deletes every leaf voxel farther than `self.max_radius_m` from
        `center_xyz`. This is this module's stand-in for `octomap_server`'s
        `clear_bbx` service (see the module docstring for why that ROS node
        isn't available here) - same effect (bound the map's spatial/memory
        footprint), done as a plain in-process method call instead of a ROS
        service round-trip, since there's no separate process to call it on.

        Returns the number of voxels evicted, so a caller (or a test) can
        confirm eviction actually happened rather than silently no-opping.
        """
        centers = self.leaf_centers()
        if centers.shape[0] == 0:
            return 0
        outside = voxels_outside_radius(centers, np.asarray(center_xyz), self.max_radius_m)
        # REAL BUG FOUND LIVE in the `octomap` PyPI binding (v1.10.0.0):
        # `deleteNode`'s own DEFAULT depth argument (depth=1) does not mean
        # "delete this one leaf voxel" - on a tree with any real structure,
        # depth=1 deletes a huge subtree near the root (confirmed live: one
        # `deleteNode` call at the default depth collapsed a 1478-leaf tree
        # down to 53 leaves - catastrophic, silent data loss far beyond the
        # single voxel asked for). Passing `depth=0` explicitly gives the
        # correct, precise, single-leaf deletion instead - confirmed live to
        # remove exactly one leaf per call and leave unrelated voxels
        # untouched. NEVER call `deleteNode` without `depth=0` in this
        # codebase. (One remaining edge case, also found live and considered
        # acceptable: a tree containing a single, never-subdivided
        # measurement can fail to delete even at depth=0 - irrelevant here
        # since a real sensor frame always inserts many points at once.)
        for coord in centers[outside]:
            self._tree.deleteNode(coord, depth=0)
        return int(np.count_nonzero(outside))

    @property
    def num_leaf_nodes(self) -> int:
        return self._tree.getNumLeafNodes()

    @property
    def memory_usage_bytes(self) -> int:
        return self._tree.memoryUsage()


# step06 Phase 4: the actual "go check ground truth before crossing" logic.
# `nav.prm_planner`'s anomaly/frontier mechanism (Phase 3, step05) already
# returns a TENTATIVE path across a region the 2.5D map couldn't confirm -
# but a tentative path is still just a hopeful guess until something
# actually looks. These two functions are that look: given this map's
# occupied/free/unknown answer at any point (BoundedVoxelMap.query), decide
# whether there's genuinely enough vertical clearance for the UGV to pass.
#
# Kept as plain functions taking a `query_fn` callable (not methods on
# BoundedVoxelMap) so they're unit-testable against a synthetic dict-backed
# fake with no OcTree/OctoMap dependency at all - same "pure core, thin
# wrapper" split as every other algorithm module in this project
# (`nav.walkability`, `nav.prm_planner`, `nav.anomaly` all separate the
# decision logic from whatever produces its inputs).
def check_column_headroom(
    query_fn,
    x: float,
    y: float,
    z_min: float,
    z_max: float,
    step_m: float,
    required_clearance_m: float,
) -> str:
    """Scan one vertical column at (x, y) from z_min to z_max in step_m
    increments and decide whether a UGV of `required_clearance_m` height
    (already including any safety margin - see check_region_headroom's
    caller) can fit through it.

    Deliberately does NOT assume a known floor height - unlike the 2.5D
    map's per-cell elevation, this function never trusts a single number
    for "the ground is here"; it just looks for the longest unbroken run of
    genuinely FREE voxels anywhere in the scanned range, which is exactly
    what "is there room for the robot to pass, wherever the floor actually
    is" means. Each free sample is counted as representing step_m of real
    clearance (the voxel grid's own resolution) - a mild simplification
    (a run of N free samples spans, strictly, (N-1)*step_m between sample
    centers) that always UNDER-counts true available clearance by less than
    one step_m, so it can never falsely report more room than exists.

    Returns one of three answers, deliberately NOT a boolean - the whole
    point of this function (see this module's docstring on why a 3D map
    exists at all) is to distinguish "confirmed passable" from "confirmed
    blocked" from "don't know yet", and collapsing the last two together
    would silently turn "haven't looked yet" into "looks bad", which is
    exactly the kind of unwarranted confidence step05 already found once
    (a data-fusion artifact wrongly read as confidently LETHAL):

      - "passable": a contiguous free run of at least `required_clearance_m`
        was found somewhere in the column.
      - "blocked": no such run exists, AND every sample in the scanned range
        resolved to a real answer (occupied or free, never unknown) - so
        this is a genuinely settled "no" backed by complete data over the
        scanned range, not a guess.
      - "unknown": no sufficient run was found, but at least one sample was
        still unresolved (`query_fn` returned "unknown") - more data could
        still reveal a passable window there, so this must NOT be reported
        as "blocked" (that would wrongly stop the UGV somewhere it might
        actually be able to cross) or as "passable" (that would wrongly
        send it somewhere still unverified - see check_region_headroom's
        caller, `waypoint_follower.py`, for how "unknown" gets handled:
        drive cautiously and rely on Nav2's own local obstacle avoidance,
        not this function's word, while more sensor data accumulates).
    """
    n_steps = max(1, int(round((z_max - z_min) / step_m)) + 1)
    z_values = z_min + np.arange(n_steps) * step_m

    best_free_run_m = 0.0
    current_run_steps = 0
    saw_unknown = False
    for z in z_values:
        state = query_fn(x, y, float(z))
        if state == "free":
            current_run_steps += 1
            best_free_run_m = max(best_free_run_m, current_run_steps * step_m)
        else:
            current_run_steps = 0
            if state == "unknown":
                saw_unknown = True

    if best_free_run_m >= required_clearance_m:
        return "passable"
    if not saw_unknown:
        return "blocked"
    return "unknown"


def check_region_headroom(
    query_fn,
    sample_points_xy,
    z_min: float,
    z_max: float,
    step_m: float,
    required_clearance_m: float,
) -> str:
    """Same three-way answer as `check_column_headroom`, but for a whole
    region (e.g. an anomaly segment's real x/y footprint) rather than one
    point - sampled at each of `sample_points_xy` ((x, y) pairs; the caller
    picks these, typically a few points spread along the tentative path
    segment actually being crossed - see `waypoint_follower.py`).

    Combines per-column verdicts the way a real safety check should: the
    WHOLE region only counts as "passable" if EVERY sampled column is
    (a single narrow column of insufficient headroom is exactly as
    disqualifying as a wide one); the region is "blocked" if ANY sampled
    column is confidently blocked (one confirmed obstruction is enough to
    stop the UGV, regardless of what the rest of the region looks like);
    otherwise "unknown" - there's at least one column that hasn't been
    fully resolved yet, so the honest answer is "don't know", not a
    falsely reassuring "passable" reached only by ignoring the unresolved
    part.
    """
    verdicts = [
        check_column_headroom(query_fn, x, y, z_min, z_max, step_m, required_clearance_m)
        for x, y in sample_points_xy
    ]
    if any(v == "blocked" for v in verdicts):
        return "blocked"
    if all(v == "passable" for v in verdicts):
        return "passable"
    return "unknown"
