"""Unit tests for nav.voxel_map (step06 Phases 1 and 4).

Things proved here, all pure NumPy/OctoMap - no ROS/Gazebo:
1. The eviction DECISION (`voxels_outside_radius`) is correct in isolation -
   the same "pure core, thin wrapper" split as every other algorithm module
   in this project.
2. `BoundedVoxelMap` actually represents a floor + roof + free-space-between
   at the same (x, y) as three distinct answers - the specific representational
   claim step06 exists to prove, checked here on synthetic geometry mirroring
   `tunnel_test.world`'s real dimensions (tunnel roof at z=0.73m, floor at
   z=0.0m - see that world file's own docstring for the measurements this
   mirrors), independent of whether Gazebo/the real sensor pipeline is available.
3. (Phase 4) `check_column_headroom`/`check_region_headroom` correctly
   distinguish "genuinely enough vertical clearance", "confirmed no room",
   and "haven't looked yet" - tested against a synthetic dict-backed
   `query_fn`, no OcTree needed, so the THREE-WAY decision logic is proved
   independent of the OctoMap binding underneath it.
"""
import numpy as np

from nav.voxel_map import (
    BoundedVoxelMap,
    check_column_headroom,
    check_region_headroom,
    voxels_outside_radius,
)


class TestVoxelsOutsideRadius:
    def test_points_within_radius_are_kept(self):
        centers = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        outside = voxels_outside_radius(centers, center=np.array([0.0, 0.0, 0.0]), radius_m=5.0)
        assert outside.tolist() == [False, False]

    def test_points_beyond_radius_are_flagged(self):
        centers = np.array([[0.0, 0.0, 0.0], [20.0, 0.0, 0.0]])
        outside = voxels_outside_radius(centers, center=np.array([0.0, 0.0, 0.0]), radius_m=5.0)
        assert outside.tolist() == [False, True]

    def test_z_axis_counts_towards_distance_not_just_xy(self):
        # A tunnel's ceiling is exactly as real a piece of "not local
        # anymore" as anything on the ground plane - this asserts the
        # eviction radius is a true 3D Euclidean distance, not a flattened
        # XY check that would never evict anything directly overhead.
        centers = np.array([[0.0, 0.0, 20.0]])
        outside = voxels_outside_radius(centers, center=np.array([0.0, 0.0, 0.0]), radius_m=5.0)
        assert outside.tolist() == [True]

    def test_recentering_changes_which_points_are_outside(self):
        centers = np.array([[10.0, 0.0, 0.0]])
        far_from_origin = voxels_outside_radius(centers, center=np.array([0.0, 0.0, 0.0]), radius_m=5.0)
        near_after_recenter = voxels_outside_radius(centers, center=np.array([10.0, 0.0, 0.0]), radius_m=5.0)
        assert far_from_origin.tolist() == [True]
        assert near_after_recenter.tolist() == [False]


def _tunnel_points() -> np.ndarray:
    """Synthetic geometry mirroring tunnel_test.world: a floor plane at
    z=0 under and around a tunnel, and a roof plane at z=0.73 only over the
    tunnel's own footprint - two distinct surfaces at overlapping (x, y).
    """
    points = []
    for x in np.arange(-3.0, 3.01, 0.1):
        for y in np.arange(-1.5, 1.51, 0.1):
            points.append([x, y, 0.0])
    for x in np.arange(-0.75, 0.76, 0.05):
        for y in np.arange(-0.65, 0.66, 0.05):
            points.append([x, y, 0.73])
    return np.array(points)


class TestBoundedVoxelMapRepresentsTunnelCorrectly:
    """The specific claim step05 proved the 2.5D map cannot make."""

    def setup_method(self):
        self.map = BoundedVoxelMap(resolution_m=0.05, max_radius_m=12.0)
        # Sensor origin above the scene, mimicking the UAV's downward camera -
        # ray-casting from here is what marks the tunnel's interior FREE.
        self.map.insert_points(_tunnel_points(), sensor_origin_xyz=[0.0, 0.0, 5.0])

    def test_floor_is_occupied(self):
        assert self.map.query([0.0, 0.0, 0.0]) == "occupied"

    def test_tunnel_interior_headroom_is_free_not_unknown_and_not_occupied(self):
        # This is the exact cell step05 found reading as a false, solid
        # "plateau" in the 2.5D elevation map - here it correctly reads FREE.
        assert self.map.query([0.0, 0.0, 0.35]) == "free"

    def test_roof_is_occupied(self):
        assert self.map.query([0.0, 0.0, 0.73]) == "occupied"

    def test_same_height_as_roof_but_outside_tunnel_footprint_is_free(self):
        # Proves the map isn't just crudely marking "z=0.73 is always
        # occupied" - it's a real per-(x, y, z) representation, and only
        # occupied where the roof geometry actually is.
        assert self.map.query([2.0, 0.0, 0.73]) == "free"

    def test_occupied_leaf_centers_excludes_free_space(self):
        # Built for scripts/export_voxel_map.py - only the SOLID structure
        # (floor + roof), not the much larger number of free-space leaves
        # an octree also tracks (see the tunnel's interior headroom test
        # above - that same "free" cell must NOT show up here).
        centers = self.map.occupied_leaf_centers()
        assert centers.shape[0] > 0
        assert centers.shape[1] == 3
        # Every returned center must itself query back as occupied - the
        # two methods (occupied_leaf_centers, query) must agree.
        for xyz in centers:
            assert self.map.query(xyz) == "occupied"
        # A known-free point (tunnel interior headroom) must not appear
        # among the returned centers - not even approximately, since these
        # are exact voxel centers on the SAME 0.05m grid.
        free_point = np.array([0.0, 0.0, 0.35])
        assert not np.any(np.all(np.isclose(centers, free_point, atol=1e-6), axis=1))


def _small_cluster(cx: float, cy: float) -> np.ndarray:
    """A 3x3 cluster instead of a single isolated point - the `octomap`
    binding's `deleteNode` has a documented edge case (see `voxel_map.py`'s
    own comment on the bug found live) where a tree containing exactly one
    never-subdivided measurement can fail to delete even at the correct
    depth. A real sensor frame always inserts many points, so tests use a
    small cluster to match that and avoid asserting against an irrelevant
    edge case.
    """
    return np.array([[cx + dx, cy + dy, 0.0] for dx in (-0.1, 0.0, 0.1) for dy in (-0.1, 0.0, 0.1)])


class TestBoundedVoxelMapEviction:
    def test_evict_outside_radius_removes_only_far_voxels_and_reports_count(self):
        voxel_map = BoundedVoxelMap(resolution_m=0.5, max_radius_m=3.0)
        near_points = _small_cluster(0.0, 0.0)
        far_points = _small_cluster(20.0, 0.0)
        voxel_map.insert_points(near_points, sensor_origin_xyz=[0.0, 0.0, 5.0])
        voxel_map.insert_points(far_points, sensor_origin_xyz=[20.0, 0.0, 5.0])

        nodes_before = voxel_map.num_leaf_nodes
        evicted = voxel_map.evict_outside_radius(center_xyz=[0.0, 0.0, 0.0])

        assert evicted > 0
        assert voxel_map.num_leaf_nodes < nodes_before
        # The far voxel is actually gone (query returns "unknown", not
        # "occupied") - proves this is real deletion, not just a count.
        assert voxel_map.query([20.0, 0.0, 0.0]) == "unknown"
        # The near voxel, inside the radius, must survive the same eviction pass.
        assert voxel_map.query([0.0, 0.0, 0.0]) == "occupied"

    def test_map_stays_bounded_over_repeated_recentering(self):
        # Simulates a UGV driving in a straight line far longer than the
        # configured radius: after each move, evicting relative to the NEW
        # center must keep the leaf count from growing without bound,
        # instead of accumulating every position ever visited.
        voxel_map = BoundedVoxelMap(resolution_m=0.5, max_radius_m=3.0)
        counts = []
        for step in range(20):
            center = [float(step) * 2.0, 0.0, 0.0]
            voxel_map.insert_points(_small_cluster(center[0], center[1]), sensor_origin_xyz=[center[0], 0.0, 5.0])
            voxel_map.evict_outside_radius(center_xyz=center)
            counts.append(voxel_map.num_leaf_nodes)

        # 20 steps * 2m spacing = 38m of travel, radius kept = 3m. Measured
        # live (see step06 Phase 1 results): leaf count grows for the first
        # couple of steps (24, 42) then holds EXACTLY flat at 42 for the
        # remaining 18 steps - a genuine steady state, not merely "small".
        # Over the identical 20-step run WITHOUT eviction, leaf count instead
        # reaches 820 (unbounded, still climbing) - so the real claim under
        # test is "converges to and stays at a constant", checked directly
        # (the back half of the run is one repeated value), not just "stays
        # under some arbitrary number".
        assert len(set(counts[-10:])) == 1, f"expected a flat steady state, got {counts}"
        assert counts[-1] < 100


def _fake_query(occupied_at: set[float] = frozenset(), unknown_at: set[float] = frozenset()):
    """Builds a synthetic query_fn(x, y, z) for check_column_headroom's
    tests - keyed purely on z (rounded to avoid float-equality flakiness),
    since every test below scans a single column and only cares about the
    vertical profile. Anything not listed in `occupied_at`/`unknown_at`
    defaults to "free" - matches how BoundedVoxelMap.query behaves for a
    genuinely well-observed empty region.
    """
    def query_fn(x, y, z):
        key = round(z, 6)
        if key in {round(v, 6) for v in unknown_at}:
            return "unknown"
        if key in {round(v, 6) for v in occupied_at}:
            return "occupied"
        return "free"
    return query_fn


class TestCheckColumnHeadroom:
    def test_tall_enough_free_run_is_passable(self):
        # Floor at z=0.0, roof at z=0.7 - the tunnel_test.world real
        # measurements this module's docstring/Phase 1 results are built
        # against. Scanning 0.0 to 0.8 at 0.1m steps with a 0.4m robot
        # requirement (UGV chassis ~0.2-0.25m tall - see nav_ugv/model.sdf -
        # plus margin) should find the free run comfortably.
        query_fn = _fake_query(occupied_at={0.0, 0.7})
        result = check_column_headroom(query_fn, x=0.0, y=0.0, z_min=0.0, z_max=0.8, step_m=0.1, required_clearance_m=0.4)
        assert result == "passable"

    def test_insufficient_but_fully_observed_run_is_blocked(self):
        # Only 0.1-0.2m of confirmed FREE clearance between two confirmed
        # occupied voxels close together - fully resolved, genuinely too
        # tight, must be reported as a confident "no", not "unknown".
        query_fn = _fake_query(occupied_at={0.0, 0.2})
        result = check_column_headroom(query_fn, x=0.0, y=0.0, z_min=0.0, z_max=0.2, step_m=0.1, required_clearance_m=0.4)
        assert result == "blocked"

    def test_entirely_unobserved_column_is_unknown_not_blocked(self):
        # Nothing has looked here yet at all - must NOT be reported as
        # "blocked" (that would wrongly refuse a route that might actually
        # be fine) nor "passable" (that would wrongly trust unverified space).
        query_fn = _fake_query(unknown_at={0.0, 0.1, 0.2, 0.3, 0.4})
        result = check_column_headroom(query_fn, x=0.0, y=0.0, z_min=0.0, z_max=0.4, step_m=0.1, required_clearance_m=0.4)
        assert result == "unknown"

    def test_short_confirmed_run_plus_an_unresolved_gap_is_unknown_not_blocked(self):
        # The confirmed-free portion alone isn't tall enough, but part of
        # the column is still unresolved - more data could still reveal a
        # passable window, so this must stay "unknown", not be prematurely
        # downgraded to "blocked" just because what HAS been seen isn't
        # enough on its own.
        query_fn = _fake_query(occupied_at={0.0}, unknown_at={0.3, 0.4})
        result = check_column_headroom(query_fn, x=0.0, y=0.0, z_min=0.0, z_max=0.4, step_m=0.1, required_clearance_m=0.4)
        assert result == "unknown"

    def test_free_run_length_uses_the_actual_tunnel_headroom_from_phase1(self):
        # Direct regression pin against Phase 1's own live-confirmed numbers
        # (tunnel floor=0.0 occupied, headroom=0.35 free, roof=0.7 occupied -
        # see this file's TestBoundedVoxelMapRepresentsTunnelCorrectly above
        # and step06_hybrid_3d_voxel_navigation.md's Phase 1 results) rather
        # than an arbitrary synthetic column - if this ever regresses, it
        # means Phase 4's check would have rejected the exact real scenario
        # step06 exists to solve.
        query_fn = _fake_query(occupied_at={0.0, 0.7})
        result = check_column_headroom(query_fn, x=0.0, y=0.0, z_min=-0.1, z_max=0.8, step_m=0.05, required_clearance_m=0.4)
        assert result == "passable"


class TestCheckRegionHeadroom:
    def test_all_columns_passable_makes_the_region_passable(self):
        query_fn = _fake_query(occupied_at={0.0, 0.7})
        result = check_region_headroom(
            query_fn, sample_points_xy=[(-0.3, 0.0), (0.0, 0.0), (0.3, 0.0)],
            z_min=0.0, z_max=0.8, step_m=0.1, required_clearance_m=0.4,
        )
        assert result == "passable"

    def test_a_single_blocked_column_blocks_the_whole_region(self):
        # One genuinely confirmed obstruction anywhere in the region is
        # enough to stop the UGV, regardless of how clear the rest of the
        # region is - a real ceiling that dips low in only one spot along
        # the tunnel is exactly as disqualifying as one that's low throughout.
        def query_fn(x, y, z):
            if x > 0.2:
                # Occupied every other step (0.0, 0.2, 0.4, 0.6, 0.8) caps
                # every free run at a single 0.1m step - nowhere near the
                # 0.4m requirement, and fully resolved (no unknowns), so
                # this column is a genuine, confident "blocked".
                return "occupied" if round(z, 6) in (0.0, 0.2, 0.4, 0.6, 0.8) else "free"
            return "occupied" if round(z, 6) in (0.0, 0.7) else "free"  # fine everywhere else
        result = check_region_headroom(
            query_fn, sample_points_xy=[(-0.3, 0.0), (0.0, 0.0), (0.3, 0.0)],
            z_min=0.0, z_max=0.8, step_m=0.1, required_clearance_m=0.4,
        )
        assert result == "blocked"

    def test_one_unresolved_column_makes_the_region_unknown_not_passable(self):
        # The two columns that HAVE been observed both look fine, but a
        # third hasn't been resolved yet - reporting "passable" here would
        # be trusting unverified space, exactly what this whole mechanism
        # exists to avoid (see check_column_headroom's own docstring).
        def query_fn(x, y, z):
            if x > 0.2:
                return "unknown"
            return "occupied" if round(z, 6) in (0.0, 0.7) else "free"
        result = check_region_headroom(
            query_fn, sample_points_xy=[(-0.3, 0.0), (0.0, 0.0), (0.3, 0.0)],
            z_min=0.0, z_max=0.8, step_m=0.1, required_clearance_m=0.4,
        )
        assert result == "unknown"
