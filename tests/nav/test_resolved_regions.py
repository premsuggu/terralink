"""Unit tests for nav.resolved_regions - step06 Phase 5's pure "remember what
step06 Phase 4 already settled" bookkeeping. No ROS - see that module's own
docstring for why this has to exist as its own store rather than being
folded into nav.anomaly/nav.walkability (both pure functions of the CURRENT
map message, with no memory of their own, by design).
"""
import numpy as np

from emap.utils.coord_transform import world_to_grid
from nav.resolved_regions import ResolvedRegion, ResolvedRegionStore


def _grid(n=20, resolution=0.5):
    # A trivial (n, n) all-False walkable/anomaly pair, centered on world
    # (0, 0) - the same convention nav.prm_planner's own tests set up.
    walkable = np.zeros((n, n), dtype=bool)
    anomaly = np.zeros((n, n), dtype=bool)
    return walkable, anomaly, resolution, 0.0, 0.0


class TestApplyEmptyStore:
    def test_empty_store_returns_masks_unchanged_and_uncopied(self):
        walkable, anomaly, res, cx, cy = _grid()
        store = ResolvedRegionStore()
        out_walkable, out_anomaly = store.apply(walkable, anomaly, res, cx, cy)
        # Same object, not just equal - see apply()'s own docstring for why
        # the empty-store case is meant to be a true zero-cost no-op.
        assert out_walkable is walkable
        assert out_anomaly is anomaly


class TestApplyPassableRegion:
    def test_forces_walkable_true_and_clears_anomaly_inside_the_region_only(self):
        walkable, anomaly, res, cx, cy = _grid()
        anomaly[:] = True  # pretend the whole grid was flagged anomalous
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True))

        out_walkable, out_anomaly = store.apply(walkable, anomaly, res, cx, cy)

        n = walkable.shape[0]
        r0, c0 = world_to_grid(0.0, 0.0, cx, cy, res, n)  # world (0,0) - inside the reported region
        assert out_walkable[int(r0), int(c0)] == True
        assert out_anomaly[int(r0), int(c0)] == False
        assert out_walkable[0, 0] == False  # far corner, outside the region - untouched


class TestApplyBlockedRegion:
    def test_forces_walkable_false_and_clears_anomaly_inside_the_region_only(self):
        walkable, anomaly, res, cx, cy = _grid()
        walkable[:] = True  # pretend the whole grid (including a false "island") reads walkable
        anomaly[:] = True
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=False))

        out_walkable, out_anomaly = store.apply(walkable, anomaly, res, cx, cy)

        n = walkable.shape[0]
        r0, c0 = world_to_grid(0.0, 0.0, cx, cy, res, n)
        assert out_walkable[int(r0), int(c0)] == False
        assert out_anomaly[int(r0), int(c0)] == False
        assert out_walkable[0, 0] == True  # untouched outside the region


class TestApplyDoesNotMutateInputs:
    def test_original_arrays_are_left_alone(self):
        walkable, anomaly, res, cx, cy = _grid()
        original_walkable = walkable.copy()
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True))

        store.apply(walkable, anomaly, res, cx, cy)

        assert np.array_equal(walkable, original_walkable)


class TestApplyWithNoAnomalyMask:
    def test_none_anomaly_mask_stays_none(self):
        walkable, _, res, cx, cy = _grid()
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True))

        _, out_anomaly = store.apply(walkable, None, res, cx, cy)

        assert out_anomaly is None


class TestApplyMultipleRegions:
    def test_two_disjoint_regions_each_apply_only_to_their_own_area(self):
        n, resolution = 40, 0.5  # 20m x 20m grid, centered on world (0, 0)
        walkable = np.ones((n, n), dtype=bool)
        anomaly = np.zeros((n, n), dtype=bool)
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-9.0, y_min=-1.0, x_max=-7.0, y_max=1.0, passable=True))  # no-op (already walkable)
        store.add(ResolvedRegion(x_min=7.0, y_min=-1.0, x_max=9.0, y_max=1.0, passable=False))  # blocks this region

        out_walkable, _ = store.apply(walkable, anomaly, resolution, 0.0, 0.0)

        r_left, c_left = world_to_grid(-8.0, 0.0, 0.0, 0.0, resolution, n)
        r_right, c_right = world_to_grid(8.0, 0.0, 0.0, 0.0, resolution, n)
        r_mid, c_mid = world_to_grid(0.0, 0.0, 0.0, 0.0, resolution, n)
        assert out_walkable[int(r_left), int(c_left)] == True
        assert out_walkable[int(r_right), int(c_right)] == False
        assert out_walkable[int(r_mid), int(c_mid)] == True  # outside both regions - untouched

    def test_a_later_region_overlapping_an_earlier_one_wins(self):
        # Freshest ground-truth check should always take precedence over a
        # stale one - see apply()'s own docstring.
        walkable, anomaly, res, cx, cy = _grid()
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True))
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=False))

        out_walkable, _ = store.apply(walkable, anomaly, res, cx, cy)

        n = walkable.shape[0]
        r0, c0 = world_to_grid(0.0, 0.0, cx, cy, res, n)
        assert out_walkable[int(r0), int(c0)] == False  # the later (blocked) report wins


class TestAddDeduplicatesExactRepeats:
    def test_adding_the_same_region_twice_does_not_grow_the_store(self):
        store = ResolvedRegionStore()
        region = ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True)
        store.add(region)
        store.add(region)
        assert len(store._regions) == 1

    def test_a_different_verdict_for_the_same_bbox_is_a_distinct_entry(self):
        store = ResolvedRegionStore()
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=True))
        store.add(ResolvedRegion(x_min=-1.0, y_min=-1.0, x_max=1.0, y_max=1.0, passable=False))
        assert len(store._regions) == 2


class TestPassableMask:
    """`passable_mask` feeds the A* planner's force_free_mask."""

    def _store(self, *regions):
        from nav.resolved_regions import ResolvedRegionStore

        store = ResolvedRegionStore()
        for r in regions:
            store.add(r)
        return store

    def test_empty_store_gives_an_all_false_mask(self):
        mask = self._store().passable_mask((40, 40), 0.1, 0.0, 0.0)
        assert mask.shape == (40, 40) and not mask.any()

    def test_a_passable_region_is_marked_and_a_blocked_one_is_not(self):
        from nav.resolved_regions import ResolvedRegion

        store = self._store(
            ResolvedRegion(-1.0, -1.0, 0.0, 0.0, passable=True),
            ResolvedRegion(0.5, 0.5, 1.5, 1.5, passable=False),
        )
        mask = store.passable_mask((40, 40), 0.1, 0.0, 0.0)
        assert mask[15, 15]  # inside the passable box (x=-0.5, y=-0.5)
        assert not mask[28, 28]  # inside the blocked box (x=0.8, y=0.8)
        assert not mask[0, 0]

    def test_a_later_blocked_report_overrides_an_earlier_passable_one(self):
        from nav.resolved_regions import ResolvedRegion

        store = self._store(
            ResolvedRegion(-1.0, -1.0, 1.0, 1.0, passable=True),
            ResolvedRegion(-0.5, -0.5, 0.5, 0.5, passable=False),
        )
        mask = store.passable_mask((40, 40), 0.1, 0.0, 0.0)
        assert mask[20 - 8, 20 - 8]  # in the outer passable ring
        assert not mask[20, 20]  # centre was later reported blocked

    def test_region_outside_the_grid_is_ignored(self):
        from nav.resolved_regions import ResolvedRegion

        store = self._store(ResolvedRegion(50.0, 50.0, 60.0, 60.0, passable=True))
        assert not store.passable_mask((40, 40), 0.1, 0.0, 0.0).any()
