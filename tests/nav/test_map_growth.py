"""Unit tests for nav.map_growth - 'is the map still being built?'."""
from nav.map_growth import MapGrowthTracker


def _feed(tracker, series):
    for t, count in series:
        tracker.update(count, t)


class TestMapGrowthTracker:
    def test_before_any_history_the_map_counts_as_growing(self):
        assert MapGrowthTracker().growing(0.0)

    def test_a_single_sample_is_not_enough_to_call_it_stable(self):
        tr = MapGrowthTracker()
        tr.update(1000, 0.0)
        assert tr.growing(0.0)

    def test_steady_growth_is_growing(self):
        tr = MapGrowthTracker(window_sec=10.0, min_growth_frac=0.01)
        _feed(tr, [(t, 1000 + 50 * t) for t in range(0, 30, 5)])  # +5%/s
        assert tr.growing(25.0)

    def test_a_flat_count_over_a_full_window_is_stable(self):
        tr = MapGrowthTracker(window_sec=10.0, min_growth_frac=0.01)
        _feed(tr, [(t, 7000) for t in range(0, 30, 5)])
        assert not tr.growing(25.0)

    def test_tiny_growth_below_the_threshold_is_stable(self):
        tr = MapGrowthTracker(window_sec=10.0, min_growth_frac=0.01)
        _feed(tr, [(t, 7000 + t) for t in range(0, 30, 5)])  # ~0.14% per 10 s
        assert not tr.growing(25.0)

    def test_growth_that_stops_is_noticed_after_a_window(self):
        tr = MapGrowthTracker(window_sec=10.0, min_growth_frac=0.01)
        _feed(tr, [(0, 1000), (5, 2000), (10, 3000), (15, 3000), (20, 3000), (25, 3000)])
        assert not tr.growing(25.0)

    def test_a_short_history_is_not_called_stable_even_if_flat(self):
        tr = MapGrowthTracker(window_sec=10.0)
        _feed(tr, [(0, 5000), (2, 5000)])  # only 2 s of history
        assert tr.growing(2.0)

    def test_no_updates_for_a_whole_window_means_not_growing(self):
        tr = MapGrowthTracker(window_sec=10.0)
        _feed(tr, [(0, 1000), (5, 2000), (10, 3000)])
        assert not tr.growing(40.0)

    def test_old_samples_are_discarded(self):
        tr = MapGrowthTracker(window_sec=10.0)
        _feed(tr, [(t, 1000 + t) for t in range(0, 200, 1)])
        assert len(tr._samples) <= 14
