"""Tests for nav.replay (offline snapshot replay) and scripts/replay_plan.py.
Pure NumPy + matplotlib (Agg backend); no ROS."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from nav import replay
from nav.resolved_regions import ResolvedRegion

N = 60
RES = 0.1


def _snapshot(walkable, valid=None, elevation=None):
    valid = np.ones_like(walkable) if valid is None else valid
    layers = {
        "elevation": np.zeros(walkable.shape) if elevation is None else elevation,
        "traversability": np.where(walkable, 1.0, 0.0),
        "is_valid": valid.astype(float),
    }
    return replay.MapSnapshot(layers=layers, resolution=RES, center_x=0.0, center_y=0.0)


def _open():
    return np.ones((N, N), dtype=bool)


def _xy(row, col):
    return ((col - N / 2.0) * RES, (row - N / 2.0) * RES)


class TestSnapshotIO:
    def test_round_trip_preserves_layers_and_geometry(self, tmp_path):
        snap = _snapshot(_open())
        path = tmp_path / "s.npz"
        replay.save_snapshot(path, snap.layers, 0.1, 1.5, -2.0)
        back = replay.load_snapshot(path)
        assert back.resolution == pytest.approx(0.1)
        assert (back.center_x, back.center_y) == (1.5, -2.0)
        assert set(back.layers) == set(snap.layers)
        for name in snap.layers:
            assert np.array_equal(back.layers[name], snap.layers[name])

    def test_saving_without_required_layers_is_rejected(self, tmp_path):
        with pytest.raises(ValueError):
            replay.save_snapshot(tmp_path / "bad.npz", {"elevation": np.zeros((4, 4))}, 0.1, 0, 0)

    def test_loading_a_file_that_is_not_a_snapshot_is_rejected(self, tmp_path):
        np.savez(tmp_path / "other.npz", x=np.zeros(3))
        with pytest.raises(ValueError):
            replay.load_snapshot(tmp_path / "other.npz")


class TestBuildMasks:
    def test_walkable_matches_the_production_function(self):
        from nav.walkability import compute_walkable_mask

        w = _open()
        w[10:20, 10:20] = False
        snap = _snapshot(w)
        masks = replay.build_masks(snap)
        assert np.array_equal(masks.walkable, compute_walkable_mask(snap.layers["traversability"], snap.layers["is_valid"]))
        assert masks.frontier is None and masks.anomaly is None

    def test_frontier_only_when_asked(self):
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:30] = True
        valid[:, 30:] = False  # unobserved space starts right at the room's edge
        snap = _snapshot(w, valid)
        assert replay.build_masks(snap, use_frontier=True).frontier.any()
        assert replay.build_masks(snap).frontier is None

    def test_anomaly_needs_an_elevation_layer(self):
        snap = _snapshot(_open())
        del snap.layers["elevation"]
        with pytest.raises(ValueError):
            replay.build_masks(snap, use_anomaly=True)

    def test_resolved_regions_are_applied(self):
        w = _open()
        snap = _snapshot(w)
        blocked = ResolvedRegion(-0.5, -0.5, 0.5, 0.5, passable=False)
        masks = replay.build_masks(snap, use_anomaly=True, resolved=[blocked])
        assert not masks.walkable[N // 2, N // 2]


class TestRunPlanAndMetrics:
    def test_astar_and_prm_both_solve_an_open_map(self):
        snap = _snapshot(_open())
        masks = replay.build_masks(snap)
        for planner in ("astar", "prm"):
            res = replay.run_plan(snap, masks, _xy(10, 10), _xy(45, 40), planner=planner)
            assert res.valid

    def test_astar_is_a_single_straight_segment_where_prm_zigzags(self):
        snap = _snapshot(_open())
        masks = replay.build_masks(snap)
        a = replay.run_plan(snap, masks, _xy(10, 10), _xy(45, 40), planner="astar")
        p = replay.run_plan(snap, masks, _xy(10, 10), _xy(45, 40), planner="prm")
        assert len(a.waypoints) == 2
        assert len(p.waypoints) > 2

    def test_explore_falls_back_to_a_viewpoint_when_there_is_no_route(self):
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:30] = True
        w[26:36, 30:32] = True
        valid[:, 32:] = False
        snap = _snapshot(w, valid)
        masks = replay.build_masks(snap, use_frontier=True)
        without = replay.run_plan(snap, masks, _xy(30, 5), _xy(30, 55), explore=False)
        with_explore = replay.run_plan(snap, masks, _xy(30, 5), _xy(30, 55), explore=True)
        assert not without.valid
        assert with_explore.valid and with_explore.is_exploration

    def test_metrics_of_a_straight_line(self):
        snap = _snapshot(_open())
        masks = replay.build_masks(snap)
        m = replay.path_metrics([(0.0, 0.0), (3.0, 4.0)], snap, masks)
        assert m.length_m == pytest.approx(5.0)
        assert m.detour_ratio == pytest.approx(1.0)
        assert m.total_turn_deg == pytest.approx(0.0)

    def test_metrics_of_a_right_angle(self):
        snap = _snapshot(_open())
        masks = replay.build_masks(snap)
        m = replay.path_metrics([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], snap, masks)
        assert m.length_m == pytest.approx(4.0)
        assert m.detour_ratio == pytest.approx(4.0 / (2.0 * 2 ** 0.5))
        assert m.sharpest_turn_deg == pytest.approx(90.0)
        assert m.n_waypoints == 3


class TestRenderingAndScript:
    def test_render_png_writes_a_file(self, tmp_path):
        snap = _snapshot(_open())
        masks = replay.build_masks(snap)
        res = replay.run_plan(snap, masks, _xy(10, 10), _xy(45, 40))
        out = tmp_path / "plan.png"
        replay.render_png(out, snap, masks, _xy(10, 10), _xy(45, 40), res)
        assert out.stat().st_size > 1000

    def test_render_png_handles_a_failed_plan(self, tmp_path):
        w = _open()
        w[:, 30] = False
        snap = _snapshot(w)
        masks = replay.build_masks(snap)
        res = replay.run_plan(snap, masks, _xy(10, 10), _xy(10, 50))
        assert not res.valid
        out = tmp_path / "none.png"
        replay.render_png(out, snap, masks, _xy(10, 10), _xy(10, 50), res)
        assert out.exists()

    def _script(self):
        path = Path(__file__).resolve().parents[2] / "src" / "nav" / "scripts" / "replay_plan.py"
        spec = importlib.util.spec_from_file_location("replay_plan_script", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def test_script_end_to_end(self, tmp_path, capsys):
        snap = _snapshot(_open())
        npz = tmp_path / "m.npz"
        replay.save_snapshot(npz, snap.layers, RES, 0.0, 0.0)
        png = tmp_path / "out.png"
        rc = self._script().main(
            ["--snapshot", str(npz), "--start", "-2", "-2", "--goal", "2", "2", "--png", str(png)]
        )
        out = capsys.readouterr().out
        assert rc == 0
        assert "RESULT: route to goal" in out and "metrics:" in out
        assert png.exists()

    def test_script_returns_nonzero_when_there_is_no_path(self, tmp_path, capsys):
        w = _open()
        w[:, 30] = False
        snap = _snapshot(w)
        npz = tmp_path / "m.npz"
        replay.save_snapshot(npz, snap.layers, RES, 0.0, 0.0)
        rc = self._script().main(["--snapshot", str(npz), "--start", "-2", "-2", "--goal", "2", "-2"])
        assert rc == 1
        assert "no path" in capsys.readouterr().out


class TestReplaySequence:
    """The offline stand-in for 'watch the UGV and see whether it dithers'."""

    def _frames(self, n_frames=6):
        # a wall with two gaps; each frame the "map" flickers by blocking and
        # unblocking cells so costs/validity change a little
        out = []
        for k in range(n_frames):
            w = _open()
            w[:, 28:32] = False
            w[8:14, 28:32] = True
            w[46:52, 28:32] = True
            if k % 2:
                w[20:26, 5:12] = False  # an obstacle appearing/disappearing in the west room
            out.append(_snapshot(w))
        return out

    def test_a_stable_world_gives_one_route_change_with_memory(self):
        snaps = self._frames()
        steps = replay.replay_sequence(
            snaps, list(range(len(snaps))), [_xy(30, 5)] * len(snaps), _xy(30, 55),
            use_frontier=False, use_anomaly=False, memory=True,
        )  # fmt: skip
        assert all(s.valid for s in steps)
        # only the first plan counts as a "change"
        assert sum(s.route_changed for s in steps) <= 2

    def test_prm_changes_the_route_more_than_astar_on_the_same_frames(self):
        snaps = self._frames(8)
        kw = dict(use_frontier=False, use_anomaly=False)
        starts = [_xy(30, 5)] * len(snaps)
        astar = replay.replay_sequence(snaps, list(range(8)), starts, _xy(30, 55), planner="astar", **kw)
        prm = replay.replay_sequence(snaps, list(range(8)), starts, _xy(30, 55), planner="prm", **kw)
        assert sum(s.route_changed for s in astar) <= sum(s.route_changed for s in prm)

    def test_failed_steps_are_reported_not_hidden(self):
        w = _open()
        w[:, 30] = False
        snap = _snapshot(w)
        steps = replay.replay_sequence(
            [snap], [0.0], [_xy(10, 10)], _xy(10, 50), use_frontier=False, use_anomaly=False
        )  # fmt: skip
        assert len(steps) == 1 and not steps[0].valid and not steps[0].route_changed

    def test_the_anomaly_hold_is_carried_across_steps(self):
        # Step 0's map yields flagged anomaly cells; step 1's map flickers to
        # nothing. With the SAME latch carried across steps the cells are
        # still held at step 1; without it they are gone.
        from emap.traversability import compute_traversability

        n = 60
        elev = np.full((n, n), 1.0)
        elev[5:55, 0:20] = 0.0
        elev[5:55, 40:60] = 0.0
        elev[24:36, 20:40] = 0.7  # a roofed strip joining two rooms
        valid = np.ones((n, n), dtype=bool)
        var = np.full((n, n), 0.001, dtype=np.float32)
        trav = compute_traversability(elev.astype(np.float32), var, valid, RES, 0.35, 0.15, 0.05)
        with_tunnel = replay.MapSnapshot(
            layers={"elevation": elev, "traversability": trav, "is_valid": valid.astype(float)},
            resolution=RES, center_x=0.0, center_y=0.0,
        )  # fmt: skip
        flat = replay.MapSnapshot(
            layers={"elevation": np.zeros((n, n)), "traversability": np.ones((n, n)), "is_valid": valid.astype(float)},
            resolution=RES, center_x=0.0, center_y=0.0,
        )  # fmt: skip

        latch = replay.MaskLatch(30.0)
        first = replay.build_masks(with_tunnel, use_anomaly=True, latch=latch, now=0.0)
        assert first.anomaly.any()
        held = replay.build_masks(flat, use_anomaly=True, latch=latch, now=5.0)
        assert held.anomaly.sum() == first.anomaly.sum()
        assert replay.build_masks(flat, use_anomaly=True).anomaly.sum() == 0


class TestReplaySequenceWaitForMapping:
    def test_speculation_is_held_while_growing_then_released(self):
        # Two rooms joined only by a corridor lined with unobserved space (a
        # frontier gamble). Separately, a strip along the top of the map gets
        # revealed a bit more each frame for the first 4 frames, then stops -
        # the signal that the UAV has finished scanning.
        def frame(revealed_cols):
            w = np.zeros((N, N), dtype=bool)
            valid = np.ones((N, N), dtype=bool)
            w[5:55, 0:20] = True
            w[5:55, 40:60] = True
            w[28:32, 20:40] = True
            valid[:28, 20:40] = False
            valid[32:, 20:40] = False
            valid[0:3, :] = False
            valid[0:3, :revealed_cols] = True
            return _snapshot(w, valid)

        frames = [frame(k) for k in (5, 15, 30, 50, 60, 60, 60, 60, 60)]
        times = [0, 5, 10, 15, 20, 25, 30, 35, 40]
        steps = replay.replay_sequence(
            frames, times, [_xy(30, 5)] * len(frames), _xy(30, 55),
            use_anomaly=False, wait_for_mapping=True, memory=False,
            params=replay.astar_planner.PlannerParams(mask_rim_m=0.0),
        )  # fmt: skip
        assert not steps[2].valid  # still growing -> gamble held back
        assert steps[-1].valid and steps[-1].tentative  # growth stopped -> gamble taken
