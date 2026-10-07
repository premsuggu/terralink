"""Tests for nav/trace_render.py (the run-trace metrics and drawing). Pure
NumPy/matplotlib, no ROS or simulator: the traces are synthetic and the map is
the repo's real tunnel fixture."""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src", "nav"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src", "emap"))

from nav import trace_render as tr  # noqa: E402
from nav.replay import load_snapshot  # noqa: E402

FIXTURE = os.path.join(os.path.dirname(__file__), "fixtures", "tunnel_test_parked.npz")


def _poses(points, dt=0.1):
    """[(x, y), ...] sampled every `dt` s -> (t, x, y, yaw) rows."""
    return np.array([[i * dt, x, y, 0.0] for i, (x, y) in enumerate(points)])


def _straight(x0, x1, speed=0.5, dt=0.1, y=0.0):
    n = int(abs(x1 - x0) / (speed * dt))
    return [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1)]


def _trace(points, goal=(3.0, 0.0), plans=None):
    return {"meta": {"goal": list(goal)}, "poses": _poses(points), "plans": plans or [], "goals": [], "verdicts": []}


def test_straight_drive_metrics():
    pts = [(-3.0, 0.0)] * 20 + _straight(-3.0, 3.0)  # 2 s parked, then 6 m at 0.5 m/s
    m = tr.track_metrics(_trace(pts))
    assert m["reached"]
    assert m["first_move_s"] == pytest.approx(2.0, abs=0.5)
    assert m["move_to_arrive_s"] == pytest.approx(11.4, abs=1.0)  # arrives 0.3 m short of the goal
    assert m["driven_m"] == pytest.approx(5.7, abs=0.2)
    assert m["detour"] == pytest.approx(0.95, abs=0.05)
    assert m["stalled_s"] == 0
    assert m["reversals"] == 0


def test_not_reaching_the_goal_is_reported():
    m = tr.track_metrics(_trace(_straight(-3.0, 0.0)))
    assert not m["reached"]
    assert m["move_to_arrive_s"] is None


def test_stall_is_counted():
    pts = _straight(-3.0, -1.0) + [(-1.0, 0.0)] * 100 + _straight(-1.0, 3.0)  # 10 s standing still mid-way
    m = tr.track_metrics(_trace(pts))
    assert m["stalled_s"] >= 5.0
    assert m["reached"]


def test_reversal_detected_for_a_turn_back_but_not_for_a_gentle_curve():
    there_and_back = _straight(0.0, 2.0) + _straight(2.0, 0.0)
    assert tr.direction_reversals(_poses(there_and_back)) >= 1
    curve = [(np.cos(a) * 2, np.sin(a) * 2) for a in np.linspace(0, np.pi / 2, 80)]
    assert tr.direction_reversals(_poses(curve)) == 0


def test_dwell_points_find_a_pause_and_ignore_a_short_one():
    pts = _straight(0.0, 1.0) + [(1.0, 0.0)] * 50 + _straight(1.0, 2.0) + [(2.0, 0.0)] * 5 + _straight(2.0, 3.0)
    dw = tr.dwell_points(_poses(pts), min_sec=3.0)
    assert len(dw) == 1
    x, y, _t0, dur = dw[0]
    assert x == pytest.approx(1.0, abs=0.1) and dur >= 4.5


def test_merge_dwells_combines_nearby_pauses_only():
    d = [(1.0, 0.0, 5.0, 4.0), (1.2, 0.1, 12.0, 6.0), (4.0, 0.0, 30.0, 5.0)]
    m = tr.merge_dwells(d, radius=0.6)
    assert len(m) == 2
    assert m[0][3] == pytest.approx(10.0) and m[0][4] == 2 and m[0][2] == 5.0
    assert m[1][4] == 1


def test_unique_plans_merges_repeats_and_keeps_order():
    a = {"valid": True, "waypoints": [[0, 0], [1, 0], [2, 0]], "t": 1.0}
    a2 = {"valid": True, "waypoints": [[0, 0.02], [1, 0.01], [2, 0]], "t": 2.0}  # same route, 2 cm off
    b = {"valid": True, "waypoints": [[0, 0], [1, 1], [2, 0]], "t": 3.0}
    bad = {"valid": False, "waypoints": [], "t": 4.0}
    u = tr.unique_plans([a, a2, b, bad])
    assert [p["count"] for p in u] == [2, 1]
    assert u[0]["first_t"] == 1.0 and u[0]["last_t"] == 2.0


def test_plan_counts_in_metrics():
    plans = [
        {"valid": True, "waypoints": [[0, 0], [1, 0]], "t": 1.0, "has_anomaly": True},
        {"valid": False, "waypoints": [], "t": 2.0},
        {"valid": True, "waypoints": [[0, 0], [1, 1]], "t": 3.0, "exploration": "frontier"},
    ]
    m = tr.track_metrics(_trace(_straight(-3.0, 3.0), plans=plans))
    assert (m["plans"], m["plans_valid"], m["plans_distinct"], m["no_path"]) == (3, 2, 2, 1)
    assert m["exploration_plans"] == 1 and m["tentative_plans"] == 1


def test_load_trace_roundtrip(tmp_path):
    data = {"meta": {"goal": [3, 0]}, "poses": [[0.0, 1.0, 2.0, 0.0], [0.1, 1.1, 2.0, 0.0]], "plans": [], "goals": [], "verdicts": []}
    p = tmp_path / "trace.json"
    p.write_text(json.dumps(data))
    t = tr.load_trace(p)
    assert t["poses"].shape == (2, 4)


@pytest.mark.skipif(not os.path.exists(FIXTURE), reason="fixture missing")
def test_renderers_write_pngs(tmp_path):
    snap = load_snapshot(FIXTURE)
    plans = [
        {"valid": True, "waypoints": [[-3, 0], [-1.3, -0.2], [1.1, -0.2], [3, 0]], "kinds": ["free", "anomaly", "free"],
         "has_anomaly": True, "has_frontier": False, "exploration": "", "t": 3.0},
    ]
    trace = _trace([(-3.0, 0.0)] * 30 + _straight(-3.0, 3.0), plans=plans)
    trace["verdicts"] = [{"t": 9.0, "text": "PASSABLE"}]
    m = tr.render_run_png(tmp_path / "d.png", trace, snap, title="t")
    assert m["reached"] and (tmp_path / "d.png").stat().st_size > 5000
    out = tr.render_comparison_png(tmp_path / "c.png", {"A": [trace, trace], "B": [trace]}, snap, title="x")
    assert set(out) == {"A", "B"} and (tmp_path / "c.png").stat().st_size > 5000
