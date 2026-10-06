"""Unit tests for nav.plan_memory - route stability. Pure Python for the
decision logic (fake evaluators), plus end-to-end checks against the real
planner and `evaluate_path` for the behaviour that matters in practice."""
import math

import numpy as np
import pytest

from nav import astar_planner as ap
from nav.astar_planner import AStarResult
from nav.plan_memory import PlanMemory, trim_path

RES = 0.1


def _route(wps, cost, kinds=None, mode="strict", infl=0.2):
    kinds = kinds or ["free"] * (len(wps) - 1)
    return AStarResult(
        waypoints=wps,
        valid=True,
        has_anomaly_segments="anomaly" in kinds,
        has_frontier_segments="frontier" in kinds,
        segment_kinds=kinds,
        cost=cost,
        inflation_m=infl,
        mode=mode,
    )


def _fixed_evaluator(table):
    """evaluate(wps, mode, infl) that looks the answer up by the number of
    waypoints, so a test can say 'the old (ahead) route costs X' directly."""

    def evaluate(wps, mode, infl):
        return table[len(wps)]

    return evaluate


GOAL = (5.0, 0.0)
OLD = [(0.0, 0.0), (2.0, 1.0), (5.0, 0.0)]
NEW = [(0.0, 0.0), (5.0, 0.0)]


class TestTrimPath:
    def test_trims_to_the_part_still_ahead(self):
        wps = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
        out = trim_path(wps, (3.0, 0.2))
        assert out[0] == pytest.approx((3.0, 0.0))
        assert out[1:] == [(4.0, 0.0), (4.0, 4.0)]

    def test_robot_before_the_path_keeps_the_whole_thing(self):
        wps = [(1.0, 0.0), (4.0, 0.0)]
        out = trim_path(wps, (0.0, 0.0))
        assert out[0] == pytest.approx((1.0, 0.0))
        assert out[-1] == (4.0, 0.0)

    def test_robot_past_the_end_leaves_only_the_goal(self):
        wps = [(0.0, 0.0), (4.0, 0.0)]
        out = trim_path(wps, (9.0, 0.0))
        assert out[-1] == (4.0, 0.0)
        assert all(math.isclose(p[1], 0.0) for p in out)

    def test_single_waypoint_is_returned_as_is(self):
        assert trim_path([(1.0, 1.0)], (0.0, 0.0)) == [(1.0, 1.0)]


class TestDecisions:
    def test_first_plan_is_adopted(self):
        mem = PlanMemory()
        new = _route(NEW, 5.0)
        d = mem.decide(new, (0.0, 0.0), GOAL, _fixed_evaluator({}))
        assert not d.kept and d.result is new
        assert mem.stored is new

    def test_similar_cost_new_route_does_not_replace_the_stored_one(self):
        mem = PlanMemory(switch_margin=0.15)
        old = _route(OLD, 5.5)
        mem.decide(old, (0.0, 0.0), GOAL, _fixed_evaluator({}))
        new = _route(NEW, 5.0)
        # old ahead costs 5.4, new costs 5.0 -> only 7% cheaper, under the 15% margin
        ev = _fixed_evaluator({len(trim_path(OLD, (0.1, 0.0))): (True, 5.4, ["free", "free"]), 2: (True, 5.0, ["free"])})
        d = mem.decide(new, (0.1, 0.0), GOAL, ev)
        assert d.kept
        assert d.result.waypoints == OLD  # the ORIGINAL list, untouched

    def test_clearly_cheaper_route_replaces_it(self):
        mem = PlanMemory(switch_margin=0.15)
        mem.decide(_route(OLD, 8.0), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        new = _route(NEW, 5.0)
        ev = _fixed_evaluator({len(trim_path(OLD, (0.1, 0.0))): (True, 8.0, ["free", "free"]), 2: (True, 5.0, ["free"])})
        d = mem.decide(new, (0.1, 0.0), GOAL, ev)
        assert not d.kept and d.result is new

    def test_invalid_stored_route_is_replaced_whatever_the_cost(self):
        mem = PlanMemory()
        mem.decide(_route(OLD, 5.0), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        new = _route(NEW, 99.0)
        ev = _fixed_evaluator({len(trim_path(OLD, (0.1, 0.0))): (False, math.inf, []), 2: (True, 99.0, ["free"])})
        d = mem.decide(new, (0.1, 0.0), GOAL, ev)
        assert not d.kept and d.result is new

    def test_confirmed_route_replaces_a_tentative_one_even_if_costlier(self):
        mem = PlanMemory()
        mem.decide(_route(OLD, 5.0, ["free", "anomaly"]), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        new = _route(NEW, 9.0)
        ev = _fixed_evaluator(
            {len(trim_path(OLD, (0.1, 0.0))): (True, 5.0, ["free", "anomaly"]), 2: (True, 9.0, ["free"])}
        )
        d = mem.decide(new, (0.1, 0.0), GOAL, ev)
        assert not d.kept and d.result is new

    def test_no_new_route_but_stored_still_valid_keeps_the_stored_one(self):
        mem = PlanMemory()
        mem.decide(_route(OLD, 5.0), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        ev = _fixed_evaluator({len(trim_path(OLD, (0.1, 0.0))): (True, 5.0, ["free", "free"])})
        d = mem.decide(AStarResult(valid=False), (0.1, 0.0), GOAL, ev)
        assert d.kept and d.result.valid

    def test_no_new_route_and_stored_invalid_reports_failure_and_forgets(self):
        mem = PlanMemory()
        mem.decide(_route(OLD, 5.0), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        ev = _fixed_evaluator({len(trim_path(OLD, (0.1, 0.0))): (False, math.inf, [])})
        d = mem.decide(AStarResult(valid=False), (0.1, 0.0), GOAL, ev)
        assert not d.result.valid
        assert mem.stored is None

    def test_a_different_goal_resets_the_memory(self):
        mem = PlanMemory()
        mem.decide(_route(OLD, 5.0), (0.0, 0.0), GOAL, _fixed_evaluator({}))
        new = _route(NEW, 50.0)
        d = mem.decide(new, (0.0, 0.0), (9.0, 9.0), _fixed_evaluator({}))
        assert not d.kept and d.result is new

    def test_kept_route_flags_describe_only_what_is_still_ahead(self):
        # the robot has driven past the tentative segment: the kept route is
        # no longer tentative
        mem = PlanMemory()
        old = _route(OLD, 5.0, ["anomaly", "free"])
        mem.decide(old, (0.0, 0.0), GOAL, _fixed_evaluator({}))
        ev = _fixed_evaluator({len(trim_path(OLD, (4.5, 0.2))): (True, 1.0, ["free"]), 2: (True, 1.0, ["free"])})
        d = mem.decide(_route(NEW, 5.0), (4.5, 0.2), GOAL, ev)
        assert d.kept
        assert not d.result.has_anomaly_segments


# --------------------------------------------------------------------------
# End to end with the real planner
# --------------------------------------------------------------------------


def _xy(row, col, n):
    return ((col - n / 2.0) * RES, (row - n / 2.0) * RES)


def _evaluator_for(walkable, start_xy, **kw):
    def evaluate(wps, mode, infl):
        return ap.evaluate_path(wps, walkable, RES, 0.0, 0.0, start_xy, mode, infl, mask_rim_m=0.0, **kw)

    return evaluate


def _plan(walkable, start_xy, goal_xy, **kw):
    return ap.plan(walkable, RES, 0.0, 0.0, start_xy, goal_xy, mask_rim_m=0.0, **kw)


class TestWithTheRealPlanner:
    def _two_route_map(self):
        # A wall block with a gap near the top (route 1) and a gap near the
        # bottom (route 2). With the start/goal vertically centred the two are
        # almost the same length - the classic flip-flop situation.
        n = 60
        w = np.ones((n, n), dtype=bool)
        w[:, 28:32] = False
        w[8:14, 28:32] = True  # gap 1
        w[46:52, 28:32] = True  # gap 2
        return w

    @staticmethod
    def _which_gap(waypoints):
        """Topology of a route on the two-gap map: went through the top gap
        (y < 0) or the bottom one (y > 0)?"""
        return "top" if any(p[1] < -1.0 for p in waypoints) else "bottom"

    @staticmethod
    def _noisy_traversability(shape, seed):
        """Fusion-like noise: a scatter of DIFFICULT cells that changes costs
        a little without ever blocking anything."""
        rng = np.random.default_rng(seed)
        trav = np.ones(shape)
        trav[rng.random(shape) < 0.08] = 0.3
        return trav

    def _run(self, use_memory):
        w = self._two_route_map()
        n = w.shape[0]
        start, goal = _xy(30, 5, n), _xy(30, 55, n)
        mem = PlanMemory()
        chosen, flips = None, 0
        for seed in range(40):
            trav = self._noisy_traversability(w.shape, seed)
            new = _plan(w, start, goal, traversability=trav)
            if use_memory:
                wps = mem.decide(new, start, goal, _evaluator_for(w, start, traversability=trav)).result.waypoints
            else:
                wps = new.waypoints
            gap = self._which_gap(wps)
            if chosen is not None and gap != chosen:
                flips += 1
            chosen = gap
        return flips

    def test_without_memory_cost_noise_flips_between_two_equal_routes(self):
        # Guards the next test against passing trivially: the raw planner
        # really does flip under this noise.
        assert self._run(use_memory=False) > 0

    def test_with_memory_cost_noise_never_flips_the_route(self):
        assert self._run(use_memory=True) == 0

    def test_a_blocked_gap_forces_a_switch(self):
        w = self._two_route_map()
        n = w.shape[0]
        start, goal = _xy(30, 5, n), _xy(30, 55, n)
        mem = PlanMemory()
        first = mem.decide(_plan(w, start, goal), start, goal, _evaluator_for(w, start))
        w2 = w.copy()
        # close whichever gap the first route used
        used_top = any(p[1] < 0 for p in first.result.waypoints)
        if used_top:
            w2[8:14, 28:32] = False
        else:
            w2[46:52, 28:32] = False
        d = mem.decide(_plan(w2, start, goal), start, goal, _evaluator_for(w2, start))
        assert not d.kept
        assert d.result.valid
        assert d.result.waypoints != first.result.waypoints

    def test_a_much_shorter_route_opening_up_is_taken(self):
        n = 60
        w = np.ones((n, n), dtype=bool)
        w[:, 28:32] = False
        w[0:4, 28:32] = True  # only a far-away gap at the very top
        start, goal = _xy(55, 5, n), _xy(55, 55, n)
        mem = PlanMemory()
        mem.decide(_plan(w, start, goal), start, goal, _evaluator_for(w, start))
        w2 = w.copy()
        w2[50:60, 28:32] = True  # a door right next to the robot's line
        d = mem.decide(_plan(w2, start, goal), start, goal, _evaluator_for(w2, start))
        assert not d.kept
