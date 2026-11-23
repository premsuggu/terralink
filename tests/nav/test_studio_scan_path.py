"""Auto-scan path + control step (no ROS): the sweep covers the area, stays inside it, and the step heads the right way."""
import json
import math

import pytest

from nav.studio.commands import CommandError, parse_command
from nav.studio.scan_path import autoscan_step, lawnmower


class TestLawnmower:
    def test_small_example_is_the_expected_zigzag(self):
        assert lawnmower((0, 0, 10, 10), 5.0) == [(0, 0), (0, 10), (5, 10), (5, 0), (10, 0), (10, 10)]

    def test_every_point_is_inside_the_area(self):
        for x, y in lawnmower((-13.5, 2.0, 21.0, 40.0), 5.0):
            assert -13.5 <= x <= 21.0 and 2.0 <= y <= 40.0

    def test_lanes_are_never_farther_apart_than_spacing_and_reach_both_edges(self):
        xs = sorted({x for x, _ in lawnmower((0, 0, 23.0, 9.0), 5.0)})
        assert xs[0] == 0 and xs[-1] == 23.0
        assert all(b - a <= 5.0 + 1e-9 for a, b in zip(xs, xs[1:]))

    def test_directions_alternate_so_no_empty_flying(self):
        pts = lawnmower((0, 0, 20, 10), 5.0)
        for i in range(1, len(pts) - 1, 2):                      # the move between lanes is short (one spacing), never a full lane back
            assert abs(pts[i][1] - pts[i + 1][1]) < 1e-9

    @pytest.mark.parametrize("bad", [(0, 0, 0, 5), (0, 0, 5, 0), (5, 0, 0, 5)])
    def test_empty_rectangles_are_rejected(self, bad):
        with pytest.raises(ValueError):
            lawnmower(bad)


class TestStep:
    def test_heads_for_the_target_at_cruise_speed_when_far(self):
        reached, (vx, vy, vz, wz) = autoscan_step((0, 0, 5, 0), (50, 0), 5, 1.2, 1.5)
        assert not reached and vx == pytest.approx(1.2) and vy == pytest.approx(0) and wz == 0

    def test_body_frame_rotation(self):
        # facing +y (yaw 90 deg), target is straight ahead in +y -> pure forward velocity
        _, (vx, vy, _, _) = autoscan_step((0, 0, 5, math.pi / 2), (0, 50), 5, 1.0, 1.5)
        assert vx == pytest.approx(1.0) and vy == pytest.approx(0, abs=1e-9)

    def test_slows_near_the_target_and_reports_arrival(self):
        _, (vx, _, _, _) = autoscan_step((0, 0, 5, 0), (1.0, 0), 5, 1.2, 1.5)
        assert 0.3 <= vx < 1.2
        assert autoscan_step((0, 0, 5, 0), (0.5, 0), 5, 1.2, 1.5)[0]

    def test_climbs_first_without_moving_sideways(self):
        reached, (vx, vy, vz, _) = autoscan_step((0, 0, 0.5, 0), (50, 0), 5, 1.2, 1.5)
        assert not reached and vx == 0 and vy == 0 and vz == pytest.approx(1.5)

    def test_holds_the_height_and_respects_the_vertical_limit(self):
        assert autoscan_step((0, 0, 5.0, 0), (50, 0), 5, 1.2, 1.5)[1][2] == pytest.approx(0)
        assert autoscan_step((0, 0, 20.0, 0), (50, 0), 5, 1.2, 0.5)[1][2] == pytest.approx(-0.5)


class TestCommand:
    def test_defaults_and_clamping(self):
        assert parse_command('{"t":"autoscan"}').data == {"alt": 5.0}
        assert parse_command(json.dumps({"t": "autoscan", "alt": 99})).data == {"alt": 10.0}
        assert parse_command(json.dumps({"t": "autoscan", "alt": 0.1})).data == {"alt": 3.0}

    def test_cancel_and_junk(self):
        assert parse_command('{"t":"autoscan_cancel"}').kind == "autoscan_cancel"
        with pytest.raises(CommandError):
            parse_command('{"t":"autoscan","alt":"high"}')
