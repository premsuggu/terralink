"""Studio commands added for the UGV: manual driving, mode, cancel, and the UAV flight bounds (no ROS)."""
import json

import pytest

from nav.studio.commands import (CommandError, UavBounds, UgvCommandState, UgvLimits, apply_uav_bounds, parse_command, ugv_twist)


def _p(**kw):
    return parse_command(json.dumps(kw))


class TestParse:
    def test_ugv_vel_is_clamped_and_defaults(self):
        c = _p(t="ugv_vel", vx=5, wz=-9)
        assert c.data == {"vx": 1.0, "wz": -1.0, "scale": 1.0}

    def test_ugv_vel_rejects_non_numbers(self):
        with pytest.raises(CommandError):
            _p(t="ugv_vel", vx="fast")

    def test_mode_accepts_only_the_two_modes(self):
        assert _p(t="mode", mode="manual").data == {"mode": "manual"}
        for bad in ("fly", None, 1):
            with pytest.raises(CommandError):
                _p(t="mode", mode=bad)

    def test_cancel_goal_has_no_payload(self):
        assert _p(t="cancel_goal").kind == "cancel_goal"


class TestUgvState:
    def test_limits_applied_on_the_server(self):
        c = _p(t="ugv_vel", vx=1, wz=1, scale=0.5)
        assert ugv_twist(c, UgvLimits(max_lin=1.0, max_ang=1.5)) == (0.5, 0.75)

    def test_dead_man_zeroes_a_stale_command(self):
        st = UgvCommandState(UgvLimits(), deadman_sec=0.5)
        st.handle(_p(t="ugv_vel", vx=1), now=10.0)
        assert st.current(10.2) == (1.0, 0.0)
        assert st.current(10.6) == (0.0, 0.0)

    def test_nothing_before_the_first_command_and_after_clear(self):
        st = UgvCommandState()
        assert st.current(0.0) == (0.0, 0.0)
        st.handle(_p(t="ugv_vel", vx=1), now=1.0)
        st.clear()
        assert st.current(1.1) == (0.0, 0.0)


class TestBounds:
    B = UavBounds(min_z=0.6, max_z=10.0, fence=(-5, -5, 5, 5))

    def test_unknown_pose_changes_nothing(self):
        assert apply_uav_bounds((1, 1, 1, 1), None, self.B) == (1, 1, 1, 1)

    def test_ceiling_stops_climbing_but_allows_descending(self):
        assert apply_uav_bounds((0, 0, 1, 0), (0, 0, 10.0, 0), self.B)[2] == 0.0
        assert apply_uav_bounds((0, 0, -1, 0), (0, 0, 10.0, 0), self.B)[2] == -1

    def test_floor_stops_descending_but_allows_take_off(self):
        assert apply_uav_bounds((0, 0, -1, 0), (0, 0, 0.5, 0), self.B)[2] == 0.0
        assert apply_uav_bounds((0, 0, 1, 0), (0, 0, 0.1, 0), self.B)[2] == 1

    def test_fence_removes_only_the_outward_part(self):
        # at the east edge facing east: forward (+x) blocked, sideways (+y = left = north) still free
        out = apply_uav_bounds((1, 1, 0, 0), (5.0, 0, 3, 0), self.B)
        assert out[0] == pytest.approx(0.0) and out[1] == pytest.approx(1.0)

    def test_fence_uses_the_heading(self):
        # facing north (yaw 90deg) at the east edge: body +y (left) points WEST = inward, body +x points north = free
        out = apply_uav_bounds((1, -1, 0, 0), (5.0, 0, 3, 1.5707963), self.B)   # body -y = east = outward -> removed
        assert out[0] == pytest.approx(1.0, abs=1e-6) and out[1] == pytest.approx(0.0, abs=1e-6)

    def test_inside_the_fence_nothing_changes(self):
        assert apply_uav_bounds((1, 1, 0.5, 0.2), (0, 0, 3, 0.7), self.B) == pytest.approx((1, 1, 0.5, 0.2))

    def test_no_fence_means_no_horizontal_limit(self):
        b = UavBounds(fence=None)
        assert apply_uav_bounds((1, 1, 0, 0), (999, 999, 3, 0), b) == (1, 1, 0, 0)
