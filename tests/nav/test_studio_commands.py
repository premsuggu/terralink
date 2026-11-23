"""Studio commands: parsing, server-side limits, dead-man and e-stop (no ROS)."""
import json

import pytest

from nav.studio.commands import Command, CommandError, UavCommandState, UavLimits, parse_command, uav_twist


def _cmd(**kw):
    return parse_command(json.dumps({"t": "uav_vel", **kw}))


class TestParse:
    def test_a_full_uav_command(self):
        c = _cmd(vx=0.5, vy=-0.25, vz=1.0, wz=-1.0, scale=0.5)
        assert c.kind == "uav_vel" and c.data == {"vx": 0.5, "vy": -0.25, "vz": 1.0, "wz": -1.0, "scale": 0.5}

    def test_missing_axes_default_to_zero_and_full_scale(self):
        assert _cmd(vx=1.0).data == {"vx": 1.0, "vy": 0.0, "vz": 0.0, "wz": 0.0, "scale": 1.0}

    def test_out_of_range_sticks_are_clamped_not_trusted(self):
        c = _cmd(vx=50, vy=-9, vz=2, wz=-3, scale=7)
        assert c.data == {"vx": 1.0, "vy": -1.0, "vz": 1.0, "wz": -1.0, "scale": 1.0}

    @pytest.mark.parametrize("bad", ['{"t":"uav_vel","vx":NaN}', '{"t":"uav_vel","vx":"1"}', '{"t":"uav_vel","vx":true}',
                                     '{"t":"uav_vel","vx":null}', '{"t":"uav_vel","vx":Infinity}'])
    def test_non_numbers_and_non_finite_numbers_are_rejected(self, bad):
        with pytest.raises(CommandError):
            parse_command(bad)

    @pytest.mark.parametrize("bad", ["", "not json", "[]", "3", '{"x":1}', '{"t":5}', '{"t":"fly_to_the_moon"}'])
    def test_malformed_or_unknown_messages_are_rejected(self, bad):
        with pytest.raises(CommandError):
            parse_command(bad)

    def test_goal_needs_both_coordinates(self):
        assert parse_command('{"t":"goal","x":1.5,"y":-2}').data == {"x": 1.5, "y": -2.0}
        with pytest.raises(CommandError):
            parse_command('{"t":"goal","x":1.5}')

    def test_goal_is_bounded_to_a_sane_region(self):
        assert parse_command('{"t":"goal","x":1e9,"y":-1e9}').data == {"x": 1000.0, "y": -1000.0}

    @pytest.mark.parametrize("kind", ["estop", "estop_release", "ping"])
    def test_simple_commands(self, kind):
        assert parse_command(json.dumps({"t": kind})) == Command(kind)


class TestLimits:
    def test_full_stick_gives_exactly_the_limit(self):
        lim = UavLimits(max_xy=3.0, max_z=1.5, max_yaw=1.0)
        assert uav_twist(_cmd(vx=1, vy=-1, vz=1, wz=-1), lim) == (3.0, -3.0, 1.5, -1.0)

    def test_slow_mode_scales_every_axis(self):
        assert uav_twist(_cmd(vx=1, vy=1, vz=1, wz=1, scale=0.5), UavLimits(2.0, 1.0, 0.8)) == (1.0, 1.0, 0.5, 0.4)

    def test_no_message_can_exceed_the_limit(self):
        lim = UavLimits(max_xy=2.5, max_z=1.0, max_yaw=0.5)
        for v in (1.0, 5.0, 1e30):
            vx, vy, vz, wz = uav_twist(_cmd(vx=v, vy=v, vz=v, wz=v, scale=v), lim)
            assert abs(vx) <= 2.5 and abs(vy) <= 2.5 and abs(vz) <= 1.0 and abs(wz) <= 0.5


class TestDeadmanAndEstop:
    def _state(self):
        return UavCommandState(UavLimits(3.0, 1.5, 1.0), deadman_sec=0.5)

    def test_nothing_is_sent_before_any_command(self):
        assert self._state().current(0.0) == (0.0, 0.0, 0.0, 0.0)

    def test_a_fresh_command_is_held_then_expires(self):
        s = self._state()
        s.handle(_cmd(vx=1), now=10.0)
        assert s.current(10.2) == (3.0, 0.0, 0.0, 0.0)
        assert s.current(10.5) == (3.0, 0.0, 0.0, 0.0)          # exactly at the limit is still fresh
        assert s.current(10.51) == (0.0, 0.0, 0.0, 0.0)         # stale: the dead-man stops the UAV

    def test_a_newer_command_replaces_the_old_one(self):
        s = self._state()
        s.handle(_cmd(vx=1), 0.0)
        s.handle(_cmd(vz=1), 0.1)
        assert s.current(0.2) == (0.0, 0.0, 1.5, 0.0)

    def test_estop_zeroes_at_once_and_ignores_commands_until_released(self):
        s = self._state()
        s.handle(_cmd(vx=1), 0.0)
        s.handle(Command("estop"), 0.1)
        assert s.current(0.1) == (0.0, 0.0, 0.0, 0.0)
        s.handle(_cmd(vx=1), 0.2)                                # a stuck stick must not restart the UAV
        assert s.current(0.25) == (0.0, 0.0, 0.0, 0.0)
        s.handle(Command("estop_release"), 0.3)
        assert s.current(0.3) == (0.0, 0.0, 0.0, 0.0)            # released, but nothing commanded since: still zero
        s.handle(_cmd(vx=1), 0.4)
        assert s.current(0.45) == (3.0, 0.0, 0.0, 0.0)

    def test_a_command_sent_during_estop_does_not_resume_after_release(self):
        s = self._state()
        s.handle(Command("estop"), 0.0)
        s.handle(_cmd(vx=1), 0.1)
        s.handle(Command("estop_release"), 0.2)
        assert s.current(0.25) == (0.0, 0.0, 0.0, 0.0)


class TestSettings:
    """P5: run-time limits. The server clamps every value to HARD_CAPS, so the page can never ask for something unsafe."""

    def test_a_partial_update_keeps_only_the_given_keys(self):
        c = parse_command('{"t":"settings","max_xy":2.0}')
        assert c.kind == "settings" and c.data == {"max_xy": 2.0}

    def test_values_are_clamped_to_the_hard_caps(self):
        from nav.studio.commands import HARD_CAPS
        c = parse_command('{"t":"settings","max_xy":99,"ugv_max_lin":9,"ceiling":100,"max_z":-5}')
        assert c.data == {"max_xy": HARD_CAPS["max_xy"][1], "ugv_max_lin": HARD_CAPS["ugv_max_lin"][1], "ceiling": 14.0, "max_z": HARD_CAPS["max_z"][0]}

    def test_the_ugv_cap_never_exceeds_the_tip_over_limit(self):
        from nav.studio.commands import HARD_CAPS
        assert HARD_CAPS["ugv_max_lin"][1] <= 1.0 and HARD_CAPS["ugv_max_ang"][1] <= 1.5

    @pytest.mark.parametrize("bad", ['{"t":"settings"}', '{"t":"settings","unknown":1}', '{"t":"settings","max_xy":"fast"}',
                                     '{"t":"settings","max_xy":NaN}', '{"t":"settings","max_xy":true}'])
    def test_nonsense_is_rejected(self, bad):
        with pytest.raises(CommandError):
            parse_command(bad)
