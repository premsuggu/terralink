"""Unit tests for nav.autopilot - pure logic, no ROS. Covers three
independent pieces: compute_hold_command (per-tick velocity from position
error, in WORLD frame), world_to_body_xy (the missing WORLD->BODY frame
conversion - see autopilot.py's AutopilotCommand docstring for the real bug
this fixes), and WaypointSequencer (dwell-and-advance state machine).
"""
import math

import pytest

from nav.autopilot import WaypointSequencer, compute_hold_command, world_to_body_xy


class TestComputeHoldCommand:
    def test_zero_error_produces_zero_command(self):
        cmd = compute_hold_command((1.0, 2.0, 3.0), (1.0, 2.0, 3.0), xy_gain=0.5, z_gain=0.5, max_xy_speed=1.0, max_z_speed=1.0)
        assert cmd.vx == cmd.vy == cmd.vz == 0.0

    def test_command_points_toward_target(self):
        cmd = compute_hold_command((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), xy_gain=0.1, z_gain=0.1, max_xy_speed=5.0, max_z_speed=5.0)
        assert cmd.vx > 0
        assert cmd.vy == 0.0

    def test_xy_speed_is_clamped_to_max_even_for_a_large_error(self):
        cmd = compute_hold_command((0.0, 0.0, 0.0), (100.0, 100.0, 0.0), xy_gain=1.0, z_gain=1.0, max_xy_speed=2.0, max_z_speed=2.0)
        speed = math.hypot(cmd.vx, cmd.vy)
        assert speed == pytest.approx(2.0)

    def test_clamping_preserves_direction_not_just_magnitude(self):
        # A 3-4-5 triangle error: clamped output must still point the same
        # way (proportionally scaled), not just be independently clamped
        # per-axis (which would distort the direction of travel).
        cmd = compute_hold_command((0.0, 0.0, 0.0), (3.0, 4.0, 0.0), xy_gain=10.0, z_gain=1.0, max_xy_speed=1.0, max_z_speed=1.0)
        assert cmd.vx == pytest.approx(0.6)  # 3/5 * 1.0
        assert cmd.vy == pytest.approx(0.8)  # 4/5 * 1.0

    def test_z_speed_is_clamped_independently_of_xy(self):
        cmd = compute_hold_command((0.0, 0.0, 0.0), (0.0, 0.0, 100.0), xy_gain=1.0, z_gain=1.0, max_xy_speed=5.0, max_z_speed=0.8)
        assert cmd.vz == pytest.approx(0.8)
        assert cmd.vx == cmd.vy == 0.0


class TestWorldToBodyXy:
    def test_zero_yaw_is_the_identity(self):
        vx, vy = world_to_body_xy(3.0, -2.0, yaw_rad=0.0)
        assert vx == pytest.approx(3.0)
        assert vy == pytest.approx(-2.0)

    def test_90_degree_yaw_matches_hand_derived_rotation(self):
        # Vehicle yawed +90deg (body-x now points world +y, body-y points
        # world -x) - to produce pure world +x motion, the body-frame
        # command must be (vx=0, vy=-1). This is exactly the live bug this
        # module fixes: applying a WORLD-frame (1, 0) command directly as
        # BODY-frame at this yaw would send the vehicle toward world +y
        # instead of +x.
        vx, vy = world_to_body_xy(1.0, 0.0, yaw_rad=math.pi / 2)
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(-1.0)

    def test_180_degree_yaw_negates_both_axes(self):
        vx, vy = world_to_body_xy(1.0, 1.0, yaw_rad=math.pi)
        assert vx == pytest.approx(-1.0)
        assert vy == pytest.approx(-1.0)

    def test_preserves_magnitude_for_any_yaw(self):
        # A rotation never changes vector length - this holds for every
        # yaw, not just the axis-aligned cases above.
        for yaw in (0.3, 1.1, 2.4, -1.7, math.pi):
            vx, vy = world_to_body_xy(2.0, -3.0, yaw_rad=yaw)
            assert math.hypot(vx, vy) == pytest.approx(math.hypot(2.0, -3.0))


class TestWaypointSequencer:
    def _seq(self, **overrides):
        defaults = dict(
            waypoints=[(0.0, 0.0, 5.0), (10.0, 0.0, 5.0), (10.0, 10.0, 5.0)],
            arrival_radius_m=0.5,
            dwell_time_sec=3.0,
            loop=True,
        )
        defaults.update(overrides)
        return WaypointSequencer(**defaults)

    def test_starts_at_the_first_waypoint(self):
        seq = self._seq()
        assert seq.current_target == (0.0, 0.0, 5.0)

    def test_does_not_advance_while_still_approaching(self):
        seq = self._seq()
        # Far from the first waypoint, dt doesn't matter - never "arrived".
        for _ in range(10):
            seq.update(current_xyz=(50.0, 50.0, 5.0), dt_sec=1.0)
        assert seq.current_target == (0.0, 0.0, 5.0)

    def test_advances_only_after_the_full_dwell_time_at_the_target(self):
        seq = self._seq()
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=1.0)
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=1.0)
        assert seq.current_target == (0.0, 0.0, 5.0)  # only 2s of a 3s dwell
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=1.0)
        assert seq.current_target == (10.0, 0.0, 5.0)  # 3s reached - advanced

    def test_drifting_out_of_radius_restarts_the_dwell_clock(self):
        seq = self._seq()
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=2.9)  # almost done dwelling
        seq.update(current_xyz=(50.0, 50.0, 5.0), dt_sec=0.1)  # drifted away
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=2.9)  # back, but clock reset
        assert seq.current_target == (0.0, 0.0, 5.0)  # still not a full fresh 3s

    def test_loops_back_to_the_first_waypoint_after_the_last(self):
        seq = self._seq(waypoints=[(0.0, 0.0, 5.0), (10.0, 0.0, 5.0)], dwell_time_sec=1.0)
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=1.5)  # arrive + dwell at wp0 -> wp1
        assert seq.current_target == (10.0, 0.0, 5.0)
        seq.update(current_xyz=(10.0, 0.0, 5.0), dt_sec=1.5)  # arrive + dwell at wp1 -> loops
        assert seq.current_target == (0.0, 0.0, 5.0)
        assert seq.finished is False

    def test_non_looping_sequence_finishes_after_the_last_waypoint(self):
        seq = self._seq(waypoints=[(0.0, 0.0, 5.0)], dwell_time_sec=1.0, loop=False)
        seq.update(current_xyz=(0.0, 0.0, 5.0), dt_sec=1.5)
        assert seq.finished is True
        assert seq.current_target is None
