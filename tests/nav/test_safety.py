"""Tests for nav/safety.py (S-01 safety layer, S-02 link-loss policy)."""
from nav.safety import OK, RETURN, STOP, SafetyMonitor, SafetyParams


def test_ok_when_nothing_is_wrong():
    m = SafetyMonitor(SafetyParams(fence=(-10, -10, 10, 10)))
    assert m.update(0.0, (1, 1), 2, 3, front_range_m=2.0).action == OK


def test_outside_the_geofence_stops():
    v = SafetyMonitor(SafetyParams(fence=(-5, -5, 5, 5))).update(0.0, (6.0, 0.0))
    assert v.action == STOP and "geofence" in v.reasons[0]


def test_tilt_and_close_obstacle_stop():
    m = SafetyMonitor(SafetyParams(max_tilt_deg=30, min_range_m=0.3))
    assert m.update(0.0, (0, 0), pitch_deg=40).action == STOP
    m2 = SafetyMonitor(SafetyParams(min_range_m=0.3))
    assert m2.update(0.0, (0, 0), front_range_m=0.2).action == STOP


def test_stop_is_held_until_the_way_has_been_clear_for_a_while():
    m = SafetyMonitor(SafetyParams(min_range_m=0.3, clear_after_s=1.5))
    assert m.update(0.0, (0, 0), front_range_m=0.1).action == STOP
    assert m.update(0.5, (0, 0), front_range_m=2.0).action == STOP          # clear, but only for 0 s
    assert m.update(1.0, (0, 0), front_range_m=2.0).action == STOP          # 0.5 s
    assert m.update(2.1, (0, 0), front_range_m=2.0).action == OK            # >= 1.5 s clear: released
    assert m.update(2.2, (0, 0), front_range_m=0.1).action == STOP          # and it can stop again


def test_battery_budget_triggers_return_when_home_is_getting_far():
    m = SafetyMonitor(SafetyParams(battery_budget_m=100.0, return_margin=1.3, reserve_m=5.0), base_xy=(0, 0))
    assert m.update(0.0, (0, 0)).action == OK
    for k in range(1, 61):                                                 # drive 60 m straight out
        v = m.update(float(k), (float(k), 0.0))
    # at x=60: left 40 m, need 60*1.3+5 = 83 m -> return (it triggered earlier, once left < need)
    assert v.action == RETURN and "battery" in v.reasons[0] and v.battery_left_m == 40.0


def test_battery_return_fires_exactly_when_the_budget_no_longer_covers_the_way_home():
    m = SafetyMonitor(SafetyParams(battery_budget_m=100.0, return_margin=1.0, reserve_m=0.0), base_xy=(0, 0))
    first = None
    for k in range(0, 80):
        v = m.update(float(k), (float(k), 0.0))
        if v.action == RETURN:
            first = k
            break
    assert first == 51          # left = 100 - k < k (distance home)  ->  k > 50


def test_stop_beats_return():
    m = SafetyMonitor(SafetyParams(battery_budget_m=1.0, min_range_m=0.3), base_xy=(0, 0))
    m.update(0.0, (0, 0))
    assert m.update(1.0, (50, 0), front_range_m=0.1).action == STOP


def test_link_loss_is_flagged_then_triggers_a_return_only_after_the_long_timeout():
    m = SafetyMonitor(SafetyParams(link_timeout_s=5.0, link_return_s=60.0))
    m.map_received(0.0)
    assert not m.update(3.0, (0, 0)).link_lost
    v = m.update(10.0, (0, 0))
    assert v.link_lost and v.action == OK                                   # finish on the last map
    assert m.update(80.0, (0, 0)).action == RETURN                          # lost for 75 s > 60
    m.map_received(81.0)
    v = m.update(82.0, (0, 0))
    assert not v.link_lost and v.action == OK                               # the link came back


def test_no_map_ever_received_is_not_link_loss():
    """The very start (the UAV has not produced a map yet) must not look like a lost link."""
    assert not SafetyMonitor().update(100.0, (0, 0)).link_lost
