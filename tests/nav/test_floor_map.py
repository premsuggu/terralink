"""Lidar occupancy ray casting of nav/floor_map_node.py (no ROS needed)."""
import numpy as np

from nav.floor_map_node import integrate_scan, to_layers

RES, N = 0.1, 200                     # 20 m x 20 m centred on (0, 0)


def _scan(ranges_by_deg, n=360, max_r=12.0):
    r = np.full(n, max_r + 1.0)         # nothing seen: reading above the maximum
    for deg, val in ranges_by_deg.items():
        r[deg % n] = val
    return r


def _cell(x, y):
    return (int(round(y / RES + N / 2)), int(round(x / RES + N / 2)))


def _run(ranges, sensor=(0.0, 0.0), yaw=0.0, repeat=1):
    lo, ob = np.zeros((N, N), np.float32), np.zeros((N, N), bool)
    for _ in range(repeat):
        integrate_scan(lo, ob, RES, 0.0, 0.0, sensor, yaw, ranges, -np.pi, 2 * np.pi / 360, 0.3, 12.0)
    return lo, ob


def test_a_wall_hit_is_lethal_and_the_ray_before_it_is_free_and_observed():
    # ray index 180 points along +x (angle_min = -pi, so index 180 = 0 rad); wall at 3 m
    lo, ob = _run(_scan({180: 3.0}))
    lay = to_layers(lo, ob, 0.0)
    assert lay["traversability"][_cell(3.0, 0.0)] == 0.0 and lay["is_valid"][_cell(3.0, 0.0)] == 1
    for x in (0.5, 1.0, 2.0, 2.7):
        assert lay["is_valid"][_cell(x, 0.0)] == 1 and lay["traversability"][_cell(x, 0.0)] == 1.0
    assert lay["is_valid"][_cell(4.0, 0.0)] == 0                          # behind the wall: never observed


def test_max_range_readings_make_free_space_and_no_obstacle():
    lo, ob = _run(_scan({180: float("inf")}))
    lay = to_layers(lo, ob, 0.0)
    assert lay["is_valid"][_cell(8.0, 0.0)] == 1 and lay["traversability"][_cell(8.0, 0.0)] == 1.0
    assert (lay["traversability"] == 0.0).sum() == 0


def test_a_wall_stays_lethal_when_seen_from_several_scans():
    lo, ob = np.zeros((N, N), np.float32), np.zeros((N, N), bool)
    for _ in range(6):
        integrate_scan(lo, ob, RES, 0.0, 0.0, (0.0, 0.0), 0.0, _scan({180: 3.0}), -np.pi, 2 * np.pi / 360, 0.3, 12.0)
    assert to_layers(lo, ob, 0.0)["traversability"][_cell(3.0, 0.0)] == 0.0


def test_the_robot_pose_and_heading_place_the_rays():
    lo, ob = _run(_scan({180: 2.0}), sensor=(1.0, 2.0), yaw=np.pi / 2)    # now ray 180 points along +y
    lay = to_layers(lo, ob, 0.0)
    assert lay["traversability"][_cell(1.0, 4.0)] == 0.0


def test_floor_elevation_follows_the_robot_height():
    lo, ob = _run(_scan({180: 3.0}))
    assert to_layers(lo, ob, 0.2)["elevation"][_cell(1.0, 0.0)] == np.float32(0.2)


def test_a_reading_inside_the_blind_zone_is_ignored():
    """-inf (Gazebo: return nearer than the minimum range) must neither clear free cells nor mark a wall."""
    lo, ob = _run(np.full(360, -np.inf))
    assert not ob.any() and not lo.any()
