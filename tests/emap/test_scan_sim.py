"""emap.scan_sim: the offline UAV-scan simulator used by the classifier evaluation (src/nav/scripts/eval_terrain.py).

The simulator is only worth anything if it puts surfaces where the truth has them: these tests check orientation (row <-> y,
col <-> x), height, coverage and the variance model on a tiny world.
"""
import numpy as np
import pytest

from emap.scan_sim import lawnmower, scan
from emap.terrain_analysis import analyze_terrain
from emap.traversability import EASY, LETHAL

RES = 0.1


@pytest.fixture(scope="module")
def world():
    h = np.zeros((160, 160))
    h[100:110, 40:60] = 0.4                                  # a 1 x 2 m box at y = +2..3, x = -4..-2 (cell centres)
    poses = lawnmower(-6, 6, -6, 6, 4.0)
    emap = scan(h, RES, -8.0, -8.0, poses, 4.0, map_length=16.0, noise_a=0.005, seed=1)
    return h, emap


def test_lawnmower_covers_the_rectangle_in_lanes():
    poses = lawnmower(-6, 6, -6, 6, 4.0)
    xs, ys = zip(*poses)
    assert min(xs) == pytest.approx(-6) and max(xs) == pytest.approx(6)
    assert len(set(np.round(ys, 3))) >= 3                    # several lanes
    first_lane = [p for p in poses if p[1] == poses[0][1]]
    second_lane = [p for p in poses if p[1] == [q for q in poses if q[1] != poses[0][1]][0][1]]
    assert first_lane[0][0] < first_lane[-1][0] and second_lane[0][0] > second_lane[-1][0]      # the second lane runs back


def test_surfaces_land_where_the_truth_has_them(world):
    h, emap = world
    e, ok = emap.layer("elevation"), emap.layer("is_valid") > 0.5
    assert abs(e[103:107, 45:55].mean() - 0.4) < 0.03       # box seen at the right place (row <-> y, col <-> x)
    assert abs(e[45:55, 103:107].mean()) < 0.03             # ... and NOT at its mirror image
    flat = ok & (h == 0)
    assert abs(np.median(e[flat])) < 0.01 and e[flat].std() < 0.02


def test_variance_grows_with_range_and_shrinks_with_overlap(world):
    _, emap = world
    ok = emap.layer("is_valid") > 0.5
    v = emap.layer("variance")[ok]
    assert 0.01 < np.median(v) < 0.5                          # 0.01 * range^2 at 4 m = 0.16 per observation, less after fusion
    assert emap.layer("variance")[~ok].min() == pytest.approx(10.0)   # unobserved: the initial variance


def test_classifier_sees_the_box_in_the_scan(world):
    h, emap = world
    ok = emap.layer("is_valid") > 0.5
    r = analyze_terrain(emap.layer("elevation").astype(float), emap.layer("variance").astype(float), ok, RES)
    assert np.all(r.tier[103:107, 45:55] == EASY)             # the box top: flat
    assert np.all(r.tier[99:101, 45:55] == LETHAL)            # its south face: a 40 cm step
    assert np.mean(r.tier[ok & (h == 0)][::7] == EASY) > 0.97


# ------------------------------------------------------------------------------------------------ UGV pass (step 11b)
from emap.elevation_map import ElevationMap
from emap.scan_sim import drive_poses, ugv_pass


def test_drive_poses_follow_the_polyline_and_face_along_it():
    poses = drive_poses([(0, 0), (2, 0), (2, 2)], spacing=0.5)
    assert len(poses) == 8 and poses[0] == pytest.approx((0, 0, 0)) and poses[-1][2] == pytest.approx(np.pi / 2)
    assert all(abs(p[0] - 2.0) < 1e-9 for p in poses[4:])                  # the second leg runs along x = 2


def test_ugv_camera_measures_the_floor_just_ahead_and_a_step_in_front():
    h = np.zeros((160, 160))
    h[:, 100:] = 0.1                                                       # a 10 cm step at x = +2 (cell 100 -> x = -8 + 10)
    emap = ElevationMap(RES, 16.0, 10.0)
    ugv_pass(emap, h, RES, -8.0, -8.0, drive_poses([(-2.0, 0.0), (1.6, 0.0)], spacing=0.4), noise_a=0.002)
    ok = emap.layer("is_valid") > 0.5
    e = emap.layer("elevation")
    assert ok[75:85, 60:95].sum() > 100                                    # a strip along the route is observed ...
    assert not ok[:40].any() and not ok[120:].any()                        # ... and only there (1 m wide, ahead of the robot)
    assert abs(np.median(e[ok & (h == 0)])) < 0.01 and abs(np.median(e[ok & (h > 0)]) - 0.1) < 0.01
    assert np.median(emap.layer("variance")[ok]) < 0.02                    # close range: ~100x more certain than a 4-7 m UAV look
