"""Tests for the world editing behind the real top-down pictures (src/nav/scripts/render_world_topdown.py).

No Gazebo here: only the text transformation of the world file is checked - that the UGV is removed and that the UAV-view camera
is moved high, covers exactly the map, clips everything above the UAV ceiling and casts no shadows.
"""
import math
import os
import re
import sys

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "nav", "scripts"))
import render_world_topdown as rt                                          # noqa: E402

WORLD = os.path.join(REPO, "src", "nav", "worlds", "w4_structure_decks_run.world")


def _camera_block(txt):
    i = txt.index('name="overhead_overview_camera"')
    return txt[i:i + txt[i:].index("</model>")]


def test_robot_removed_rest_kept(tmp_path):
    out = tmp_path / "w.world"
    rt.world_without_robot(WORLD, str(out))
    txt, src = out.read_text(), open(WORLD).read()
    assert "model://nav_ugv" not in txt
    # every other include survives
    assert txt.count("<include>") == src.count("<include>") - 1
    # without uav_view_of the camera is untouched
    assert _camera_block(txt) == _camera_block(src)


def test_uav_view_camera(tmp_path):
    out = tmp_path / "w.world"
    cx, cy, side = 0.05, 0.05, 50.0
    rt.world_without_robot(WORLD, str(out), (cx, cy, side))
    cam = _camera_block(out.read_text())
    x, y, z = (float(v) for v in re.search(r"<pose>([-\d.]+) ([-\d.]+) ([-\d.]+) ", cam).groups())
    assert (x, y) == pytest.approx((cx, cy))
    fov = float(re.search(r"<horizontal_fov>([\d.]+)", cam).group(1))
    assert fov >= 0.1                                                       # SDF rejects a narrower lens
    assert 2 * z * math.tan(fov / 2) == pytest.approx(side, rel=1e-5)      # the picture spans exactly the map
    near = float(re.search(r"<near>([\d.]+)", cam).group(1))
    assert z - near == pytest.approx(rt.UAV_CEILING)                        # nothing above the ceiling is drawn
    assert "<cast_shadows>true</cast_shadows>" not in out.read_text()


def test_map_area_matches_planner_map():
    cx, cy, side = rt.map_area("w4_structure_decks")
    assert side == pytest.approx(50.0)
    assert abs(cx) < 0.5 and abs(cy) < 0.5


def test_bad_world_rejected(tmp_path):
    src = tmp_path / "no_robot.world"
    src.write_text("<sdf><world></world></sdf>")
    with pytest.raises(RuntimeError):
        rt.world_without_robot(str(src), str(tmp_path / "out.world"))
