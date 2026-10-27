"""tests/nav/integration/make_limits_world.py: the X-A1 world (twelve lanes, one obstacle each) must be what the driver assumes."""
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.join(os.path.dirname(__file__), "integration")
sys.path.insert(0, HERE)
import make_limits_world as mw  # noqa: E402


@pytest.fixture(scope="module")
def models():
    root = ET.fromstring(mw.world())
    return {m.get("name"): m for m in root.find("world").findall("model")}


def _pose(m):
    return [float(v) for v in m.findtext("pose").split()]


def _size(m):
    return [float(v) for v in m.find("link/collision/geometry/box/size").text.split()]


def test_twelve_lanes_three_metres_apart_with_the_obstacles_the_driver_expects(models):
    ys = sorted({round(_pose(m)[1], 3) for n, m in models.items() if n.startswith(("ramp_", "step_"))})
    assert len(ys) == 12 and all(abs((b - a) - 3.0) < 1e-6 for a, b in zip(ys, ys[1:]))
    for d in mw.RAMPS_DEG:
        assert f"ramp_{d}" in models and f"top_{d}" in models
    for h in mw.STEPS_CM:
        assert f"step_{h}cm" in models


@pytest.mark.parametrize("deg", mw.RAMPS_DEG)
def test_ramp_starts_on_the_ground_and_ends_at_the_plateau(models, deg):
    th = math.radians(deg)
    x, _, z, _, pitch, _ = _pose(models[f"ramp_{deg}"])
    L = _size(models[f"ramp_{deg}"])[0]
    assert pitch == pytest.approx(-th, abs=1e-4)
    assert x - L / 2 * math.cos(th) == pytest.approx(3.0, abs=1e-3) and z - L / 2 * math.sin(th) == pytest.approx(0.0, abs=1e-3)
    tz, tsz = _pose(models[f"top_{deg}"])[2], _size(models[f"top_{deg}"])[2]
    # plateau top = the ramp's centre-line height at its upper end (the slab's surface is 2.5 cm higher: a lip, harmless next to 8 cm)
    assert tz + tsz / 2 == pytest.approx(L * math.sin(th), abs=1e-3)


def test_step_heights_match_their_names(models):
    for h in mw.STEPS_CM:
        m = models[f"step_{h}cm"]
        assert _size(m)[2] == pytest.approx(h / 100) and _pose(m)[2] == pytest.approx(h / 200)       # sits on the ground


def test_driver_and_world_share_the_diff_drive_settings():
    text = mw.world()
    assert "<wheel_separation>0.49</wheel_separation>" in text and "<wheel_radius>0.098</wheel_radius>" in text
    assert "<dimensions>3</dimensions>" in text           # roll/pitch need the 3-D odometry
