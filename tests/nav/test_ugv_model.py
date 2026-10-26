"""The UGV model (src/nav/models/nav_ugv, a Clearpath Jackal-class rover) and the worlds that include it.

No Gazebo needed: the SDF is plain XML. What these tests protect are the mistakes that already happened once while the
model was swapped (2026-10-26) and that cost an hour each to find in simulation:
  * link poses (wheels, cameras) are relative to the MODEL origin, sensors inside base_link relative to base_link
  * a sensor mounted inside the body shell sees nothing (depth camera: 94 % -inf) or the robot itself (lidar)
  * the world's DiffDrive plugin must list the joints the model really has, with the model's wheel radius
"""
from __future__ import annotations

import glob
import os
import re
import xml.etree.ElementTree as ET

import pytest

NAV = os.path.join(os.path.dirname(__file__), "..", "..", "src", "nav")
MODEL_DIR = os.path.join(NAV, "models", "nav_ugv")
BASE_Z = 0.125            # base_link height above ground (kept from the old model: spawn heights depend on it)
BODY_TOP = 0.25           # Jackal height above ground
FRONT_FENDER_X = 0.255    # front edge of the fender mesh (jackal-fender.stl)
WHEEL_R = 0.098


@pytest.fixture(scope="module")
def model():
    return ET.parse(os.path.join(MODEL_DIR, "model.sdf")).getroot().find("model")


def _pose(el):
    txt = el.findtext("pose") or "0 0 0 0 0 0"
    return [float(v) for v in txt.split()]


def test_meshes_exist_and_licence_is_shipped(model):
    uris = {m.text for m in model.iter("uri")}
    assert uris, "the model should use the Jackal meshes"
    for u in uris:
        assert u.startswith("model://nav_ugv/meshes/")
        assert os.path.isfile(os.path.join(MODEL_DIR, "meshes", u.rsplit("/", 1)[1])), u
    assert os.path.isfile(os.path.join(MODEL_DIR, "meshes", "LICENSE_clearpath_jackal"))     # BSD: keep the notice with the meshes


def test_four_wheel_joints_with_the_names_the_worlds_use(model):
    joints = {j.get("name"): j for j in model.findall("joint")}
    for name in ("left_wheel_joint", "rear_left_wheel_joint", "right_wheel_joint", "rear_right_wheel_joint"):
        assert name in joints and joints[name].get("type") == "revolute"
    links = {l.get("name"): l for l in model.findall("link")}
    for name in ("left_wheel", "rear_left_wheel", "right_wheel", "rear_right_wheel"):
        z = _pose(links[name])[2]
        assert z == pytest.approx(WHEEL_R, abs=1e-3), f"{name}: a model-frame z IS the height above ground"
        sphere = links[name].find("collision/geometry/sphere/radius")
        assert float(sphere.text) == pytest.approx(WHEEL_R)


def test_base_link_height_unchanged(model):
    base = next(l for l in model.findall("link") if l.get("name") == "base_link")
    assert _pose(base)[2] == pytest.approx(BASE_Z)          # floor_map_node / spawn heights assume this


def test_lidar_is_above_the_body(model):
    base = next(l for l in model.findall("link") if l.get("name") == "base_link")
    lidar = base.find("sensor[@name='lidar']")
    z_ground = BASE_Z + _pose(lidar)[2]                       # sensor pose is relative to base_link
    assert z_ground > BODY_TOP + 0.02, "a lidar inside the body shell hits the robot itself"
    assert lidar.findtext("topic") == "nav_ugv/scan"
    assert lidar.findtext("lidar/range/min") == "0.3" and lidar.findtext("lidar/scan/horizontal/samples") == "360"


def test_front_camera_is_ahead_of_the_body_and_above_the_floor(model):
    link = next(l for l in model.findall("link") if l.get("name") == "front_camera_link")
    x, _, z = _pose(link)[:3]                                  # link pose: relative to the MODEL origin = the ground
    assert x > FRONT_FENDER_X, "inside the shell the depth image is all -inf"
    assert 0.08 < z < 0.2, "must see the floor ahead (not be inside the floor or above the fender's view)"
    assert link.find("sensor[@type='rgbd_camera']") is not None


def test_video_camera_is_above_the_body(model):
    link = next(l for l in model.findall("link") if l.get("name") == "video_camera_link")
    assert _pose(link)[2] > BODY_TOP + 0.03


def test_launch_tf_matches_the_model():
    """The static TFs are relative to base_link: z_tf = z_model - 0.125 for a link, = the sensor pose for the lidar."""
    src = open(os.path.join(NAV, "launch", "tunnel_demo.launch.py")).read()
    assert "'--x', '0.1', '--y', '0', '--z', '0.165'" in src                       # lidar
    assert "'--x', '0.27', '--y', '0', '--z', '0.03'" in src                       # front camera: 0.155 - 0.125
    m = ET.parse(os.path.join(MODEL_DIR, "model.sdf")).getroot().find("model")
    cam = next(l for l in m.findall("link") if l.get("name") == "front_camera_link")
    assert _pose(cam)[0] == pytest.approx(0.27) and _pose(cam)[2] - BASE_Z == pytest.approx(0.03)


@pytest.mark.parametrize("world", sorted(glob.glob(os.path.join(NAV, "worlds", "*.world"))))
def test_worlds_drive_all_four_wheels(world):
    text = open(world).read()
    if "model://nav_ugv" not in text:
        pytest.skip("no UGV in this world")
    for j in ("left_wheel_joint", "rear_left_wheel_joint", "right_wheel_joint", "rear_right_wheel_joint"):
        assert f"<{'left' if 'left' in j else 'right'}_joint>{j}</" in text, f"{os.path.basename(world)}: {j}"
    assert float(re.search(r"<wheel_radius>([\d.]+)</wheel_radius>", text).group(1)) == pytest.approx(WHEEL_R)
    sep = float(re.search(r"<wheel_separation>([\d.]+)</wheel_separation>", text).group(1))
    assert sep == pytest.approx(0.49), "effective (not geometric) separation: skid-steer slip, measured in step 10"


def test_world_generator_template_matches():
    src = open(os.path.join(NAV, "scripts", "worldgen", "run_world.py")).read()
    assert "rear_left_wheel_joint" in src and "<wheel_radius>0.098</wheel_radius>" in src and "0.49" in src
