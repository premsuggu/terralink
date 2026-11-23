"""Studio scenery (the world's boxes sent to the page) and the take-off command (no ROS, no simulator)."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from nav.studio.commands import CommandError, parse_command, takeoff_climb
from nav.studio.scenery import load_scenery

NAV = Path(__file__).resolve().parents[2] / "src" / "nav"
WORLD = NAV / "worlds" / "w2_container_yard_studio.world"
MODELS = NAV / "models"


class TestSceneryFromWorldFile:
    def test_the_shipped_world_gives_fences_walls_and_containers(self):
        items = load_scenery(WORLD, MODELS)
        assert len(items) > 50                                              # 5 fences + walls + 31 containers + crates ...
        assert all(set(i) >= {"x", "y", "z", "yaw", "sx", "sy", "sz", "rgb"} for i in items)
        assert all(i["sx"] > 0 and i["sy"] > 0 and i["sz"] > 0 for i in items)

    def test_robots_ground_plane_and_camera_are_not_scenery(self):
        # the studio world also contains iris_quad / nav_ugv includes and a ground plane: none may appear as a box
        items = load_scenery(WORLD, MODELS)
        assert not any(i["sz"] > 100 or i["sx"] >= 300 for i in items)       # the 300 m ground plane would show up as a huge box

    def test_container_boxes_match_the_ground_truth_map(self):
        """The strongest check: the cell under every container's centre in the saved ground-truth map is as tall as the container."""
        d = np.load(NAV / "maps" / "w2_container_yard_groundtruth.npz")
        elev, res, cx, cy = d["layer_elevation"], float(d["resolution"]), float(d["center_x"]), float(d["center_y"])
        rows, cols = elev.shape
        containers = [i for i in load_scenery(WORLD, MODELS) if abs(i["sz"] - 2.6) < 0.05]
        assert len(containers) >= 25
        for b in containers:
            c, r = int((b["x"] - cx) / res + cols / 2), int((b["y"] - cy) / res + rows / 2)
            assert elev[r, c] > 2.0, f"container at ({b['x']:.1f}, {b['y']:.1f}) is not where the ground-truth map has it"

    def test_colours_come_from_the_material_or_the_model_name(self):
        items = load_scenery(WORLD, MODELS)
        assert any(abs(i["rgb"][0] - 0.9) < 1e-6 and abs(i["rgb"][1] - 0.7) < 1e-6 for i in items)   # the yellow crates' <diffuse>
        blue = [i for i in items if i["rgb"][2] > i["rgb"][0] + 0.3]                                   # blue containers
        assert blue

    def test_missing_or_broken_files_give_an_empty_list_not_an_exception(self, tmp_path):
        assert load_scenery(tmp_path / "nope.world", MODELS) == []
        bad = tmp_path / "bad.world"
        bad.write_text("<sdf><world")
        assert load_scenery(bad, MODELS) == []

    def test_an_include_is_placed_by_its_mesh_box_and_yaw(self, tmp_path):
        models = tmp_path / "models" / "crate_blue"
        models.mkdir(parents=True)
        # a mesh spanning x in [-1, 1], y in [0, 4], z in [0, 2]: centre (0, 2, 1), size 2 x 4 x 2
        (models / "mesh.obj").write_text("v -1 0 0\nv 1 4 2\nv 1 0 0\n")
        world = tmp_path / "t.world"
        world.write_text("<sdf><world name='w'><include><name>c</name><uri>model://crate_blue</uri><pose>10 20 0 0 0 1.5707963</pose></include></world></sdf>")
        (it,) = load_scenery(world, tmp_path / "models")
        # yaw 90 deg: the model's +y centre offset of 2 m points along world -x
        assert (it["sx"], it["sy"], it["sz"]) == (2.0, 4.0, 2.0)
        assert math.isclose(it["x"], 10 - 2, abs_tol=1e-2) and math.isclose(it["y"], 20.0, abs_tol=1e-2) and math.isclose(it["z"], 1.0)
        assert it["rgb"][2] > it["rgb"][0]                                   # "blue" in the model name

    def test_the_message_stays_small_enough_to_send_to_every_new_tab(self):
        assert len(json.dumps(load_scenery(WORLD, MODELS))) < 60_000


class TestTakeoffCommand:
    def test_default_and_explicit_altitude(self):
        assert parse_command('{"t":"takeoff"}').data == {"alt": 6.0}
        assert parse_command('{"t":"takeoff","alt":4.5}').data == {"alt": 4.5}

    def test_altitude_is_clamped_to_a_safe_range(self):
        assert parse_command('{"t":"takeoff","alt":500}').data["alt"] == 13.0         # below the 14 m ceiling
        assert parse_command('{"t":"takeoff","alt":-3}').data["alt"] == 1.0           # above the 0.6 m floor

    @pytest.mark.parametrize("bad", ['{"t":"takeoff","alt":"high"}', '{"t":"takeoff","alt":NaN}', '{"t":"takeoff","alt":true}'])
    def test_non_numbers_are_rejected(self, bad):
        with pytest.raises(CommandError):
            parse_command(bad)


class TestTakeoffClimbRule:
    def test_climbs_towards_the_target_and_never_faster_than_the_limit(self):
        target, vz = takeoff_climb(0.1, 6.0, 0.0, 1.5)
        assert target == 6.0 and vz == 1.5

    def test_slows_down_near_the_target_but_keeps_moving(self):
        _, vz = takeoff_climb(5.6, 6.0, 0.0, 1.5)
        assert 0.3 <= vz < 0.5

    def test_finishes_just_below_the_target(self):
        assert takeoff_climb(5.85, 6.0, 0.0, 1.5) == (None, 0.0)

    def test_a_pilot_input_cancels_it_and_is_passed_through(self):
        assert takeoff_climb(2.0, 6.0, -0.7, 1.5) == (None, -0.7)
