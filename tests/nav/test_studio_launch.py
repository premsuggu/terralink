"""studio.launch.py / tunnel_demo.launch.py guards that need neither ROS nor a simulator.

`resolve_map` is pulled out of the launch file's source with `ast` (importing the file would need the ROS launch libraries), so what is tested
is the real function, not a copy.
"""
import ast
import os
from pathlib import Path

import pytest

NAV = Path(__file__).resolve().parents[2] / "src" / "nav"
LAUNCH = (NAV / "launch" / "studio.launch.py").read_text()


def _load_resolve_map():
    tree = ast.parse(LAUNCH)
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "resolve_map")
    ns = {"os": os}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "studio.launch.py", "exec"), ns)  # noqa: S102
    return ns["resolve_map"]


resolve_map = _load_resolve_map()


class TestResolveMap:
    def test_live_means_no_snapshot(self, tmp_path):
        assert resolve_map("w2", "live", str(tmp_path)) == ""
        assert resolve_map("w2", "", str(tmp_path)) == ""

    def test_a_name_is_looked_up_in_the_maps_folder(self, tmp_path):
        (tmp_path / "w2_uavview.npz").write_bytes(b"x")
        assert resolve_map("w2", "uavview", str(tmp_path)) == str(tmp_path / "w2_uavview.npz")

    def test_a_path_is_used_as_given(self, tmp_path):
        f = tmp_path / "mine.npz"
        f.write_bytes(b"x")
        assert resolve_map("w2", str(f), str(tmp_path)) == str(f)

    @pytest.mark.parametrize("bad", ["groundtruth", "typo.npz"])
    def test_a_missing_map_is_an_error_naming_the_file(self, tmp_path, bad):
        with pytest.raises(ValueError, match="not found"):
            resolve_map("w2", bad, str(tmp_path))


class TestShippedMapsAndWorlds:
    @pytest.mark.parametrize("name", ["uavview", "groundtruth"])
    def test_the_default_site_has_both_saved_maps(self, name):
        assert (NAV / "maps" / f"w2_container_yard_{name}.npz").is_file()

    def test_the_default_site_has_a_studio_world(self):
        assert (NAV / "worlds" / "w2_container_yard_studio.world").is_file()


class TestStudioWiring:
    def test_the_studio_flag_makes_the_follower_wait_for_a_goal_and_listen_on_the_goal_topic(self):
        text = (NAV / "launch" / "tunnel_demo.launch.py").read_text()
        assert "'wait_for_goal': ParameterValue(LaunchConfiguration('studio')" in text
        assert "LaunchConfiguration('studio'), \"' == 'true'" in text          # goal_topic is set when studio:=true

    def test_the_studio_launch_never_starts_an_autopilot_and_always_sets_studio(self):
        assert '"autonomous_uav": "false"' in LAUNCH and '"studio": "true"' in LAUNCH

    def test_the_follower_default_is_unchanged(self):
        src = (NAV / "nav" / "waypoint_follower.py").read_text()
        assert 'declare_parameter("wait_for_goal", False)' in src


class TestStartFootprint:
    """A hand-flown UAV scan leaves the parked UGV boxed in by its own pollution unless the start circle is big enough (found live, step 16)."""

    def test_the_studio_launch_widens_the_start_circle_and_the_default_is_unchanged(self):
        assert '"footprint_radius_m": "0.5"' in LAUNCH
        tunnel = (NAV / "launch" / "tunnel_demo.launch.py").read_text()
        assert "DeclareLaunchArgument('footprint_radius_m', default_value='0.3'" in tunnel

    def test_on_a_real_live_scan_the_ugv_has_a_route_with_the_studio_radius(self):
        """tests/nav/fixtures/studio_live_corridor.npz = /elevation_map saved from the live studio after flying the UAV over the UGV (W2, start -3,-26)."""
        import dataclasses

        import numpy as np

        from nav import astar_planner, replay

        snap = replay.load_snapshot(str(Path(__file__).parent / "fixtures" / "studio_live_corridor.npz"))
        masks = replay.build_masks(snap)
        params = dataclasses.replace(astar_planner.PlannerParams(), footprint_radius_m=0.5)
        for goal in ((-3, -23), (-3, -20), (-2.95, -14.95)):
            result = replay.run_plan(snap, masks, (-3, -26), goal, params=params)
            assert result.valid, f"no route to {goal}"
            wp = np.array(result.waypoints)
            assert abs(wp[-1][0] - goal[0]) < 0.6 and abs(wp[-1][1] - goal[1]) < 0.6


class TestFollowerCancelAndHost:
    def test_cancel_forgets_the_stored_goal_so_the_same_goal_can_be_sent_again(self):
        src = (NAV / "nav" / "waypoint_follower.py").read_text()
        body = src[src.index("def _cancel_callback"):src.index("def _plan_blocked_callback")]
        assert 'self._goal_x = self._goal_y = float("nan")' in body       # NaN != any goal, so _mission_goal_callback accepts the next one
        assert "abs(gx - self._goal_x) < 1e-3" in src                      # ...the equality test that made this necessary

    def test_the_studio_launch_exposes_the_host_with_a_safe_default(self):
        assert 'DeclareLaunchArgument("host", default_value="127.0.0.1"' in LAUNCH
        assert '"host": context.launch_configurations["host"]' in LAUNCH


class TestSceneryWiring:
    def test_the_studio_launch_hands_the_world_file_and_models_to_the_server(self):
        """Without these two the page has no 'real site' boxes and a live scan looks like an empty floor."""
        assert '"world_file": os.path.join(nav_share, "worlds", world)' in LAUNCH
        assert '"models_dir": os.path.join(nav_share, "models")' in LAUNCH

    def test_the_node_sends_scenery_in_hello_and_handles_takeoff(self):
        src = (NAV / "nav" / "studio" / "studio_node.py").read_text()
        assert '"scenery": self._scenery' in src
        assert 'cmd.kind == "takeoff"' in src and "def _with_takeoff" in src
