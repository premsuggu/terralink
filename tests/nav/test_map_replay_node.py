"""map_replay_node: a saved snapshot republished as /elevation_map must be
indistinguishable, to the consumers, from emap's own message. Needs ROS 2
sourced (skipped otherwise); no Gazebo, no executor."""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("grid_map_msgs")

from emap.utils.gridmap_utils import decode_gridmap  # noqa: E402
from nav import replay  # noqa: E402
from nav.map_replay_node import snapshot_to_gridmap  # noqa: E402

RES = 0.1


def _snap():
    n = 40
    walkable = np.ones((n, n), dtype=bool)
    walkable[:, 20] = False
    layers = {
        "elevation": np.random.default_rng(0).random((n, n)).astype(np.float32),
        "variance": np.full((n, n), 0.01, dtype=np.float32),
        "traversability": np.where(walkable, 1.0, 0.0).astype(np.float32),
        "is_valid": np.ones((n, n), dtype=np.float32),
    }
    return replay.MapSnapshot(layers=layers, resolution=RES, center_x=1.5, center_y=-0.5)


class TestSnapshotToGridMap:
    def test_every_layer_survives_the_round_trip(self):
        snap = _snap()
        decoded = decode_gridmap(snapshot_to_gridmap(snap, "iris_quad/odom"))
        assert set(decoded) == set(snap.layers)
        for name, arr in snap.layers.items():
            assert np.array_equal(decoded[name], arr), name

    def test_geometry_and_frame_are_carried(self):
        msg = snapshot_to_gridmap(_snap(), "iris_quad/odom")
        assert msg.header.frame_id == "iris_quad/odom"
        assert msg.info.resolution == pytest.approx(RES)
        assert (msg.info.pose.position.x, msg.info.pose.position.y) == (1.5, -0.5)
        assert msg.info.length_x == pytest.approx(40 * RES)

    def test_the_planner_node_accepts_it_and_plans(self):
        from nav_msgs.srv import GetPlan

        rclpy.init(args=["--ros-args", "-p", "planner_type:=astar"])
        try:
            from nav.planner_node import PlannerNode

            node = PlannerNode()
            node._map_callback(snapshot_to_gridmap(_snap(), "iris_quad/odom"))
            req = GetPlan.Request()
            req.start.pose.position.x, req.start.pose.position.y = (0.0, -0.5)
            req.goal.pose.position.x, req.goal.pose.position.y = (1.0, 0.5)
            resp = node._get_plan_callback(req, GetPlan.Response())
            assert len(resp.plan.poses) >= 2
            node.destroy_node()
        finally:
            rclpy.try_shutdown()
