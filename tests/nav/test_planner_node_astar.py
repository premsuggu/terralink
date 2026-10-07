"""Node-level tests for nav.planner_node's "astar" planner type.

These need ROS 2 (rclpy + grid_map_msgs) sourced - they call the node's own
callbacks directly with hand-built messages, so no executor, no Gazebo and no
other node is involved. They check the WIRING (parameters -> planner ->
response + published flags); the algorithms themselves are covered by
test_astar_planner.py / test_plan_memory.py / test_exploration.py.

Skipped automatically when ROS is not sourced.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("grid_map_msgs")
pytest.importorskip("nav_msgs")

from grid_map_msgs.msg import GridMap  # noqa: E402
from nav_msgs.srv import GetPlan  # noqa: E402

from emap.utils.gridmap_utils import encode_layer_to_multiarray  # noqa: E402

RES = 0.1
N = 60


def _xy(row, col):
    return ((col - N / 2.0) * RES, (row - N / 2.0) * RES)


def _gridmap(walkable, valid=None):
    """A /elevation_map message whose traversability is 1.0 where `walkable`
    and 0.0 (LETHAL) elsewhere, flat elevation, all cells observed unless
    `valid` says otherwise."""
    valid = np.ones_like(walkable) if valid is None else valid
    msg = GridMap()
    msg.header.frame_id = "iris_quad/odom"
    msg.info.resolution = RES
    msg.info.length_x = msg.info.length_y = N * RES
    msg.info.pose.position.x = 0.0
    msg.info.pose.position.y = 0.0
    layers = {
        "elevation": np.zeros(walkable.shape),
        "traversability": np.where(walkable, 1.0, 0.0),
        "is_valid": valid.astype(float),
    }
    msg.layers = list(layers)
    msg.data = [encode_layer_to_multiarray(a.astype("float32")) for a in layers.values()]
    return msg


def _request(start, goal):
    req = GetPlan.Request()
    req.start.pose.position.x, req.start.pose.position.y = start
    req.goal.pose.position.x, req.goal.pose.position.y = goal
    return req


def _make_node(*params):
    args = ["--ros-args"]
    for p in params:
        args += ["-p", p]
    rclpy.init(args=args)
    from nav.planner_node import PlannerNode

    node = PlannerNode()
    node.published = {"frontier": [], "anomaly": []}
    node._frontier_pub.publish = lambda m: node.published["frontier"].append(m.data)
    node._anomaly_plan_pub.publish = lambda m: node.published["anomaly"].append(m.data)
    node.published["trace"] = []
    node._plan_trace_pub.publish = lambda m: node.published["trace"].append(m.data)
    return node


@pytest.fixture
def node_factory():
    nodes = []

    def make(*params):
        node = _make_node(*params)
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()


def _poses(response):
    return [(p.pose.position.x, p.pose.position.y) for p in response.plan.poses]


def _open_map():
    return np.ones((N, N), dtype=bool)


def _tunnel_map():
    """Two rooms joined by an island, with anomaly bands at each end - what
    the anomaly detector would flag; handed to the node via a patched detector."""
    w = np.zeros((N, N), dtype=bool)
    a = np.zeros((N, N), dtype=bool)
    w[5:55, 0:20] = True
    w[5:55, 40:60] = True
    w[26:34, 22:38] = True
    a[26:34, 20:22] = True
    a[26:34, 38:40] = True
    return w, a


class TestPlannerTypeSwitch:
    def test_default_is_prm_and_still_plans(self, node_factory):
        node = node_factory()
        node._map_callback(_gridmap(_open_map()))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        assert node._planner_type == "prm"
        assert len(resp.plan.poses) >= 2

    def test_astar_plans_an_open_map_as_one_straight_segment(self, node_factory):
        node = node_factory("planner_type:=astar")
        node._map_callback(_gridmap(_open_map()))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        assert len(resp.plan.poses) == 2
        assert node.published["frontier"][-1] is False and node.published["anomaly"][-1] is False

    def test_unknown_planner_type_is_rejected(self, node_factory):
        with pytest.raises(ValueError):
            node_factory("planner_type:=dijkstra")

    def test_astar_reports_no_path_for_a_walled_off_goal(self, node_factory):
        node = node_factory("planner_type:=astar")
        w = _open_map()
        w[:, 30] = False
        node._map_callback(_gridmap(w))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(10, 50)), GetPlan.Response())
        assert resp.plan.poses == []


class TestRouteMemoryThroughTheNode:
    def test_repeated_identical_requests_return_identical_waypoints(self, node_factory):
        node = node_factory("planner_type:=astar")
        w = _open_map()
        w[:, 28:32] = False
        w[8:14, 28:32] = True
        w[46:52, 28:32] = True
        node._map_callback(_gridmap(w))
        first = _poses(node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()))
        for _ in range(5):
            again = _poses(node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()))
            assert again == first

    def test_kept_route_is_returned_unchanged_even_as_the_robot_moves(self, node_factory):
        # The follower only treats a plan as unchanged if every waypoint is
        # within 5 cm - so as the robot advances along the route, the planner
        # must keep handing back the SAME list, not one trimmed to the robot.
        node = node_factory("planner_type:=astar")
        w = _open_map()
        w[:, 28:32] = False
        w[8:14, 28:32] = True
        node._map_callback(_gridmap(w))
        first = _poses(node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()))
        moved = _poses(node._get_plan_callback(_request(_xy(25, 8), _xy(30, 55)), GetPlan.Response()))
        assert moved == first

    def test_memory_can_be_switched_off(self, node_factory):
        node = node_factory("planner_type:=astar", "enable_plan_memory:=false")
        node._map_callback(_gridmap(_open_map()))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        assert len(resp.plan.poses) == 2


class TestTentativeRoutesThroughTheNode:
    def _node_with_tunnel(self, node_factory, monkeypatch, *params):
        w, a = _tunnel_map()
        monkeypatch.setattr("nav.planner_node.compute_anomaly_mask", lambda *args, **kw: a)
        node = node_factory("planner_type:=astar", "enable_anomaly_mode:=true", *params)
        node._map_callback(_gridmap(w))
        return node

    def test_tunnel_route_is_found_and_flagged_anomalous(self, node_factory, monkeypatch):
        node = self._node_with_tunnel(node_factory, monkeypatch)
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses
        assert node.published["anomaly"][-1] is True

    def test_standoff_waypoint_is_inserted_before_the_tunnel(self, node_factory, monkeypatch):
        node = self._node_with_tunnel(node_factory, monkeypatch, "standoff_m:=1.0")
        # start well back from the tunnel so the standoff point is a separate
        # waypoint from the start
        poses = _poses(node._get_plan_callback(_request(_xy(30, 3), _xy(30, 55)), GetPlan.Response()))
        xs = [p[0] for p in poses]
        mouth_x = _xy(0, 20)[0]  # west mouth
        entries = [x for x in xs if mouth_x - 0.7 <= x <= mouth_x]
        assert entries, f"no waypoint at the tunnel entry: {xs}"
        entry = entries[0]
        # a waypoint about 1 m before the entry, where the UGV stops to look
        assert any(abs((entry - 1.0) - x) < 0.25 for x in xs), xs

    def test_a_voxel_confirmed_blocked_region_removes_the_tunnel_route(self, node_factory, monkeypatch):
        node = self._node_with_tunnel(node_factory, monkeypatch)
        from std_msgs.msg import Float64MultiArray

        report = Float64MultiArray(data=[-1.0, -0.5, 1.0, 0.5, 0.0])  # blocked box over the tunnel
        node._resolved_region_callback(report)
        node._map_callback(_gridmap(_tunnel_map()[0]))  # next map message applies it
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses == []

    def test_a_voxel_confirmed_passable_region_makes_the_route_confirmed(self, node_factory, monkeypatch):
        node = self._node_with_tunnel(node_factory, monkeypatch)
        from std_msgs.msg import Float64MultiArray

        node._resolved_region_callback(Float64MultiArray(data=[-1.6, -0.7, 1.6, 0.7, 1.0]))
        node._map_callback(_gridmap(_tunnel_map()[0]))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses
        assert node.published["anomaly"][-1] is False


class TestExplorationThroughTheNode:
    def _world(self):
        # A room with one opening on its east wall leading to unobserved space.
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:30] = True
        w[26:36, 30:32] = True  # opening
        valid[:, 32:] = False
        return w, valid

    def test_without_exploration_an_unreachable_goal_gives_no_path(self, node_factory):
        node = node_factory("planner_type:=astar")
        w, valid = self._world()
        node._map_callback(_gridmap(w, valid))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses == []

    def test_with_exploration_the_ugv_is_sent_to_the_opening(self, node_factory):
        node = node_factory("planner_type:=astar", "enable_exploration:=true")
        w, valid = self._world()
        node._map_callback(_gridmap(w, valid))
        poses = _poses(node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()))
        assert poses
        end_x, end_y = poses[-1]
        assert end_x == pytest.approx(_xy(0, 31)[0], abs=0.3)  # right at the opening
        assert abs(end_y - _xy(30, 0)[1]) < 0.7
        assert node.published["frontier"][-1] is True  # tentative: keep replanning

    def test_after_arriving_the_visited_spot_is_not_chosen_again(self, node_factory):
        node = node_factory("planner_type:=astar", "enable_exploration:=true")
        w, valid = self._world()
        node._map_callback(_gridmap(w, valid))
        first = _poses(node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()))
        # the robot is now at the viewpoint; the frontier never resolved
        at_target = first[-1]
        second = node._get_plan_callback(_request(at_target, _xy(30, 55)), GetPlan.Response())
        assert second.plan.poses == []  # nothing else to explore


class TestFlagsDescribeTheDrivenPlan:
    """REAL BUG FOUND LIVE: a replan that found no path published
    has_frontier=False/has_anomaly=False, which told the follower its current
    plan was no longer tentative and switched off periodic replanning."""

    def test_a_failed_request_does_not_overwrite_the_published_flags(self, node_factory, monkeypatch):
        w, a = _tunnel_map()
        monkeypatch.setattr("nav.planner_node.compute_anomaly_mask", lambda *args, **kw: a)
        node = node_factory("planner_type:=astar", "enable_anomaly_mode:=true")
        node._map_callback(_gridmap(w))
        node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert node.published["anomaly"] == [True]
        # now a request that cannot succeed (goal inside a wall)
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(2, 2)), GetPlan.Response())
        assert resp.plan.poses == []
        assert node.published["anomaly"] == [True]  # nothing new was published
        assert node.published["frontier"] == [False]

    def test_the_prm_planner_has_the_same_protection(self, node_factory):
        node = node_factory()  # default planner: prm
        w = _open_map()
        w[:, 30] = False
        node._map_callback(_gridmap(w))
        node._get_plan_callback(_request(_xy(10, 10), _xy(10, 20)), GetPlan.Response())
        before = list(node.published["frontier"])
        node._get_plan_callback(_request(_xy(10, 10), _xy(10, 50)), GetPlan.Response())  # unreachable
        assert node.published["frontier"] == before


class TestAnomalyHoldThroughTheNode:
    def test_a_flickering_anomaly_mask_keeps_the_tunnel_route_alive(self, node_factory, monkeypatch):
        w, a = _tunnel_map()
        masks = iter([a, np.zeros_like(a), np.zeros_like(a)])
        monkeypatch.setattr("nav.planner_node.compute_anomaly_mask", lambda *args, **kw: next(masks))
        node = node_factory("planner_type:=astar", "enable_anomaly_mode:=true", "anomaly_hold_sec:=30.0")
        req = lambda: _request(_xy(30, 5), _xy(30, 55))  # noqa: E731
        node._map_callback(_gridmap(w))
        assert node._get_plan_callback(req(), GetPlan.Response()).plan.poses
        node._map_callback(_gridmap(w))  # detector flickers off...
        assert node._get_plan_callback(req(), GetPlan.Response()).plan.poses  # ...route survives

    def test_without_the_hold_the_same_flicker_loses_the_route(self, node_factory, monkeypatch):
        w, a = _tunnel_map()
        masks = iter([a, np.zeros_like(a)])
        monkeypatch.setattr("nav.planner_node.compute_anomaly_mask", lambda *args, **kw: next(masks))
        node = node_factory("planner_type:=astar", "enable_anomaly_mode:=true", "enable_plan_memory:=false")
        node._map_callback(_gridmap(w))
        node._map_callback(_gridmap(w))
        assert node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response()).plan.poses == []


class TestWaitForMappingThroughTheNode:
    def _gamble_map(self):
        # two rooms joined only by a 4-wide corridor lined with unobserved
        # space: a frontier gamble
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:20] = True
        w[5:55, 40:60] = True
        w[28:32, 20:40] = True
        valid[:28, 20:40] = False
        valid[32:, 20:40] = False
        return w, valid

    def test_the_gamble_is_held_back_while_the_map_is_growing(self, node_factory):
        node = node_factory(
            "planner_type:=astar", "enable_frontier_mode:=true", "wait_for_mapping:=true", "mask_rim_m:=0.0"
        )
        w, valid = self._gamble_map()
        node._map_callback(_gridmap(w, valid))  # first message: no history yet -> counts as growing
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses == []

    def test_the_gamble_is_allowed_after_the_wait_cap(self, node_factory):
        node = node_factory(
            "planner_type:=astar", "enable_frontier_mode:=true", "wait_for_mapping:=true",
            "mapping_max_wait_sec:=0.0", "mask_rim_m:=0.0",
        )  # fmt: skip
        w, valid = self._gamble_map()
        node._map_callback(_gridmap(w, valid))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses

    def test_the_gamble_is_allowed_when_the_rule_is_off(self, node_factory):
        node = node_factory("planner_type:=astar", "enable_frontier_mode:=true", "mask_rim_m:=0.0")
        w, valid = self._gamble_map()
        node._map_callback(_gridmap(w, valid))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses


class TestStuckFeedbackThroughTheNode:
    def test_off_by_default(self, node_factory):
        assert node_factory("planner_type:=astar")._global_planner.stuck is None

    def test_can_be_switched_on(self, node_factory):
        node = node_factory("planner_type:=astar", "stuck_feedback:=true", "stuck_window_sec:=9.0")
        assert node._global_planner.stuck is not None
        assert node._global_planner.stuck.window_sec == 9.0

    def test_planning_still_works_with_it_on(self, node_factory):
        node = node_factory("planner_type:=astar", "stuck_feedback:=true")
        node._map_callback(_gridmap(_open_map()))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        assert len(resp.plan.poses) == 2


class TestFrontierMinWidthDefault:
    """A* without the thin-strip pruning never reached the goal in 3 of 3 live
    runs (it kept gambling through unscanned wall tops), so the astar planner
    applies it automatically; the original prm planner is left unchanged."""

    def test_astar_defaults_to_the_robot_diameter(self, node_factory):
        assert node_factory("planner_type:=astar")._frontier_min_width_m == pytest.approx(0.44)

    def test_prm_default_is_off(self, node_factory):
        assert node_factory()._frontier_min_width_m == 0.0

    def test_an_explicit_zero_turns_it_off_for_astar(self, node_factory):
        assert node_factory("planner_type:=astar", "frontier_min_width_m:=0.0")._frontier_min_width_m == 0.0

    def test_an_explicit_value_applies_to_prm_too(self, node_factory):
        node = node_factory("planner_type:=prm", "frontier_min_width_m:=0.3")
        assert node._frontier_min_width_m == pytest.approx(0.3)

    def _two_rooms_with_an_unscanned_wall_top(self):
        # two free rooms separated ONLY by a 2-cell-thick unobserved strip (an
        # unscanned wall top) - no walkable gap anywhere
        w = np.zeros((N, N), dtype=bool)
        valid = np.ones((N, N), dtype=bool)
        w[5:55, 0:29] = True
        w[5:55, 31:60] = True
        valid[5:55, 29:31] = False
        return w, valid

    def test_the_thin_seam_is_not_a_gamble_for_astar(self, node_factory):
        node = node_factory("planner_type:=astar", "enable_frontier_mode:=true", "mask_rim_m:=0.0")
        w, valid = self._two_rooms_with_an_unscanned_wall_top()
        node._map_callback(_gridmap(w, valid))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses == []  # a wall is a wall

    def test_with_pruning_switched_off_the_same_seam_becomes_a_gamble(self, node_factory):
        node = node_factory(
            "planner_type:=astar", "enable_frontier_mode:=true", "mask_rim_m:=0.0", "frontier_min_width_m:=0.0"
        )  # fmt: skip
        w, valid = self._two_rooms_with_an_unscanned_wall_top()
        node._map_callback(_gridmap(w, valid))
        resp = node._get_plan_callback(_request(_xy(30, 5), _xy(30, 55)), GetPlan.Response())
        assert resp.plan.poses
        assert node.published["frontier"][-1] is True


class TestPlanTraceTopic:
    """`plan_trace` is the debug side channel scripts/trace_run.py records to draw
    every plan; it must describe each reply faithfully and change nothing else."""

    def test_a_valid_plan_is_published_with_waypoints_and_kinds(self, node_factory):
        import json

        node = node_factory("planner_type:=astar")
        node._map_callback(_gridmap(_open_map()))
        resp = node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        rec = json.loads(node.published["trace"][-1])
        assert rec["valid"] is True and rec["planner"] == "astar"
        assert len(rec["waypoints"]) == len(resp.plan.poses) == 2
        assert rec["kinds"] == ["free"]
        assert rec["has_frontier"] is False and rec["has_anomaly"] is False and rec["exploration"] == ""

    def test_a_no_path_reply_is_published_too(self, node_factory):
        import json

        node = node_factory("planner_type:=astar")
        w = _open_map()
        w[:, 30] = False
        node._map_callback(_gridmap(w))
        node._get_plan_callback(_request(_xy(10, 10), _xy(10, 50)), GetPlan.Response())
        rec = json.loads(node.published["trace"][-1])
        assert rec["valid"] is False and rec["waypoints"] == []
        # the request is recorded so a drawing can show where the UGV asked from
        assert rec["start"] == pytest.approx(list(_xy(10, 10)))

    def test_prm_plans_are_published_without_per_stretch_kinds(self, node_factory):
        import json

        node = node_factory()
        node._map_callback(_gridmap(_open_map()))
        node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response())
        rec = json.loads(node.published["trace"][-1])
        assert rec["planner"] == "prm" and rec["valid"] is True and rec["kinds"] == []

    def test_the_trace_does_not_alter_the_plan_itself(self, node_factory):
        node = node_factory("planner_type:=astar")
        node._map_callback(_gridmap(_open_map()))
        a = _poses(node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response()))
        b = _poses(node._get_plan_callback(_request(_xy(10, 10), _xy(45, 40)), GetPlan.Response()))
        assert a == b and len(node.published["trace"]) == 2
