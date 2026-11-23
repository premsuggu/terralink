"""Guard for R-06 (maze stall): nav_sim.launch.py must keep the options that let the stack recover from a wrong first plan.

Why they matter (docs/work-docs/nav/step15_prm_removal.md, section "Maze stall"): while the UAV is still scanning, thin maze walls can read
as floor, so A* may answer with a route through a wall. Without these options that route is never revised and the UGV stalls at the wall.
The check reads the launch file as text (no ROS needed), so it also runs where launch_ros is not installed.
"""
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[2] / "src" / "nav" / "launch" / "nav_sim.launch.py"


def _text() -> str:
    return LAUNCH.read_text()


def test_planner_node_asks_for_stuck_feedback():
    assert "'stuck_feedback': True" in _text()


def test_follower_replans_when_stuck_or_blocked():
    text = _text()
    assert "'enable_stuck_recovery': True" in text
    assert "'enable_blocked_replan': True" in text


def test_launch_no_longer_offers_a_planner_type_switch():
    # the PRM planner was removed (step 15); A* is the only planner
    assert "planner_type" not in _text()
