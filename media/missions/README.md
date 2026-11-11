# Mission and safety layer, live on W2 (2026-11-11)

Launch: `enable_safety:=true safety_fence:=-12,-32,32,-8 battery_budget_m:=90 mission_goals:=5,-24,20,-15 mission_mode:=once base_x:=-3 base_y:=-26`
(saved UAV map of W2, Gazebo, real Nav2).

| file | what it shows |
|---|---|
| `w2_mission_D_F_home_safety_on.mp4`, `w2_mission_D_F_home_path.png` | goal D at 20 s (7.9 m), goal F at 56 s (27.7 m), back at base at 105 s (55.4 m), 56.1 m in total, always inside the geofence; the mission reports DONE |
| `w2_mission_D_then_E_unreachable_skipped_home.mp4` | goal E has no route on this map: the planner says "no path", after 8 attempts the mission skips E, drives home (17 m) and finishes - the skip rule working |

Not staged live: an actual geofence breach, a person stepping in front of the robot, a lost UAV link (these are covered by the unit tests in
`tests/nav/test_safety.py`). Details: `docs/work-docs/nav/step14_finalisation.md` sections 3 and 8.
