# RUN.md - Runtime Commands for TerraLink (`emap` + `nav`)

Run `SETUP.md` first. Each package's roadmap holds the full details: `docs/work-docs/emap/IMPLEMENTATION_PLAN.md`, `docs/work-docs/nav/IMPLEMENTATION_PLAN.md`.

> Earlier versions of this file also covered the Direction 1 / 2 / 3 reference packages, which were removed (last present in commit `8a86dab`).

## Every session

```bash
source /opt/ros/humble/setup.bash
source /home/prem/terralink/install/local_setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

## `emap` - elevation mapping only

```bash
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump
ros2 launch emap uav_sim.launch.py headless:=true            # no RViz
```
Fly the UAV by publishing `geometry_msgs/Twist` on `/cmd_vel` (it is velocity-controlled and does not fly itself):
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.6}}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Verify: `ros2 topic hz /elevation_map` (GridMap layers: elevation, variance, is_valid, traversability).

## `nav` - UAV maps, UGV navigates

```bash
# Hands-off demo in the maze world (UAV patrols and maps, UGV plans and drives)
ros2 launch nav nav_sim.launch.py headless:=false autonomous_uav:=true goal_x:=1.7 goal_y:=-0.5

# Tunnel demo (occluded passage), PRM planner (default) or A*
ros2 launch nav tunnel_demo.launch.py headless:=true
ros2 launch nav tunnel_demo.launch.py headless:=true planner_type:=astar wait_for_mapping:=true anomaly_hold_sec:=30
```
Launch sequence and options: `nav_sim.launch.py` docstring; planner options: `docs/work-docs/nav/step07_astar_planner.md`.

### Skip the UAV scan (start at the planning stage)

```bash
ros2 launch nav tunnel_demo.launch.py headless:=true planner_type:=astar \
    map_snapshot:=$PWD/tests/nav/fixtures/tunnel_test_parked.npz
```

### Offline replay (no simulator)

```bash
export PYTHONPATH=src/nav:src/emap
python3 src/nav/scripts/replay_plan.py --snapshot tests/nav/fixtures/tunnel_test_parked.npz \
    --start -3 0 --goal 3 0 --anomaly --frontier --frontier-min-width 0.44
python3 src/nav/scripts/replay_plan.py ... --planner prm      # the PRM planner on the same map
```

### Record and draw a run (where did the UGV go, which plans did it get)

```bash
# with a simulation running in another terminal:
python3 src/nav/scripts/trace_run.py --out-dir runs/tunnel_astar_1 --world tunnel_test --goal 3 0 --planner astar   # maze: --world room_maze --goal 1.7 -0.5
python3 src/nav/scripts/render_run_trace.py detail --run runs/tunnel_astar_1 --out run.png
python3 src/nav/scripts/render_run_trace.py compare --group "PRM=runs/p1,runs/p2" --group "A*=runs/a1,runs/a2" --out cmp.png
```
Images from the 12 runs of step 08 are in `media/figures/path_comparison/`; details in `docs/work-docs/nav/step08_path_trace_images.md`.

## Tests

```bash
source /opt/ros/humble/setup.bash
(cd tests/nav && python3 -m pytest -q)                          # 382 tests
cd tests/emap && for f in test_*.py; do python3 -m pytest -q $f; done   # 44 CPU tests; run file by file
```
Running `tests/emap` as one folder collects as "skipped" here because the GPU test module's cupy import fails against the installed numpy.

## Debug commands

```bash
colcon list                          # emap, nav
ros2 topic list | grep -E "(camera|goal|odom|elevation|plan)"
ros2 run tf2_tools view_frames
ros2 topic hz /elevation_map
ros2 topic echo /ugv/odom_ground_truth --once    # ground truth; /ugv/odom is NOT ground truth in tunnel_test.world
```

## Quick fixes

| Issue | Fix |
|-------|-----|
| `ros2 launch` not found | `source /opt/ros/humble/setup.bash` |
| Package not found | `source install/local_setup.bash` |
| DDS discovery fails | `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` |
| RViz shows no map | check `/elevation_map` is publishing |
| Leftover processes after a run | kill the leftover `gz`/`ruby`/`ros2` PIDs explicitly (avoid `pkill -f` patterns that match your own shell) |
