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
(cd tests/nav && python3 -m pytest -q)                          # 405 tests
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

## Stress-test worlds (step 09)

```bash
source /opt/ros/humble/setup.bash
python3 src/nav/scripts/build_world.py w2_container_yard     # also: w1_house_site w3_excavation_slopes w4_structure_decks
# -> src/nav/worlds/<w>.world, src/nav/maps/<w>_uavview.npz (+ _groundtruth.npz), media/worlds/<w>_world.html (open in a browser), media/3d_maps/<w>_elevation_uavview.ply
explorer.exe "$(wslpath -w media/3d_maps/viewer.html)"   # then Open .ply File (or drag a .ply in)
```
The ground-truth maps are normal nav snapshots, so e.g.
`python3 src/nav/scripts/replay_plan.py --snapshot src/nav/maps/w2_container_yard_uavview.npz --start -3 -26 --goal 24 -16 --png /tmp/plan.png`
re-plans on them without any simulator.

**One live run + video of a world (complete map given to A*)** - details in the step09 doc:
```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select nav && source install/local_setup.bash
python3 src/nav/scripts/build_world.py w2_container_yard            # also writes <w>_run.world (UGV + overhead camera)
src/nav/scripts/run_world_scenario.sh w2_container_yard -3 -26 -12 13.8 "W2 title" [uavview|groundtruth]
# -> media/runs/<w>_run.mp4. EXTRA="..." adds launch args, FPS=10 composes a long run faster, MAX_SIM=<sim s> caps it.
```
**Door gauntlet (how narrow a door can the UGV pass?)** - 8 doors, 1.2 ... 0.5 m; goal `k` (x = -14 + 4(k-1), y = 0) is behind door `k`:
```bash
python3 src/nav/scripts/build_world.py w5_door_gauntlet && colcon build --packages-select nav && source install/local_setup.bash
MAX_SIM=420 FPS=10 src/nav/scripts/run_world_scenario.sh w5_door_gauntlet -20 0 6 0 "W5 door gauntlet" uavview     # goal 6 = behind the 0.6 m door
# per-door crossing times from the recording:  poses.csv in /tmp/w5_door_gauntlet_run (columns t, x, y, yaw); walls at x = -16 + 4k
```
**Goals inside a roofed structure (the UGV must discover the interior itself)** - NOT solved live yet, see "Hidden interiors" in the step09 doc:
```bash
EXTRA="overhead_islands:=true enable_exploration:=true stuck_feedback:=true stuck_progress_window_sec:=30.0 anomaly_hold_sec:=30.0 \
       global_map_length:=50.0 max_roughness:=0.5 viewpoint_standoff_m:=0.8 viewpoint_block_m:=3.0 uncertain_waive_m:=0.0" \
MAX_SIM=900 FPS=10 src/nav/scripts/run_world_scenario.sh w4_structure_decks -21.5 -6 -12 8 "W4 hidden interior" uavview
# full pipeline incl. a real UAV scan (~30 min wall time, only worth it once): src/nav/scripts/run_world_mission.sh (see its header)
# offline simulator (no Gazebo, ~10-40 s per goal): the same planner + lidar mapping code as the live node
python3 - <<'PY'
import sys; sys.path[:0] = ["src/nav", "src/emap", "tests/nav"]
from nav.replay import load_snapshot
from explore_sim import ExploreSim
base  = load_snapshot("src/nav/maps/w4_structure_decks_uavview.npz")       # what the planner is given
truth = load_snapshot("src/nav/maps/w4_structure_decks_groundtruth.npz")   # what the simulated lidar sees
r = ExploreSim(base, truth, (-21.5, -6.0), (-12.0, 8.0), replan_every_m=8.0).run(max_plans=250)
print(r.reached, round(r.travelled_m), "m", round(r.seconds), "s")
PY
```
Before any run: `ps -eo pid,args | grep -E "ign gazebo|nav/lib|ros2 launch"` must be empty (stale nodes silently corrupt a new run); after it, check again.
