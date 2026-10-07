# TerraLink - Collaborative UAV-UGV Navigation Framework

ROS 2 Humble heterogeneous robotics framework for unstructured environments (construction sites, disaster response). UAV provides global overhead awareness; UGV executes ground tasks with payload capacity.

## Packages

| Package | Role | Location | Docs |
|---------|------|----------|------|
| `emap` | UAV elevation mapping (Bayesian 2.5D fusion, traversability, drift compensation, optional GPU fusion) | `src/emap/` | `docs/work-docs/emap/` |
| `nav` | UGV navigation: walkability, anomaly detection, PRM / A* global planning, route memory, exploration, bounded 3D voxel verification, Nav2 following | `src/nav/` | `docs/work-docs/nav/` |
| `terralink_semantic` | Semantic vision (planned, not started) | - | - |

The project began by evaluating three approaches (geometric elevation mapping, semantic vision, OpenCV + PRM baseline - see `docs/directions.md`). The two reference implementations that were tried (`src/d1`, `src/d3`) have been removed from the tree; the last commit containing them is `8a86dab`.

---

## ACTIVE: `emap` - From-Scratch Elevation Mapping (Ignition Gazebo)

**Package**: `emap` (`src/emap/`) - the current, active from-scratch elevation-mapping rebuild. An earlier from-scratch attempt, `terralink_elevation` (Gazebo Classic-based, never verified working), has been removed now that `emap` fully supersedes it - see git history if ever needed for reference.

**Why a rebuild**: this environment has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge`, not Gazebo Classic (`gazebo_ros`), which the original reference baseline needed. `emap` targets the stack that's actually installed here.

**Status**: see `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for the current, up-to-date step-by-step status (kept current there, unlike this line).

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch emap uav_sim.launch.py          # headless by default; headless:=false for GUI
```

Roadmap and per-step write-ups: `docs/work-docs/emap/`.

### Controlling the UAV

**The drone does not fly on its own.** It's velocity-controlled: Gazebo's `MulticopterVelocityControl` plugin (see `step01_uav_gazebo_deployment.md`) constantly asks "what body-frame velocity was I just told to fly at?" and holds that - with nothing publishing to `/cmd_vel`, the commanded velocity is zero, so it just sits on the ground under gravity. This is normal for a velocity-controlled vehicle (a real drone does the same with no stick input) - it's not a bug, and nothing else in the simulation is supposed to move it by itself.

To actually fly it, publish `geometry_msgs/msg/Twist` messages to `/cmd_vel` (linear x/y/z in m/s, angular z for yaw rate, all in the drone's own body frame):

```bash
# One-shot: climb, then hold that velocity until told otherwise
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.6}}"

# Hover (zero velocity) once at altitude
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.0}}"

# Continuous manual control from the keyboard (ros-humble-teleop-twist-keyboard,
# already installed) - a terminal UI that publishes /cmd_vel as you press keys
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Is this critical right now? No.** Manual `/cmd_vel` commands are enough to test the sensor/TF/mapping pipeline (exactly how steps 1-2 were verified) - none of the mapping algorithm work (steps 3-4, pure CPU code) depends on the drone moving by itself at all. Real autonomy - flying a survey pattern, or a planner publishing `/cmd_vel` on the UAV's behalf - is a separate, later concern once the mapping pipeline itself is wired into a live node and there's an actual map worth flying to complete.

---

## `nav` - UAV-mapped-elevation-driven UGV navigation

**Package**: `nav` (`src/nav/`) - depends on `emap`. Classifies `emap`'s `traversability` layer into walkable/non-walkable space (replacing the earlier baseline's hardcoded pixel-color threshold), plans through it with a NumPy/SciPy PRM planner (default) or an opt-in deterministic A* planner (`planner_type:=astar`, see `docs/work-docs/nav/step07_astar_planner.md`), and drives the result with the same Nav2/DWB stack `d3` uses.

**Status**: see `docs/work-docs/nav/IMPLEMENTATION_PLAN.md`; comparison with published systems, remaining work and compute profile: `docs/work-docs/nav/sota_comparison_and_roadmap.md`. Mapping → classification → planning is live-verified; low-level UGV following needs Nav2 installed first (`sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup` - not installed in this environment by default).

```bash
ros2 launch nav nav_sim.launch.py   # headless by default; headless:=false for GUI
```

---

## Quick Start

Full one-time setup: `docs/SETUP.md`. Run commands: `docs/RUN.md`.

```bash
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
colcon build --packages-select emap nav
source install/local_setup.bash
```

---

## Run Commands

### `emap` (Our Active Elevation Mapping)

```bash
# With RViz
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump

# Headless, no RViz
ros2 launch emap uav_sim.launch.py headless:=true
```

### `nav` (Our UAV→UGV Navigation Pipeline)

```bash
# Requires Nav2 installed (see the section above)
ros2 launch nav nav_sim.launch.py headless:=true goal_x:=1.7 goal_y:=-0.5
```

Then fly the UAV manually (see "Controlling the UAV" above) over enough of `room_maze.world` for `nav_ugv` to have a walkable path to the configured goal. `goal_x`/`goal_y` must land in genuinely open, wall-clear space - a point close to a wall reads `LETHAL` and will never be reachable no matter how long you wait (this is exactly what happened with an earlier default of `(4.0, 0.0)`, ~0.1m from a real wall - see `docs/work-docs/nav/step03_real_bugs_from_a_live_user_run.md`). Check any custom goal against a live-flown `/elevation_map` before relying on it.

**Hands-off demo mode** - the UAV flies itself (a fixed patrol of hover waypoints, see `nav/uav_autopilot.py`) so nothing needs manual `/cmd_vel` control:

```bash
ros2 launch nav nav_sim.launch.py headless:=false autonomous_uav:=true goal_x:=1.7 goal_y:=-0.5
```

`headless:=false` here on purpose - the point of this mode is to watch it work. Give it a couple of minutes: Gazebo/Nav2/the UAV takeoff are staggered on purpose (a few seconds each - see `nav/launch/nav_sim.launch.py`'s docstring) so nothing starts probing for a robot that hasn't spawned yet.

To plan without re-running the UAV scan (saved map, offline or live): see `docs/work-docs/nav/step07_astar_planner.md` and `docs/RUN.md`.

---

## Project Structure

```
terralink/
├── README.md                    # This file
├── AGENTS.md                    # Agent instructions
├── docs/
│   ├── PS.md                    # Problem statement
│   ├── directions.md            # The three approaches evaluated at the start (+ outcome)
│   ├── SETUP.md / RUN.md        # Environment setup / run commands
│   ├── STATUS.md                # Current status
│   ├── resource/                # Literature reviews, optimization blueprints
│   └── work-docs/               # emap/, nav/ (roadmaps + step docs), learning/ (tool tutorials)
├── media/                       # videos, figures, TerraLink_Project_Report.docx
├── src/
│   ├── emap/                    # OUR elevation mapping
│   │   ├── package.xml, config/, launch/, models/, worlds/, rviz/
│   │   └── emap/                # elevation_map, fusion(+gpu), drift, traversability, ...
│   └── nav/                     # OUR UAV-mapped-elevation-driven UGV navigation
│       ├── package.xml, config/, launch/, models/nav_ugv/, worlds/
│       └── nav/                 # walkability, anomaly, prm_planner, astar_planner, global_planner, planner_node, waypoint_follower, voxel_map, replay, ...
└── tests/
    ├── emap/                    # unit tests for emap
    └── nav/                     # unit tests for nav
```

---

## References

- **Elevation Mapping (ETH, the idea `emap` re-implements)**: https://github.com/swiss-mile/elevation_mapping_cupy and https://arxiv.org/pdf/2204.12876
- **Grid Map Library**: https://github.com/ANYbotics/grid_map
- **CuPy**: https://cupy.dev/
- **Nav2**: https://github.com/ros-planning/navigation2
- **Grid Map RViz Plugin**: https://github.com/ANYbotics/grid_map_rviz_plugin