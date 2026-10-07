# AGENTS.md - Terralink UAV-UGV Project

## Project Overview

**TerraLink**: ROS 2 Humble collaborative UAV-UGV navigation framework for unstructured environments (construction sites, disaster response). UAV provides global overhead awareness; UGV executes ground tasks with payload capacity.

**History**: the project started by evaluating three technical directions (geometric elevation mapping, semantic vision, OpenCV+PRM baseline - see `docs/directions.md`). The two reference implementations that were tried (`src/d1`, a GPU elevation-mapping reference, and `src/d3`, the OpenCV+PRM baseline) have been **removed from the working tree**: nothing depended on them at build or run time, and our own packages superseded them. The last commit that still contains them is `8a86dab` (`git show 8a86dab:src/d1/...`). Older step docs under `docs/work-docs/` still mention those paths as they were at the time.

| Direction | Outcome |
|-----------|---------|
| **1. Geometric** (2.5D elevation mapping) | Realised as our own package `emap` (+ `nav`). Reference copy removed. |
| **2. Semantic** (AI vision to costmap) | Not started (future package `terralink_semantic`). |
| **3. Baseline** (OpenCV color filter + PRM) | Retired; its PRM idea lives on in `nav/prm_planner.py`. Reference copy removed. |

---

## ACTIVE: From-Scratch Implementation (`emap` + `nav`)

**Goal**: Build our own elevation mapping package from scratch (the Direction 1 reference code that guided it is no longer in the tree). An earlier from-scratch attempt, `src/terralink_elevation/` (Gazebo Classic-based, never verified working), was removed entirely once `emap` fully superseded it - see git history if the old code is ever needed for reference.

**Environment note**: this machine has Ignition Gazebo Fortress (`gz sim`) + `ros_gz_sim`/`ros_gz_bridge` for ROS 2 Humble, NOT Gazebo Classic (`gazebo_ros`) - the old `src/d3` baseline depended on Classic and could not run here. `emap` targets Ignition/`ros_gz`. Also: `fuel.gazebosim.org` (Ignition Fuel) connects but stalls on downloads in this sandbox - don't add runtime/setup dependencies on it; vendor external assets locally instead (GitHub raw is reliable).

| Component | Package | Location | Status |
|-----------|---------|----------|--------|
| **Elevation Mapping** | `emap` | `src/emap/` | **Active - see `docs/work-docs/emap/`** |
| **Navigation Core** | `nav` | `src/nav/` | **Active - see `docs/work-docs/nav/`** |
| **Semantic Vision** | `terralink_semantic` | `src/terralink_semantic/` | Not started |

**Key Principles**:
- ✅ New packages in `src/` (not `src/d<N>/`)
- ✅ Each step verified with unit tests in `tests/emap/` and `tests/nav/`
- ✅ Concepts documented in `docs/work-docs/<package>/stepXX_*.md`
- ✅ Modular, testable, learning-focused

---

## Workspace Structure

```
terralink/
├── AGENTS.md                    # This file
├── README.md                    # Project overview
├── docs/                        # Technical documentation
│   ├── PS.md                    # Problem statement
│   ├── directions.md            # The three approaches evaluated at the start (+ outcome)
│   ├── SETUP.md / RUN.md        # Environment setup / run commands for emap + nav
│   ├── STATUS.md                # Current project status
│   ├── build_report.py          # Generates media/reports/TerraLink_Project_Report.docx
│   ├── resource/                # Literature reviews and optimization blueprints
│   └── work-docs/               # OUR implementation logs
│       ├── emap/                # emap roadmap + step docs
│       ├── nav/                 # nav roadmap + step docs (+ SOTA comparison)
│       └── learning/            # Tool/concept tutorials (ROS 2, CuPy, Gazebo, ...)
├── media/                       # Videos, figures, report
├── tests/                       # emap/ and nav/ unit tests (no ROS/Gazebo needed)
└── src/
    ├── emap/                    # OUR elevation mapping (Ignition Gazebo, Python)
    └── nav/                       # OUR navigation core (active)
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── config/
        │   ├── nav2_params.yaml
        │   └── bridge.yaml
        ├── launch/
        │   └── nav_sim.launch.py
        ├── models/nav_ugv/        # UGV SDF model
        ├── worlds/room_maze.world
        ├── resource/
        └── nav/
            ├── prm_planner.py         # PRM path planning
            ├── astar_planner.py       # A* grid planner (opt-in: planner_type:=astar)
            ├── global_planner.py      # plan -> route memory -> exploration decision (shared by node + replay)
            ├── plan_memory.py / exploration.py / mask_latch.py / map_growth.py / stuck_feedback.py
            ├── replay.py              # offline replay of planning on saved maps (no simulator)
            ├── autopilot.py           # UAV autonomous flight logic
            ├── uav_autopilot_node.py  # UAV autopilot ROS node
            ├── planner_node.py        # PRM planner ROS node
            ├── waypoint_follower.py   # UGV waypoint following
            └── walkability.py         # Traversability analysis
See `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for the step-by-step build history.

---

## Build & Install

### Build `nav` (our navigation core package)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav
```
See `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` for full build/run/test instructions - it's kept current as the package evolves.

---

### Build `emap` (our active elevation mapping package)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select emap
```
See `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for full build/run/test instructions - it's kept current as the package evolves, unlike this section.

---

## How to Run (`emap` - our active elevation mapping)

```bash
source /opt/ros/humble/setup.bash
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump
```
Full launch args, verification steps, and expected output are documented (and kept current) in `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` and its per-step docs - refer there rather than this file for exact commands/output, which will otherwise drift out of date as `emap` evolves.

---

## How to Run (`nav` - our navigation core)

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true
```

**Launch Arguments:**
| Arg | Default | Description |
|-----|---------|-------------|
| `headless` | `true` | Run gz sim server-only (no GUI) |
| `autonomous_uav` | `false` | UAV flies patrol waypoints to build map |
| `goal_x` | `1.7` | UGV goal X (world frame) |
| `goal_y` | `-0.5` | UGV goal Y (world frame) |

**Launch Sequence (see `nav_sim.launch.py` docstring for full rationale):**
| Time | Event |
|------|-------|
| 0s | Gazebo starts (`room_maze.world`) |
| 5s | Bridges, TFs, elevation mapping, UAV autopilot start |
| 9s | Nav2, PRM planner, UGV waypoint follower start |

Optional: `planner_type:=astar` (A* planner; default `prm`). `tunnel_demo.launch.py` also takes `enable_exploration`, `anomaly_hold_sec`, `wait_for_mapping`, `stuck_feedback`. To test planning WITHOUT re-running the UAV scan, save a map once (`src/nav/scripts/save_map_snapshot.py`) and either re-plan offline (`replay_plan.py` / `replay_sequence.py`) or run the live Nav2/follower stack from the planning stage with `tunnel_demo.launch.py map_snapshot:=<file.npz>` (fixture: `tests/nav/fixtures/tunnel_test_parked.npz`) - see `docs/work-docs/nav/step07_astar_planner.md`.

**Verification Checklist:**
- [ ] Gazebo loads `room_maze.world`
- [ ] UAV (`iris_quad`) and UGV (`nav_ugv`) visible
- [ ] `/elevation_map` publishing GridMap
- [ ] Nav2 costmap receives UGV lidar scans (no TF drops)
- [ ] PRM planner finds path through traversable cells
- [ ] UGV follows waypoints to goal

Full details in `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` and per-step docs.

---

## Key Executables

| Executable | Package | Description |
|------------|---------|-------------|
| `elevation_mapping_node` | emap | Elevation mapping ROS node (fusion -> `/elevation_map`) |
| `planner_node` | nav | Global planner ROS node (PRM default, `planner_type:=astar`) |
| `map_replay_node` | nav | Publishes a saved map as `/elevation_map` (start-at-planning mode) |
| `uav_autopilot_node` | nav | UAV autopilot ROS node |
| `waypoint_follower` | nav | UGV waypoint following |

---

### `nav` Tests (our navigation core)

```bash
cd tests/nav
python3 -m pytest -v
```
All algorithm-level tests (PRM planner, autopilot, walkability) live here and require no ROS/Gazebo - see `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` for what each covers.

---

### `emap` Tests (our active elevation mapping)

```bash
cd tests/emap
python3 -m pytest -v
```
All algorithm-level tests (fusion, GPU fusion, drift compensation, traversability, map shifting) live here and require no ROS/Gazebo - see `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` for what each covers.

---

## Documentation Reference

| Document | Purpose |
|----------|---------|
| `docs/PS.md` | Problem statement |
| `docs/directions.md` | The three approaches evaluated at the start, and what happened to them |
| `docs/SETUP.md` / `docs/RUN.md` | One-time setup / run commands for emap + nav |
| `docs/STATUS.md` | Current project status |
| `docs/work-docs/emap/IMPLEMENTATION_PLAN.md` | emap roadmap and per-step write-ups |
| `docs/work-docs/nav/IMPLEMENTATION_PLAN.md` | nav roadmap and per-step write-ups |
| `docs/work-docs/nav/step07_astar_planner.md` | nav: A* planner, route memory, exploration, offline replay, live results |
| `docs/work-docs/nav/sota_comparison_and_roadmap.md` | nav: comparison with published systems, remaining work, compute profile |
| `docs/work-docs/nav/step08_path_trace_images.md` | nav: path-trace images (UGV track + every plan), PRM vs A* in two worlds, bugs found |
| `docs/work-docs/learning/` | Tutorials for the tools used (ROS 2, CuPy, GridMap, Gazebo, TF2, colcon, RViz, pytest) |
| `docs/resource/` | Literature reviews and optimization blueprints |
| `media/reports/TerraLink_Project_Report.docx` | Technical report (regenerate with `python3 docs/build_report.py`) |

---

## Development Guidelines

### Modularity Rules

1. **Each package is independent** - `nav` depends only on `emap`'s public pieces (`emap.traversability`, `emap.utils.gridmap_utils`)
2. **Build separately** - Use `--packages-select`
3. **Shared interfaces only** - Common: `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `grid_map_msgs`

### Code Style

- Python 3.10, ROS 2 Humble conventions
- Parameters via `declare_parameter()` (and launch arguments)
- `rclpy` sensor subscriptions use sensor-data QoS where the publisher is a sensor
- Thorough inline comments (project preference)

---

## From-Scratch Implementation Guidelines (TerraLink Custom)

### Package Creation Rules

1. **New packages in `src/` root** - NOT in `src/d<N>/` (e.g., `src/emap/`)
2. **Study published/open-source algorithms, then write our own** - never copy-paste (the old `src/d1` reference was removed; see History above)
3. **Each step = testable module** - Unit test before integration
4. **Document everything** - Work-logs in `docs/work-logs/` with concept explanations

### Testing Discipline

1. **Unit test per step** - `tests/emap/test_*.py`
2. **No ROS in unit tests** - Pure algorithm verification
3. **Integration test with ROS** - After all unit tests pass

### Documentation Standards

1. **Concept-first explanations** - Math, intuition, then code
2. **Reference line numbers** - Link to our own `src/...` code lines
3. **Work-log per step** - What, why, how, pitfalls
4. **Examples** - Synthetic data, expected outputs

See the earlier directory tree (`emap/` under "ACTIVE: From-Scratch Implementation") for the current code architecture.

---
 
## Quick Reference Commands
 
```bash
# Build emap (our elevation mapping) and nav (our navigation core)
colcon build --packages-select emap nav

# Run emap
source install/local_setup.bash
ros2 launch emap uav_sim.launch.py headless:=true launch_rviz:=true world:=bump

# Run nav
source install/local_setup.bash
ros2 launch nav nav_sim.launch.py headless:=true autonomous_uav:=true

# Tests (no ROS/Gazebo needed for the algorithm tests)
(cd tests/nav && python3 -m pytest -q)

# Check topics
ros2 topic list | grep -E "(camera|waypoint|goal|odom|elevation)"

# RViz for emap (or pass launch_rviz:=true to the launch file above)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix emap)/share/emap/rviz/elevation_mapping.rviz

# RViz for nav
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav)/share/nav/rviz/nav.rviz
```
---


## Agent Guidelines

### Code Quality & Maintenance
- Ensure the codebase is well-maintained and follows best practices in terms of modularity, readability, documentation, and performance.
- Always use best practices for code quality, including but not limited to:
    - Code reviews before merging changes
    - Static analysis tools for linting and type checking
    - Adherence to established coding standards and conventions
- Use coding examples wherever necessary to explain concepts in a simple and understandable way.

### Planning & Implementation Process
- **Before starting any task:** Always propose a detailed implementation plan and obtain explicit approval before proceeding.
- **Implementation plan must include:**
    - Clear description of the task objectives and scope
    - All technical details (algorithms, data structures, design patterns)
    - Complete list of dependencies, libraries, and frameworks that will be used
    - Important code snippets or pseudocode to illustrate the approach
    - Estimated timeline and potential risks or challenges
    - After each implementation step, provide a summary of the changes made and any relevant code snippets or examples to illustrate the modifications.

### Testing & Quality Assurance
- After completing any task, ensure the code is properly tested with:
    - Unit tests for individual components and functions
    - Integration tests to verify interactions between modules
    - All tests should pass before considering a task complete
- **Important:** Never create test or debug related files in the root directory. All test files must be placed in the `tests/` directory with appropriate subdirectories matching the source structure.

### Version Control & Deployment
- **Never push code to git independently.** Only push code to git after obtaining explicit approval from the user.
- Wait for user confirmation at each milestone before committing changes to the repository.

### Documentation Standards
- After each completed task, update the documentation to reflect all changes made to the codebase.
- Continuously update `docs/work-docs/` and `docs/STATUS.md` so that we are consistently tracking the current state of the project and its progress.
- Provide documentation in a modular way:
    - High-level documentation explaining overall concepts and workflows
    - Detailed explanations with in-depth technical details
    - All documentation should be placed in the `docs/` directory with clear organization
    - Explanatory docs and work logs shall be maintained separately in modular way.
- Always include references to relevant code snippets in the documentation.
- Provide both conceptual explanations and practical examples to help users understand the codebase better. Provide clear explanations such that even a new developer can understand the codebase and its functionality.
- I have created separate directories for GNN and TabNet documentation in the `docs/` folder. This is where we will maintain all the relevant documentation for these components.

### Communication & Clarification
- Never assume anything about requirements or implementation details.
- When explaining something, in chat, always provide clear explanations like you are to a newbie. 
- Always ask for clarifications if any requirements, specifications, or details are unclear.
- Request explicit approval before making architectural decisions or significant changes.

### Dependency Management
- Never install unnecessary packages or libraries in the system without explicit user permission.
- If you have installed packages without prior permission, immediately inform the user with:
    - A list of packages that were installed
    - Reasons why they were needed
    - A request for permission to keep or remove them
- Follow up accordingly based on user feedback.

### Privilege & Capability Limitations
- If you lack the necessary privileges to perform any task, inform the user immediately.
- Provide a detailed explanation of:
    - What task requires elevated privileges
    - Why those privileges are needed
    - Step-by-step instructions for the user to complete the task manually

---