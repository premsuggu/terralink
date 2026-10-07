# TerraLink Technical Resource & Literature Library

This directory contains reference papers, literature reviews, root cause analyses, and architectural optimization blueprints for the **TerraLink** collaborative UAV-UGV navigation framework.

---

## 📚 Document Index

| Document | Topic | Description |
| :--- | :--- | :--- |
| **[`Vision-Based_UAV-UGV_Collaboration_for_Autonomous_Construction_Site_Preparation.pdf`](file:///home/prem/terralink/docs/resource/Vision-Based_UAV-UGV_Collaboration_for_Autonomous_Construction_Site_Preparation.pdf)** | Primary Reference Paper (PDF) | Full IEEE Access 2022 paper by Oren Elmakis, Tom Shaked, and Amir Degani (Technion). |
| **[`01_paper_review_elmakis2022_vision_uav_ugv.md`](file:///home/prem/terralink/docs/resource/01_paper_review_elmakis2022_vision_uav_ugv.md)** | Reference Paper Deep Dive | In-depth breakdown of the Elmakis et al. (2022) system: collaborative EKF, invariant ArUco scale transformation, adaptive RGB material mapping, and CAD trajectory generation. |
| **[`02_root_cause_analysis_terralink_inefficiencies.md`](file:///home/prem/terralink/docs/resource/02_root_cause_analysis_terralink_inefficiencies.md)** | Root Cause Analysis | Exhaustive, line-by-line diagnosis of why TerraLink takes **> 3 minutes** for a 6-meter crossing. Analyzes the stop-and-go goal pumping bug, PRM narrow passage bottlenecks, inflation overlap, and blind aerial flight delays. |
| **[`03_literature_review_narrow_passages_and_optimal_planning.md`](file:///home/prem/terralink/docs/resource/03_literature_review_narrow_passages_and_optimal_planning.md)** | Advanced Planning Literature Review | Survey of state-of-the-art algorithms: Bridge Test PRM (Hsu 2003), Medial Axis PRM (Wilmarth 1999), Any-Angle Theta\* (Nash 2007), Hybrid A\* (Dolgov 2010), and DARPA SubT subterranean planning (CERBERUS / CoSTAR). |
| **[`04_nav2_trajectory_execution_architectures.md`](file:///home/prem/terralink/docs/resource/04_nav2_trajectory_execution_architectures.md)** | Nav2 Trajectory Execution Guide | Architectural comparison of Nav2 execution tiers (`NavigateToPose` vs `NavigateThroughPoses` vs `FollowPath`) and controller plugins (DWB vs Regulated Pure Pursuit vs TEB). |
| **[`05_optimization_blueprint_for_terralink.md`](file:///home/prem/terralink/docs/resource/05_optimization_blueprint_for_terralink.md)** | Actionable Optimization Roadmap | Step-by-step engineering blueprint to upgrade TerraLink, slashing mission traversal time from **~160 s** to **< 20 s** (an 89% reduction). |

---

## 🎯 Summary of Key Findings

1. **Why Current Navigation is Slow (> 3 min)**:
   - **The Waypoint-Pumping Trap** ([`waypoint_follower.py`](file:///home/prem/terralink/src/nav/nav/waypoint_follower.py)): Dispatches each waypoint individually to `/goal_pose`, causing Nav2 to execute a full stop, in-place rotation to East (`yaw = 0`), and behavior-tree re-spin at every single point along the path.
   - **The Narrow Passage Dilemma** ([`prm_planner.py`](file:///home/prem/terralink/src/nav/nav/prm_planner.py)): Uniform random PRM sampling places very few nodes in the 1m tunnel ($< 2.5\%$ area), producing sharp zigzag paths near walls with no smoothing.
   - **Costmap Inflation Overlap** ([`nav2_params.yaml`](file:///home/prem/terralink/src/nav/config/nav2_params.yaml)): An inflation radius of $0.55\text{ m}$ in a $1.0\text{ m}$ tunnel causes wall inflation zones to overlap, leaving no zero-cost path and forcing DWB to throttle speed down to $0.15\text{ m/s}$.
   - **Uncoordinated Aerial Patrol** ([`tunnel_demo.launch.py`](file:///home/prem/terralink/src/nav/launch/tunnel_demo.launch.py)): The UAV flies an open-loop room orbit with dwell delays, forcing the ground vehicle to wait 35–40 seconds before obtaining its first plan.

2. **The Recommended Path Forward** *(note: the planner recommendation below was written before step 07; we built a deterministic A* planner instead of Theta\*, see `docs/work-docs/nav/step07_astar_planner.md`; the execution recommendations are still open)*:
   - Upgrade trajectory dispatch to continuous **`FollowPath`** or **`NavigateThroughPoses`**, eliminating all intermediate stops.
   - Implement **Theta\*** (optimal any-angle search) with **Medial-Axis corridor centering** and B-spline smoothing.
   - Replace DWB with **Regulated Pure Pursuit (RPP)** and adjust inflation radius to $0.40\text{ m}$.
   - Direct the UAV to actively scout the transit corridor on takeoff.
