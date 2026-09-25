# Collaborative UAV–UGV Autonomous Navigation Architecture

## 1. System Pipeline Overview

```mermaid
flowchart TD
    %% Clean Academic Styling
    classDef default fill:#ffffff,stroke:#94a3b8,stroke-width:1.2px,color:#0f172a,font-size:11px;
    classDef sectionHeader fill:#f1f5f9,stroke:#cbd5e1,stroke-width:1px,color:#1e293b,font-weight:bold;
    classDef primaryNode fill:#ffffff,stroke:#2563eb,stroke-width:1.4px,color:#1e293b;
    classDef successNode fill:#f0fdf4,stroke:#059669,stroke-width:1.4px,color:#065f46;
    classDef tentativeNode fill:#fffbeb,stroke:#d97706,stroke-width:1.4px,color:#92400e;
    classDef anomalyNode fill:#faf5ff,stroke:#7c3aed,stroke-width:1.4px,color:#5b21b6;
    classDef alertNode fill:#fef2f2,stroke:#dc2626,stroke-width:1.4px,color:#991b1b;

    %% 1. PERCEPTION & MAPPING
    subgraph PERCEPTION ["1. Aerial Perception & 2.5D Mapping (emap)"]
        UAV_Cam["UAV RGB-D Sensor (10 Hz PointCloud2)"]
        UGV_Cam["UGV Front Depth Camera (Secondary)"]
        Preprocess["Spatial Range Filter & TF World Projection"]
        BayesFusion["Recursive Bayesian Height & Variance Fusion"]:::primaryNode
        GlobalMap["Persistent Global GridMap (h, σ², valid)"]:::primaryNode
        TravFilter["Analytical Traversability Filter (Slope, Step, Variance)"]:::primaryNode

        UAV_Cam --> Preprocess
        UGV_Cam -.-> Preprocess
        Preprocess --> BayesFusion
        BayesFusion --> GlobalMap
        GlobalMap --> TravFilter
    end

    %% 2. CLASSIFICATION & ANOMALY
    subgraph CLASSIFICATION ["2. Spatial Masks & Anomaly Detection (nav)"]
        WalkMask["Walkable Mask Mw: is_valid ∧ trav ≠ LETHAL"]:::successNode
        FrontierMask["Frontier Mask Mf: ¬is_valid ∧ 4-conn(Mw)"]:::tentativeNode
        AnomalyMask["Anomaly Mask Ma (v3): Elevated Island Topology"]:::anomalyNode
        ResolvedStore["Resolved Region Store Sr (Headroom Memory)"]:::primaryNode

        TravFilter --> WalkMask
        TravFilter --> FrontierMask
        TravFilter --> AnomalyMask
        ResolvedStore -.->|"Override"| WalkMask
        ResolvedStore -.->|"Prune"| AnomalyMask
    end

    %% 3. HIERARCHICAL PRM PLANNING
    subgraph PLANNING ["3. Hierarchical PRM Planning (planner_node)"]
        QueryInit["Navigation Query: Start pose s, Goal pose g"]
        ClearStart["Start Footprint Clearance: Disk(s, r=0.3m)"]
        PRMSample["Roadmap Sampling: V = {s, g} ∪ Uniform(Mw, N=400)"]
        PRMEdges["Line-of-Sight Edge Evaluation: Raycast(r=3.0m)"]
        DijkstraSearch["Cost-Weighted Dijkstra: w_normal = d, w_anom = 20×d"]:::primaryNode

        QueryInit --> ClearStart
        ClearStart --> PRMSample
        PRMSample --> PRMEdges
        PRMEdges --> DijkstraSearch
        WalkMask --> PRMSample
        FrontierMask -.-> PRMEdges
        AnomalyMask -.-> PRMEdges
    end

    %% 4. ROUTE DISPATCH
    subgraph ROUTES ["4. Route Classification"]
        RouteA["Route A: Confirmed Safe Route"]:::successNode
        RouteB["Route B: Tentative Frontier Route"]:::tentativeNode
        RouteC["Route C: Tentative Anomaly Route"]:::anomalyNode
        RouteD["Route D: Infeasible (Hold & Expand Map)"]:::alertNode

        DijkstraSearch --> RouteA
        DijkstraSearch --> RouteB
        DijkstraSearch --> RouteC
        DijkstraSearch --> RouteD
    end

    %% 5. 3D VERIFICATION & EXECUTION
    subgraph EXECUTION ["5. 3D Verification & Motion Execution"]
        VoxelCheck{"Headroom Check:<br/>OctoMap Query"}:::anomalyNode
        PruneWaypoints["Waypoint Pruning: Skip Reached Leading Poses"]
        DWBControl["Nav2 DWB Controller: Trajectory Rollout"]:::primaryNode
        GoalCheck{"Goal Convergence:<br/>||p - g|| ≤ 0.25m"}

        RouteC --> VoxelCheck
        VoxelCheck -- "PASSABLE" --> ResolvedStore
        VoxelCheck -- "PASSABLE" --> PruneWaypoints
        VoxelCheck -- "BLOCKED" --> ResolvedStore
        VoxelCheck -- "BLOCKED" ==>|"Immediate Replan"| QueryInit
        VoxelCheck -- "UNKNOWN" --> PruneWaypoints

        RouteA --> PruneWaypoints
        RouteB --> PruneWaypoints
        PruneWaypoints --> DWBControl
        DWBControl --> GoalCheck

        %% Supervisory Loops
        DWBControl -.->|"Stuck: Δd < 0.15m / 20s"| QueryInit
        DWBControl -.->|"Progress Gate: Δd < 0.3m / 5s"| QueryInit
    end

    GoalCheck -- "Converged" --> Success["Mission Complete"]:::successNode
    RouteD -.->|"UAV Survey"| BayesFusion
```

---

## 2. Mathematical Formulation & System Modules

### 2.1 Perception & Elevation Mapping (`emap`)
* **Range Bounds**: Point measurements $\mathbf{p} \in \mathbb{R}^3$ are retained if $d_{\min} \le \|\mathbf{p} - \mathbf{p}_{\text{sensor}}\| \le d_{\max}$, where $d_{\min} = 0.3\,\text{m}$ and $d_{\max} = 19.5\,\text{m}$.
* **Recursive Bayesian Fusion**:
  $$\mu_{t} = \frac{\mu_{t-1} \sigma_{z}^2 + z_t \sigma_{t-1}^2}{\sigma_{t-1}^2 + \sigma_{z}^2}, \quad \sigma_t^2 = \frac{\sigma_{t-1}^2 \sigma_z^2}{\sigma_{t-1}^2 + \sigma_z^2}$$
  where measurement variance scales quadratically with range: $\sigma_z^2 = k \cdot r^2$.
* **Analytical Traversability**:
  * Slope: $s = \|\nabla h\| = \sqrt{(\partial h / \partial x)^2 + (\partial h / \partial y)^2}$
  * Step Height: $\Delta h_{\text{local}} = \max_{(3\times 3)}(h) - \min_{(3\times 3)}(h)$
  * Cell Cost:
    $$\tau(x, y) = \begin{cases} 
      \text{LETHAL}\ (0.0), & \text{if } s > s_{\max} \lor \Delta h > \Delta h_{\max} \lor \sigma^2 > \sigma^2_{\max} \\
      \text{DIFFICULT}\ (0.3), & \text{if } s > 0.5 s_{\max} \lor \Delta h > 0.5 \Delta h_{\max} \\
      \text{EASY}\ (1.0), & \text{otherwise}
    \end{cases}$$

---

### 2.2 Spatial Classification & Anomaly Detection (`nav`)
* **Walkable Mask ($M_w$)**:
  $$M_w(x, y) = \mathbb{I}(\text{is\_valid}(x, y) \land \tau(x, y) \neq \text{LETHAL})$$
* **Frontier Mask ($M_f$)**:
  $$M_f(x, y) = \mathbb{I}(\neg \text{is\_valid}(x, y) \land \exists (u, v) \in \mathcal{N}_4(x, y) \text{ s.t. } M_w(u, v) = 1)$$
* **Occlusion Anomaly Mask ($M_a$, v3)**:
  Identifies false-lethal bands caused by 2.5D overhead depth projection on opaque overhangs (tunnel roofs). Anomaly candidates are detected using connected-component analysis: small elevated component "islands" bounded between two distinct large walkable "rooms" separated by narrow lethal transitions.
* **Resolved Region Store ($S_r$)**:
  Maintains verified 3D bounding boxes. Once headroom is settled:
  * If $\text{PASSABLE}$: Overrides $M_w(x, y) \leftarrow 1$ and clears $M_a(x, y) \leftarrow 0$.
  * If $\text{BLOCKED}$: Overrides $M_w(x, y) \leftarrow 0$ and triggers an immediate global replan.

---

### 2.3 PRM Path Planning & Decision Routes
* **Start Footprint Clearance**: A square disk of radius $r = 0.3\,\text{m}$ centered at the UGV's live start coordinates is unconditionally marked walkable to prevent robot chassis self-detection deadlocks.
* **Roadmap Construction**: Nodes $V = \{\mathbf{s}, \mathbf{g}\} \cup \text{Sample}(M_w, N=400)$. Edges are formed between nodes within $r_{\text{connect}} = 3.0\,\text{m}$ verified via rasterized line-of-sight checking.
* **Cost Metric**:
  $$c(e_{ij}) = \begin{cases}
    \|\mathbf{p}_i - \mathbf{p}_j\|, & \text{if } e_{ij} \subseteq M_w \cup M_f \\
    20 \times \|\mathbf{p}_i - \mathbf{p}_j\|, & \text{if } e_{ij} \cap M_a \neq \emptyset
  \end{cases}$$

#### Route Classifications
| Route | Condition | Downstream Action |
| :--- | :--- | :--- |
| **Route A (Confirmed)** | Path passes strictly through $M_w$ (`has_frontier=F`, `has_anomaly=F`). | Directly executed by Nav2 DWB controller. |
| **Route B (Frontier)** | Path utilizes unobserved reachable space $M_f$. | Tentatively executed; replanned every 5s if progress slows. |
| **Route C (Anomaly)** | Path utilizes suspected overhang cells $M_a$. | Pauses before entry to query local 3D voxel headroom. |
| **Route D (Infeasible)** | No path found through $V$. | UGV holds position; UAV autonomous patrol expands map. |

---

### 2.4 3D Headroom Resolution & Supervisory Control
* **Bounded OctoMap (`voxel_map_node`)**: Maintains an in-memory 3D occupancy octree ($0.05\,\text{m}$ resolution) with spatial bounding ($r_{\max} = 12.0\,\text{m}$) centered at the UGV.
* **Headroom Query (`check_region_headroom`)**:
  * $\text{PASSABLE}$: Continuous vertical clearance confirmed between ground ($z \approx 0.0\,\text{m}$) and ceiling ($z \approx 0.7\,\text{m}$).
  * $\text{BLOCKED}$: Solid obstacle obstruction confirmed; region recorded in $S_r$ and triggers immediate replan.
  * $\text{UNKNOWN}$: Sensor returns pending; UGV creeps forward until resolved.
* **Waypoint Sequence Pruning (`_skip_reached_leading_waypoints`)**: Skips leading waypoints already within $r_{\text{reached}} = 0.5\,\text{m}$ to prevent Nav2 action-server goal preemption races.
* **Stuck Watchdog**: If UGV displacement $\Delta d < 0.15\,\text{m}$ over $20.0\,\text{s}$, a fresh plan is automatically requested from the live pose.
* **Progress-Gated Replan**: During tentative routes, replanning is only executed if recent displacement $\Delta d < 0.3\,\text{m}$ within the $5.0\,\text{s}$ window, preventing unnecessary trajectory churn during smooth driving.
