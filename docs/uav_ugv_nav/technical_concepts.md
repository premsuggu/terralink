# Technical Concepts - Direction 3: UAV-UGV Navigation

This document explains the **core concepts** you need to understand the Direction 3 implementation. Written for beginners with no prior robotics background.

---

## Table of Contents
1. [ROS 2 Fundamentals](#1-ros-2-fundamentals)
2. [Coordinate Frames & TF](#2-coordinate-frames--tf)
3. [Probabilistic Roadmap (PRM)](#3-probabilistic-roadmap-prm)
4. [A* Search Algorithm](#4-a-search-algorithm)
5. [Nav2 Navigation Stack](#5-nav2-navigation-stack)
5. [OpenCV Image Processing](#5-opencv-image-processing)
6. [Gazebo Simulation](#6-gazebo-simulation)
7. [System Integration](#7-system-integration)

---

## 1. ROS 2 Fundamentals

### What is ROS 2?
**ROS 2 (Robot Operating System 2)** is a middleware framework for robotics. It provides:
- **Nodes**: Independent processes that communicate
- **Topics**: Publish/subscribe messaging (many-to-many)
- **Services**: Request/response RPC (one-to-one)
- **Actions**: Long-running tasks with feedback
- **Parameters**: Configuration values
- **TF2**: Coordinate frame transformations

### Key ROS 2 Concepts for This Project

#### Nodes
Each `.cpp` file with `main()` creates a **node**. Our nodes:
- `waypoints_server` - Image processing + PRM service
- `waypoints_client` - Path requests + waypoint tracking
- `nav2_handler` - Simple message forwarder
- Nav2 nodes (planner, controller, etc.)

#### Topics (Pub/Sub)
```cpp
// Publisher: sends messages
auto pub = this->create_publisher<MsgType>("topic_name", queue_size);
pub->publish(message);

// Subscriber: receives messages
auto sub = this->create_subscription<MsgType>(
    "topic_name", queue_size,
    std::bind(&Class::callback, this, _1));
```

#### Services (Request/Response)
```cpp
// Server
auto srv = this->create_service<SrvType>(
    "service_name",
    std::bind(&Class::callback, this, _1, _2, _3));

// Client
auto client = this->create_client<SrvType>("service_name");
auto request = std::make_shared<SrvType::Request>();
auto future = client->async_send_request(request, callback);
```

#### QoS (Quality of Service)
```cpp
// Sensor data: BEST_EFFORT, small queue (latest matters most)
rclcpp::SensorDataQoS()  // Used for camera images

// Default: RELIABLE, larger queue
rclcpp::QoS(10)          // Used for commands, odometry
```

---

## 2. Coordinate Frames & TF

### Why Coordinate Frames?
Robots have multiple sensors/links at different positions. **TF (Transform)** tracks relationships between frames.

### Our TF Tree
```
map (world origin)
  │
  └── odom (static: 0,0,0)  ← Published by static_transform_publisher
        │
        └── base_link (UGV center)  ← Published by robot_state_publisher from URDF
              │
              └── camera_link (UAV camera)  ← From UAV URDF/SDF
                    │
                    └── camera_optical_frame  ← Standard camera frame (Z forward)
```

### Frame Conventions (REP-103)
| Frame | Axis Convention |
|-------|-----------------|
| `map` | X=East, Y=North, Z=Up (ENU) |
| `odom` | Same as map, but can drift |
| `base_link` | X=Forward, Y=Left, Z=Up (FLU) |
| `camera_link` | X=Right, Y=Down, Z=Forward |
| `camera_optical_frame` | Z=Forward (optical axis) |

### Using TF in Code
```cpp
// Look up transform
tf2::Buffer buffer;
tf2::TransformListener listener(buffer);
geometry_msgs::msg::TransformStamped t = buffer.lookupTransform(
    "target_frame", "source_frame", tf2::TimePointZero);

// Transform a point
geometry_msgs::msg::PointStamped in, out;
in.point.x = 1.0; in.point.y = 2.0; in.point.z = 0.0;
in.header.frame_id = "base_link";
buffer.transform(in, out, "map");  // out now in map frame
```

### Static vs Dynamic Transforms
| Type | Publisher | Use Case |
|------|-----------|----------|
| **Static** | `static_transform_publisher` | Fixed offsets (map→odom, base→camera) |
| **Dynamic** | `robot_state_publisher` | Moving joints (wheel rotation, robot pose) |

---

## 3. Probabilistic Roadmap (PRM)

### The Problem: Motion Planning
Given a robot in a 2D map with obstacles, find a collision-free path from A to B.

### Two Main Approaches
| Approach | How it Works | Pros | Cons |
|----------|--------------|------|------|
| **Grid-based (A*)** | Search on discretized grid | Optimal, complete | Slow for large maps |
| **Sampling-based (PRM/RRT)** | Random samples + connections | Fast, scales to high-D | Not optimal, probabilistic |

### PRM Algorithm (Two Phases)

#### Phase 1: Learning (Offline/Incremental)
```
1. Sample N random points in free space
2. For each point, connect to k nearest neighbors
3. Check line-of-sight (collision check) for each edge
4. Store as graph: nodes + adjacency list
```

#### Phase 2: Query (Online)
```
1. Connect start/goal to nearest PRM nodes
2. Run A* on graph from start-node to goal-node
3. Convert graph path → waypoints
```

### Our PRM Implementation Details

#### Configuration Space
- **2D grid** from camera image (1000×1000 pixels)
- **Binary**: 255 = free, 0 = obstacle
- **World coords**: Meters in `map` frame

#### Key Parameters
```cpp
maxNodes = 1000                    // Max graph nodes
connection_radius = width/12       // ~83 pixels (1000px image)
line_of_sight_check = Bresenham    // Iterate pixels on line
sampling = uniform random          // getRandomPoint()
```

#### Graph Data Structures
```cpp
// Node: pixel coordinate in image
std::map<uint32_t, std::pair<int, int>> points;

// Edge: destination node + cost (manhattan distance)
struct Edge { uint32_t nodeIdx; int cost; };
std::map<uint32_t, std::vector<Edge>> adjacencyList;
```

#### Line-of-Sight Check (Bresenham)
```cpp
bool checkLine(pair<int,int> start, pair<int,int> end) {
    cv::LineIterator it(grid, cv::Point(x1,y1), cv::Point(x2,y2), 8);
    for (int i = 0; i < it.count; i++, ++it) {
        if (grid.at<uchar>(it.pos().y, it.pos().x) != 255) 
            return false;  // Hit obstacle
    }
    return true;  // Clear path
}
```

---

## 4. A* Search Algorithm

### What is A*?
**A*** (A-star) is a **best-first search** algorithm that finds the shortest path using a heuristic.

### Formula
```
f(n) = g(n) + h(n)
g(n) = actual cost from start to n
h(n) = estimated cost from n to goal (heuristic)
```

### Our Implementation
```cpp
struct ANode {
    uint32_t ind;      // Node index
    uint32_t parent;   // Previous node in path
    int cost;          // g(n): actual cost from start
    int priority;      // f(n): g(n) + h(n)
};

// Heuristic: Manhattan distance (grid-aligned)
int manhattanDistance(p1, p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}
```

### Algorithm Steps
```cpp
openList = {start_node}
processed = {}

while openList not empty:
    current = node with LOWEST f(n)  // priority queue
    if current == goal: RECONSTRUCT PATH
    
    move current to processed
    
    for each neighbor:
        if neighbor in processed: continue
        
        new_cost = current.cost + edge_cost
        
        if neighbor not in openList OR new_cost < neighbor.cost:
            neighbor.parent = current
            neighbor.cost = new_cost
            neighbor.priority = new_cost + heuristic(neighbor, goal)
            add/update in openList
```

### Why Manhattan Distance?
- Grid is axis-aligned (no diagonal movement in cost calc)
- Admissible (never overestimates) → guarantees optimal path
- Fast to compute

---

## 5. Nav2 Navigation Stack

### What is Nav2?
**Nav2 (Navigation 2)** is ROS 2's official navigation stack. It handles:
- **Global Planner**: Long-term path (A*, Dijkstra, Navfn)
- **Local Controller**: Short-term obstacle avoidance (DWB, TEB, RPP)
- **Behavior Trees**: Recovery behaviors (spin, backup, wait)
- **Costmaps**: 2D grids with traversability costs

### Nav2 Architecture
```
┌─────────────────────────────────────────────────────────┐
│                    Navigation Server                     │
├─────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Global     │  │   Local      │  │  Behavior    │  │
│  │  Planner     │──►│  Controller  │◄──│  Server      │  │
│  │  (Navfn)     │  │  (DWB)       │  │  (Recovery)  │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘  │
│         │                 │                              │
│         ▼                 ▼                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │              Costmap 2D (Global + Local)         │   │
│  │  - Obstacle layer    - Inflation layer           │   │
│  │  - Static layer      - Voxel layer               │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

### Our Integration
```
Waypoint Client  ──/goal_pose──► Nav2 (Global Planner)
                                           │
                    UGV /odom ────────────┘
                                           │
                    LaserScan ────────────┘
                                           │
                    Costmap (obstacles) ───┘
                                           │
                    /cmd_vel ─────────────► UGV
```

### Key Nav2 Parameters (nav2_params.yaml)
```yaml
global_planner: nav2_navfn_planner/NavfnPlanner  # Dijkstra-based
local_controller: nav2_dwb_controller/DWBController
costmap:
  global:
    resolution: 0.05
    robot_radius: 0.22
    inflation_radius: 0.55
  local:
    resolution: 0.05
```

---

## 5. OpenCV Image Processing

### Basic Pipeline
```cpp
// 1. ROS Image → OpenCV Mat
cv_bridge::toCvCopy(msg, "bgr8") → cv::Mat (BGR, 8-bit per channel)

// 2. Color Thresholding
cv::Scalar lower(100, 100, 100);  // BGR
cv::Scalar upper(180, 180, 180);
cv::inRange(src, lower, upper, mask);  // Output: binary (0 or 255)

// 3. Morphological Operations (optional cleanup)
cv::erode(mask, mask, kernel);
cv::dilate(mask, mask, kernel);

// 4. Threshold to strict binary
cv::threshold(mask, grid, 0, 255, cv::THRESH_BINARY);
```

### Color Spaces
| Space | Channels | Best For |
|-------|----------|----------|
| **BGR** | Blue, Green, Red | Display, simple threshold |
| **HSV** | Hue, Saturation, Value | Lighting-invariant color |
| **LAB** | Lightness, A, B | Perceptual uniformity |

### Why HSV is Better for Real Robots
```cpp
cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
// Hue: 0-179 (color type)
// Saturation: 0-255 (color intensity)  
// Value: 0-255 (brightness)

// Floor color in HSV (example)
cv::Scalar lower(0, 0, 50);     // Dark gray floor
cv::Scalar upper(180, 50, 200); // Any hue, low saturation, medium brightness
```

---

## 6. Gazebo Simulation

### What is Gazebo?
**Gazebo** is a 3D robotics simulator with physics engine (ODE, Bullet, DART, Simbody).

### Key Components in Our Simulation

#### World File (`roomWithObstacles.world`)
```xml
<world name="default">
  <physics>...</physics>
  <scene>...</scene>
  <model name="ground_plane">...</model>
  <model name="walls">...</model>
  <model name="obstacles">...</model>
  <light name="sun">...</light>
</world>
```

#### Robot Description (URDF/SDF/Xacro)
```
my_bot/
├── description/
│   ├── robot.urdf.xacro      # UGV: diff drive, LiDAR, wheels
│   ├── uav.sdf               # UAV: simple box + camera
│   └── camera_uav.xacro      # Downward camera sensor
```

#### Spawning Entities
```python
# Robot (from topic)
spawn_entity.py -topic robot_description -entity my_bot

# UAV (from file)
spawn_entity.py -file uav.sdf -entity my_uav
```

#### ROS-Gazebo Bridge
- `gazebo_ros` package provides plugins
- `ros_gz_bridge` (Gazebo Garden+) or `gazebo_ros` (Gazebo Classic)
- Camera plugin publishes `sensor_msgs/Image`

---

## 7. System Integration

### Complete Data Flow Summary

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SIMULATION LOOP                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Gazebo Physics (1000Hz)                                           │
│       │                                                             │
│       ▼                                                             │
│  UAV Camera Plugin (30Hz) ──► /my_uav/camera_uav/image_raw        │
│       │                                                             │
│       ▼                                                             │
│  waypoints_server (10Hz)                                           │
│  - cv_bridge → OpenCV                                              │
│  - inRange(100-180) → binary mask                                  │
│  - GridSpace.setGrid() → threshold                                 │
│  - runPRM(100 nodes/frame) → build graph                           │
│  - getPoints() → 1000 nodes = "complete"                           │
│       │                                                             │
│       ▼ Service: GetWaypoints                                      │
│  waypoints_client (timer @ 10s)                                    │
│  - start=current /odom, goal=fixed                                 │
│  - coordToPixel() → pixels                                         │
│  - findNearest() → graph nodes                                     │
│  - A* search → node path                                           │
│  - pixelToCoord() → world waypoints                                │
│  - Return PoseStamped[] with yaw                                   │
│       │                                                             │
│       ▼ /waypoints topic                                           │
│  nav2_handler ──► /goal_pose ──► Nav2                             │
│       │                                                             │
│       ▼                                                             │
│  Nav2 Global Planner (Navfn) + Local Controller (DWB)             │
│  - Global costmap (static + inflation)                            │
│  - Local costmap (laser scan + inflation)                         │
│       │                                                             │
│       ▼                                                             │
│  /cmd_vel ──► twist_mux ──► UGV (diff drive controller)          │
│       │                                                             │
│       ▼                                                             │
│  UGV moves → /odom updates → client tracks progress               │
│       │                                                             │
│       ▼                                                             │
│  When dist < 0.5m → publish next waypoint                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Integration Points
| Connection | Mechanism | Frequency |
|------------|-----------|-----------|
| UAV → Server | ROS Topic (Image) | 30 Hz |
| Client ↔ Server | ROS Service | On-demand |
| Client → Handler | ROS Topic (PoseStamped) | On waypoint reach |
| Handler → Nav2 | ROS Topic (PoseStamped) | On waypoint |
| Nav2 → UGV | ROS Topic (Twist) | 20-50 Hz |
| UGV → Client | ROS Topic (Odometry) | 20-50 Hz |

---

## Common Beginner Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Forgetting `source install/local_setup.bash` | `ros2 run` not found | Always source workspace |
| Wrong frame_id in messages | Nav2 "out of bounds" | Use `map` for global poses |
| QoS mismatch | No messages received | Use `SensorDataQoS()` for sensors |
| Not waiting for service | Client hangs | `wait_for_service()` with timeout |
| Coordinate frame confusion | Robot drives wrong way | Draw TF tree, verify with `view_frames` |

---

## Further Reading

### Books
- **"Probabilistic Robotics"** - Thrun, Burgard, Fox (Ch. 5: Sampling-based Planning)
- **"Robotics, Vision and Control"** - Corke (Ch. 4: Navigation)
- **"ROS 2 Documentation"** - https://docs.ros.org/en/humble/

### Papers
- **PRM**: Kavraki et al., IEEE Trans. Robotics Automation (1996)
- **RRT**: LaValle, IEEE Trans. Automatic Control (1998)
- **Nav2**: Macenski et al., ICRA 2020

### Tutorials
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Nav2 Tutorials: https://navigation.ros.org/getting_started/index.html
- OpenCV Tutorials: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html