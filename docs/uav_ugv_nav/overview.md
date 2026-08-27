# Direction 3: UAV-UGV Collaborative Navigation (OpenCV PRM Baseline)

## Overview

This document provides a **complete beginner-friendly guide** to the Direction 3 implementation: a working UAV-UGV collaborative navigation system using OpenCV color filtering + Probabilistic Roadmap (PRM) path planning.

**Status**: ✅ **Working in Gazebo Simulation**

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        UAV-UGV COLLABORATIVE NAVIGATION                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐     /my_uav/camera_uav/image_raw      ┌──────────────┐   │
│   │     UAV      │ ─────────────────────────────────────►│ Waypoint     │   │
│   │  (Gazebo)    │     sensor_msgs/Image (RGB)           │ Server       │   │
│   │  Downward    │                                       │ (PRM + OpenCV)│  │
│   │  Camera      │                                       └──────┬───────┘   │
│   └──────────────┘                                              │           │
│                                                                 │           │
│                              waypoints_service (srv)            ▼           │
│                              tutorial_interfaces/srv/      ┌──────────────┐  │
│                              GetWaypoints                  │ Waypoint     │  │
│                              ────────────────────────────►│ Client       │  │
│                                                             │ (ROS2 Client)│  │
│                                                             └──────┬───────┘  │
│                                                                    │          │
│                                                       /waypoints  ▼          │
│                                       geometry_msgs/PoseStamped  ┌──────────┐ │
│                                                                    │ Nav2     │ │
│                                                                    │ Handler  │ │
│                                                                    └────┬─────┘ │
│                                                                         │      │
│                                                          /goal_pose   ▼      │
│                                       geometry_msgs/PoseStamped  ┌──────────┐ │
│                                                                    │  Nav2    │ │
│                                                                    │ (Global  │ │
│                                                                    │  Planner │ │
│                                                                    └────┬─────┘ │
│                                                                         │      │
│                                                          /cmd_vel     ▼      │
│                                       geometry_msgs/Twist      ┌──────────┐ │
│                                                                    │  UGV     │ │
│                                                                    │ (Gazebo) │ │
│                                                                    └──────────┘ │
│                                                                              │
│                    ┌─────────────────────────────────────────────────────┐   │
│                    │                   TF TREE                           │   │
│                    │  map → odom (static) → base_link → camera_link      │   │
│                    └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Components

| Component | Package | Executable | Role |
|-----------|---------|------------|------|
| **UAV Camera** | `my_bot` | Gazebo plugin | Publishes `/my_uav/camera_uav/image_raw` |
| **Waypoint Server** | `my_bot` | `waypoints_server` | OpenCV color filter → Binary grid → PRM graph → Service |
| **Waypoint Client** | `my_bot` | `waypoints_client` | Requests path, tracks odom, publishes waypoints |
| **Nav2 Handler** | `my_bot` | `nav2_handler` | Forwards `/waypoints` → `/goal_pose` for Nav2 |
| **Nav2 Stack** | `nav2_bringup` | `navigation_launch.py` | Global planner + local controller |
| **UGV Robot** | `my_bot` | Gazebo + ros2_control | Differential drive robot with LiDAR |

---

## Data Flow (Step by Step)

### 1. **UAV Camera → Image Processing**
```
Gazebo Camera Plugin
        │
        ▼
/my_uav/camera_uav/image_raw (sensor_msgs/Image, BGR8)
        │
        ▼
waypoints_server::image_callback()
        │
        ▼
cv_bridge → OpenCV Mat (BGR)
        │
        ▼
cv::inRange(lowerColor=(100,100,100), upperColor=(180,180,180))
        │
        ▼
Binary Mask (white = traversable floor, black = obstacles)
```

### 2. **Binary Grid → PRM Graph**
```
Binary Mask (cv::Mat)
        │
        ▼
GridSpace::setGrid() → threshold → binary grid (0=obstacle, 255=free)
        │
        ▼
GridSpace::runPRM() - Build Probabilistic Roadmap
        │
        ├─ Random sampling in free space
        ├─ Connect nearby nodes (line-of-sight check)
        └─ Store as adjacency list
        │
        ▼
1000 nodes max, connection radius = width/12
```

### 3. **Service Request → Path → Waypoints**
```
Client calls /waypoints_service (GetWaypoints.srv)
        │
        ▼
GridSpace::getWaypoints(start, goal)
        │
        ├─ coordToPixel() - World coords → Image pixels (pinhole camera model)
        ├─ findNearest() - Find closest PRM nodes
        ├─ A* search on PRM graph (manhattan distance heuristic)
        └─ pixelToCoord() - Path pixels → World coordinates
        │
        ▼
Response: valid + PoseStamped[] waypoints (with yaw orientation)
```

### 4. **Waypoint Following → Nav2**
```
Waypoint Client receives path
        │
        ▼
Tracks /odom (UGV position)
        │
        ▼
Publishes next waypoint to /waypoints when within 0.5m
        │
        ▼
Nav2 Handler forwards to /goal_pose
        │
        ▼
Nav2 Global Planner (Navfn) + Local Controller (DWB)
        │
        ▼
/cmd_vel → UGV moves
```

---

## Coordinate Systems Explained

### World Frame (`map`)
- Origin: Gazebo world origin (0,0,0)
- All navigation happens in this frame
- UGV odometry, PRM waypoints, Nav2 goals all in `map` frame

### UAV Camera Frame
- Downward-facing camera at height `captureHeight = 10.0m`
- FOV: 1.25 rad (~71.6°) horizontal & vertical
- Image: 1000×1000 pixels

### Coordinate Transformation (World ↔ Image Pixels)

**World → Pixel (coordToPixel):**
```
focal_length = image_width / (2 * tan(FOV/2))
pixel_x = (focal_length * (-world_x)) / captureHeight + image_width/2
pixel_y = (focal_length * (-world_y)) / captureHeight + image_height/2
```

**Pixel → World (pixelToCoord):**
```
world_x = -((pixel_x - image_width/2) * captureHeight) / focal_length
world_y = -((pixel_y - image_height/2) * captureHeight) / focal_length
```

> **Note**: Negative signs because camera looks DOWN (Z-axis), and image Y increases downward.

---

## PRM (Probabilistic Roadmap) - Beginner Explanation

### What is PRM?
PRM is a **sampling-based motion planning algorithm** that builds a graph of collision-free paths in a configuration space.

### How it Works (Simple Terms)
1. **Sample**: Randomly pick points in free space
2. **Connect**: Connect nearby points if straight line is collision-free
3. **Search**: Use A* to find shortest path from start to goal

### Our Implementation Details
```cpp
// GridSpace class members
maxNodes = 1000                    // Maximum PRM nodes
connection_radius = width/12       // ~83 pixels for 1000px image
line_of_sight = checkLine()        // Bresenham line iteration on grid
heuristic = manhattanDistance      // A* heuristic (grid-aligned)
```

### PRM Building Process (Incremental)
```
Frame 1-2:  "Warming up" (warming > 2 check)
Frame 3+:   Build 100 nodes per frame
            RunPRM({0,0}, {1,1}, 100)  // Adds 100 nodes each frame
            connectNewNode()           // Connect new node to existing
Until:      1000 nodes reached → "PRM graph generation complete!"
```

---

## Color Filtering - Beginner Explanation

### Why Color Filtering?
Simple, fast way to distinguish "floor" from "obstacles" in simulation.

### Current Implementation (HARDCODED - Simulation Only)
```cpp
// waypoints_server.cpp line 53-56
cv::Scalar lowerColor = cv::Scalar(100, 100, 100);  // Dark gray (shadowed floor)
cv::Scalar upperColor = cv::Scalar(180, 180, 180);  // Light gray (lit floor)
cv::inRange(cv_image, lowerColor, upperColor, mask);
```

### Limitations
- ❌ Only works in this specific Gazebo world lighting
- ❌ Real world: shadows, textures, varying lighting break this
- ❌ No semantic understanding (can't distinguish "mud" from "concrete")

### Future Improvements (See Direction 2)
- HSV color space (more robust to lighting)
- Adaptive thresholding (learn floor color from first frames)
- Semantic segmentation (Direction 2: YOLOv8-seg → costmap)

---

## Quick Reference: Topics & Services

| Topic/Service | Type | Direction | Description |
|---------------|------|-----------|-------------|
| `/my_uav/camera_uav/image_raw` | `sensor_msgs/Image` | UAV → Server | Downward RGB camera |
| `/waypoints_service` | `GetWaypoints.srv` | Client ↔ Server | Start+Goal → Waypoints |
| `/waypoints` | `geometry_msgs/PoseStamped` | Client → Handler | Waypoint sequence |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Handler → Nav2 | Nav2 navigation goal |
| `/odom` | `nav_msgs/Odometry` | UGV → Client | Robot pose for tracking |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → UGV | Velocity commands |

---

## References

### Core Concepts
- **PRM**: Kavraki et al., "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces" (1996)
- **A* Search**: Hart et al., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (1968)
- **Bresenham Line Algorithm**: Bresenham, "Algorithm for Computer Control of a Digital Plotter" (1965)

### ROS 2 / Nav2
- **Nav2 Documentation**: https://navigation.ros.org/
- **ROS 2 Services**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLIENT-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
- **cv_bridge**: https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge

### OpenCV
- **inRange**: https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html
- **LineIterator**: https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html#ga746c0625f1781f0ffc905625c7a65c2b

### Coordinate Transforms
- **Pinhole Camera Model**: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
- **ROS 2 TF2**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

---

## Next Steps for Learning

1. **Read**: `technical_concepts.md` - Deep dive on PRM, Nav2, TF, coordinate transforms
2. **Run**: `simulation_workflow.md` - Step-by-step simulation guide
3. **Study**: `code_implementation.md` - Line-by-line code walkthrough
4. **Extend**: `integration_guide.md` - How to add features (stuck detection, adaptive threshold, etc.)