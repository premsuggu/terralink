# Simulation Workflow - Direction 3: UAV-UGV Navigation

This guide provides **step-by-step instructions** to run, verify, and troubleshoot the Direction 3 simulation.

---

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] ROS 2 Humble installed (`source /opt/ros/humble/setup.bash`)
- [ ] Gazebo Classic (comes with `ros-humble-gazebo-ros-pkgs`)
- [ ] Workspace built: `colcon build --packages-select my_bot tutorial_interfaces`
- [ ] Workspace sourced: `source install/local_setup.bash`
- [ ] DDS fix exported: `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`

---

## Quick Start (TL;DR)

```bash
# Terminal 1: Build and source
cd /home/prem/terralink
source /opt/ros/humble/setup.bash
colcon build --packages-select my_bot tutorial_interfaces
source install/local_setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Terminal 1: Launch simulation
ros2 launch my_bot launch_sim.launch.py

# Watch console for:
# [Sim] PRM graph generation complete!
# [Sim] Received valid path! Beginning waypoint navigation.
```

---

## Detailed Launch Sequence

### What Happens When You Launch

| Time | Event | What to Expect |
|------|-------|----------------|
| **0s** | Gazebo starts | Gazebo GUI opens, loads `roomWithObstacles.world` |
| **5s** | Robot spawns | `my_bot` (UGV) appears in Gazebo |
| **15s** | UAV spawns | `my_uav` appears at height 10m |
| **~17s** | Camera publishes | `/my_uav/camera_uav/image_raw` active |
| **~17-23s** | PRM building | Console: `[Init] Building PRM graph... (X/1000 nodes)` |
| **~23s** | PRM complete | `[Sim] PRM graph generation complete!` |
| **25s** | First path request | Waypoint client timer fires |
| **~26s** | Navigation starts | `[Sim] Received valid path!` + robot moves |

---

## Step-by-Step Verification

### 1. Verify Gazebo Opens Correctly
```bash
# Should see:
# - Maze-like world with walls and obstacles
# - UGV (my_bot): 4-wheeled robot on ground
# - UAV (my_uav): Floating box at 10m height
```

### 2. Verify ROS Topics
```bash
# In new terminal (with workspace sourced):
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Check camera topic
ros2 topic hz /my_uav/camera_uav/image_raw
# Expected: ~30 Hz

# Check odometry
ros2 topic hz /odom
# Expected: ~20-50 Hz

# Check laser scan (UGV LiDAR)
ros2 topic hz /scan
# Expected: ~10-20 Hz
```

### 3. Verify PRM Building Progress
```bash
# Watch waypoints_server logs
ros2 log get waypoints_server | grep -E "(Building|complete|nodes)"

# Expected output:
# [Init] Building PRM graph... (100/1000 nodes)
# [Init] Building PRM graph... (200/1000 nodes)
# ...
# [Sim] PRM graph generation complete! System is ready for path requests.
```

### 4. Verify Service Availability
```bash
ros2 service list | grep waypoints
# Should show: /waypoints_service

# Test manually (after PRM complete):
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
# Should return valid=true with waypoint array
```

### 5. Verify Navigation Starts
```bash
# Watch waypoints_client logs
ros2 log get waypoints_client | grep -E "(Requesting|Received|Publishing)"

# Expected:
# [Init] Requesting path from waypoints_service...
# [Sim] Received valid path! Beginning waypoint navigation.
# Publishing waypoint to /waypoints
```

### 6. Verify Robot Movement
- In Gazebo: UGV should move toward goal
- Check `/cmd_vel` topic:
```bash
ros2 topic echo /cmd_vel
# Should show non-zero linear.x when moving
```

---

## RViz Visualization (Optional but Recommended)

### Launch RViz with Nav2 Config
```bash
# Terminal 3:
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Recommended RViz Panels
1. **TF** - Shows coordinate frames
2. **Map** - Shows costmap (enable Global Costmap)
3. **RobotModel** - Shows UGV + UAV
3. **Camera** - Shows `/my_uav/camera_uav/image_raw`
4. **Path** - Shows `/plan` (Nav2 global plan)
5. **Odometry** - Shows `/odom`

---

## Common Issues & Fixes

### Issue: Gazebo Hangs on Startup
```
Symptom: Gazebo window opens but stays gray/empty
Fix:
export GAZEBO_MODEL_DATABASE_URI=""
# Add to ~/.bashrc for persistence
```

### Issue: DDS Discovery Fails (No Topics)
```
Symptom: ros2 topic list shows nothing or very few topics
Fix:
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Or try CycloneDDS:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Issue: UAV Spawns But Camera Topic Missing
```
Symptom: /my_uav/camera_uav/image_raw not in ros2 topic list
Fix:
1. Check UAV SDF has camera plugin:
   <sensor name="camera" type="camera">...</sensor>
2. Check camera_uav.xacro included in UAV
3. Verify gazebo_ros_camera plugin loaded
```

### Issue: PRM Never Completes (Stays at "Building...")
```
Symptom: Stuck at "[Init] Building PRM graph... (X/1000 nodes)" for long time
Causes:
1. Camera image all black/white → color filter fails
   - Check: ros2 topic echo /my_uav/camera_uav/image_raw --once
2. Color thresholds wrong for lighting
   - Lower: (100,100,100), Upper: (180,180,180) in waypoints_server.cpp
3. Max range too small / FOV wrong
   - Check coordToPixel() parameters match camera
```

### Issue: "No Valid Path" / Service Returns valid=false
```
Symptom: [Init] Requested path could not be found
Causes:
1. PRM not ready (warming < 2)
   - Wait for "PRM graph generation complete!"
2. Start/goal in obstacle (black pixels)
   - Verify coordinates in free space
3. PRM graph disconnected
   - Increase connection_radius or maxNodes
```

### Issue: Robot Stuck / Doesn't Move
```
Symptom: Path received but robot doesn't move
Check:
1. /cmd_vel published? → ros2 topic echo /cmd_vel
2. twist_mux running? → ros2 node list | grep twist
3. Nav2 active? → ros2 node list | grep -E "(planner|controller|bt_nav)"
4. Costmap has obstacles? → Check RViz Global Costmap
5. Goal in costmap? → Goal must be in free space
```

### Issue: Nav2 "Out of Bounds" Error
```
Symptom: [Navfn] Goal is out of bounds
Fix:
1. Static transform map→odom missing
   - Check launch_sim.launch.py has static_tf publisher
2. Costmap size too small
   - Increase global_costmap.size in nav2_params.yaml
3. Goal outside map bounds
   - Verify goal coordinates within world
```

### Issue: Robot Gets Stuck Mid-Path (Known Limitation)
```
Symptom: Robot stops, no recovery
Cause: No stuck detection in waypoints_client.cpp
Fix (TODO): Add watchdog timer + progress tracking
Workaround: Restart simulation or manually drive
```

---

## Debugging Commands Reference

### Topic Inspection
```bash
# Topic rates
ros2 topic hz /my_uav/camera_uav/image_raw
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /goal_pose
ros2 topic hz /plan

# Message content
ros2 topic echo /my_uav/camera_uav/image_raw --once
ros2 topic echo /odom --once
ros2 topic echo /goal_pose --once

# Topic types
ros2 topic type /my_uav/camera_uav/image_raw
```

### Node Inspection
```bash
# List nodes
ros2 node list

# Node info
ros2 node info /waypoints_server
ros2 node info /waypoints_client
ros2 node info /nav2_handler

# Parameters
ros2 param dump /waypoints_server
```

### TF Debugging
```bash
# View TF tree (generates PDF)
ros2 run tf2_tools view_frames
# Opens frames.pdf - check map→odom→base_link chain

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_link
```

### Service Testing
```bash
# List services
ros2 service list | grep waypoints

# Call service manually
ros2 service call /waypoints_service tutorial_interfaces/srv/GetWaypoints \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 1.1, y: -0.4, z: 0.0}}"
```

### Log Analysis
```bash
# Get logs for specific node
ros2 log get waypoints_server
ros2 log get waypoints_client
ros2 log get nav2_handler

# Filter for key events
ros2 log get waypoints_server | grep -E "(Building|complete|Valid|Invalid)"
ros2 log get waypoints_client | grep -E "(Requesting|Received|Publishing|Waiting)"
```

---

## Customizing the Simulation

### Change Goal Position
Edit `waypoints_client.cpp` line 23-25:
```cpp
goal_pose_.x = -4.5;  // Change X
goal_pose_.y = 3.0;   // Change Y
goal_pose_.z = 0.0;
```
Rebuild: `colcon build --packages-select my_bot`

### Change Color Thresholds
Edit `waypoints_server.cpp` line 53-56:
```cpp
cv::Scalar lowerColor = cv::Scalar(100, 100, 100);  // Darker = more sensitive
cv::Scalar upperColor = cv::Scalar(180, 180, 180);  // Brighter = more tolerant
```

### Change PRM Parameters
Edit `processImage.h` / `processImage.cpp`:
```cpp
// Max nodes (line 25 in .h, line 26 in server)
maxNodes = 1000;

// Connection radius (line 81, 99 in .cpp)
int r = int(width/8);    // connectNeighbors
int r = int(width/12);   // connectNewNode

// Nodes per frame (line 35 in server)
runPRM({0, 0}, {1, 1}, 100);  // 100 nodes per frame
```

### Change World
Edit `launch_sim.launch.py` line 28:
```python
world_file_path = os.path.join(get_package_share_directory(package_name), 
                               'worlds', 'YOUR_WORLD.world')
```

---

## Performance Tuning

### Reduce CPU Usage
```bash
# Lower camera rate in UAV SDF/xacro
<update_rate>10</update_rate>  # Default 30, reduce to 10

# Lower PRM nodes per frame
runPRM({0, 0}, {1, 1}, 50);  # 50 instead of 100
```

### Improve Path Quality
```cpp
// Increase max nodes
maxNodes = 2000;

// Increase connection radius
int r = int(width/6);  // More connections = better paths

// Use Euclidean distance for A* heuristic
int euclideanDistance(p1, p2) {
    return sqrt(pow(p1.first-p2.first,2) + pow(p1.second-p2.second,2));
}
```

---

## Next Steps After Successful Run

1. **Read** `code_implementation.md` - Understand every line
2. **Modify** color thresholds for your environment
3. **Add** stuck detection to `waypoints_client.cpp`
4. **Implement** adaptive color learning (Direction 2)
5. **Integrate** elevation mapping (Direction 1) for 3D traversability

---

## Full Verification Checklist

Run this after every launch:

- [ ] Gazebo opens with `roomWithObstacles.world`
- [ ] Robot (my_bot) visible at ground level
- [ ] UAV (my_uav) visible at ~10m height
- [ ] `ros2 topic hz /my_uav/camera_uav/image_raw` → ~30 Hz
- [ ] `ros2 topic hz /odom` → ~20-50 Hz
- [ ] Console shows PRM building: `[Init] Building PRM graph... (X/1000 nodes)`
- [ ] `[Sim] PRM graph generation complete!` appears
- [ ] `[Init] Requesting path from waypoints_service...` appears
- [ ] `[Sim] Received valid path! Beginning waypoint navigation.` appears
- [ ] `[Sim] Publishing waypoint to /waypoints` appears
- [ ] UGV moves in Gazebo toward goal
- [ ] `ros2 topic hz /cmd_vel` shows non-zero when moving

---

## When Things Go Wrong: Debugging Flowchart

```
SIMULATION NOT WORKING?
    │
    ├─► Gazebo won't start?
    │      └─► export GAZEBO_MODEL_DATABASE_URI=""
    │
    ├─► No ROS topics?
    │      └─► export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    │      └─► source install/local_setup.bash
    │
    ├─► Camera topic missing?
    │      └─► Check UAV SDF camera plugin
    │      └─► Check gazebo_ros_camera plugin loaded
    │
    ├─► PRM not building?
    │      └─► Check camera image: ros2 topic echo /my_uav/camera_uav/image_raw
    │      └─► Adjust color thresholds in waypoints_server.cpp
    │
    ├─► No valid path?
    │      └─► Wait for "PRM graph generation complete!"
    │      └─► Check start/goal in free space
    │
    ├─► Robot not moving?
    │      └─► Check /cmd_vel topic
    │      └─► Check twist_mux running
    │      └─► Check Nav2 nodes active
    │
    └─► Nav2 errors?
           └─► Check map→odom static transform
           └─► Check costmap bounds
           └─► Check goal within map
```