# RViz Visualization

**Goal**: Visualize elevation maps, point clouds, and TF frames in RViz.  
**Time to Read**: ~15 minutes  
**Prerequisites**: [01_ros2_fundamentals.md](01_ros2_fundamentals.md), [04_gridmap.md](04_gridmap.md)

---

## 1. What is RViz?

RViz (ROS Visualization) is a **3D visualizer for ROS data**. It displays:
- Robot models (URDF)
- Sensor data (PointCloud2, LaserScan, Images)
- Maps (OccupancyGrid, GridMap)
- TF frames (coordinate axes)
- Custom markers

---

## 2. Starting RViz

```bash
# Standalone
ros2 run rviz2 rviz2

# With config file (recommended)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix terralink_elevation)/share/terralink_elevation/rviz/elevation_mapping.rviz

# From launch file
ros2 launch terralink_elevation elevation_mapping_sim.launch.py launch_rviz:=true
```

---

## 3. Essential Displays for Elevation Mapping

### 3.1 GridMap Display (Our Main Output)
```
Add → By topic → /elevation_map (grid_map_msgs/msg/GridMap)
```
**Settings**:
- **Layer**: `elevation` / `variance` / `traversability`
- **Color Scheme**: `Rainbow`, `Viridis`, `Grayscale`, `Autumn`
- **Min/Max**: Set manually or `Auto`
- **Alpha**: 0.5-1.0
- **Plane**: `XY` (default), `XZ`, `YZ`
- **Height Mode**: `Flat` / `Height` (3D elevation)

**Height Mode = Height**: Shows 3D terrain!
- Color = layer value (e.g., elevation)
- Z position = elevation value

### 3.2 PointCloud2 Display (Input Sensor Data)
```
Add → By topic → /my_uav/camera/depth/points (sensor_msgs/msg/PointCloud2)
```
**Settings**:
- **Style**: `Points`, `Spheres`, `Boxes`
- **Size**: 0.01 - 0.1 (meters)
- **Color**: `RGB8` (if colored), `Z-axis` (height), `Intensity`
- **Decay**: 0 (persist) or time (fade old points)

### 3.3 TF Display (Coordinate Frames)
```
Add → By display type → TF
```
**Settings**:
- **Frame Timeout**: 1.0 (seconds)
- **Show Arrows**: ✓
- **Show Axes**: ✓
- **Axis Length**: 0.5
- **Axis Radius**: 0.03

**Shows**: All frames, parent-child relationships, updates in real-time

### 3.4 RobotModel Display (UAV Visualization)
```
Add → By display type → RobotModel
```
**Settings**:
- **Description Topic**: `/robot_description` (from robot_state_publisher)
- **Visual Enabled**: ✓
- **Collision Enabled**: ✗ (usually)
- **Alpha**: 0.5

### 3.5 Grid Display (Ground Reference)
```
Add → By display type → Grid
```
**Settings**:
- **Plane**: `XY`
- **Color**: Dark gray
- **Line Style**: `Dotted`
- **Cell Size**: 1.0

---

## 4. Creating RViz Config File

### 4.1 Save Current Config
1. Set up all displays as desired
2. File → Save Config As → `rviz/elevation_mapping.rviz`

### 4.2 Config File Structure (YAML)
```yaml
# rviz/elevation_mapping.rviz
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz2/Grid
      Enabled: true
      Name: Ground Grid
      Plane: XY
      Color: 100; 100; 100
      Line Style: Dotted
    
    - Class: rviz2/TF
      Enabled: true
      Name: TF Frames
      Frame Timeout: 1.0
      Show Arrows: true
      Show Axes: true
      Axis Length: 0.5
      Axis Radius: 0.03
    
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: Depth Points
      Topic: /my_uav/camera/depth/points
      Style: Points
      Size: 0.02
      Color Transformer: Z-axis
      Decay: 0.5
    
    - Class: grid_map_rviz_plugin/GridMap
      Enabled: true
      Name: Elevation Map
      Topic: /elevation_mapping_node/elevation_map
      Layer: elevation
      Color Scheme: viridis
      Min Color Value: -1.0
      Max Color Value: 2.0
      Alpha: 0.8
      Plane: XY
      Height Mode: Height
      Height Factor: 1.0
    
    - Class: grid_map_rviz_plugin/GridMap
      Enabled: false
      Name: Traversability
      Topic: /elevation_mapping_node/elevation_map
      Layer: traversability
      Color Scheme: autumn
      Min Color Value: 0.0
      Max Color Value: 1.0
      Alpha: 0.8
  
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
    Frame Rate: 30
```

---

## 5. Common RViz Workflows

### 5.1 Debugging Elevation Map
1. **GridMap (elevation)** - Check map builds correctly
2. **GridMap (traversability)** - Verify ramp/step classification
3. **PointCloud2** - Verify sensor data quality
4. **TF** - Verify frame tree: `map → base_link → camera_link → camera_depth_optical_frame`

### 5.2 Checking Coordinate Convention
If map appears **rotated/flipped**:
1. Check `msg.info.pose.orientation.w = 1.0` (no rotation)
2. Verify `internal_to_gridmap()` transform in code
3. Compare raw PointCloud2 vs GridMap alignment

### 5.3 Measuring Heights
1. Enable **GridMap (elevation)** with **Height Mode: Height**
2. Use **3D View** (mouse drag to rotate)
3. Hover over terrain → shows height in status bar
4. Or use **Measure** tool (ruler icon)

---

## 6. RViz Config for Our Project

```bash
# Location
src/terralink_elevation/rviz/elevation_mapping.rviz

# Key settings:
# - Fixed Frame: map
# - GridMap (elevation): Layer=elevation, Height Mode=Height
# - GridMap (traversability): Layer=traversability, Color Scheme=autumn
# - PointCloud2: /my_uav/camera/depth/points, Color=Z-axis
# - TF: Show Arrows/Axes
```

---

## 7. Troubleshooting

| Issue | Solution |
|-------|----------|
| "No transform from X to Y" | Check TF display shows all frames, add missing static transforms |
| GridMap not visible | Check Layer name matches `msg.layers`, Min/Max range |
| Map flickering | Increase `msg.info.pose.orientation.w = 1.0`, check frame_id |
| PointCloud2 empty | Verify topic name, QoS (SENSOR_DATA), frame_id |
| RViz crashes | Update graphics drivers, try `rviz2 --opengl 1` |
| High CPU | Reduce PointCloud2 decay, disable unused displays |

---

## 8. Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Mouse drag` | Rotate view |
| `Shift + drag` | Pan view |
| `Scroll` | Zoom |
| `F` | Focus on selection |
| `Space` | Toggle pause |
| `Ctrl+S` | Save config |

---

## Next: [10_pytest.md](10_pytest.md) - Unit testing with pytest