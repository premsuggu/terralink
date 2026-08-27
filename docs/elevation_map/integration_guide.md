# Elevation Mapping → Nav2 Integration Guide

## Overview

This document describes how to connect the **elevation_mapping_cupy** output (GridMap) to the **UGV Nav2 stack** for autonomous navigation.

---

## 1. Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              UAV                                        │
│  ┌─────────────┐    ┌──────────────────┐    ┌─────────────────────┐   │
│  │ RGB-D Camera│───▶│ PointCloud2      │───▶│ Elevation Mapping   │   │
│  │ (Depth+RGB) │    │ /camera/depth    │    │ Node (GPU/CuPy)     │   │
│  └─────────────┘    └──────────────────┘    │ Publishes GridMap   │   │
│                                              └──────────┬──────────┘   │
└─────────────────────────────────────────────────────────┼──────────────┘
                                                          │
                                              /elevation_map (GridMap)
                                                          │
                                                          ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                              UGV                                        │
│  ┌──────────────────────┐    ┌─────────────────┐    ┌──────────────┐  │
│  │ ElevationToCostmap   │───▶│ Nav2 Global     │───▶│ Nav2 Local   │  │
│  │ Converter Node       │    │ Costmap         │    │ Planner      │  │
│  └──────────────────────┘    └─────────────────┘    └──────────────┘  │
│         ▲                                                 │            │
│         │                                                 ▼            │
│  ┌──────┴──────┐                              ┌──────────────────┐   │
│  │ Traversability│                              │ Diff Drive       │   │
│  │ Parameters   │                              │ Controller       │   │
│  └─────────────┘                              └──────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. ElevationToCostmap Converter Node

### 2.1 Purpose

Convert `grid_map_msgs/msg/GridMap` (multi-layer) → `nav_msgs/msg/OccupancyGrid` (Nav2 costmap).

### 2.2 Implementation (`elevation_to_costmap_node.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import ros2_numpy as rnp

class ElevationToCostmap(Node):
    def __init__(self):
        super().__init__('elevation_to_costmap')
        
        # Robot traversability parameters (tune for your robot)
        self.declare_parameter('max_slope', 0.35)       # ~20 deg
        self.declare_parameter('max_step', 0.15)        # 15cm
        self.declare_parameter('max_roughness', 0.05)   # 5cm variance
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('inflation_radius', 0.55)
        self.declare_parameter('cost_scaling_factor', 3.0)
        self.declare_parameter('use_traversability_layer', True)
        
        self.max_slope = self.get_parameter('max_slope').value
        self.max_step = self.get_parameter('max_step').value
        self.max_roughness = self.get_parameter('max_roughness').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.use_traversability = self.get_parameter('use_traversability_layer').value
        
        # Subscribe to GridMap
        self.gridmap_sub = self.create_subscription(
            GridMap,
            '/elevation_mapping_node/elevation_map',  # Default publisher topic
            self.gridmap_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        # Publish OccupancyGrid for Nav2
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/elevation_costmap',
            10
        )
        
        self.get_logger().info('ElevationToCostmap node started')
    
    def gridmap_callback(self, msg: GridMap):
        try:
            # Extract layers
            elevation = self.extract_layer(msg, 'elevation')
            variance = self.extract_layer(msg, 'variance')
            traversability = self.extract_layer(msg, 'traversability')
            is_valid = self.extract_layer(msg, 'is_valid')
            
            if elevation is None:
                self.get_logger().warn('No elevation layer in GridMap')
                return
            
            # Compute costmap
            if self.use_traversability and traversability is not None:
                # Direct traversability → cost (invert: 1=traversable → 0=free)
                costmap = self.traversability_to_cost(traversability, is_valid)
            else:
                # Compute from elevation + variance
                costmap = self.compute_traversability_cost(elevation, variance, is_valid)
            
            # Publish
            self.publish_costmap(costmap, msg)
            
        except Exception as e:
            self.get_logger().error(f'Costmap conversion failed: {e}')
    
    def extract_layer(self, gridmap: GridMap, layer_name: str) -> np.ndarray:
        """Extract a layer from GridMap as 2D numpy array."""
        if layer_name not in gridmap.layers:
            return None
        
        idx = gridmap.layers.index(layer_name)
        data = gridmap.data[idx]
        
        # Decode Float32MultiArray
        layout = data.layout
        if len(layout.dim) >= 2:
            rows = layout.dim[0].size
            cols = layout.dim[1].size if len(layout.dim) > 1 else layout.dim[0].size
        else:
            size = len(data.data)
            rows = cols = int(np.sqrt(size))
        
        arr = np.array(data.data, dtype=np.float32).reshape(rows, cols)
        return arr
    
    def traversability_to_cost(self, traversability: np.ndarray, is_valid: np.ndarray) -> np.ndarray:
        """Convert traversability (0-1, higher=better) to costmap (0-100, higher=lethal)."""
        costmap = np.full_like(traversability, 255, dtype=np.uint8)  # Unknown
        
        valid_mask = is_valid > 0.5
        trav_valid = traversability[valid_mask]
        
        # Invert and scale: traversability 1.0 → cost 0, traversability 0.0 → cost 100
        cost = (1.0 - trav_valid) * 100
        cost = np.clip(cost, 0, 100).astype(np.uint8)
        
        costmap[valid_mask] = cost
        return costmap
    
    def compute_traversability_cost(self, elevation: np.ndarray, variance: np.ndarray, 
                                     is_valid: np.ndarray) -> np.ndarray:
        """Compute cost from elevation gradient (slope), step height, roughness."""
        from scipy.ndimage import maximum_filter, minimum_filter
        
        h, w = elevation.shape
        costmap = np.full((h, w), 255, dtype=np.uint8)  # Unknown
        
        valid_mask = is_valid > 0.5
        if not np.any(valid_mask):
            return costmap
        
        # Resolution from GridMap metadata (assumed square cells)
        resolution = 0.05  # TODO: get from GridMap info
        
        # 1. Slope (gradient magnitude)
        grad_x = np.gradient(elevation, axis=1) / resolution
        grad_y = np.gradient(elevation, axis=0) / resolution
        slope = np.sqrt(grad_x**2 + grad_y**2)  # radians
        
        # 2. Step height (local max - min in 3x3)
        step_height = maximum_filter(elevation, size=3) - minimum_filter(elevation, size=3)
        
        # 3. Roughness (variance)
        roughness = variance
        
        # Classify
        lethal = (slope > self.max_slope) | \
                 (step_height > self.max_step) | \
                 (roughness > self.max_roughness)
        
        difficult = (slope > self.max_slope * 0.5) | \
                    (step_height > self.max_step * 0.5)
        
        costmap[valid_mask] = 0  # Free by default
        costmap[valid_mask & difficult] = 50   # High cost
        costmap[valid_mask & lethal] = 100     # Lethal
        
        return costmap
    
    def publish_costmap(self, costmap: np.ndarray, gridmap_msg: GridMap):
        """Publish as OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header = gridmap_msg.header
        msg.header.frame_id = 'map'  # Nav2 expects 'map' frame
        
        msg.info = MapMetaData()
        msg.info.resolution = gridmap_msg.info.resolution
        msg.info.width = costmap.shape[1]
        msg.info.height = costmap.shape[0]
        
        # Origin: GridMap center is at pose.position, convert to bottom-left
        origin_x = gridmap_msg.info.pose.position.x - gridmap_msg.info.length_x / 2
        origin_y = gridmap_msg.info.pose.position.y - gridmap_msg.info.length_y / 2
        msg.info.origin = Pose()
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten row-major (GridMap is column-major for GridMap msg)
        msg.data = costmap.flatten().tolist()
        
        self.costmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ElevationToCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 3. Nav2 Configuration

### 3.1 `nav2_params.yaml` Additions

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # Use elevation costmap as static layer
      plugins: ["elevation_layer", "inflation_layer"]
      
      elevation_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/elevation_costmap"
        subscribed_topics: []
        track_unknown_space: true
        use_maximum: false
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      # ... existing LiDAR config ...
```

### 3.2 Static Transform (Map → Elevation Map)

If elevation map frame differs from `map`:

```bash
# In launch file:
static_transform_publisher = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'elevation_map_frame']
)
```

---

## 4. Launch File Integration

### 4.1 Complete Launch (`launch_elevation_nav.py`)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. UAV with depth camera (Gazebo)
    uav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('my_bot'), '/launch/spawn_uav_depth.launch.py'
        ])
    )
    
    # 2. Elevation mapping node
    elevation_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('elevation_mapping_cupy'),
            'config', 'elevation_mapping.yaml'
        )]
    )
    
    # 3. Costmap converter
    converter_node = Node(
        package='terralink_elevation',  # Your new package
        executable='elevation_to_costmap_node.py',
        name='elevation_to_costmap',
        output='screen',
        parameters=[{
            'max_slope': 0.35,
            'max_step': 0.15,
            'max_roughness': 0.05,
        }]
    )
    
    # 4. Nav2 stack (delayed until map ready)
    nav2_launch = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('my_bot'), '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'params_file': os.path.join(
                    get_package_share_directory('my_bot'),
                    'config', 'nav2_params_elevation.yaml'
                )
            }.items()
        )
    ])
    
    # 5. Static TF: map → elevation_map_frame (if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'elevation_mapping_node_map']
    )
    
    return LaunchDescription([
        uav_launch,
        elevation_node,
        converter_node,
        static_tf,
        nav2_launch,
    ])
```

---

## 5. Parameter Tuning Guide

### 5.1 Robot-Specific Parameters

| Parameter | Small Robot (0.3m) | Medium Robot (0.5m) | Large Robot (1.0m) |
|-----------|-------------------|---------------------|-------------------|
| `max_slope` | 0.40 (22°) | 0.35 (20°) | 0.25 (14°) |
| `max_step` | 0.10m | 0.15m | 0.25m |
| `max_roughness` | 0.03m | 0.05m | 0.08m |
| `robot_radius` | 0.20m | 0.25m | 0.50m |
| `inflation_radius` | 0.40m | 0.55m | 0.80m |

### 5.2 Sensor-Specific Parameters

| Sensor | `min_valid_distance` | `max_height_range` | `sensor_noise_factor` |
|--------|---------------------|-------------------|----------------------|
| RealSense D435i | 0.3m | 1.0m | 0.05 |
| ZED 2i | 0.5m | 2.0m | 0.03 |
| Livox Mid-360 | 0.2m | 5.0m | 0.01 |

### 5.3 Environment-Specific

| Environment | `cleanup_cos_thresh` | `enable_visibility_cleanup` |
|-------------|---------------------|----------------------------|
| Structured (indoor) | 0.5 | True |
| Unstructured (outdoor) | 0.3 | True |
| Vegetation heavy | 0.7 | False (rays hit leaves) |

---

## 6. Debugging & Visualization

### 6.1 Verify Topics

```bash
# Check GridMap publishing
ros2 topic hz /elevation_mapping_node/elevation_map
ros2 topic echo /elevation_mapping_node/elevation_map --once | head -50

# Check costmap
ros2 topic hz /elevation_costmap
ros2 topic echo /elevation_costmap --once

# Check Nav2 costmap
ros2 topic hz /global_costmap/costmap
```

### 6.2 RViz Setup

1. Add **GridMap** display → Topic: `/elevation_mapping_node/elevation_map`
2. Add **Map** display → Topic: `/elevation_costmap` (for converter debug)
3. Add **Map** display → Topic: `/global_costmap/costmap` (Nav2 view)
4. Add **TF** display → Check `map` → `base_link` chain

### 6.3 Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Costmap all 255 (unknown) | `is_valid` layer all 0 | Check sensor TF, pointcloud frame |
| Robot avoids flat ground | Slope threshold too low | Increase `max_slope` |
| Map doesn't follow robot | Pose update not working | Check `base_frame` param, TF tree |
| RViz shows map rotated | Coordinate convention mismatch | Verify `_transform_to_grid_map_convention` |
| Drift compensation triggers falsely | Pose noise high | Increase `position_noise_thresh` |

---

## 7. Performance Optimization

### 7.1 GPU Memory

```python
# In parameter.py or YAML:
# Reduce map size for limited VRAM:
map_length: 10.0    # Instead of 20.0
resolution: 0.05    # Instead of 0.04
# cell_n = 10/0.05 + 2 = 202 → 202² × 7 layers × 4 bytes ≈ 1.1 MB
```

### 7.2 Update Rates

```yaml
# In elevation_mapping.yaml:
publishers:
  elevation_map:
    fps: 2.0          # Lower if CPU limited
update_pose_fps: 10.0 # Match odom rate
update_variance_fps: 5.0
```

---

## 8. Testing Checklist

- [ ] UAV publishes PointCloud2 on `/camera/depth/points`
- [ ] TF: `map` → `camera_depth_optical_frame` exists
- [ ] Elevation mapping node receives pointcloud (check counter)
- [ ] GridMap published on `/elevation_mapping_node/elevation_map`
- [ ] Converter node outputs `/elevation_costmap`
- [ ] Nav2 global_costmap receives elevation costmap
- [ ] Robot navigates avoiding steep/rough terrain
- [ ] Drift compensation doesn't cause map jumps

---

*This integration guide assumes you have the elevation_mapping_cupy package built and running. The converter node is a new package you'll create in your workspace.*