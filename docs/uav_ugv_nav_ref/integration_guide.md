# Integration Guide - Direction 3: UAV-UGV Navigation

This guide explains how to **extend, modify, and integrate** the Direction 3 system with other components.

---

## Table of Contents
1. [Adding New Features](#1-adding-new-features)
2. [Integrating with Direction 1 (Elevation Mapping)](#2-integrating-with-direction-1-elevation-mapping)
3. [Integrating with Direction 2 (Semantic Vision)](#3-integrating-with-direction-2-semantic-vision)
4. [Real Robot Deployment](#4-real-robot-deployment)
5. [Multi-Robot Support](#5-multi-robot-support)
6. [Testing & CI/CD](#6-testing--cicd)

---

## 1. Adding New Features

### 1.1 Stuck Detection (High Priority)

**Problem**: Robot stops mid-path with no recovery.

**Solution**: Add watchdog in `waypoints_client.cpp`:

```cpp
// Add to class members:
rclcpp::Time last_progress_time_;
geometry_msgs::msg::Point last_position_;
const double STUCK_TIMEOUT = 5.0;  // seconds
const double MIN_PROGRESS = 0.1;   // meters

// In odom_callback():
void odom_callback(const Odometry::SharedPtr msg) {
    // ... existing code ...
    
    // Check progress
    double dist = hypot(msg->pose.pose.position.x - last_position_.x,
                        msg->pose.pose.position.y - last_position_.y);
    
    if (dist > MIN_PROGRESS) {
        last_progress_time_ = this->now();
        last_position_ = msg->pose.pose.position;
    }
    
    // Stuck detection
    if (waypoints_ && waypoints_->valid && 
        (this->now() - last_progress_time_).seconds() > STUCK_TIMEOUT) {
        RCLCPP_WARN(logger, "[Stuck] No progress for %.1fs, requesting new path!", STUCK_TIMEOUT);
        waypoints_.reset();  // Force re-plan
        timer_callback();    // Request new path immediately
    }
}
```

### 1.2 Path Retry Logic

**Current**: Retries every 10s only if initial request failed.

**Improved**: Retry on any failure (invalid path, stuck, timeout):

```cpp
// Add method:
void request_new_path() {
    if (!waypoints_client_->wait_for_service(1s)) return;
    
    auto request = make_shared<GetWaypoints::Request>();
    request->start.x = start_pose_.x;
    request->start.y = start_pose_.y;
    request->start.z = start_pose_.z;
    request->goal = goal_pose_;
    
    auto callback = [this](auto future) {
        auto result = future.get();
        if (result->valid) {
            waypoints_ = result;
            way_ind = 0;
            last_progress_time_ = this->now();
            RCLCPP_INFO(logger, "[Retry] New path received!");
        } else {
            RCLCPP_WARN(logger, "[Retry] Still invalid, will retry...");
        }
    };
    waypoints_client_->async_send_request(request, callback);
}
```

### 1.3 Adaptive Color Thresholding

**Problem**: Hardcoded BGR thresholds fail in varying lighting.

**Solution**: Learn floor color from first frames:

```cpp
// In waypoints_server.cpp - add to class:
std::vector<cv::Vec3b> floor_samples_;
const int LEARNING_FRAMES = 30;
bool learned_ = false;
cv::Scalar learned_lower_, learned_upper_;

// In image_callback():
if (!learned_ && warming < LEARNING_FRAMES) {
    // Sample center region (assumed to be floor)
    cv::Rect roi(cv_image.cols/4, cv_image.rows/4, cv_image.cols/2, cv_image.rows/2);
    cv::Mat center = cv_image(roi);
    cv::Scalar mean = cv::mean(center);
    floor_samples_.push_back(cv::Vec3b(mean[0], mean[1], mean[2]));
    
    if (warming == LEARNING_FRAMES - 1) {
        // Compute statistics
        cv::Vec3d sum(0,0,0);
        for (auto& s : floor_samples_) sum += s;
        cv::Vec3d avg = sum / floor_samples_.size();
        
        // Set adaptive thresholds (±30 in each channel)
        learned_lower_ = cv::Scalar(max(0,avg[0]-30), max(0,avg[1]-30), max(0,avg[2]-30));
        learned_upper_ = cv::Scalar(min(255,avg[0]+30), min(255,avg[1]+30), min(255,avg[2]+30));
        learned_ = true;
        RCLCPP_INFO(logger, "[Adaptive] Learned floor color: lower=%d,%d,%d upper=%d,%d,%d",
            (int)learned_lower_[0], (int)learned_lower_[1], (int)learned_lower_[2],
            (int)learned_upper_[0], (int)learned_upper_[1], (int)learned_upper_[2]);
    }
    return;  // Skip PRM during learning
}

// In process_image():
if (learned_) {
    cv::inRange(cv_image, learned_lower_, learned_upper_, mask);
} else {
    // Fallback to hardcoded
    cv::inRange(cv_image, cv::Scalar(100,100,100), cv::Scalar(180,180,180), mask);
}
```

### 1.4 HSV Color Space (More Robust)

```cpp
cv::Mat hsv;
cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

// Floor in HSV: Low saturation, medium value
cv::Scalar lower(0, 0, 50);      // Any hue, low saturation, min brightness
cv::Scalar upper(179, 60, 200);  // Any hue, max saturation, max brightness
cv::inRange(hsv, lower, upper, mask);
```

---

## 2. Integrating with Direction 1 (Elevation Mapping)

### Architecture
```
UAV Sensors:
├── Downward RGB Camera → Direction 3 (PRM)
└── RGB-D / LiDAR → Direction 1 (Elevation Map)

Fusion:
Elevation Map (GridMap) + PRM Graph → Traversability Costmap → Nav2
```

### 2.1 Elevation to Costmap Converter

Create new package `elevation_costmap_converter`:

```cpp
// elevation_to_costmap_node.cpp
class ElevationToCostmap : public rclcpp::Node {
public:
    ElevationToCostmap() : Node("elevation_to_costmap") {
        // Subscribe to elevation map
        sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
            "/elevation_mapping_node/elevation_map", 10,
            bind(&ElevationToCostmap::elevation_callback, this, _1));
        
        // Publish Nav2 costmap
        pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/elevation_costmap", 10);
    }

private:
    void elevation_callback(const GridMap::SharedPtr msg) {
        // Extract layers
        const auto& elevation = msg->data[0];  // elevation layer
        const auto& variance = msg->data[1];   // variance layer
        const auto& traversability = msg->data[2]; // traversability layer
        
        // Convert to OccupancyGrid
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header = msg->header;
        costmap.info.resolution = msg->info.resolution;
        costmap.info.width = msg->info.length_x / msg->info.resolution;
        costmap.info.height = msg->info.length_y / msg->info.resolution;
        costmap.info.origin = msg->info.pose;
        
        // Map traversability (0-1) to cost (0-255)
        // traversability=1 (easy) → cost=0 (free)
        // traversability=0 (hard) → cost=255 (lethal)
        costmap.data.resize(costmap.info.width * costmap.info.height);
        for (size_t i = 0; i < traversability.data.size(); ++i) {
            float trav = traversability.data[i];
            if (std::isnan(trav)) {
                costmap.data[i] = -1;  // Unknown
            } else {
                costmap.data[i] = static_cast<int8_t>((1.0 - trav) * 255);
            }
        }
        
        pub_->publish(costmap);
    }
};
```

### 2.2 Nav2 Costmap Fusion

In `nav2_params.yaml`, add elevation costmap as plugin:

```yaml
global_costmap:
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: elevation_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

elevation_layer:
  topic: "/elevation_costmap"
  subscription_duration: 0.5
  lethal_cost_threshold: 200
  unknown_cost_value: 255
  track_unknown_space: true
```

---

## 3. Integrating with Direction 2 (Semantic Vision)

### Architecture
```
UAV RGB Camera → Semantic Segmentation (YOLOv8-seg/SegFormer)
                    ↓
            Semantic Classes → Costmap Converter
                    ↓
            Semantic Costmap (class-specific costs) → Nav2
```

### 3.1 Semantic Classes to Costmap Mapping

Based on SegFormer Aerial results:
| Class | Nav2 Cost | Rationale |
|-------|-----------|-----------|
| paved-area (road) | 0 (FREE) | Primary traversable |
| dirt/grass/gravel | 50-100 | Traversable but slower |
| vegetation | 150 | Difficult but possible |
| obstacle/water/wall | 255 (LETHAL) | Impassable |

### 3.2 Costmap Converter Node

```cpp
// semantic_costmap_converter.cpp
class SemanticCostmapConverter : public rclcpp::Node {
    std::map<int, int8_t> class_to_cost_ = {
        {1, 0},    // paved-area
        {2, 50},   // dirt
        {3, 50},   // grass
        {4, 75},   // gravel
        {5, 255},  // water
        {6, 200},  // rocks
        {7, 255},  // pool
        {8, 150},  // vegetation
        {9, 255},  // roof
        {10, 255}, // wall
        {22, 255}, // obstacle
    };

    void semantic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert class IDs to costs
        cv::Mat semantic = cv_bridge::toCvCopy(msg, "mono8")->image;
        cv::Mat costmap(semantic.size(), CV_8SC1);
        
        for (int y = 0; y < semantic.rows; ++y) {
            for (int x = 0; x < semantic.cols; ++x) {
                int cls = semantic.at<uchar>(y, x);
                costmap.at<int8_t>(y, x) = class_to_cost_.count(cls) 
                    ? class_to_cost_[cls] : 100;  // Default medium cost
            }
        }
        
        // Project to world frame using camera model
        // Publish as OccupancyGrid
    }
};
```

---

## 4. Real Robot Deployment

### 4.1 Hardware Requirements

| Component | Specification | Notes |
|-----------|---------------|-------|
| **UAV** | DJI Matrice 300 / custom | Downward RGB + RGB-D |
| **UGV** | Clearpath Jackal / Husky | Diff drive, LiDAR |
| **Compute (UAV)** | Jetson Orin NX / AGX | 70-275 TOPS |
| **Compute (UGV)** | Intel NUC / Jetson | Nav2 + Localization |
| **Comms** | WiFi 6 / 4G/5G / DDS | <100ms latency |

### 4.2 Software Changes for Real Robots

#### Replace Gazebo with Real Drivers
```yaml
# launch_real.launch.py
# Instead of Gazebo:
- UAV: dji_osdk_ros / mavros for camera
- UGV: ros2_control + hardware_interface
- Localization: slam_toolbox / amcl + map_server
```

#### Camera Calibration (Critical!)
```cpp
// Replace hardcoded coordToPixel with calibrated model
// Use camera_info_manager + OpenCV calib3d
cv::FileStorage fs("camera_calib.yaml", cv::FileStorage::READ);
cv::Mat camera_matrix, dist_coeffs;
fs["camera_matrix"] >> camera_matrix;
fs["distortion_coefficients"] >> dist_coeffs;

// Use cv::projectPoints / cv::undistortPoints
```

#### Time Synchronization
```bash
# On both robots:
sudo apt install chrony
# Configure /etc/chrony/chrony.conf with common NTP server
# Or use PTP (Precision Time Protocol) for sub-ms sync
```

#### DDS Configuration for Multi-Robot
```xml
<!-- FASTDDS profiles.xml -->
<participant profile_name="uav_participant">
    <rtps>
        <builtin>
            <domainId>1</domainId>  <!-- UAV domain -->
        </builtin>
    </rtps>
</participant>

<participant profile_name="ugv_participant">
    <rtps>
        <builtin>
            <domainId>2</domainId>  <!-- UGV domain -->
        </builtin>
    </rtps>
</participant>
```

---

## 5. Multi-Robot Support

### 5.1 Namespacing
```python
# launch_multi.launch.py
def generate_launch_description():
    robots = ['ugv_1', 'ugv_2', 'ugv_3']
    
    for robot in robots:
        # Each robot gets own namespace
        nav2 = IncludeLaunchDescription(nav2_launch.py, 
            launch_arguments={'namespace': robot, 'use_sim_time': 'true'})
        
        # Remap topics
        remappings = [
            ('/cmd_vel', f'/{robot}/cmd_vel'),
            ('/odom', f'/{robot}/odom'),
            ('/scan', f'/{robot}/scan'),
            ('/goal_pose', f'/{robot}/goal_pose'),
        ]
```

### 5.2 Shared UAV (Single UAV, Multiple UGVs)
```python
# UAV publishes once, all UGVs subscribe
# UAV topic: /uav/camera/image_raw
# Each UGV runs own waypoints_server with different goal
```

---

## 6. Testing & CI/CD

### 6.1 Unit Tests (GoogleTest)

```cpp
// test/test_prm.cpp
#include <gtest/gtest.h>
#include "processImage.h"

TEST(GridSpaceTest, ManhattanDistance) {
    GridSpace gs(100);
    EXPECT_EQ(gs.manhattanDistance({0,0}, {3,4}), 7);
    EXPECT_EQ(gs.manhattanDistance({10,10}, {10,10}), 0);
}

TEST(GridSpaceTest, CoordToPixelRoundTrip) {
    GridSpace gs(100);
    gs.width = 1000; gs.height = 1000;
    
    auto pixel = gs.coordToPixel({0.0, 0.0});
    auto coord = gs.pixelToCoord(pixel);
    
    EXPECT_NEAR(coord.first, 0.0, 0.01);
    EXPECT_NEAR(coord.second, 0.0, 0.01);
}

TEST(GridSpaceTest, CheckLineObstacle) {
    GridSpace gs(100);
    gs.width = 10; gs.height = 10;
    gs.grid = cv::Mat::ones(10, 10, CV_8UC1) * 255;
    gs.grid.at<uchar>(5, 5) = 0;  // Obstacle at center
    
    EXPECT_TRUE(gs.checkLine({0,0}, {9,0}));   // Clear horizontal
    EXPECT_FALSE(gs.checkLine({0,5}, {9,5}));  // Through obstacle
}
```

### 6.2 Integration Tests (launch_testing)

```python
# test/test_integration.py
import launch_testing
import pytest
import rclpy
from tutorial_interfaces.srv import GetWaypoints

@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node('test_node')
    yield n
    n.destroy_node()
    rclpy.shutdown()

def test_waypoints_service(node):
    client = node.create_client(GetWaypoints, 'waypoints_service')
    assert client.wait_for_service(timeout_sec=10.0)
    
    request = GetWaypoints.Request()
    request.start.x = 0.0; request.start.y = 0.0
    request.goal.x = 1.0; request.goal.y = 1.0
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    assert result.valid == True
    assert len(result.waypoints) > 0
```

### 6.3 CI Pipeline (GitHub Actions)

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build_and_test:
    runs-on: ubuntu-22.04
    container: ros:humble-ros-base-jammy
    
    steps:
    - uses: actions/checkout@v4
    
    - name: Install dependencies
      run: |
        apt update && apt install -y python3-colcon-common-extensions
        rosdep install --from-paths src --ignore-src -r -y
    
    - name: Build
      run: |
        source /opt/ros/humble/setup.bash
        colcon build --packages-select my_bot tutorial_interfaces
    
    - name: Unit tests
      run: |
        source install/local_setup.bash
        colcon test --packages-select my_bot --event-handlers console_direct+
    
    - name: Integration test
      run: |
        source install/local_setup.bash
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        ros2 launch my_bot launch_sim.launch.py &
        sleep 30
        # Run test_integration.py
        python3 -m pytest test/test_integration.py -v
```

---

## Migration Path: Direction 3 → Production

### Phase 1: Robustness (Current → 1 month)
- [ ] Stuck detection + auto-replan
- [ ] Adaptive color thresholds
- [ ] HSV color space
- [ ] Orientation in waypoints (fix nav2_handler)
- [ ] Unit tests for PRM

### Phase 2: Sensor Fusion (1-2 months)
- [ ] Elevation mapping integration (Direction 1)
- [ ] Semantic vision integration (Direction 2)
- [ ] Multi-layer costmap fusion
- [ ] Camera calibration pipeline

### Phase 3: Real Robot (2-3 months)
- [ ] Hardware drivers (UAV camera, UGV base)
- [ ] Localization (SLAM/AMCL)
- [ ] Time sync (PTP/Chrony)
- [ ] DDS multi-robot config
- [ ] Field testing

### Phase 4: Production (3+ months)
- [ ] Safety certification
- [ ] Fail-safe behaviors
- [ ] Remote monitoring/teleop
- [ ] Fleet management

---

## Quick Reference: Extension Points

| Extension | File(s) | Difficulty |
|-----------|---------|------------|
| Stuck detection | `waypoints_client.cpp` | Easy |
| Path retry | `waypoints_client.cpp` | Easy |
| Adaptive color | `waypoints_server.cpp` | Medium |
| HSV threshold | `waypoints_server.cpp` | Easy |
| Elevation fusion | New package + nav2_params.yaml | Medium |
| Semantic fusion | New package + costmap converter | Medium |
| Real robot drivers | New launch files + hw interfaces | Hard |
| Multi-robot | Namespaced launch files | Medium |