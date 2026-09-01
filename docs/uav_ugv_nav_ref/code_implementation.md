# Code Implementation Walkthrough - Direction 3

This document provides a **line-by-line explanation** of the Direction 3 source code. Perfect for understanding exactly how the system works and how to modify it.

---

## File Structure

```
src/d3/
├── my_bot/
│   ├── includes/
│   │   ├── processImage.h      # GridSpace class declaration
│   │   └── processImage.cpp    # GridSpace implementation (PRM, A*, coords)
│   ├── src/
│   │   ├── waypoints_server.cpp  # ROS2 node: Image → PRM → Service
│   │   ├── waypoints_client.cpp  # ROS2 node: Path requests + tracking
│   │   ├── nav2_handler.cpp      # ROS2 node: /waypoints → /goal_pose
│   │   └── follow_waypoints.py   # Alternative Python client
│   ├── launch/
│   │   └── launch_sim.launch.py  # Main simulation launch
│   └── config/
│       ├── nav2_params.yaml      # Nav2 configuration
│       └── twist_mux.yaml        # Velocity multiplexer
└── tutorial_interfaces/
    └── srv/GetWaypoints.srv      # Service definition
```

---

## 1. `includes/processImage.h` - GridSpace Class Declaration

### Purpose
Core class implementing **Probabilistic Roadmap (PRM)** with A* search and coordinate transformations.

### Key Data Structures

```cpp
// Line 3-7: Path result container
struct Path {
    Path() = default;
    std::vector<std::pair<double, double>> waypoints;  // World coordinates
    bool valid = true;
};

// Line 9-12: Graph edge
struct Edge {
    uint32_t nodeIdx;  // Destination node index
    int cost;          // Edge weight (manhattan distance)
};

// Line 14-19: A* search node
struct ANode {
    uint32_t ind;      // Node index in graph
    uint32_t parent;   // Previous node in optimal path
    int cost;          // g(n): actual cost from start
    int priority;      // f(n) = g(n) + h(n): priority for queue
};
```

### Public Interface (Line 23-42)

```cpp
class GridSpace {
public:
    GridSpace(uint32_t maxNodes);           // Constructor
    void fillGrid(const cv::Mat& image);    // Initialize from binary mask
    void setGrid(const cv::Mat& image);     // Update grid (used each frame)
    void showPRM(const cv::Mat& image);     # Debug visualization
    bool checkLine(pair<int,int>, pair<int,int>);  # Line-of-sight (Bresenham)
    pair<int,int> getRandomPoint();         # Uniform random sampling
    void runPRM(init, goal, n);             # Build PRM (add n nodes)
    list<uint32_t> searchGraph(init, goal); # A* search
    Path getWaypoints(init_coord, goal_coord); # Full pipeline
    void connectNieghbors();                # Full graph connection (unused)
    void connectNewNode(node);              # Incremental connection
    int manhattanDistance(p1, p2);          # A* heuristic
    pair<int,int> coordToPixel(coord);      # World → Image pixels
    pair<double,double> pixelToCoord(pixel); # Image pixels → World
    uint32_t findNearest(pixel);            # Nearest PRM node
    map<uint32_t, pair<int,int>> getPoints(); // Expose nodes for debug
};
```

### Private Members (Line 43-50)
```cpp
int width, height;                           // Grid dimensions
uint32_t maxNodes;                           // Max PRM nodes (1000)
double captureHeight = 10.0;                 // UAV camera height (meters)
cv::Mat grid;                                // Binary occupancy grid
map<uint32_t, pair<int,int>> points;         // Valid PRM nodes
map<uint32_t, pair<int,int>> failedPoints;   # Failed samples (obstacles)
map<uint32_t, vector<Edge>> adjacencyList;   # Graph edges
```

---

## 2. `includes/processImage.cpp` - GridSpace Implementation

### `applyMask()` (Lines 10-18) - **Standalone Function**
```cpp
cv::Mat applyMask(const cv::Mat& image) {
    cv::Scalar lowerColor = cv::Scalar(154, 154, 154);  // Very narrow range
    cv::Scalar upperColor = cv::Scalar(156, 156, 156);  # Specific gray
    cv::Mat mask;
    cv::inRange(image, lowerColor, upperColor, mask);
    return mask;
}
```
> **Note**: This is NOT used in the main pipeline (server uses different thresholds). Legacy function.

---

### `fillGrid()` (Lines 21-27) - Initialize Grid
```cpp
void GridSpace::fillGrid(const cv::Mat& image) {
    width = image.cols;
    height = image.rows;
    grid = image.clone();                    // Copy input
    cv::threshold(grid, grid, 0, 255, cv::THRESH_BINARY);  // Ensure 0/255
    std::vector<std::vector<int>> occupancyGrid(height, vector<int>(width, 0));
}
```
**Purpose**: One-time initialization. Stores dimensions, binarizes grid.
**Note**: `occupancyGrid` created but unused (dead code).

---

### `setGrid()` (Lines 29-35) - Update Grid Each Frame
```cpp
void GridSpace::setGrid(const cv::Mat& image) {
    width = image.cols;
    height = image.rows;
    grid = image.clone();
    // cv::imshow("GRID", grid);  // Debug
    // cv::waitKey(0);
}
```
**Called from**: `waypoints_server::image_callback()` each frame.

---

### `checkLine()` (Lines 37-49) - Bresenham Line-of-Sight
```cpp
bool GridSpace::checkLine(pair<int,int> start, pair<int,int> end) {
    int x1 = start.second;  // Note: start.first = row (y), start.second = col (x)
    int y1 = start.first;
    int x2 = end.second;
    int y2 = end.first;
    
    // OpenCV LineIterator: iterates pixels along line
    cv::LineIterator it(grid, cv::Point(x1, y1), cv::Point(x2, y2), 8);
    for (int i = 0; i < it.count; i++, ++it) {
        int x = it.pos().x;
        int y = it.pos().y;
        if (grid.at<uchar>(y, x) != 255)  // 255 = free, 0 = obstacle
            return false;  // Hit obstacle
    }
    return true;  // Clear line of sight
}
```
**Key**: Uses OpenCV's `LineIterator` with 8-connectivity. Checks if ANY pixel on line is obstacle (0).

---

### `getRandomPoint()` (Lines 51-59) - Uniform Sampling
```cpp
pair<int,int> GridSpace::getRandomPoint() {
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    mt19937 gen(seed);
    uniform_int_distribution<int> random_row(0, width - 1);
    uniform_int_distribution<int> random_col(0, height - 1);
    int i = random_row(gen);  // x (col)
    int j = random_col(gen);  // y (row)
    return {i, j};
}
```
**Note**: Creates NEW random generator each call (inefficient). Uses `chrono` seed.

---

### `runPRM()` (Lines 61-76) - Main PRM Builder
```cpp
void GridSpace::runPRM(const pair<int,int>& init, const pair<int,int>& goal, int n) {
    int ind = 0;
    pair<int,int> qRand;
    if (points.size() == 0) points[0] = {0, 0};  // Seed with origin
    
    while (ind < n && points.size() < maxNodes) {
        qRand = getRandomPoint();
        if (grid.at<uchar>(qRand.first, qRand.second) != 0) {  // Free space?
            points[points.size()] = qRand;        // Add node
            connectNewNode(points.size() - 1);    // Connect to graph
        } else {
            failedPoints[failedPoints.size()] = qRand;  // Track failures
        }
        ind++;
    }
    cout << "NODES IN PRM:" << points.size() << "\n";
}
```
**Called from**: `waypoints_server::image_callback()` with `n=100` per frame.
**Builds incrementally**: Adds 100 nodes per camera frame until `maxNodes=1000`.

---

### `connectNieghbors()` (Lines 78-94) - Full Connection (UNUSED)
```cpp
void GridSpace::connectNieghbors() {
    int n = points.size();
    int r = int(width/8);  // Larger radius for full connection
    
    for (i = 0; i < n; i++) {
        for (j = i+1; j < n; j++) {
            norm = manhattanDistance(points[i], points[j]);
            if (norm < r && checkLine(points[i], points[j])) {
                adjacencyList[i].push_back({j, norm});
                adjacencyList[j].push_back({i, norm});
            }
        }
    }
}
```
**Not called** - `connectNewNode()` used instead for incremental building.

---

### `connectNewNode()` (Lines 96-113) - Incremental Connection
```cpp
void GridSpace::connectNewNode(uint32_t node) {
    int n = points.size();
    int r = int(width/12);  // Smaller radius = local connections
    uint32_t j = node;      // New node index
    
    for (i = 0; i < n; i++) {
        if (i != j) {
            norm = manhattanDistance(points[i], points[j]);
            if (norm < r && checkLine(points[i], points[j])) {
                adjacencyList[i].push_back({j, norm});
                adjacencyList[j].push_back({i, norm});
            }
        }
    }
}
```
**Key**: Only connects NEW node to existing nodes within radius `width/12`. Much faster than full rebuild.

---

### `manhattanDistance()` (Lines 115-117) - A* Heuristic
```cpp
int GridSpace::manhattanDistance(const pair<int,int>& cell1, const pair<int,int>& cell2) {
    return abs(cell1.first - cell2.first) + abs(cell1.second - cell2.second);
}
```
**Used for**: A* heuristic (admissible for grid) + connection radius comparison.

---

### `showPRM()` (Lines 119-145) - Debug Visualization
```cpp
void GridSpace::showPRM(const cv::Mat& image) {
    for (const auto& pair : adjacencyList) {
        uint32_t ind = pair.first;
        // Draw edges
        for (const Edge& edge : pair.second) {
            cv::line(image, cv::Point(y, x), cv::Point(destY, destX), 
                     cv::Scalar(10, 135, 215), 1);  // Orange lines
        }
        // Draw nodes
        cv::circle(image, cv::Point(y, x), 5, cv::Scalar(243, 150, 33), -1);  // Blue circles
    }
    cv::imshow("Nodes and Edges", image);
    cv::waitKey(0);
}
```
**Note**: Blocking `waitKey(0)` - only for manual debugging.

---

### `searchGraph()` (Lines 147-203) - A* Search
```cpp
list<uint32_t> GridSpace::searchGraph(uint32_t init, uint32_t goal) {
    ANode currNode, node;
    currNode.ind = init;
    list<ANode> queue = {currNode};           // Priority queue (sorted by f)
    map<uint32_t, ANode> openList;            // Nodes in queue
    map<uint32_t, ANode> processed;           // Expanded nodes
    openList[init] = currNode;
    
    while (success < 0) {
        queue.sort(compareCost);              // Sort by priority (f = g+h)
        currNode = queue.back();              // Get LOWEST f(n)
        queue.pop_back();
        openList.erase(currNode.ind);
        processed[currNode.ind] = currNode;
        
        for (const Edge& edge : adjacencyList[currNode.ind]) {
            index = edge.nodeIdx;
            cost = edge.cost;
            if (index == goal) cout << "BING\n";
            
            if (processed.count(index) == 0) {
                // New path cost = current g + edge cost
                node = {index, currNode.ind, currNode.cost + cost, 
                        currNode.cost + cost};  // priority = g+h (h=0 here?)
                
                if (openList.count(index) == 0) {
                    queue.push_back(node);
                    openList[index] = node;
                } else if (openList[index].cost > currNode.cost + cost) {
                    // Found better path to existing open node
                    queue.remove_if([index](const ANode& a) { return a.ind == index; });
                    queue.push_back(node);
                    openList[index] = node;
                }
            }
        }
        if (currNode.ind == goal) success = 0;
        else if (queue.empty()) success = 1;
    }
    
    // Reconstruct path
    if (success == 0) {
        index = currNode.ind;
        while (index != init) {
            path.push_front(index);
            index = processed[index].parent;
        }
    }
    return path;
}
```
**Note**: Priority = cost (no separate heuristic h(n) added). Manhattan used only for connection radius.

---

### `getWaypoints()` (Lines 205-217) - Full Pipeline
```cpp
Path GridSpace::getWaypoints(const pair<double,double>& init, const pair<double,double>& goal) {
    Path path;
    uint32_t initNode = findNearest(coordToPixel(init));
    uint32_t goalNode = findNearest(coordToPixel(goal));
    list<uint32_t> nodePath = searchGraph(initNode, goalNode);
    
    for (uint32_t node : nodePath) {
        path.waypoints.push_back(pixelToCoord(points[node]));
    }
    if (nodePath.size() == 0) path.valid = false;
    return path;
}
```
**Pipeline**: World coords → Pixel coords → Nearest PRM nodes → A* → World coords

---

### `compareCost()` (Lines 219-221) - Queue Sorting
```cpp
bool GridSpace::compareCost(const ANode& a, const ANode& b) {
    return a.priority > b.priority;  // Higher priority = lower cost = back of list
}
```
**Note**: `list.sort()` puts highest at back, so `>` gives min-heap behavior.

---

### `findNearest()` (Lines 223-235) - Linear Search
```cpp
uint32_t GridSpace::findNearest(const pair<int,int>& pixel) {
    pair<int,int> nearest = points[0];
    int n = points.size();
    uint32_t ind = 0;
    for (uint32_t i = 1; i < points.size(); i++) {
        if (manhattanDistance(pixel, points[i]) < manhattanDistance(pixel, nearest)) {
            nearest = points[i];
            ind = i;
        }
    }
    return ind;
}
```
**O(n)** search - fine for 1000 nodes. Could use KD-tree for larger graphs.

---

### `coordToPixel()` (Lines 237-256) - World → Image Pixels
```cpp
pair<int,int> GridSpace::coordToPixel(const pair<double,double>& coordinate) {
    float fov_horizontal_rad = 1.25f;  // ~71.6°
    float fov_vertical_rad = 1.25f;
    int image_width = 1000;
    int image_height = 1000;
    float x = -coordinate.first;   // Negative: world +X = image -X
    float y = -coordinate.second;  // Negative: world +Y = image -Y

    // Focal length from FOV
    float focal_length_horizontal = image_width / (2 * tan(fov_horizontal_rad / 2));
    float focal_length_vertical = image_height / (2 * tan(fov_vertical_rad / 2));

    // Pinhole projection: pixel = (f * X) / Z + center
    int pixel_x = static_cast<int>((focal_length_horizontal * x) / captureHeight + (image_width / 2));
    int pixel_y = static_cast<int>((focal_length_vertical * y) / captureHeight + (image_height / 2));

    return {pixel_x, pixel_y};
}
```
**Camera Model**: Downward-facing pinhole at height `captureHeight=10m`.
**Coordinate Flip**: Negative signs because camera looks DOWN (Z-axis).

---

### `pixelToCoord()` (Lines 259-277) - Image Pixels → World
```cpp
pair<double,double> GridSpace::pixelToCoord(const pair<int,int>& pixel) {
    float fov_horizontal_rad = 1.25f;
    float fov_vertical_rad = 1.25f;
    int image_width = 1000;
    int image_height = 1000;

    float focal_length_horizontal = image_width / (2 * tan(fov_horizontal_rad / 2));
    float focal_length_vertical = image_height / (2 * tan(fov_vertical_rad / 2));

    // Inverse projection
    float x = ((pixel.first - (image_width / 2)) * captureHeight) / focal_length_horizontal;
    float y = ((pixel.second - (image_height / 2)) * captureHeight) / focal_length_vertical;

    return {-x, -y};  // Flip back to world coordinates
}
```
**Inverse** of `coordToPixel()`. Used to convert path pixels back to world waypoints.

---

### `getPoints()` (Lines 280-282) - Debug Accessor
```cpp
map<uint32_t, pair<int,int>> GridSpace::getPoints() {
    return points;
}
```

---

## 3. `src/waypoints_server.cpp` - ROS2 Service Node

### Class Structure (Lines 13-122)
```cpp
class WaypointServerNode : public rclcpp::Node {
public:
    WaypointServerNode() : Node("waypoints_server") {
        // Image subscription (sensor data QoS)
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/my_uav/camera_uav/image_raw", rclcpp::SensorDataQoS(),
            bind(&WaypointServerNode::image_callback, this, _1));

        // Service server
        waypoints_service_ = create_service<GetWaypoints>(
            "waypoints_service",
            bind(&WaypointServerNode::metadata_service_callback, this, _1, _2, _3));
        
        maxNodes = 1000;
        GridSpacePtr = make_shared<GridSpace>(maxNodes);
    }
```

### `image_callback()` (Lines 31-47) - Main Pipeline
```cpp
void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (warming > 2) {  // Skip first 2 frames
        if (GridSpacePtr->getPoints().size() < maxNodes) {
            GridSpacePtr->setGrid(process_image(msg));  // Update binary grid
            GridSpacePtr->runPRM({0, 0}, {1, 1}, 100);  // Add 100 PRM nodes
            
            // Log progress
            size_t nodes_built = GridSpacePtr->getPoints().size();
            if (nodes_built >= maxNodes) {
                RCLCPP_INFO(logger, "[Sim] PRM graph generation complete!");
            } else {
                RCLCPP_INFO(logger, "[Init] Building PRM graph... (%ld/%d)", nodes_built, maxNodes);
            }
        }
    }
    warming++;
}
```
**Key Logic**:
- `warming > 2`: Skip first 2 frames (let camera stabilize)
- `runPRM(..., 100)`: Add 100 nodes per frame
- Stops at 1000 nodes

---

### `process_image()` (Lines 49-62) - Color Filtering
```cpp
cv::Mat process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // Stored for debug
    
    // HARDCODED color range for gray floor
    cv::Scalar lowerColor = cv::Scalar(100, 100, 100);  // Dark gray
    cv::Scalar upperColor = cv::Scalar(180, 180, 180);  // Light gray
    cv::Mat mask;
    cv::inRange(cv_image, lowerColor, upperColor, mask);
    return mask;  // Binary: 255=floor, 0=obstacle
}
```
**Limitations**: Only works in this specific simulation lighting.

---

### `metadata_service_callback()` (Lines 64-112) - Service Handler
```cpp
void metadata_service_callback(request_header, request, response) {
    geometry_msgs::msg::Point start = request->start;
    geometry_msgs::msg::Point goal = request->goal;
    
    // Get path from PRM
    Path path = GridSpacePtr->getWaypoints({start.x, start.y}, {goal.x, goal.y});
    response->valid = path.valid;
    
    if (path.valid) {
        RCLCPP_INFO(logger, "[Sim] Valid path generated and sent to client.");
    } else {
        RCLCPP_WARN(logger, "[Init] Requested path could not be found...");
    }
    
    // Convert to PoseStamped[] with YAW orientation
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        prev = path.waypoints[i-1];
        curr = path.waypoints[i];
        
        auto pose_msg = make_shared<geometry_msgs::msg::PoseStamped>();
        pose_msg->header.frame_id = "map";
        pose_msg->pose.position.x = curr.first;
        pose_msg->pose.position.y = curr.second;
        pose_msg->pose.position.z = 0.0;
        
        // Calculate yaw from direction vector
        double dx = curr.first - prev.first;
        double dy = curr.second - prev.second;
        double yaw = atan2(dy, dx);
        
        pose_msg->pose.orientation.w = cos(yaw / 2.0);
        pose_msg->pose.orientation.z = sin(yaw / 2.0);
        response->waypoints.push_back(*pose_msg);
    }
}
```
**Key**: Computes orientation from consecutive waypoints (direction vector → yaw → quaternion).

---

### Member Variables (Lines 114-121)
```cpp
bool open = true;
int warming = 0;
cv_bridge::CvImagePtr cv_ptr;
map<rclcpp::Time, int> metadata_;  // Unused
uint32_t maxNodes;
shared_ptr<GridSpace> GridSpacePtr;
Subscription<Image>::SharedPtr image_subscription_;
Service<GetWaypoints>::SharedPtr waypoints_service_;
```

---

## 4. `src/waypoints_client.cpp` - Path Request + Tracking

### Constructor (Lines 14-29)
```cpp
WaypointsClientNode() : Node("waypoints_client") {
    // Subscribe to UGV odometry
    odom_subscriber_ = create_subscription<Odometry>(
        "/odom", 10, bind(&WaypointsClientNode::odom_callback, this, _1));
    
    // Publish waypoints to Nav2 handler
    waypoints_publisher_ = create_publisher<PoseStamped>("/goal_pose", 10);
    
    // Service client
    waypoints_client_ = create_client<GetWaypoints>("waypoints_service");
    
    // Fixed goal (in map frame)
    goal_pose_.x = -4.5;
    goal_pose_.y = 3.0;
    goal_pose_.z = 0.0;
    
    // Timer: first request after 10s
    timer_ = create_wall_timer(10s, bind(&WaypointsClientNode::timer_callback, this));
}
```

### `odom_callback()` (Lines 40-62) - Waypoint Tracking
```cpp
void odom_callback(const Odometry::SharedPtr msg) {
    // Update current position
    start_pose_.x = msg->pose.pose.position.x;
    start_pose_.y = msg->pose.pose.position.y;
    start_pose_.z = msg->pose.pose.position.z;
    
    if (waypoints_) {  // Have valid path?
        if (waypoints_->valid && way_ind < waypoints_->waypoints.size()) {
            const auto waypoint = waypoints_->waypoints[way_ind];
            
            if (way_ind == 0) {  // First waypoint: publish immediately
                waypoints_publisher_->publish(waypoint);
                way_ind++;
            } else {
                // Check distance to PREVIOUS waypoint
                double distance = sqrt(pow(waypoints_->waypoints[way_ind-1].pose.position.x - start_pose_.x, 2) 
                                     + pow(waypoints_->waypoints[way_ind-1].pose.position.y - start_pose_.y, 2));
                if (distance < 0.5) {  // Within 0.5m → next waypoint
                    RCLCPP_INFO(logger, "Publishing waypoint to /waypoints");
                    waypoints_publisher_->publish(waypoint);
                    way_ind++;
                }
            }
        }
    }
}
```
**Logic**: Publish waypoint 0 immediately. For subsequent waypoints, wait until robot within 0.5m of PREVIOUS waypoint.

---

### `timer_callback()` (Lines 64-95) - Service Request
```cpp
void timer_callback() {
    if (!waypoints_) {  // Only if no path yet
        auto request = make_shared<GetWaypoints::Request>();
        request->start = start_pose_;  // Current position from odom
        request->goal = goal_pose_;    // Fixed goal
        
        // Wait for service
        while (!waypoints_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(logger, "[Init] Waiting for waypoints_service...");
        }
        
        RCLCPP_INFO(logger, "[Init] Requesting path from waypoints_service...");
        
        // Async call with callback
        auto callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (result->valid) {
                waypoints_ = result;
                RCLCPP_INFO(logger, "[Sim] Received valid path! Beginning waypoint navigation.");
            } else {
                RCLCPP_WARN(logger, "[Init] Received invalid path. Retrying in 10s...");
            }
        };
        waypoints_client_->async_send_request(request, callback);
    }
}
```
**Behavior**: Requests path once (10s after startup). Retries every 10s if invalid.

---

## 5. `src/nav2_handler.cpp` - Simple Forwarder

### Purpose
Receives `geometry_msgs/Point` on `/waypoints`, converts to `PoseStamped`, publishes to `/goal_pose` for Nav2.

```cpp
class Nav2Handler : public rclcpp::Node {
public:
    Nav2Handler() : Node("nav2_handler") {
        subscription_ = create_subscription<Point>(
            "/waypoints", 10,
            bind(&Nav2Handler::waypoint_callback, this, _1));
        nav_goal_publisher_ = create_publisher<PoseStamped>("/goal_pose", 10);
    }

private:
    void waypoint_callback(const Point::SharedPtr msg) {
        auto nav_goal = make_unique<PoseStamped>();
        nav_goal->header.frame_id = "map";
        nav_goal->pose.position = *msg;  // Copy x,y,z
        nav_goal->pose.orientation.w = 0.0;  // No orientation (bug: should be 1.0)
        nav_goal_publisher_->publish(move(nav_goal));
        RCLCPP_INFO(logger, "Sent navigation goal to Nav2");
    }
};
```
**Note**: `orientation.w = 0.0` is invalid quaternion. Should be `w=1.0` for no rotation.

---

## 6. `launch/launch_sim.launch.py` - Simulation Launch

### Key Components
```python
# 1. Robot State Publishers (URDF → TF)
rsp = IncludeLaunchDescription(rsp.launch.py, use_sim_time=true)
rsp_uav = IncludeLaunchDescription(rsp_uav.launch.py, use_sim_time=true)

# 2. Gazebo with custom world
gazebo = IncludeLaunchDescription(gazebo.launch.py, 
    world=roomWithObstacles.world, 
    extra_gazebo_args=gazebo_params.yaml)

# 3. Spawn entities with delays
spawn_robot = TimerAction(5.0, spawn_entity.py -topic robot_description -entity my_bot)
spawn_uav = TimerAction(15.0, spawn_entity.py -file uav.sdf -entity my_uav)

# 4. Joystick + twist_mux
joystick = IncludeLaunchDescription(joystick.launch.py)
twist_mux = Node(package="twist_mux", executable="twist_mux", 
                 parameters=[twist_mux.yaml], remappings=[/cmd_vel_out→/cmd_vel])

# 5. Our nodes
waypoints_server_node = Node(package=my_bot, executable=waypoints_server)
waypoints_client_node = Node(package=my_bot, executable=waypoints_client)
nav2_handler_node = Node(package=my_bot, executable=nav2_handler)  # COMMENTED OUT

# 6. Nav2 Stack
nav2_launch = IncludeLaunchDescription(navigation_launch.py, 
    use_sim_time=true, params_file=nav2_params.yaml)

# 7. Static TF: map → odom
static_tf = Node(package=tf2_ros, executable=static_transform_publisher,
    arguments=[0,0,0,0,0,0,map,odom])
```

### Launch Order (Critical)
```
1. Gazebo starts (0s)
2. Robot spawns (5s) → /robot_description → TF: map→odom→base_link
3. UAV spawns (15s) → camera publishes
4. waypoints_server starts → builds PRM
5. waypoints_client waits 10s → requests path
6. Nav2 starts with params
```

---

## 7. `config/nav2_params.yaml` - Nav2 Configuration

### Key Sections
```yaml
# Global Planner (Navfn - Dijkstra)
global_planner: nav2_navfn_planner/NavfnPlanner

# Local Controller (DWB - Dynamic Window Approach)
local_controller: nav2_dwb_controller/DWBController

# Global Costmap
global_costmap:
  resolution: 0.05
  robot_radius: 0.22
  inflation_radius: 0.55
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

# Local Costmap
local_costmap:
  resolution: 0.05
  robot_radius: 0.22
  inflation_radius: 0.55
  plugins: ["obstacle_layer", "inflation_layer"]
```

---

## 8. `tutorial_interfaces/srv/GetWaypoints.srv`

```idl
geometry_msgs/Point start
geometry_msgs/Point goal
---
bool valid
geometry_msgs/PoseStamped[] waypoints
```

**Request**: Start + Goal in world coordinates (meters, `map` frame)
**Response**: Valid flag + array of poses with orientation

---

## Summary: Key Parameters to Tune

| Parameter | File | Default | Effect |
|-----------|------|---------|--------|
| `lowerColor` | waypoints_server.cpp:53 | (100,100,100) | Floor color lower bound |
| `upperColor` | waypoints_server.cpp:54 | (180,180,180) | Floor color upper bound |
| `maxNodes` | waypoints_server.cpp:26 | 1000 | PRM graph size |
| `nodes per frame` | waypoints_server.cpp:35 | 100 | Build speed |
| `connection radius` | processImage.cpp:81,99 | width/8, width/12 | Graph connectivity |
| `goal_pose` | waypoints_client.cpp:23-25 | (-4.5, 3.0) | Navigation target |
| `arrival threshold` | waypoints_client.cpp:54 | 0.5m | Waypoint switching |
| `timer period` | waypoints_client.cpp:28 | 10s | First request delay |

---

## Common Modifications

### Add Stuck Detection (TODO)
```cpp
// In waypoints_client.cpp odom_callback:
// Track position history
// If position unchanged > 5s → request new path
```

### Adaptive Color Threshold
```cpp
// In waypoints_server.cpp process_image:
// Learn floor color from first N frames
// Use HSV instead of BGR
```

### Orientation in Waypoints (Already Implemented)
```cpp
// In waypoints_server.cpp metadata_service_callback:
// Yaw calculated from direction vector
double yaw = atan2(dy, dx);
pose_msg->pose.orientation.w = cos(yaw/2);
pose_msg->pose.orientation.z = sin(yaw/2);
```