cmake_minimum_required(VERSION 3.5)
project(my_bot)

set(CMAKE_CXX_STANDARD 17)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    include
    includes
    ${OpenCV_INCLUDE_DIRS} 
)

# Add executables
add_executable(waypoints_server src/waypoints_server.cpp includes/processImage.cpp)
add_executable(waypoints_client src/waypoints_client.cpp)
add_executable(nav2_handler src/nav2_handler.cpp)

# Link libraries and dependencies
ament_target_dependencies(waypoints_server
    rclcpp
    sensor_msgs
    tutorial_interfaces
    cv_bridge
    OpenCV
)

ament_target_dependencies(waypoints_client
    rclcpp
    nav_msgs
    tutorial_interfaces
)

ament_target_dependencies(nav2_handler
    rclcpp
    nav_msgs
)

target_link_libraries(waypoints_server
    ${OpenCV_LIBS}
)

# Install executables to lib/${PROJECT_NAME} for ROS 2 launch/run compliance
install(TARGETS
    waypoints_server
    waypoints_client
    nav2_handler
    DESTINATION lib/${PROJECT_NAME}
)

# Install resource directories to share/${PROJECT_NAME}
install(
    DIRECTORY config description launch models worlds includes
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
