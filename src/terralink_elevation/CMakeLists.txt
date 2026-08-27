cmake_minimum_required(VERSION 3.8)
project(terralink_elevation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_python REQUIRED)

# Install Python package (handles entry points and scripts)
ament_python_install_package(${PROJECT_NAME})

# Install scripts to lib/${PROJECT_NAME} (matching reference - required for ros2 launch)
# Use install(CODE) to create directory and copy scripts at install time
install(CODE "
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/scripts/synthetic_pointcloud.py
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/synthetic_pointcloud.py
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/scripts/synthetic_pointcloud_tf_publisher.py
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/synthetic_pointcloud_tf_publisher.py
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/scripts/elevation_mapping_node.py
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/elevation_mapping_node.py
  )
")

# Install launch, config, rviz, worlds
install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()