"""
Gazebo Simulation with Real Depth Camera - REQUIRES NATIVE LINUX WITH GPU

This simulation uses Gazebo for physics and a real depth camera sensor.
It requires:
  - Native Linux (not WSL) with GPU drivers
  - Gazebo 11 + ROS 2 Humble
  - Proper GPU acceleration (NVIDIA drivers, CUDA)

WSL does NOT support Gazebo physics properly - gzserver crashes due to
shared memory / FastDDS issues.

For development/testing in WSL, use the synthetic demo instead:
  ros2 launch terralink_elevation elevation_mapping.launch.py launch_rviz:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'terralink_elevation'
    
    # UAV SDF file
    uav_sdf = os.path.join(get_package_share_directory(package_name), 'description', 'terralink_uav.sdf')
    
    # World file
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'elevation_test.world')
    
    # Config files
    config_file = os.path.join(get_package_share_directory(package_name), 'config', 'elevation_mapping.yaml')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn UAV with depth camera
    spawn_uav = TimerAction(period=10.0, actions=[Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        name='spawn_uav',
        arguments=['-file', uav_sdf, '-entity', 'terralink_uav', '-x', '0', '-y', '0', '-z', '5'],
        output='screen'
    )])
    
    # Elevation mapping node - run script directly from bin (installed by setup.py)
    elevation_mapping = TimerAction(period=15.0, actions=[Node(
        package='terralink_elevation',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        parameters=[config_file],
        output='screen'
    )])
    
    # Static TF for map frame (map -> odom)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Static TF for UAV base link (map -> uav_base_link at height 5m)
    static_tf_uav = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_uav',
        arguments=['0', '0', '5', '0', '0', '0', 'map', 'uav_base_link']
    )
    
    # Static TF for camera optical frame (uav_base_link -> depth_camera_optical)
    # Camera pose in SDF: 0 0 5 3.14159 1.57079 3.14159 relative to uav_base_link
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=['0', '0', '5', '3.14159', '1.57079', '3.14159', 'uav_base_link', 'depth_camera_optical']
    )
    
    # RViz
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'elevation_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_uav,
        elevation_mapping,
        static_tf,
        static_tf_uav,
        static_tf_camera,
        rviz,
    ])