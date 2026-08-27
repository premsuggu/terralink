from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('terralink_elevation')
    my_bot_share = get_package_share_directory('my_bot')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='gaussian_bump',
        description='World to load: gaussian_bump or construction_site'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Launch RViz'
    )
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(pkg_share, 'config', 'elevation_mapping.yaml'),
        description='Parameter file'
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_share, 'worlds', 
                LaunchConfiguration('world')
            ] + '.world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn UAV (from my_bot)
    spawn_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                my_bot_share, 'launch', 'spawn_uav.launch.py'
            ])
        ]),
        launch_arguments={
            'x': '0', 'y': '0', 'z': '5',  # 5m altitude for gaussian_bump
        }.items()
    )
    
    # Elevation mapping node
    elevation_node = Node(
        package='terralink_elevation',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )
    
    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'elevation_mapping.rviz')],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([
        world_arg,
        launch_rviz_arg,
        param_file_arg,
        gazebo,
        spawn_uav,
        elevation_node,
        rviz,
    ])