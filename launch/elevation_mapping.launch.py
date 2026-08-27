import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('terralink_elevation')

    param_file = os.path.join(pkg_share, 'config', 'elevation_mapping.yaml')
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'elevation_mapping.rviz'])

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', description='Launch RViz2'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use /clock if true'
    )

    # Synthetic pointcloud + TF publisher (matches reference implementation exactly)
    synthetic_pub = Node(
        package='terralink_elevation',
        executable='synthetic_pointcloud_tf_publisher.py',
        name='synthetic_pointcloud_tf_publisher',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'base_link',
            'pointcloud_topic': '/camera/depth/points',
            'publish_rate_hz': 10.0,
            'max_range_m': 10.0,
            'front_only': False,
            'trajectory_speed_mps': 0.25,
            'trajectory_segment_s': 5.0,
            'enable_yaw': True,
            'yaw_rate_rps': 0.15,
        }],
    )

    # Elevation mapping node - run script directly from bin (installed by setup.py)
    elevation_node = Node(
        package='terralink_elevation',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[param_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        synthetic_pub,
        elevation_node,
        rviz,
    ])