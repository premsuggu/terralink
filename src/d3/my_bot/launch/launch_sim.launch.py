import os
import os
os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    rsp_uav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_uav.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'roomWithObstacles.world')
    gazebo_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': world_file_path,
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path
                    }.items()
            )

    from launch.actions import TimerAction
    spawn_robot = TimerAction(period=5.0, actions=[Node(package='gazebo_ros', executable='spawn_entity.py', 
                        name='spawn_robot',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')])
    
    uav_sdf = os.path.join(get_package_share_directory(package_name), 'description', 'uav.sdf')
    spawn_uav = TimerAction(period=15.0, actions=[Node(package='gazebo_ros', executable='spawn_entity.py', 
                        name='spawn_uav',
                        arguments=['-file', uav_sdf, '-entity', 'my_uav'],
                        output='screen')])

    

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    waypoints_server_node = Node(
        package='my_bot',  # Replace with your package name
        executable='waypoints_server',  # Name of your waypoints_server executable
        output='screen',
    )

    # Add waypoints_client node
    waypoints_client_node = Node(
        package='my_bot',  # Replace with your package name
        executable='waypoints_client',  # Name of your waypoints_client executable
        output='screen',
    )

    nav2_handler_node = Node(
        package='my_bot',  # Replace with your package name
        executable='nav2_handler',  # Name of your waypoints_server executable
        output='screen',
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
        }.items()
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        # rsp_uav,
        gazebo,
        spawn_robot,
        spawn_uav,
        joystick,
        twist_mux,
        waypoints_server_node,
        waypoints_client_node,
        nav2_launch,
        static_tf,
        # nav2_handler_node,
        # waypoint_publisher,
    ])
