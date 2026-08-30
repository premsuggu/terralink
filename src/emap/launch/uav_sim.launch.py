"""emap step 1: spawn the iris_quad UAV in Ignition Gazebo Fortress and bridge
its control/odometry topics to ROS 2 via ros_gz_bridge.

Headless by default (AGENTS.md: run simulations headlessly unless GUI is
needed) - pass launch arg headless:=false to see the Gazebo GUI (WSLg-capable
environments can render it).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('emap')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    mapping_config = os.path.join(pkg_share, 'config', 'elevation_mapping.yaml')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run gz sim server-only (no GUI window)'
    )
    headless = LaunchConfiguration('headless')

    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping', default_value='true',
        description='Start the step-6 elevation_mapping_node alongside the simulation'
    )
    enable_mapping = LaunchConfiguration('enable_mapping')

    # step 7: 'flat' (default, every prior step's exact ground-truth world)
    # or 'bump' (a real Gaussian-bump terrain - see worlds/bump_test.world
    # and scripts/generate_bump_heightmap.py) - resolved to a filename with
    # PythonExpression since, unlike headless above, there's no simple
    # true/false condition to pick between two whole IncludeLaunchDescriptions
    # for every world we might add in the future.
    world_arg = DeclareLaunchArgument(
        'world', default_value='flat',
        description="Which world to load: 'flat' (uav_test.world) or 'bump' (bump_test.world)"
    )
    world = LaunchConfiguration('world')
    world_filename = PythonExpression(["'bump_test.world' if '", world, "' == 'bump' else 'uav_test.world'"])
    world_file = PathJoinSubstitution([worlds_dir, world_filename])

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Start rviz2 with this package\'s elevation_mapping.rviz config'
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    # model://iris_quad (and, since step 7, model://heightmaps/...) must
    # resolve under either <pkg_share>/models or <pkg_share>/worlds.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [models_dir, os.pathsep, worlds_dir, os.pathsep,
         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )

    # This machine's GPU driver is WSL2's D3D12-translated Mesa OpenGL, which is
    # missing a texture-copy path Ignition's Ogre2 renderer needs to render a
    # camera sensor's image - gz sim aborts (Ogre::UnimplementedException in
    # GL3PlusTextureGpu::copyTo) as soon as the rgbd_camera below starts
    # rendering. Forcing Mesa's llvmpipe software rasterizer avoids that gap
    # (slower, but correct) and isn't needed on a native Linux GPU driver.
    force_software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # camera_link is rigidly bolted to base_link (a `fixed` joint in model.sdf),
    # so its pose relative to base_link never changes - publish it once as a
    # static transform instead of asking Gazebo for it every frame. Values match
    # camera_link's <pose> in model.sdf exactly. The child frame name matches the
    # rgbd_camera sensor's actual PointCloud2 frame_id (iris_quad/camera_link/rgbd_camera).
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '-0.08',
            '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
            '--frame-id', 'iris_quad/base_link',
            '--child-frame-id', 'iris_quad/camera_link/rgbd_camera',
        ],
    )

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -s ', world_file]}.items(),
        condition=IfCondition(headless),
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
        condition=UnlessCondition(headless),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='emap_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # step 6: the live mapping node - name matches elevation_mapping.yaml's
    # top-level key so its ros__parameters actually get applied.
    mapping_node = Node(
        package='emap',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        parameters=[mapping_config],
        output='screen',
        condition=IfCondition(enable_mapping),
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'elevation_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        headless_arg,
        enable_mapping_arg,
        world_arg,
        launch_rviz_arg,
        resource_path,
        force_software_gl,
        gz_sim_headless,
        gz_sim_gui,
        bridge,
        camera_static_tf,
        mapping_node,
        rviz,
    ])
