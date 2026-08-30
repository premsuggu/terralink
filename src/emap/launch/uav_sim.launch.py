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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('emap')
    world_file = os.path.join(pkg_share, 'worlds', 'uav_test.world')
    models_dir = os.path.join(pkg_share, 'models')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run gz sim server-only (no GUI window)'
    )
    headless = LaunchConfiguration('headless')

    # model://iris_quad must resolve to <pkg_share>/models/iris_quad
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [models_dir, os.pathsep, EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
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
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
        condition=IfCondition(headless),
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=UnlessCondition(headless),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='emap_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        resource_path,
        force_software_gl,
        gz_sim_headless,
        gz_sim_gui,
        bridge,
        camera_static_tf,
    ])
