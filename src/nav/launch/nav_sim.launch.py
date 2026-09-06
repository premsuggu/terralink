"""nav's own top-level simulation launch: the UAV (emap's iris_quad +
depth camera + elevation_mapping_node), the UGV (nav_ugv, driven by Nav2),
and the PRM planning layer connecting them, all in worlds/room_maze.world.

This does NOT `IncludeLaunchDescription` emap's own uav_sim.launch.py,
even though most of what it needs (gz_sim launch, the resource-path/
software-GL environment fixes, the camera static TF, elevation_mapping_node)
is identical to what that file already sets up - because that file's world
selection (`world:=flat|bump|construction`) is scoped to emap's OWN
`worlds/`/`models/` package-share directories, and `room_maze.world` lives
in THIS package's share directory instead (see worlds/room_maze.world's own
docstring for why the world itself lives here rather than in emap). Rather
than reach into emap's launch internals to redirect it at a path it wasn't
designed to look in, the handful of UAV-side actions are set up directly
here, each one a straight copy of the corresponding action in
emap/launch/uav_sim.launch.py - same reasoning, same values, just no
world-argument branching needed since this launch only ever loads one world.

STARTUP STAGGERING: a real issue found live (see cli.txt from an early test
run) - launching everything at once let Nav2's costmaps start probing for
`nav_ugv/base_link -> nav_ugv/odom` before Gazebo had even finished spawning
`nav_ugv` into the world, producing a burst of "Timed out waiting for
transform" warnings (benign - Nav2 retries and would recover on its own -
but a user hitting Ctrl+C during exactly that window catches several
lifecycle nodes mid-transition, which is what produced that log's
class_loader/SIGABRT teardown mess). Same idea as d3's own
`launch_sim.launch.py`, which staggers its UGV/UAV spawns with
`TimerAction(period=5.0/15.0, ...)` - here, Gazebo itself starts
immediately (it's the slowest thing to come up), the ROS-side bridges/
static TFs/mapping node get a few seconds' head start once Gazebo should be
running and spawning models, and Nav2/the planning layer wait a few
seconds more so TF is already flowing by the time anything tries to use it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    nav_share = get_package_share_directory('nav')
    emap_share = get_package_share_directory('emap')

    world_file = os.path.join(nav_share, 'worlds', 'room_maze.world')
    nav_bridge_config = os.path.join(nav_share, 'config', 'bridge.yaml')
    nav2_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    emap_bridge_config = os.path.join(emap_share, 'config', 'bridge.yaml')
    mapping_config = os.path.join(emap_share, 'config', 'elevation_mapping.yaml')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description="Run gz sim server-only (no GUI window). Pass headless:=false to actually "
                    "SEE the simulation - this is almost always what you want alongside "
                    "autonomous_uav:=true, since the whole point of that mode is to watch it run."
    )
    headless = LaunchConfiguration('headless')

    # (1.7, -0.5) - see nav/waypoint_follower.py's own declare_parameter
    # comment for why this specific default (not the room's more obvious-
    # looking (4.0, 0.0), which turned out to sit ~0.1m from a real wall).
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='1.7')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='-0.5')

    # Opt-in "demo mode": no one needs to send /cmd_vel by hand - the UAV
    # flies itself around a fixed patrol of hover points to build up the
    # elevation map on its own. See nav/autopilot.py's module docstring for
    # why this is a small patrol rather than a single static hover, and
    # nav/uav_autopilot_node.py for the default waypoint list.
    autonomous_uav_arg = DeclareLaunchArgument(
        'autonomous_uav', default_value='false',
        description='If true, the UAV flies a fixed patrol of hover waypoints on its own '
                    '(no /cmd_vel needed from you) to scan the room and build the elevation map.'
    )
    autonomous_uav = LaunchConfiguration('autonomous_uav')

    # model://nav_ugv (this package) and model://iris_quad (emap) both need
    # to resolve - same pattern as emap/launch/uav_sim.launch.py's own
    # resource_path action, just listing both packages' models/ dirs.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [nav_share, '/models', os.pathsep,
         emap_share, '/models', os.pathsep,
         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )

    # See emap/launch/uav_sim.launch.py's identical action for the full
    # explanation (WSL2's software-rendered GL path, needed for the depth
    # camera AND nav_ugv's own gpu_lidar sensor to render at all here).
    force_software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # Two separate IncludeLaunchDescriptions gated by headless/not - same
    # pattern emap/launch/uav_sim.launch.py uses (a single Substitution-based
    # gz_args string can't conditionally include/omit the "-s" flag).
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

    # --- UAV side: straight copies of emap/launch/uav_sim.launch.py's own
    # actions (see that file for the full rationale behind each one). ---
    emap_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='emap_bridge',
        parameters=[{'config_file': emap_bridge_config}],
        output='screen',
    )
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
    mapping_node = Node(
        package='emap',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        parameters=[mapping_config],
        output='screen',
    )

    # --- UGV side ---
    nav_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='nav_bridge',
        parameters=[{'config_file': nav_bridge_config}],
        output='screen',
    )

    # Ties nav_ugv's own dead-reckoning odom tree into the same frame emap's
    # map already uses (iris_quad/odom, confirmed live to be WORLD-ABSOLUTE
    # - see config/bridge.yaml and config/nav2_params.yaml's top comments
    # for the full live-verified explanation). The translation below MUST
    # match nav_ugv's <include><pose> in worlds/room_maze.world exactly -
    # both are the single source of truth for "where nav_ugv actually
    # spawns", so if one changes, the other must too.
    ugv_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ugv_odom_static_tf',
        arguments=[
            '--x', '-0.7', '--y', '-4.5', '--z', '0.0',
            '--frame-id', 'iris_quad/odom',
            '--child-frame-id', 'nav_ugv/odom',
        ],
    )

    # REAL BUG FOUND LIVE: without this, every `/scan` message (frame_id
    # `nav_ugv/base_link/lidar` - Ignition's auto-derived <model>/<link>/
    # <sensor> naming, confirmed by echoing the live topic) had nowhere to
    # resolve to relative to the robot, and Nav2's costmaps silently dropped
    # every single one ("Message Filter dropping message... timestamp
    # earlier than all the data in the transform cache" - the real message,
    # misleadingly, since the actual problem was a MISSING transform, not a
    # stale one). Exactly the same gap `camera_static_tf` above fixes for
    # the UAV's camera - just missed for the UGV's lidar the first time.
    # Values match the lidar <sensor>'s own <pose> in
    # models/nav_ugv/model.sdf exactly (no rotation - the sensor is mounted
    # flat).
    ugv_lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ugv_lidar_static_tf',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0.095',
            '--frame-id', 'nav_ugv/base_link',
            '--child-frame-id', 'nav_ugv/base_link/lidar',
        ],
    )

    # --- Navigation layer ---
    # REAL BUG FOUND LIVE: the INSTALLED nav2_bringup/navigation_launch.py
    # (checked directly - not assumed from d3's own, apparently older/
    # different, vendored copy) remaps its final velocity_smoother output
    # straight to the literal, unnamespaced topic `cmd_vel`
    # (`[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]`), not
    # `cmd_vel_smoothed` - and that hardcoded remap is baked into the
    # installed launch file with no `namespace` argument actually applied to
    # it (confirmed: no `PushRosNamespace`/`namespace=` anywhere in that
    # file - its own `namespace` launch arg only feeds `RewrittenYaml`'s
    # root_key, not real node namespacing). Left alone, that would collide
    # directly with `emap_bridge`'s own `/cmd_vel` (the UAV's command
    # topic) - the UGV's velocity commands would either silently vanish
    # into the UAV's controller or the two would fight over the same name.
    # `SetRemap` (unlike a Node's own `remappings=`) applies across an
    # entire `IncludeLaunchDescription`'s worth of nodes without needing to
    # fork/vendor navigation_launch.py just to change one remap - the
    # standard, documented way to redirect a topic inside a launch file you
    # don't own. Scoped to a GroupAction so it only affects nodes started
    # inside this specific include, not the rest of this launch file.
    nav2_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/nav2_cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
    ])

    planner_node = Node(
        package='nav',
        executable='planner_node',
        name='planner_node',
        output='screen',
    )
    prm_waypoint_follower = Node(
        package='nav',
        executable='waypoint_follower',
        name='prm_waypoint_follower',
        parameters=[{'goal_x': LaunchConfiguration('goal_x'), 'goal_y': LaunchConfiguration('goal_y')}],
        output='screen',
    )
    uav_autopilot_node = Node(
        package='nav',
        executable='uav_autopilot_node',
        name='uav_autopilot_node',
        output='screen',
        condition=IfCondition(autonomous_uav),
    )

    # --- Staggering (see this file's top docstring) ---
    # Gazebo itself (gz_sim_headless/gz_sim_gui above) starts at t=0 with no
    # delay - it's the slowest thing to come up, so it needs the longest
    # head start, not a delay of its own.
    #
    # uav_autopilot_node deliberately starts alongside the bridges (t=5s),
    # NOT with the rest of the Nav2/planning layer (t=9s) - it only needs
    # `/odom`/`/cmd_vel` (from emap_bridge), nothing Nav2-related. This
    # matters for a real reason found live: `MulticopterVelocityControl`
    # gives the UAV no thrust at all until something actually commands it
    # (see the README's "Controlling the UAV" section), so every second
    # this node isn't running yet is a second the UAV free-falls
    # uncontrolled from its spawn altitude - and a bad landing can leave it
    # significantly yawed (confirmed live: 120 degrees off level after one
    # such fall), which is exactly the scenario `world_to_body_xy` in
    # autopilot.py exists to recover correctly from. Starting this node as
    # early as the bridge allows shrinks that uncontrolled window as much
    # as possible, on top of (not instead of) that correctness fix.
    bridges_and_tf_group = TimerAction(period=5.0, actions=[
        emap_bridge, camera_static_tf, mapping_node, nav_bridge, ugv_odom_static_tf, ugv_lidar_static_tf,
        uav_autopilot_node,
    ])
    nav_layer_group = TimerAction(period=9.0, actions=[
        nav2_launch, planner_node, prm_waypoint_follower,
    ])

    return LaunchDescription([
        headless_arg,
        goal_x_arg,
        goal_y_arg,
        autonomous_uav_arg,
        resource_path,
        force_software_gl,
        gz_sim_headless,
        gz_sim_gui,
        bridges_and_tf_group,
        nav_layer_group,
    ])
