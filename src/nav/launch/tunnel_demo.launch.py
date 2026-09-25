"""nav step 5 demo: the frontier-aware tunnel-navigation capability, watched
end to end with zero manual control - same "hands-off demo" philosophy as
nav_sim.launch.py's autonomous_uav mode, applied to worlds/tunnel_test.world
(see that file's own docstring for the scenario: a UGV whose only route to
its goal passes through a semi-cylindrical tunnel the UAV's overhead camera
can never see the inside of).

This is a SEPARATE launch file from nav_sim.launch.py on purpose, not a
branch inside it - per this project's own instruction, nav_sim.launch.py and
the default planner_node/waypoint_follower/walkability/prm_planner BEHAVIOR
must stay exactly as they were until this new capability is proven live
(see docs/work-docs/nav/step05_frontier_tunnel_navigation.md's "prototype
first" framing). Structurally, this file is a straight copy of
nav_sim.launch.py's own bridging/staggering pattern (same reasoning
throughout - see that file for the full rationale behind each action),
with four real differences:

  1. worlds/tunnel_test.world instead of room_maze.world, and matching
     goal_x/goal_y/ugv-spawn defaults (the tunnel world's own green/red
     markers - see that file's docstring).
  2. A second static TF (ugv_camera_static_tf) for nav_ugv's own new
     front-facing depth camera (models/nav_ugv/model.sdf's
     front_camera_link), and elevation_mapping_node is told about it via
     secondary_points_topic - closing the actual sensing gap (see
     emap/elevation_mapping_node.py's module docstring).
  3. planner_node's enable_frontier_mode/enable_anomaly_mode and
     waypoint_follower's enable_frontier_replan are all turned ON (default
     OFF - see their own declare_parameter comments) - this is the one
     launch file in the whole project meant to exercise those capabilities.
  4. Defaults skew toward "watch it run": headless defaults to false (the
     opposite of nav_sim.launch.py's default) and autonomous_uav defaults
     to true, since the entire point of this file is to see the tunnel
     capability work, not to run headless CI-style.

step06 Phase 4 adds a fifth difference: `voxel_map_node` (step06 Phase 1)
is now actually launched here, and waypoint_follower's
`enable_voxel_verification` is turned on - the plan produced by Phase 3's
anomaly-aware planning is no longer just trusted blindly; before the UGV
advances onto a tentative segment, it's checked against the same shared 3D
map's real headroom data (see nav.voxel_map/nav.waypoint_follower's own
docstrings for the full mechanism).

step06 Phase 5 adds a sixth: `enable_anomaly_replan` is turned on too - a
CONFIRMED-BLOCKED verdict (ground truth from the voxel map, not just the
2.5D map's suspicion) now both gets reported back to `planner_node` (so it
stops offering the same already-refuted segment as a tentative route) and
immediately triggers a fresh `get_plan` request, closing the loop Phase 4
deliberately left open. See waypoint_follower.py/planner_node.py/
nav.resolved_regions's own docstrings for the full mechanism.

IMPORTANT - a real bug found from a live user run (see step06's own doc,
"Real-user-run finding" section): if ANY process from a previous launch of
this file (or nav_sim.launch.py) is still alive - a leftover
`elevation_mapping_node`, `voxel_map_node`, `ign gazebo`, etc, from a run
that was Ctrl+C'd, closed via the GUI window, or crashed - it will keep
publishing/consuming on the SAME topic names as a fresh launch, silently
corrupting or freezing the new run's state (e.g. the shared elevation/voxel
maps never resolving, robots never moving) with NO error message pointing
at the real cause. This has independently bitten this project at least five
separate times across steps 3, 4, and step06's own build (see those docs).
ALWAYS run this before every launch, not just after a crash:

    pgrep -af "ros2 launch|ign gazebo|gz sim|parameter_bridge|static_transform_publisher|elevation_mapping_node|planner_node|voxel_map_node|waypoint_follower|autopilot|nav2_" \
        | grep -v pgrep | awk '{print $1}' | xargs -r kill -9

A clean re-launch after doing this reliably reaches the goal in well under
five real minutes (headless) - if it still doesn't, that's a genuine new
bug, not this one.
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

    world_file = os.path.join(nav_share, 'worlds', 'tunnel_test.world')
    nav_bridge_config = os.path.join(nav_share, 'config', 'bridge.yaml')
    nav2_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    emap_bridge_config = os.path.join(emap_share, 'config', 'bridge.yaml')
    mapping_config = os.path.join(emap_share, 'config', 'elevation_mapping.yaml')

    # Defaults flipped relative to nav_sim.launch.py - see this file's top
    # docstring, point 4.
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description="Run gz sim server-only (no GUI window). This demo defaults to false "
                    "(unlike nav_sim.launch.py) because its entire point is to watch the UGV "
                    "actually drive through the tunnel."
    )
    headless = LaunchConfiguration('headless')

    # Matches worlds/tunnel_test.world's goal_marker (red) exactly.
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='3.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')

    autonomous_uav_arg = DeclareLaunchArgument(
        'autonomous_uav', default_value='true',
        description='Whether the UAV flies its own fixed patrol (see uav_autopilot_node below) - '
                    'defaults to true here (unlike nav_sim.launch.py) since this demo is meant to '
                    'be watched end to end with no manual /cmd_vel at all.'
    )
    autonomous_uav = LaunchConfiguration('autonomous_uav')

    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [nav_share, '/models', os.pathsep,
         emap_share, '/models', os.pathsep,
         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )
    force_software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

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

    # --- UAV side (identical to nav_sim.launch.py's own actions) ---
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
    # The one real difference from nav_sim.launch.py's mapping_node: a
    # SECOND parameters entry overriding secondary_points_topic (see
    # elevation_mapping_node.py's module docstring/declare_parameter
    # comment). A later entry in a Node's `parameters` list overrides
    # matching keys from an earlier one (standard ROS 2 launch behavior) -
    # so emap's own config/elevation_mapping.yaml is read completely
    # unmodified, then this ONE key is layered on top, rather than needing
    # a second copy of that whole file just to change one line.
    # point_cloud_stride=4: real, measured GUI-mode resource contention
    # (see docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md's own
    # section on this) found the `ign gazebo gui` render client alone
    # costing ~284% CPU on top of this node's own CPU-only fusion (no GPU/
    # cupy in this sandbox) processing TWO full-resolution 320x240@10Hz
    # depth cameras - severe enough to stall Gazebo's physics outright, not
    # just slow it down. A stride of 4 keeps 1-in-4 points (applied to the
    # flattened point array, before any transform/fusion work - a straight
    # 4x volume reduction, not a 2D per-row/per-column one) - a real
    # density tradeoff, but at this project's 0.1m grid resolution over a
    # ~10m world, full 320x240 density was always far more than any single
    # cell needs. Live-measured effect: elevation_mapping_node's CPU dropped
    # from ~220% to ~40%, voxel_map_node's from ~200% to ~23%, overall
    # system load from a peak of 22+ (causing a full physics stall,
    # confirmed live) down to 9-12 (two clean GUI-mode runs afterward, both
    # completing a real tunnel crossing). See `elevation_mapping_node.py`'s
    # own `point_cloud_stride` declare_parameter comment for the full
    # reasoning; unset (1, no change) everywhere else in this project.
    mapping_node = Node(
        package='emap',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        parameters=[
            mapping_config,
            {'secondary_points_topic': '/ugv/camera/points', 'point_cloud_stride': 4},
        ],
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

    # MUST match worlds/tunnel_test.world's nav_ugv <include><pose> exactly
    # (-3.0, 0.0) - same single-source-of-truth convention room_maze.world/
    # nav_sim.launch.py already use.
    # REAL BUG FOUND LIVE (GUI-mode run, 2026-09-24): this -3.0 offset was
    # correct for the OLD DiffDrive-sourced nav_ugv/odom, which (like all
    # DiffDrive dead-reckoning) resets to (0,0,0) at the model's own spawn
    # pose - -3.0 converted "distance since nav_ugv's spawn" into absolute
    # world position. worlds/tunnel_test.world now sources nav_ugv/odom
    # from OdometryPublisher instead (ground-truth, not wheel-integrated -
    # see that plugin's own comment for why), and OdometryPublisher does
    # NOT reset to zero at spawn - it reports the model's true ABSOLUTE
    # world pose directly (confirmed live: at spawn, before any movement,
    # the composed TF read -6.0 - exactly -3.0 (this offset) + -3.0
    # (OdometryPublisher's own already-absolute reading) - while Ignition's
    # own physics ground truth read -3.0). Zero offset now, since
    # nav_ugv/odom already sits in the same absolute space as
    # iris_quad/odom (also OdometryPublisher-sourced - see
    # emap/worlds/bump_test.world) - no additional shift needed, matching
    # iris_quad's own already-correct convention of using OdometryPublisher
    # with no compensating static transform anywhere else in this project.
    ugv_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ugv_odom_static_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--frame-id', 'iris_quad/odom',
            '--child-frame-id', 'nav_ugv/odom',
        ],
    )
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
    # NEW for step05: nav_ugv's own front-facing depth camera (see
    # models/nav_ugv/model.sdf's front_camera_link). Values match that
    # link's own <pose> (0.18, 0, 0.05, roll=0, pitch=0.6, yaw=0) exactly -
    # same convention camera_static_tf/ugv_lidar_static_tf above already
    # use (a static TF mirroring the SDF pose it's describing).
    ugv_camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ugv_camera_static_tf',
        arguments=[
            '--x', '0.18', '--y', '0', '--z', '0.05',
            '--roll', '0', '--pitch', '0.6', '--yaw', '0',
            '--frame-id', 'nav_ugv/base_link',
            '--child-frame-id', 'nav_ugv/front_camera_link/front_camera',
        ],
    )

    # --- Navigation layer (SetRemap rationale identical to nav_sim.launch.py) ---
    # REAL BUG FOUND LIVE (fixed here): a single SetRemap keyed on `/cmd_vel`
    # only catches controller_server's own raw output - velocity_smoother's
    # real, final output lands on a DIFFERENT internal source name
    # (`cmd_vel_smoothed`, remapped by nav2_bringup's own installed launch
    # file straight to literal `cmd_vel`) that the original single SetRemap
    # never matched at all, letting it sail through to `/cmd_vel` and get
    # forwarded by `emap_bridge` straight into the UAV's own flight
    # controller (`/iris_quad/gazebo/command/twist`) - a very likely direct
    # contributor to "the UAV moves uselessly" behavior seen live. See
    # nav_sim.launch.py's matching comment for the full live evidence
    # (`ros2 node info`/`ros2 topic info` output) this was found with.
    nav2_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/nav2_cmd_vel'),
        SetRemap(src='cmd_vel_smoothed', dst='/nav2_cmd_vel_smoothed'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
    ])

    # enable_frontier_mode=true - the one node in the whole project meant to
    # actually plan through unconfirmed frontier cells (see
    # nav.prm_planner/nav.walkability and this file's own top docstring).
    planner_node = Node(
        package='nav',
        executable='planner_node',
        name='planner_node',
        # enable_anomaly_mode=true (step06 Phase 2) alongside frontier mode -
        # publishes /anomaly_map purely for inspection/verification; doesn't
        # change what get_plan actually returns yet (that's Phase 3).
        parameters=[{'enable_frontier_mode': True, 'enable_anomaly_mode': True}],
        output='screen',
    )
    # enable_frontier_replan=true - keep re-requesting a plan while the
    # current one is only tentative, so once the UGV's own front camera
    # resolves the tunnel's interior with real data, the route gets
    # confirmed/refined instead of trusting one static guess forever.
    # enable_voxel_verification=true (step06 Phase 4) - before advancing
    # onto an anomaly-tentative segment, actually check the shared 3D
    # voxel map's real headroom data rather than trusting the 2.5D plan's
    # guess unconditionally. enable_anomaly_replan=true (step06 Phase 5) -
    # a CONFIRMED-BLOCKED verdict reports back to planner_node and
    # immediately requests a fresh plan, rather than leaving the UGV
    # holding position forever (Phase 4's own deliberately-left-open gap).
    # See waypoint_follower.py's own docstring for both.
    #
    # enable_stuck_recovery=true - a real, live-found gap none of the above
    # covers: Nav2 itself can simply give up on a goal (DWB finds no valid
    # trajectory, "Controller patience exceeded", "Goal failed") with
    # NOTHING here noticing or asking for a fresh plan - see
    # waypoint_follower.py's own declare_parameter comment for the exact
    # live failure this was found from.
    prm_waypoint_follower = Node(
        package='nav',
        executable='waypoint_follower',
        name='prm_waypoint_follower',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'enable_frontier_replan': True,
            'enable_voxel_verification': True,
            'enable_anomaly_replan': True,
            'enable_stuck_recovery': True,
        }],
        output='screen',
    )
    # step06 Phase 1's bounded 3D map, now actually consumed (Phase 4) -
    # default topics/frame already match what emap_bridge/nav_bridge
    # publish in this world (same /camera/points, /ugv/camera/points,
    # nav_ugv/odom used elsewhere in this file). point_cloud_stride=4 -
    # same GUI-mode resource-contention reasoning as mapping_node's own
    # override above (this node independently inserts the SAME two
    # cameras' points into its own octree, a second real CPU cost on top
    # of elevation_mapping_node's).
    voxel_map_node = Node(
        package='nav',
        executable='voxel_map_node',
        name='voxel_map_node',
        parameters=[{'point_cloud_stride': 4}],
        output='screen',
    )
    # Patrol tailored to tunnel_test.world's own layout (see that file's
    # docstring): room A center/corners, room B center/corners, and a pass
    # directly over the tunnel itself - covers both rooms thoroughly while,
    # by the very nature of the scenario, NEVER actually seeing the
    # tunnel's interior floor no matter how many times it flies over it.
    #
    # REAL BUG FOUND LIVE (GUI-mode run, 2026-09-24, user report: "make
    # sure to look at shortening the mandatory UAV-patrol/scan phase...
    # i need this fast"): the ORIGINAL waypoint order below visited both of
    # room A's far corners BEFORE ever reaching the tunnel waypoint - full
    # room-A coverage is genuinely useful for anomaly-detection accuracy
    # elsewhere in the room, but it is NOT needed for the very FIRST viable
    # plan, which only needs the direct corridor from the UGV's start point
    # through the tunnel to the goal. With the class default
    # `dwell_time_sec=15.0` (a long hold at EVERY waypoint, for a stable
    # multi-frame elevation-map read there) and `max_xy_speed=1.0`/
    # `max_z_speed=0.8`, reaching just the 4th waypoint (the tunnel itself)
    # took ~90-100s live before any plan could even be attempted - most of
    # this project's earlier "why does this take 10 minutes" frustration
    # traced back to THIS wait, not anything the UGV side does.
    #
    # Fix, two parts:
    # 1. Reordered so the three waypoints that actually matter for a first
    #    plan - room A center (start), directly over the tunnel, room B
    #    center (goal) - are visited FIRST, before the corner waypoints
    #    that only improve coverage elsewhere. Nothing is removed - this
    #    patrol still loops forever (`loop=True`, the class default) and
    #    keeps refining full-room coverage afterward; only the ORDER
    #    changed. The camera fuses continuously throughout flight, not
    #    just while dwelling, so the direct start->tunnel->goal corridor
    #    also gets scanned incidentally while transiting between these
    #    three points, not only at the hover points themselves.
    # 2. dwell_time_sec cut from 15.0 to 3.0 (still ~30 camera frames at
    #    this sensor's 10Hz rate - plenty for a stable read, live-confirmed
    #    below to not regress anomaly-detection correctness) and
    #    max_xy_speed/max_z_speed raised from 1.0/0.8 to 2.5/2.0 (a stable
    #    multirotor under `MulticopterVelocityControl` - unlike the UGV's
    #    own wheeled-chassis tipping bug this session found, there is no
    #    equivalent physical instability risk here to raising cruise
    #    speed). Live-verified: time from launch to the first viable plan
    #    dropped from ~90-100s to well under 30s - see
    #    step06_hybrid_3d_voxel_navigation.md's "Patrol speed-up" section
    #    for the exact measured numbers.
    uav_autopilot_node = Node(
        package='nav',
        executable='uav_autopilot_node',
        name='uav_autopilot_node',
        parameters=[{
            'dwell_time_sec': 3.0,
            'max_xy_speed': 2.5,
            'max_z_speed': 2.0,
            'waypoints_flat': [
                -3.0, 0.0, 3.0,    # room A center (also above the UGV's start point) - CRITICAL, first
                0.0, 0.0, 3.0,     # directly over the tunnel (sees only its roof, never inside) - CRITICAL, second
                3.0, 0.0, 3.0,     # room B center (also above the goal) - CRITICAL, third
                -3.0, 2.5, 3.0,    # room A far corner - coverage fill-in, not on the critical path
                -3.0, -2.5, 3.0,   # room A near corner
                3.0, 2.5, 3.0,     # room B far corner
                3.0, -2.5, 3.0,    # room B near corner
            ],
        }],
        output='screen',
        condition=IfCondition(autonomous_uav),
    )

    # --- Staggering (identical rationale to nav_sim.launch.py) ---
    bridges_and_tf_group = TimerAction(period=5.0, actions=[
        emap_bridge, camera_static_tf, mapping_node, nav_bridge,
        ugv_odom_static_tf, ugv_lidar_static_tf, ugv_camera_static_tf,
        uav_autopilot_node, voxel_map_node,
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
