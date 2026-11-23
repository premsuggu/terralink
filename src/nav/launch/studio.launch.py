"""TerraLink Studio in one command: the simulator + the navigation stack + the studio server (open http://localhost:8765).

    ros2 launch nav studio.launch.py                       # fly the UAV yourself and build the map live
    ros2 launch nav studio.launch.py map:=uavview          # skip the scan: use the saved "what the UAV sees" map and go straight to planning
    ros2 launch nav studio.launch.py map:=groundtruth      # same, with the oracle (ground-truth) map
    ros2 launch nav studio.launch.py map:=/path/to/map.npz # any snapshot written by scripts/save_map_snapshot.py

`map:=live` (default) runs the real elevation-mapping node: the map is whatever your flying has scanned so far.
Any other value replaces it with `map_replay_node` publishing a SAVED map, so the planner, Nav2 and the UGV run exactly as in a live
mission but start at the point where planning begins (no 95 s scan). The page shows which map source is active.

Everything else is `tunnel_demo.launch.py` (see its docstring) with the Studio settings: the UAV is NOT flown by an autopilot, the UGV
follower waits idle until you give it a goal, and `studio_node` is started alongside. Arguments:
    site        world name (needs worlds/<site>_studio.world, made by scripts/make_studio_world.py; default w2_container_yard)
    map         live | uavview | groundtruth | <path to .npz>
    headless    true (default) | false (also open the Gazebo window - slow without a GPU)
    port        studio web port (default 8765)
    host        address the web server listens on (default 127.0.0.1 = this machine only; WSL2 forwards it to a Windows browser at
                http://localhost:8765). Use 0.0.0.0 ONLY if that forwarding does not work for you: it exposes the server (which can fly
                the UAV) to everything that can reach this machine, and a browser may refuse the controller on a non-localhost address.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def resolve_map(site: str, map_arg: str, maps_dir: str) -> str:
    """The snapshot file for the `map` argument, or '' for the live map. Raises ValueError for a name that does not exist."""
    if map_arg in ("", "live"):
        return ""
    path = map_arg if map_arg.endswith(".npz") else os.path.join(maps_dir, f"{site}_{map_arg}.npz")
    if not os.path.isfile(path):
        raise ValueError(f"saved map not found: {path} (use map:=live, uavview, groundtruth, or a path to an .npz)")
    return path


def _launch(context, *_):
    nav_share = get_package_share_directory("nav")
    site = context.launch_configurations["site"]
    snapshot = resolve_map(site, context.launch_configurations["map"], os.path.join(nav_share, "maps"))
    world = f"{site}_studio.world"
    if not os.path.isfile(os.path.join(nav_share, "worlds", world)):
        raise ValueError(f"{world} not found - run scripts/make_studio_world.py {site} first (and rebuild)")
    source = os.path.splitext(os.path.basename(snapshot))[0] if snapshot else "live"
    args = {
        "world": world,
        "nav2_params": "nav2_params_large.yaml",          # 130 m global costmap (the stress worlds are up to 120 m across)
        "headless": context.launch_configurations["headless"],
        "autonomous_uav": "false",                        # the person flies the UAV
        "studio": "true",                                 # follower idles until a goal arrives
        "map_snapshot": snapshot,
        # live-scan settings proven on W2 (step14): a lighter point stream, tolerant roughness, no drift compensation (ground-truth pose)
        "global_map_length": "56.0", "emap_point_stride": "16", "max_roughness": "0.5", "drift_compensation": "false",
        "stuck_feedback": "true",
        # a hand-flown UAV scan paints a blocked ring ~0.4 m round the parked UGV: with the default 0.3 m "own body is free" circle the UGV
        # is boxed in and every plan fails ("no route"). Measured offline on a live scan: 0.3 -> no path, >= 0.4 -> routes (step16 doc).
        "footprint_radius_m": "0.5",
    }
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "tunnel_demo.launch.py")),
                                 launch_arguments=args.items()),
        Node(package="nav", executable="studio_node", name="studio_node", output="screen",
             parameters=[{"site": site, "port": int(context.launch_configurations["port"]), "map_source": source,
                          "host": context.launch_configurations["host"],
                           "world_file": os.path.join(nav_share, "worlds", world), "models_dir": os.path.join(nav_share, "models")}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="w2_container_yard"),
        DeclareLaunchArgument("map", default_value="live", description="live | uavview | groundtruth | path to an .npz snapshot"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="web server address; 0.0.0.0 exposes it to the network"),
        OpaqueFunction(function=_launch),
    ])
