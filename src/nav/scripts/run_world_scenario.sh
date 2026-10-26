#!/usr/bin/env bash
# One live A* run in a stress world + its video (see docs/work-docs/nav/step09_stress_worlds.md).
#   run_world_scenario.sh <world> <start_x> <start_y> <goal_x> <goal_y> <title> [map_variant=uavview] [out_dir=/tmp/<world>_run]
# Needs: ROS sourced, `colcon build --packages-select nav` done, `build_world.py <world>` run (writes <world>_run.world).
# Extra launch arguments: EXTRA="overhead_islands:=true enable_exploration:=true global_map_length:=50.0 ..." (environment variable).
# The recorder is started BEFORE the simulation so the first seconds of motion are not missed.
set -u
# goals are formatted as floats: the follower's goal_x/goal_y ROS parameters are doubles ("30" would be rejected)
W=$1; SX=$2; SY=$3; GX=$(printf "%.3f" "$4"); GY=$(printf "%.3f" "$5"); TITLE=$6; MAP=${7:-uavview}; OUT=${8:-/tmp/${W}_run}
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$REPO"
# stale processes from an earlier launch silently corrupt a new one (see tunnel_demo.launch.py)
pgrep -af "ros2 launch|ign gazebo|gz sim|parameter_bridge|static_transform_publisher|map_replay|planner_node|voxel_map_node|floor_map_node|waypoint_follower|nav2_|controller_server|bt_navigator|planner_server|behavior_server|smoother|lifecycle_manager|velocity_smoother" | grep -v pgrep | awk '{print $1}' | xargs -r kill -9
sleep 2
rm -rf "${OUT:?}"
python3 src/nav/scripts/record_world_run.py --out "$OUT" --goal "$GX" "$GY" --max-sim-sec "${MAX_SIM:-700}" > "$OUT.rec.log" 2>&1 &
REC=$!
sleep 2
ros2 launch nav tunnel_demo.launch.py world:=${W}_run.world nav2_params:=nav2_params_large.yaml headless:=true \
    autonomous_uav:=false planner_type:=astar map_snapshot:="$REPO/src/nav/maps/${W}_${MAP}.npz" \
    goal_x:=$GX goal_y:=$GY ${EXTRA:-} > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
wait $REC
pgrep -af "ros2 launch|ign gazebo|gz sim|parameter_bridge|static_transform_publisher|map_replay|planner_node|voxel_map_node|floor_map_node|waypoint_follower|nav2_|controller_server|bt_navigator|planner_server|behavior_server|smoother|lifecycle_manager|velocity_smoother" | grep -v pgrep | awk '{print $1}' | xargs -r kill -9
cat "$OUT.rec.log" | tail -2
python3 src/nav/scripts/make_run_video.py --run "$OUT" --world "$W" --start "$SX" "$SY" --goal "$GX" "$GY" \
    --map "$MAP" --fps "${FPS:-20}" --out "media/runs/${W}_run.mp4" --title "$TITLE"
