#!/usr/bin/env bash
# A full mission in a stress world with the REAL pipeline (no ground-truth map is handed to the planner):
#   1. the UAV flies its scan patrol; emap builds the 2.5D map from its depth camera
#   2. when the patrol has covered the site, the map is saved (<world>_uavscan.npz) and the UGV mission starts:
#      A* + frontier/anomaly/exploration logic; the UGV's own camera and the 3D voxel map discover what the UAV cannot see
#   3. the whole run is recorded; the video is composed afterwards
#   run_world_mission.sh <world> <sx> <sy> <gx> <gy> <end_x> <end_y> "<title>" [global_map_length=56] [out_dir]
# <end_x,end_y> = the patrol's last waypoint (scan is finished when the UAV is within 2.5 m of it).
# Needs: ROS sourced, nav built, build_world.py <world> run (writes <world>_scan.world and <world>_patrol.txt).
set -u
W=$1; SX=$2; SY=$3; GX=$(printf "%.3f" "$4"); GY=$(printf "%.3f" "$5"); EX=$6; EY=$7; TITLE=$8; GML=${9:-56.0}; OUT=${10:-/tmp/${W}_mission}
REPO=$(cd "$(dirname "$0")/../../.." && pwd); cd "$REPO"
PAT='ros2 launch|ign gazebo|gz sim|parameter_bridge|static_transform|map_replay|planner_node|voxel_map_node|elevation_mapping|uav_autopilot|waypoint_follower|nav2_|controller_server|bt_navigator|planner_server|behavior_server|smoother|lifecycle_manager|velocity_smoother|record_world_run'
pgrep -af "$PAT" | grep -v pgrep | awk '{print $1}' | xargs -r kill -9; sleep 2
rm -rf "${OUT:?}"; mkdir -p "$OUT"
ros2 launch nav tunnel_demo.launch.py world:=${W}_scan.world nav2_params:=nav2_params_large.yaml headless:=true autonomous_uav:=true \
    patrol_file:=${W}_patrol.txt global_map_length:=$GML uav_dwell_sec:=0.3 emap_point_stride:=${EMAP_STRIDE:-16} max_roughness:=${MAX_ROUGH:-0.5} drift_compensation:=false planner_type:=astar \
    enable_exploration:=true overhead_islands:=true anomaly_hold_sec:=30.0 stuck_feedback:=true start_follower:=false goal_x:=$GX goal_y:=$GY > "$OUT/launch.log" 2>&1 &
# 1. wait for the UAV to reach the end of its patrol
until p=$(timeout 8 ros2 topic echo --once /odom --field pose.pose.position 2>/dev/null | tr '\n' ' '); \
      python3 -c "
import re,sys
m=re.search(r'x: ([-\d.e]+) y: ([-\d.e]+)', '''$p''')
sys.exit(0 if m and ((float(m.group(1))-$EX)**2+(float(m.group(2))-$EY)**2)**.5<2.5 else 1)"; do sleep 10; done
echo "scan finished: UAV at $p"
# 2. save the map the UAV built, then start recording and the UGV mission
python3 src/nav/scripts/save_map_snapshot.py --out "$OUT/${W}_uavscan.npz" > "$OUT/snapshot.log" 2>&1
cp "$OUT/${W}_uavscan.npz" "src/nav/maps/${W}_uavscan.npz"      # a copy is kept in the run folder: the maps/ one is overwritten by the next scan
python3 src/nav/scripts/record_world_run.py --out "$OUT/rec" --goal "$GX" "$GY" --max-sim-sec "${MAX_SIM:-900}" > "$OUT/rec.log" 2>&1 &
REC=$!
sleep 3
ros2 run nav waypoint_follower --ros-args -r __node:=prm_waypoint_follower -p goal_x:=$GX -p goal_y:=$GY -p use_sim_time:=false \
    -p enable_frontier_replan:=true -p enable_voxel_verification:=true -p enable_anomaly_replan:=true -p enable_stuck_recovery:=true > "$OUT/follower.log" 2>&1 &
wait $REC
pgrep -af "$PAT" | grep -v pgrep | awk '{print $1}' | xargs -r kill -9
tail -2 "$OUT/rec.log"
python3 src/nav/scripts/make_run_video.py --run "$OUT/rec" --world "$W" --start "$SX" "$SY" --goal "$GX" "$GY" \
    --map uavscan --out "media/runs/${W}_mission.mp4" --title "$TITLE"
