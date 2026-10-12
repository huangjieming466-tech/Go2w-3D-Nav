#!/usr/bin/env bash
# Verify the execution safety layer independently from trajectory generation.
set -eo pipefail

ROOT=/home/hjj/Go2w_3D_Nav
LOG=/tmp/scan_cmd_safety_l_wall
rm -rf "$LOG"
mkdir -p "$LOG"

cleanup() {
  if [ -n "${SCAN_PID:-}" ]; then
    pkill -TERM -P "$SCAN_PID" 2>/dev/null || true
    kill "$SCAN_PID" 2>/dev/null || true
  fi
  if [ -n "${BAG_PID:-}" ]; then
    kill "$BAG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=53
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" init_x:=4.55 init_y:=4.00 init_z:=0.30 map_size_x:=12.0 map_size_y:=12.0 map_size_z:=3.0 sim_sensing_rate:=10.0 sim_sensing_horizon:=5.0 sim_vertical_fov:=90.0 sim_min_raylength:=0.10 sim_lidar_pitch:=0.0 use_sim_time:=false launch_rviz:=false >"$LOG/scan.log" 2>&1 &
SCAN_PID=$!
sleep 8

ros2 bag record -o "$LOG/bag" /planning/cmd_collision /cmd_vel_raw /quad_0/cmd_vel >"$LOG/bag.log" 2>&1 &
BAG_PID=$!
sleep 1
timeout 3 ros2 topic pub -r 20 /cmd_vel_raw geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >"$LOG/raw_pub.log" 2>&1 || true
sleep 1
kill "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
unset BAG_PID

python3 "$ROOT/tools/analyze_cmd_safety_bag.py" "$LOG/bag" >"$LOG/result.log"
cat "$LOG/result.log"
