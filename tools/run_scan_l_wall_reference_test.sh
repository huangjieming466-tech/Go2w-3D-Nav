#!/usr/bin/env bash
# Test SCAN Mode 3 against a manually verified, wall-safe reference path.
set -eo pipefail

ROOT=/home/hjj/Go2w_3D_Nav
LOG=/tmp/scan_l_wall_reference
rm -rf "$LOG"
mkdir -p "$LOG"

cleanup() {
  if [ -n "${SCAN_PID:-}" ]; then
    pkill -TERM -P "$SCAN_PID" 2>/dev/null || true
    kill "$SCAN_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=52
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" init_x:=3.5 init_y:=4.8 init_z:=0.30 map_size_x:=12.0 map_size_y:=12.0 map_size_z:=3.0 sim_sensing_rate:=10.0 sim_sensing_horizon:=5.0 sim_vertical_fov:=90.0 sim_min_raylength:=0.10 sim_lidar_pitch:=0.0 planning_horizon:=3.5 follow_reference_path:=true fsm_replan_threshold:=100.0 sim_freeze_after_sec:=6.0 sim_freeze_duration_sec:=45.0 use_sim_time:=false launch_rviz:=false >"$LOG/scan.log" 2>&1 &
SCAN_PID=$!
sleep 5
set +e
python3 "$ROOT/tools/scan_l_wall_probe.py" --publish-reference --corner-start --detect-cmd-wall --duration 58 >"$LOG/probe.log" 2>&1
PROBE_STATUS=$?
set -e
printf '%s\n' "$PROBE_STATUS" >"$LOG/status"
exit "$PROBE_STATUS"
