#!/usr/bin/env bash
# Isolated SCAN Mode 3 regression cases around the stair4 sharp corner.
set -euo pipefail

ROOT=/home/hjj/Go2w_3D_Nav
LOG_DIR=/tmp/scan_corner_cases
FULL_PCD="$ROOT/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd"
LOCAL_PCD="$ROOT/tools/test_maps/stair4_corner_crop.pcd"
START_X=5.284109
START_Y=10.908431
START_Z=3.775287

mkdir -p "$LOG_DIR"
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

stop_tree() {
  local parent="$1"
  pkill -TERM -P "$parent" 2>/dev/null || true
  kill "$parent" 2>/dev/null || true
}

run_case() {
  local name="$1" goal_x="$2" goal_y="$3" goal_z="$4" duration="$5"
  local dddmr_pid scan_pid
  rm -f "$LOG_DIR/$name".{dddmr,scan,probe}.log

  ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:="$FULL_PCD" frame_id:=world launch_rviz:=false use_odom_start:=false start_x:="$START_X" start_y:="$START_Y" start_z:="$START_Z" use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=38.0 max_roughness:=0.08 robot_radius:=0.22 robot_height:=0.50 footprint_radius:=0.22 footprint_offset:=0.13 footprint_edge_samples:=3 max_step:=0.30 inflation_radius:=0.05 connection_radius:=1.50 transition_radius:=1.50 transition_neighbors:=16 max_neighbors:=32 >"$LOG_DIR/$name.dddmr.log" 2>&1 &
  dddmr_pid=$!
  for _ in $(seq 1 30); do
    grep -q 'DDMR map ready' "$LOG_DIR/$name.dddmr.log" && break
    sleep 1
  done
  grep -q 'DDMR map ready' "$LOG_DIR/$name.dddmr.log"

  ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:="$LOCAL_PCD" init_x:="$START_X" init_y:="$START_Y" init_z:="$START_Z" sim_sensing_rate:=5.0 sim_sensing_horizon:=5.0 sim_vertical_fov:=90.0 sim_min_raylength:=0.25 planning_horizon:=3.5 follow_reference_path:=true use_sim_time:=false launch_rviz:=false >"$LOG_DIR/$name.scan.log" 2>&1 &
  scan_pid=$!
  sleep 5
  timeout "$((duration + 8))" python3 "$ROOT/tools/scan_wall_follow_probe.py" --goal-x "$goal_x" --goal-y "$goal_y" --goal-z "$goal_z" --frame-id world --duration "$duration" >"$LOG_DIR/$name.probe.log" 2>&1 || true
  stop_tree "$scan_pid"
  stop_tree "$dddmr_pid"
  sleep 2
}

run_case approach 5.640514 12.555037 3.778285 18
run_case exit_near 4.445992 13.758462 3.753714 28
run_case exit_far 3.853292 14.367986 3.741004 32
run_case reverse 4.364973 9.630803 3.762944 22

for log in "$LOG_DIR"/*.probe.log; do
  printf '\n== %s ==\n' "$(basename "$log" .probe.log)"
  cat "$log"
done
