#!/usr/bin/env bash
# Run the controlled L-wall test in a DDS domain separate from real navigation.
set -eo pipefail

ROOT=/home/hjj/Go2w_3D_Nav
LOG=/tmp/scan_l_wall_mode3
START_X=${START_X:-2.0}
START_Y=${START_Y:-2.0}
FREEZE_AFTER=${FREEZE_AFTER:-6.0}
FREEZE_DURATION=${FREEZE_DURATION:-45.0}
rm -rf "$LOG"
mkdir -p "$LOG"

cleanup() {
  for pid in "${SCAN_PID:-}" "${DDDMR_PID:-}"; do
    if [ -n "$pid" ]; then
      pkill -TERM -P "$pid" 2>/dev/null || true
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=51
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
TEST_DDS_INTERFACE=${SCAN_TEST_DDS_INTERFACE:-eth5}
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${TEST_DDS_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

DDDMR_INFLATION=${SCAN_TEST_DDDMR_INFLATION:-0.15}
ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" frame_id:=world launch_rviz:=false use_odom_start:=false start_x:="$START_X" start_y:="$START_Y" start_z:=0.30 use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=30.0 max_roughness:=0.05 robot_radius:=0.35 robot_height:=0.50 footprint_radius:=0.22 footprint_offset:=0.13 footprint_edge_samples:=12 max_step:=0.25 inflation_radius:="$DDDMR_INFLATION" connection_radius:=0.60 transition_radius:=0.60 transition_neighbors:=16 max_neighbors:=32 >"$LOG/dddmr.log" 2>&1 &
DDDMR_PID=$!
for _ in $(seq 1 45); do
  grep -q 'DDMR map ready' "$LOG/dddmr.log" && break
  sleep 1
done
grep -q 'DDMR map ready' "$LOG/dddmr.log"

CONFIG_ARGS=()
if [ -n "${SCAN_TEST_PLANNER_CONFIG:-}" ]; then
  CONFIG_ARGS+=(planner_config:="$SCAN_TEST_PLANNER_CONFIG")
fi
if [ -n "${SCAN_TEST_CONTROLLER_CONFIG:-}" ]; then
  CONFIG_ARGS+=(controller_config:="$SCAN_TEST_CONTROLLER_CONFIG")
fi
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" init_x:="$START_X" init_y:="$START_Y" init_z:=0.30 map_size_x:=12.0 map_size_y:=12.0 map_size_z:=3.0 sim_sensing_rate:=10.0 sim_sensing_horizon:=5.0 sim_vertical_fov:=90.0 sim_min_raylength:=0.10 sim_lidar_pitch:=0.0 planning_horizon:=3.5 follow_reference_path:=true fsm_replan_threshold:=100.0 sim_freeze_after_sec:="$FREEZE_AFTER" sim_freeze_duration_sec:="$FREEZE_DURATION" "${CONFIG_ARGS[@]}" use_sim_time:=false launch_rviz:=false >"$LOG/scan.log" 2>&1 &
SCAN_PID=$!
sleep 5
set +e
python3 "$ROOT/tools/scan_l_wall_probe.py" --duration 65 --detect-cmd-wall >"$LOG/probe.log" 2>&1
PROBE_STATUS=$?
set -e
printf '%s\n' "$PROBE_STATUS" >"$LOG/status"
exit "$PROBE_STATUS"
