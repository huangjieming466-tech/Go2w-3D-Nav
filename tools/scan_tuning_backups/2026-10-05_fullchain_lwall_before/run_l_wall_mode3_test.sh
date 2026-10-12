#!/usr/bin/env bash
# Run the controlled L-wall test in a DDS domain separate from real navigation.
set -eo pipefail

ROOT=/home/hjj/Go2w_3D_Nav
LOG=/tmp/scan_l_wall_mode3
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
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth5" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" frame_id:=world launch_rviz:=false use_odom_start:=false start_x:=2.0 start_y:=2.0 start_z:=0.30 use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=30.0 max_roughness:=0.05 robot_radius:=0.22 robot_height:=0.50 footprint_radius:=0.22 footprint_offset:=0.13 footprint_edge_samples:=3 max_step:=0.25 inflation_radius:=0.15 connection_radius:=1.50 transition_radius:=1.50 transition_neighbors:=16 max_neighbors:=32 >"$LOG/dddmr.log" 2>&1 &
DDDMR_PID=$!
for _ in $(seq 1 45); do
  grep -q 'DDMR map ready' "$LOG/dddmr.log" && break
  sleep 1
done
grep -q 'DDMR map ready' "$LOG/dddmr.log"

ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:="$ROOT/tools/test_maps/l_wall_mode3.pcd" init_x:=2.0 init_y:=2.0 init_z:=0.30 map_size_x:=12.0 map_size_y:=12.0 map_size_z:=3.0 sim_sensing_rate:=10.0 sim_sensing_horizon:=5.0 sim_vertical_fov:=90.0 sim_min_raylength:=0.10 sim_lidar_pitch:=0.0 planning_horizon:=3.5 follow_reference_path:=true use_sim_time:=false launch_rviz:=false >"$LOG/scan.log" 2>&1 &
SCAN_PID=$!
sleep 5
set +e
python3 "$ROOT/tools/scan_l_wall_probe.py" --duration 65 >"$LOG/probe.log" 2>&1
PROBE_STATUS=$?
set -e
printf '%s\n' "$PROBE_STATUS" >"$LOG/status"
exit "$PROBE_STATUS"
