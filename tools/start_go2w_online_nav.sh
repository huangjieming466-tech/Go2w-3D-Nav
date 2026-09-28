#!/usr/bin/env bash
set -e

workspace="/home/hjj/Go2w_3D_Nav"
interface="${GO2W_INTERFACE:-eth0}"

source /opt/ros/humble/setup.bash
source /home/hjj/unitree_ros2/install/setup.bash
source "${workspace}/install/setup.bash"

export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

exec ros2 launch go2_3d_nav_viz go2w_online_navigation.launch.py "$@"
