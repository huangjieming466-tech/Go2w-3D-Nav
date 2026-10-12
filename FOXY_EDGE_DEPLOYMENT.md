# ROS 2 Foxy Edge Navigation

The robot runs localization and the navigation planners. The workstation may
continue to send goals and visualize ROS topics, but it does not run SCAN or
DDDMR planner nodes.

## Robot Environment

The target is Ubuntu 20.04, ROS 2 Foxy, Python 3.8, and CycloneDDS. The DDDMR
planner supports the system NumPy 1.17 and SciPy 1.3; do not upgrade those
packages on the robot just for this planner.

## Build

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
source ~/livox_ws/install/setup.bash
cd ~/Go2w_3D_Nav
colcon build --symlink-install --parallel-workers 3 --base-paths src \
  --packages-up-to scan_planner dddmr_python_ros go2_3d_nav_viz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Build from the Foxy branch because Foxy uses Python 3.8 and older rclcpp,
TF2, and SciPy interfaces than the Humble workstation.

## Run

Localization must already publish `/baselink2map_kalman` and SCAN's configured
sensor streams. The launch below starts both planning nodes on the robot. It
does not start Super-LIO or Open3D localization.

```bash
ros2 launch go2_3d_nav_viz go2w_edge_navigation.launch.py pcd_file:=/home/unitree/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd
```

The DDDMR node publishes `/global_path`; SCAN consumes that route in Mode 3 and
publishes the configured `/cmd_vel`. Do not issue a goal until localization,
map frame alignment, sensor topics, and the robot's command interface have
been checked. Keep the robot physically secured during first bring-up.

The default PCD is deployment data and is not part of this source branch. Copy
the selected map to the robot or override `pcd_file` with its edge path.
