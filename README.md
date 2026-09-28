# Go2W 3D Navigation

ROS 2 workspace for real-world, multi-floor navigation with a Unitree Go2W and Livox MID-360. The current tested architecture uses Super-LIO for continuous local odometry, Open3D registration for global localization, DDDMR for global 3D path planning, and SCAN-Planner for local trajectory generation and velocity control.

## Current Runtime Chain

```text
Go2W IMU + MID-360
        |
        v
Super-LIO (world frame)
  /lio/odom + /lio/cloud_world
        |                     offline oriented-normal PCD (map frame)
        |                                      |
        v                                      v
Open3D localization ----------------------> DDDMR
  T_map_world, /baselink2map_kalman          /initial_path (map)
        |                                      |
        +---------------- map_path_to_world_bridge
                                               |
                                               v
                                  /initial_path_world (world)
                                               |
                                               v
                       SCAN-Planner mode 3 + local obstacle map
                                               |
                                               v
                                      /cmd_vel -> Go2 bridge
```

The frame split is intentional:

- `world` is Super-LIO's continuous local odometry frame. SCAN consumes local odometry, live point cloud and the transformed reference path in this frame.
- `map` is the offline PCD/DDMR global frame. Open3D estimates `T_map_world` from the live Super-LIO cloud against the offline map.
- `/baselink2map_kalman` is the globally localized robot pose used as DDDMR's path start.
- `/lio/base_odom` is derived from Super-LIO's IMU pose using `base_link -> imu_link = [-0.02557, 0, 0.04232]`. It is SCAN's body pose.

Do not feed map-corrected odometry and a world-frame cloud into SCAN as a mixed pair. Its local planning inputs must share the `world` frame.

## Repository Layout

- `src/Super-LIO`: MID-360/Go2W Super-LIO configuration and launch files.
- `src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc`: Open3D global localization used with Super-LIO.
- `src/dddmr_python_3d_global_planner`: DDDMR planning implementation.
- `src/dddmr_python_ros`: ROS 2 wrapper, static PCD publisher and path interfaces.
- `src/scan_planner_ros2`: ROS 2 SCAN-Planner with real-robot mode 3 integration.
- `src/go2_3d_nav_viz`: frame/topic adapters and unified RViz configuration.
- `tools`: map preprocessing, oriented-normal generation and startup helpers.
- `src/elevator_lio`, `src/jie_3d_nav`, `src/pct`: retained experimental alternatives; they are not part of the current runtime chain.

PCD maps, rosbag files, generated tomograms, build output and logs are excluded from Git because they are machine-specific and too large for a normal GitHub repository.

Imported upstream baselines:

| Component | Upstream | Baseline |
| --- | --- | --- |
| Super-LIO | <https://github.com/Liansheng-Wang/Super-LIO> | `f89f48d` (`ros2`) |
| FAST-LIO Localization Humanoid | <https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID> | `df4772e` (`humble`) |
| DDDMR Python | <https://github.com/xiaoqi371317/dddmr_python_3d_global_planner> | `01ee5da` (`main`) |
| Elevator-LIO | <https://github.com/xiaofan4122/Elevator-LIO> | `1d79af7` (`main`) |
| jie_3d_nav | <https://github.com/6-robot/jie_3d_nav> | `995a3a5` (`main`) |
| SCAN-Planner ROS 2 | <https://github.com/wuyi2121/SCAN-Planner> | `d0b921c` (`ros2-community`) |

Each imported component retains its upstream license and notices. This repository adds the Go2W integration, configuration, ROS interfaces and local modifications on top of those baselines.

## Build

Host (Ubuntu 22.04, ROS 2 Humble):

```bash
cd /home/hjj/Go2w_3D_Nav && source /opt/ros/humble/setup.bash && source /home/hjj/unitree_ros2/install/setup.bash && colcon build --symlink-install --packages-up-to go2_3d_nav_viz dddmr_python_ros scan_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Endpoint (Unitree ARM64, ROS 2 Foxy):

```bash
cd /home/unitree/Go2w_3D_Nav && source /opt/ros/foxy/setup.bash && source /home/unitree/cyclonedds_ws/install/setup.bash && source /home/unitree/capo_ws/install/setup.bash && colcon build --symlink-install --packages-select path_searching traj_utils scan_planner dddmr_python_ros go2_3d_nav_viz --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After rebuilding, restart the corresponding ROS nodes so they load the new binaries.

## Required Map

The commands below use this map on both computers:

```text
src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd
```

The file is deliberately not committed. Copy it to the same relative path on the host and endpoint, or replace `map_file`/`pcd_file` in the commands. DDDMR should use a PCD containing consistently oriented normals so upward floor surfaces can be separated from downward ceiling surfaces.

## Endpoint Startup

The endpoint currently uses `eth2`. Run each command in a separate terminal. Every terminal must source the same environment.

Endpoint environment:

```bash
source /opt/ros/foxy/setup.bash && source /home/unitree/cyclonedds_ws/install/setup.bash && source /home/unitree/capo_ws/install/setup.bash && source /home/unitree/Go2w_3D_Nav/install/setup.bash && export ROS_LOCALHOST_ONLY=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
```

1. Start the Unitree IMU adapter:

```bash
ros2 run lio go2_lowstate_imu_bridge
```

2. Start the MID-360 driver. This project was tested with the driver's `rviz_MID360s_launch.py` data path; close its RViz panel if visualization is being done on the host:

```bash
ros2 launch livox_ros_driver2 rviz_MID360s_launch.py
```

3. Start Super-LIO without saving a map:

```bash
ros2 launch super_lio go2w_mid360.py rviz:=false save_map:=false
```

4. Start Open3D localization using Super-LIO as its odometry/cloud source:

```bash
ros2 launch open3d_loc localization_go2w.launch.py map_file:=/home/unitree/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd launch_fast_lio:=false odom_topic:=/lio/odom cloud_topic:=/lio/cloud_world launch_rviz:=false
```

5. Start the Go2 velocity bridge. Its runtime libraries come from the Unitree and Go2 workspaces:

```bash
source /opt/ros/foxy/setup.bash && source /home/unitree/cyclonedds_ws/install/setup.bash && source /home/unitree/capo_ws/install/setup.bash && source /home/unitree/go2-nav2-amcl/install/setup.bash && source /home/unitree/Go2w_3D_Nav/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml && ros2 run go2_twist_bridge twist_bridge --ros-args -p cmd_vel_topic:=/cmd_vel
```

To build a new map instead, launch Super-LIO with timestamped PCD saving:

```bash
ros2 launch super_lio go2w_mid360.py rviz:=true save_map:=true
```

The launch file creates a timestamped directory and filename, so existing maps are not overwritten on a clean shutdown.

## Host Startup

The host currently uses `eth0`. Run this environment command in every host terminal:

```bash
cd /home/hjj/Go2w_3D_Nav && source /opt/ros/humble/setup.bash && source /home/hjj/unitree_ros2/install/setup.bash && source install/setup.bash && export ROS_LOCALHOST_ONLY=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

1. Start DDDMR in the global `map` frame:

```bash
ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd frame_id:=map launch_rviz:=false use_odom_start:=true odom_topic:=/baselink2map_kalman use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=38.0 max_roughness:=0.08 robot_radius:=0.22 robot_height:=0.50 max_step:=0.30 inflation_radius:=0.05 connection_radius:=1.50 transition_radius:=1.50 transition_neighbors:=16 max_neighbors:=32
```

2. Start SCAN mode 3. This launch also starts the map-to-world path adapter, IMU-to-base odometry adapter, closed-loop controller and unified RViz:

```bash
ros2 launch scan_planner run.launch.py is_real_world:=true lio_backend:=super_global navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_sim_time:=false cmd_vel_topic:=/cmd_vel launch_rviz:=true rviz_fixed_frame:=map
```

Alternatively, start the complete host planning chain with one command:

```bash
GO2W_INTERFACE=eth0 /home/hjj/Go2w_3D_Nav/tools/start_go2w_online_nav.sh pcd_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd
```

## Localization And Goal Workflow

1. In RViz, use **2D Pose Estimate** once to provide an approximate map pose. Open3D then refines and continuously publishes the map-to-world alignment.
2. Verify `/baselink2map_kalman` follows the robot and the live cloud overlaps the offline map.
3. Set the DDDMR destination in RViz. DDDMR publishes `/initial_path` in `map`.
4. The adapter publishes `/initial_path_world`; SCAN aligns its Z origin to the current `/lio/base_odom`, plans locally and publishes `/cmd_vel`.
5. A new reachable DDDMR path immediately replaces the active SCAN target, even while the previous target is still executing. SCAN resets its failure/emergency state and enters `GEN_NEW_TRAJ`.

Expected messages after a new route include:

```text
Reference path Z aligned to odometry: offset=...
FSM from ... to GEN_NEW_TRAJ: NEW_PATH
```

If DDDMR cannot produce a path to the new point, it does not publish a replacement path and SCAN continues the existing route.

## Runtime Checks

Check the essential endpoint data from the host:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /go2/imu
ros2 topic hz /lio/odom
ros2 topic hz /lio/cloud_world
ros2 topic hz /baselink2map_kalman
```

Check the planner/control chain:

```bash
ros2 topic echo /initial_path --once
ros2 topic echo /initial_path_world --once
ros2 topic hz /cmd_vel
```

The SCAN dynamic-feasibility diagnostic refers to the planned B-spline acceleration in `m/s^2`, not measured IMU acceleration. The planner automatically stretches trajectory time and retries before reporting failure.

## Safety

- Test with the robot lifted or with wheels disabled before enabling closed-loop motion.
- Keep an emergency stop available and start with low velocity limits.
- Do not run Fast-LIO and Super-LIO simultaneously on the same input topics.
- Confirm localization visually before sending a global target.
- MID-360 blind zones cannot be recovered by the planner; sensor placement or additional sensors are required for reliable close-range coverage.
