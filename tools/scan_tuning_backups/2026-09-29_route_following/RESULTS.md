# SCAN Mode 3 Wall-Following Test

## Scope

- Test map: `/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd`
- DDDMR route: start `[-0.751, 2.456, 3.650]`, goal `[-2.5007, 20.7320, 3.6449]`
- Global route: 240 poses, 23.818 m. The direct start-goal distance is about 18.4 m, so this route exposes corner cutting through walls.
- Simulated lidar: 10 Hz, 59 degree vertical FOV, 0.1 m minimum range, 5.0 m SCAN sensing horizon, 13 degree pitch.

## Before And After

| Setting or behavior | Before | After |
| --- | --- | --- |
| `fsm.planning_horizon` | 7.5 m | 3.5 m |
| `manager.planning_horizon` | 7.5 m | 3.5 m |
| `fsm.follow_reference_path` | unavailable | `true` |
| Mode 3 initialization | Direct minimum-snap chord to one look-ahead point | Resampled local DDDMR route segment |
| Route fitness during rebound | inactive | active with existing `lambda_fitness=1.0` |
| Stair route Z profile | linearized start-to-target | preserve DDDMR intermediate Z values |

The following tuned stair and robot-envelope values were intentionally unchanged:

```yaml
grid_map.double_cylinder_radius: 0.22
grid_map.double_cylinder_offset: 0.03
grid_map.collision_query_z_offset: 0.30
grid_map.obstacles_inflation_z_up: 0.0
grid_map.obstacles_inflation_z_down: 0.15
optimization.lambda_collision: 3.0
optimization.lambda_fitness: 1.0
optimization.dist0: 0.35
```

## Results

Parameter-only tests with the old direct-chord logic:

| Horizon | Observed maximum 3D deviation from DDDMR path |
| --- | --- |
| 7.5 m | 3.17-3.99 m |
| 3.5 m | 2.24-2.87 m |
| 2.0 m | up to 2.09 m, with unstable 4-5 m local paths |

Route-following logic with a 3.5 m horizon:

- 67 consecutive local trajectories generated successfully.
- Worst XY deviation: 0.392 m, occurring during the main turn/rebound section.
- Mean of per-trajectory maximum XY deviation: 0.119 m.
- Straight-section maximum XY deviation: approximately 0.085 m.
- Worst 3D deviation: 0.593 m. About 0.44 m is the expected body-odom versus ground-path height offset.
- The route returned to about 0.05 m XY deviation after the turn.

This removes the large direct chord responsible for the observed wall crossing while leaving enough optimizer freedom to avoid locally sensed obstacles. It is still a deterministic PCD simulation, so the change must be validated on the real robot at low speed before normal operation.

## Commands

Build:

```bash
cd /home/hjj/Go2w_3D_Nav && colcon build --symlink-install --base-paths src/scan_planner_ros2/src/planner --packages-up-to scan_planner --event-handlers console_direct+
```

DDDMR test route:

```bash
ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd frame_id:=world launch_rviz:=false use_odom_start:=false start_x:=-0.751 start_y:=2.456 start_z:=3.65 use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=38.0 max_roughness:=0.08 robot_radius:=0.22 robot_height:=0.50 max_step:=0.30 inflation_radius:=0.05 connection_radius:=1.50 transition_radius:=1.50 transition_neighbors:=16 max_neighbors:=32
```

SCAN realistic PCD simulation:

```bash
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd init_x:=-0.751 init_y:=2.456 init_z:=4.121 map_size_x:=40.0 map_size_y:=40.0 map_size_z:=10.0 planning_horizon:=3.5 follow_reference_path:=true sim_sensing_horizon:=5.0 sim_sensing_rate:=10.0 sim_vertical_fov:=59.0 sim_min_raylength:=0.1 sim_lidar_pitch:=13.0 launch_rviz:=true use_sim_time:=false
```

Metric probe:

```bash
python3 /home/hjj/Go2w_3D_Nav/tools/scan_wall_follow_probe.py --duration 75
```

Old behavior can be selected without restoring files:

```bash
ros2 launch scan_planner run.launch.py is_real_world:=true lio_backend:=super_global navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_sim_time:=false planning_horizon:=7.5 follow_reference_path:=false cmd_vel_topic:=/cmd_vel
```

## Backups And Rollback

- Exact pre-change files: `before_wall_route_following.tar.gz`
- Exact tested post-change files: `after_wall_route_following.tar.gz`
- Earlier baseline bundle: `../2026-09-29_pre_wall_test/`

Restore the pre-change implementation and rebuild:

```bash
cd /home/hjj/Go2w_3D_Nav && tar -xzf tools/scan_tuning_backups/2026-09-29_route_following/before_wall_route_following.tar.gz -C /home/hjj/Go2w_3D_Nav && colcon build --symlink-install --base-paths src/scan_planner_ros2/src/planner --packages-up-to scan_planner
```

Restore the tested implementation:

```bash
cd /home/hjj/Go2w_3D_Nav && tar -xzf tools/scan_tuning_backups/2026-09-29_route_following/after_wall_route_following.tar.gz -C /home/hjj/Go2w_3D_Nav && colcon build --symlink-install --base-paths src/scan_planner_ros2/src/planner --packages-up-to scan_planner
```

Archive SHA-256:

```text
before: 334afcaeb43cd0b35b9bbee65d458bb597ffa8530c2fb6104bf7e550598beee8
after:  ebc7249555a7f5c66215894c701e063782fe43792dc58d5c87bc6f8a1b1af884
```
