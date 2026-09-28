# Fast-LIO 回放到 PCT 楼梯规划

本流程以 Fast-LIO 的输出包保留每帧 `cloud_registered_body` 和对应的
`Odometry`。它们可恢复每个点被传感器观测时的视角，用来给平面法向定向。
不要直接用最终拼接的 XYZ PCD 做法向符号判断，因为其不保留观测方向。

## 已生成的数据

| 用途 | 文件 |
| --- | --- |
| Fast-LIO 原始回放输入 | `/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/2026-09-10_16-57-43/fastlio_input/rosbag2/stair_capture` |
| Fast-LIO 结果包 | `/home/hjj/fastlio_go2/stair_fastlio_local_20260914` |
| Fast-LIO 拼接地图 | `/home/hjj/fastlio_go2/src/FAST_LIO/PCD/scans_126_09_14_11_20_28.pcd` |
| 重力对齐、带定向法向量的 PCD | `/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/pcd/stair_fastlio_gravity_oriented_normals.pcd` |
| PCT tomogram | `/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/stair_fastlio_gravity_oriented_normals.pickle` |

## 1. 可视化 Fast-LIO 结果包

在两个终端都使用相同的 ROS 域。先启动 RViz，再启动回放，避免错过短生命周期话题。

终端 A：

```bash
source /opt/ros/humble/setup.bash
source /home/hjj/fastlio_go2/install/setup.bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/hjj/fastlio_go2/src/FAST_LIO/rviz/fastlio.rviz
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source /home/hjj/fastlio_go2/install/setup.bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 bag play /home/hjj/fastlio_go2/stair_fastlio_local_20260914 \
  --clock --rate 0.3
```

RViz 固定坐标系是 `map`。启用 `CloudRegistered` 显示 `/cloud_registered`，
并启用 `Odometry` 显示 `/Odometry`。结果包不需要 TF 才能显示这两个话题，
因为它们的消息头已经是 `map`。

## 2. 从结果包建立带符号法向的 PCD

```bash
cd /home/hjj/Go2w_3D_Nav
python3 tools/fastlio_bag_to_oriented_pcd.py \
  /home/hjj/fastlio_go2/stair_fastlio_local_20260914 \
  src/elevator_lio/PCD/pct_go2/pcd/stair_fastlio_gravity_oriented_normals.pcd \
  --imu-bag src/elevator_lio/PCD/2026-09-10_16-57-43/fastlio_input/rosbag2/stair_capture \
  --frame-voxel 0.10 --global-voxel 0.10 --normal-radius 0.25
```

脚本对每一帧 `/cloud_registered_body` 估算法向量，并将法向量指向传感器原点，
再用同时间戳 `/Odometry` 转到 `map`。`--imu-bag` 会使用原始包开始静止阶段的
IMU 加速度，将 Fast-LIO 的初始 IMU 坐标系旋转到重力 Z 轴；这是 PCT 切片的必要条件。
从上方观测的地面/踏步顶面通常是
`normal_z >= 0.7`，从下方观测的天花板或楼板底面通常是 `normal_z <= -0.7`。
该判断依赖重力已由 Fast-LIO 对齐；若 IMU 外参或重力方向错误，不能使用本筛选。

## 3. 生成 PCT tomogram

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch tomography tomography.launch.py \
  rsg_root:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2 \
  scene_name:=fastlio
```

完成日志应包含 `Normal-marked ground points` 和 `Tomogram exported`。按 `Ctrl-C`
结束节点即可，pickle 已写入 `tomogram/`。`scene_fastlio.py` 的关键参数是：

- `ground_h=-9.25`：这是本次 Fast-LIO `map` 原点下的最低地图高度，不是物理地面必须为零。
- `normal_ground_min_z=0.70`：只有向上的观测面可作为支撑面；所有点仍参与净空和障碍计算。
- `slice_dh=0.12`：相邻高度层采样间距。
- `step_max=0.24`、`slope_max=0.70`：Go2 楼梯测试的初始通行条件，必须按真实机器人能力验证。
- `connectivity_enable=false`：先不执行此前只保留最低连通分量的实验过滤，以免楼梯被裁断。

## 4. 在 RViz 中规划 PCT 路径

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pct_planner planner.launch.py \
  scene_name:=go2 \
  tomogram_path:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/stair_fastlio_gravity_oriented_normals.pickle \
  frame_id:=map \
  optimize_path:=false \
  launch_rviz:=true \
  rviz_profile:=pct_fastlio_stair.rviz \
  publish_raw_pcd:=true \
  raw_pcd_file:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/pcd/stair_fastlio_gravity_oriented_normals.pcd \
  raw_pcd_voxel_size:=0.10
```

在 RViz 中：

1. 保持 `PCT Tomogram` 启用，它是 PCT 实际可选的格点；`Raw PCD Map` 只是原始地图对照。
2. 使用 `Interact` 工具拖动 `Start` 和 `Goal`，右键任意一个标记并选择 `Plan route`。
3. 或用 `Publish Point` 依次点击起点和终点，默认模式会交替设置两者。
4. 路径输出话题为 `/pct_path`。它只用于全局参考路径，不能直接接 Go2 的 `cmd_vel`。

## 与 SCAN-Planner 的关系

PCT 在此链路中是离线地图上的全局/跨层路径生成器；SCAN-Planner 应在机器人
在线时订阅实时点云、里程计和 PCT 路径，负责短视距轨迹、局部避障以及最终运动控制。
在接入真实 Go2 前，先验证 PCT 路径在 RViz 中贴合踏步并且不落在天花板/楼板底面，
再做 `/pct_path` 到 SCAN-Planner 全局参考路径的适配。
