# Go2w 3D Navigation 链路记录

本文档记录当前工作空间中两条已经验证过的导航思路：

1. `Elevator-LIO -> JIE 3D Nav/OctoMap -> SCAN-Planner`
2. `Elevator-LIO/PCD -> PCT Planner -> SCAN-Planner`

当前测试机器是主机上的 Livox Mid-360，尚未接入 Go2w 本体。因此本文中的 SCAN 控制器以仿真/开环测试为主，不能直接驱动真实机器狗。

工作空间：

```text
/home/hjj/Go2w_3D_Nav
```

## 一、系统职责

```text
Mid-360 驱动
    -> 点云 + IMU
Elevator-LIO
    -> world/map 坐标系下的里程计、当前点云、累计 PCD 地图
全局规划器（二选一）
    -> nav_msgs/Path
SCAN-Planner
    -> 局部避障、轨迹优化、速度控制
机器狗驱动
    -> cmd_vel 或对应的 Unitree 控制接口
```

Elevator-LIO 主要负责定位和建图，不是全局路径规划器。当前配置输出的主要话题为：

| 话题 | 类型/含义 |
|---|---|
| `/LIO/odom_vehicle` | 车体中心里程计，父坐标系为 `world` |
| `/LIO/odom_imu` | IMU 中心里程计 |
| `/LIO/clouds_lidar` | 当前去畸变点云，已经转换到 `world` 坐标系 |
| `/LIO/global_map` | 累计全局点云，需打开对应发布开关 |
| `/LIO/ikdtree` | ikd-tree 地图点云，需打开对应发布开关 |

当前 Elevator-LIO 配置文件是：

```text
src/elevator_lio/yaml/root_config.yaml
src/elevator_lio/yaml/sensors/livox.yaml
src/elevator_lio/yaml/runtime/mapping.yaml
```

## 二、公共环境初始化

普通 ROS 2 / Elevator-LIO 测试终端：

```bash
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
source /home/hjj/livox_ws/install/setup.bash
source install/setup.bash
```

PCT 相关终端使用工作空间提供的辅助脚本。它还会加入本地 GTSAM、OSQP 和 CuPy NVRTC 库路径：

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh
```

如果 ROS 2 CLI 被旧 daemon 卡住，可优先使用：

```bash
ros2 topic list --no-daemon
ros2 topic echo --no-daemon /LIO/odom_vehicle
```

或者先清理 daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

如果要跨主机连接 Unitree 设备，需要在相关终端统一设置 CycloneDDS 网络接口。例如之前主机连接机器狗时使用过：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

仅使用本机 Mid-360 测试时，不需要连接机器狗，也不要把 Unitree 专用环境当成 Livox 驱动的必要条件。

## 三、从 Mid-360 开始建图

### 3.1 启动 Livox 驱动

在 Livox 工作空间终端：

```bash
source /opt/ros/humble/setup.bash
source /home/hjj/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

先确认驱动是否发布数据：

```bash
ros2 topic list --no-daemon | grep -E 'livox|mid|imu|cloud'
ros2 topic echo --no-daemon /livox/lidar
ros2 topic echo --no-daemon /livox/imu
```

Elevator-LIO 当前 Livox 配置默认订阅：

```yaml
lidar_topic_name: "/livox/lidar"
imu_topic_name: "/livox/imu"
```

如果驱动实际发布的话题不同，应修改 `src/elevator_lio/yaml/sensors/livox.yaml`，而不是在后面的规划器中反复改话题名。

### 3.2 启动 Elevator-LIO 建图

```bash
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
source /home/hjj/livox_ws/install/setup.bash
source install/setup.bash

ros2 launch lio start_ros2.launch.py
```

如果只进行普通室内建图，`root_config.yaml` 当前使用：

```yaml
runtime_config: "runtime/mapping.yaml"
```

建议先在 RViz2 检查：

- Fixed Frame 是否为 `world`；
- `/LIO/odom_vehicle` 是否有数据；
- `/LIO/clouds_lidar` 是否随运动更新；
- 点云和里程计的坐标方向是否正确。

可以单独启动 Elevator-LIO 自带 RViz2：

```bash
ros2 launch lio start_ros2.launch.py
```

如果不需要自带 RViz2：

```bash
ros2 launch lio start_ros2.launch.py use_rviz:=false
```

### 3.3 保存 PCD 地图

建图结束后使用 `Ctrl-C` 退出。当前 `mapping.yaml` 开启全局地图保存时，Elevator-LIO 会在配置的 PCD 目录生成地图文件。当前测试中使用过：

```text
/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6.pcd
/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd
```

`mapv6_level.pcd` 是当前用于规划对比的水平化 PCD。若原始地图存在俯仰角，应该先确认坐标变换，再把同一坐标系的 PCD 用于 JIE、PCT 和 SCAN，避免不同规划器看到不同地图。

## 四、链路 A：Elevator-LIO -> JIE 3D Nav -> SCAN

### 4.1 JIE 的工作方式

JIE 不是直接在原始 PCD 上搜索路径，而是：

```text
PCD
  -> OctoMap
  -> 占据/地面约束和代价处理
  -> jie_path_node A* 路径
  -> /planned_path (nav_msgs/Path)
```

JIE 的 PCD 导入工具会把天花板、墙面等点转换为占据体素。因此完整室内点云需要配置合适的分辨率、地面支持约束和人工编辑，否则天花板等高处结构可能影响可选路径。

### 4.2 编译 JIE

```bash
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select jie_map_msgs jie_octomap octo_planner
source install/setup.bash
```

Open3D C++ 依赖已经安装：

```bash
sudo apt install libopen3d-dev libglm-dev
```

### 4.3 PCD 导入为 OctoMap

```bash
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch jie_octomap import_pcd_map.launch.py
```

该命令会启动 PCD 导入 GUI。GUI 中选择：

1. `/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd`；
2. 检查点云方向和高度；
3. 必要时裁剪墙外区域或删除异常点；
4. 转换为 OctoMap；
5. 保存地图包。

之前保存过的地图包示例：

```text
/home/hjj/Go2w_3D_Nav/src/elevator_lio/Octomap/1788934114_ikdtree_scans
/home/hjj/Go2w_3D_Nav/src/elevator_lio/Octomap/mapv6_level
```

### 4.4 加载已保存的 JIE 地图

```bash
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch jie_octomap web_octomap.launch.py \
  map_package:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/Octomap/mapv6_level \
  launch_map_gui:=false
```

浏览器访问：

```text
http://127.0.0.1:8080
```

该 launch 中：

- JIE 全局路径发布在 `/planned_path`；
- 起点话题为 `/start_point`；
- 终点话题为 `/goal_point`；
- 坐标系为 `map`。

也可以在 RViz2 中查看 JIE 路径：

```bash
ros2 topic echo --no-daemon /planned_path
ros2 topic info /planned_path
```

### 4.5 JIE 到 SCAN 的现状

SCAN 的 `navi_mode:=3` 会订阅全局路径，并在局部范围内重新规划。其默认接口是：

```text
/initial_path : nav_msgs/Path
```

JIE 输出的是：

```text
/planned_path : nav_msgs/Path
```

因此消息类型兼容，但当前工作空间还没有现成的 JIE→SCAN 联合 launch。需要增加一个桥接节点或 topic relay，将：

```text
/planned_path -> /initial_path
```

同时确认以下内容：

- JIE 路径的 `header.frame_id` 与 SCAN 使用的地图坐标系一致；
- 路径高度是地面高度还是机体中心高度；
- SCAN 的 `init_x/init_y/init_z` 与 Elevator-LIO 当前位姿一致；
- SCAN 的实时点云和里程计话题已经重映射到实际机器人话题。

SCAN 单独启动的基本形式是：

```bash
source /opt/ros/humble/setup.bash
source /home/hjj/Go2w_3D_Nav/install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=false \
  navi_mode:=3 \
  sensor_type:=lidar \
  controller_mode:=open_loop \
  use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd
```

这条命令只启动 SCAN，不会自动把 JIE 的 `/planned_path` 变成 `/initial_path`。因此当前链路 A 是“JIE 已可单独规划，SCAN 已支持接收全局路径，二者还需要桥接节点”。

## 五、链路 B：Elevator-LIO/PCD -> PCT Planner -> SCAN

### 5.1 PCT 的预处理

PCT 不是直接对任意 PCD 做一次 A*。流程是：

```text
PCD
  -> tomography 预处理
  -> tomogram .pickle
  -> PCT 图搜索
  -> /pct_path (nav_msgs/Path)
  -> SCAN-Planner
```

当前 Go2 场景配置：

```text
src/pct/tomography/tomography/config/scene_go2.py
```

当前测试生成的 tomogram：

```text
/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle
```

对单层走廊，`slice_dh` 不应随意改成很大的值。它是垂直层间采样间隔，不是“是否跨楼层”的开关。较大的值可能使天花板高度被选为规划层。当前配置恢复为：

```python
self.map.slice_dh = 0.5
```

### 5.2 生成 tomogram

如果需要重新生成，先删除旧结果，避免误加载旧 pickle：

```bash
rm /home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle
```

启动 tomography：

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh

ros2 launch tomography tomography.launch.py \
  rsg_root:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2 \
  scene_name:=go2
```

本机 GPU 可用时，CuPy 会使用 CUDA。检查命令：

```bash
source tools/source_pct_workspace.sh
python3 -c "import cupy as cp; print(cp.__version__); print(cp.cuda.runtime.getDeviceCount()); print(cp.arange(3).get())"
```

预期最后输出类似：

```text
[0 1 2]
```

### 5.3 只运行 PCT Planner

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh

ros2 launch pct_planner planner.launch.py \
  scene_name:=go2 \
  tomogram_path:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle \
  frame_id:=map \
  optimize_path:=false \
  launch_rviz:=true \
  publish_raw_pcd:=true \
  raw_pcd_file:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd \
  raw_pcd_voxel_size:=0.10
```

RViz 中可以选择显示：

- `/pct_tomogram`：PCT 预处理后的可通行 tomogram 单元；
- `/raw_pcd_map`：原始 PCD 对照；
- `/pct_path`：PCT 全局路径；
- `/pct_planner/waypoint_markers`：起点和终点标记。

路径输入方式：

- RViz `Publish Point` 交替选择起点和终点；
- 或使用 PCT RViz 中的交互式起点/终点标记；
- PCT 输出路径话题为 `/pct_path`。

### 5.4 PCT 和 SCAN 联合启动

当前工作空间已经提供联合 launch：

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_pct_workspace.sh

ros2 launch pct_scan_bridge paper_scene_pct_scan.launch.py \
  scene_name:=go2 \
  tomogram_path:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle \
  pcd_map_file:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd \
  use_rviz:=true \
  use_gpu:=false \
  enable_local_sensing:=false \
  controller_mode:=open_loop \
  plan_on_startup:=false
```

该 launch 的主要连接关系：

```text
pct_scan_bridge
  -> /pct_path
  -> SCAN initial_path

SCAN
  -> 局部规划/轨迹优化
  -> 仿真或开环 cmd_vel
```

联合 launch 内部相当于：

```text
PCT: /pct_path
SCAN: initial_path remap 到 /pct_path
SCAN: navi_mode:=3
```

当前测试阶段建议使用：

```text
enable_local_sensing:=false
controller_mode:=open_loop
is_real_world:=false
```

这样先验证全局路径能否被 SCAN 正确接收，再接入实时 Mid-360 点云和真实机器狗控制。

## 六、实时机器人阶段的接口要求

SCAN 真实运行默认使用以下 Elevator-LIO 话题：

```text
/LIO/odom_vehicle
/LIO/odom_imu
/LIO/clouds_lidar
```

SCAN 的真实模式启动形式：

```bash
source /opt/ros/humble/setup.bash
source /home/hjj/Go2w_3D_Nav/install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=3 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

但是这条命令只适用于已经完成以下适配的情况：

1. Elevator-LIO 输出的 `world` 坐标系与 SCAN 地图坐标系一致；
2. SCAN 订阅到真实的 `/LIO/odom_vehicle` 和 `/LIO/clouds_lidar`；
3. `/pct_path` 或桥接后的 `/initial_path` 使用相同坐标系；
4. SCAN 输出的 `/cmd_vel` 已经接到机器狗底层控制节点；
5. 已确认速度、机体高度和坐标轴方向符合 Go2w 实际接口。

在没有机器狗时，不要使用 `is_real_world:=true` 和 `closed_loop` 进行运动控制测试。

## 七、当前状态总结

### 已经完成

- Livox Mid-360 驱动能够单独启动；
- Elevator-LIO ROS 2 工作空间和 Livox 输入链路已建立；
- 已生成并使用 Elevator-LIO PCD 地图；
- JIE 能够将 PCD 导入 OctoMap 并进行 Web 3D 路径规划；
- PCT tomography 能够生成 `mapv6_level.pickle`；
- PCT Planner 能在 RViz 中显示 tomogram、原始 PCD 和全局路径；
- PCT→SCAN 联合 launch 已存在，路径接口为 `/pct_path` 到 SCAN 的 `initial_path`；
- SCAN 的 `navi_mode:=3` 已用于接收全局路径并进行局部规划。

### 尚未完成

- JIE `/planned_path` 到 SCAN `initial_path` 的专用桥接节点；
- Elevator-LIO 与真实 Go2w 的最终 TF、外参和坐标系确认；
- SCAN 输出到 Unitree Go2w 的真实控制接口适配；
- 用真实 Mid-360 在线点云验证 SCAN 动态局部避障；
- DDDMR 全局规划器核心、`perception_3d`、`dddmr_sys_core` 已加入当前工作空间并完成编译；由于官方依赖外部 `mcl_3dl/pcl_publisher`，已增加 `dddmr_pcd_publisher` 适配节点用于离线 PCD 测试。

## 八、推荐的后续顺序

1. 继续使用已保存 PCD，分别比较 JIE 和 PCT 的全局路径质量。
2. 先修正 PCT 单层地图的天花板/地面层选择问题。
3. 增加 JIE→SCAN 的 `nav_msgs/Path` 桥接节点。
4. 使用 Mid-360 在线点云和 Elevator-LIO 里程计运行 SCAN，但保持开环控制。
5. 最后才接入 Go2w，并切换到真实闭环控制。

## 九、DDDMR 路线（新增）

DDDMR 的静态全局规划链路与 PCT 不同。它不直接读取 tomogram，而是由 `perception_3d::StaticLayer` 根据两类点云建立地面图：

- `/mapground`：可行走地面点；
- `/mapcloud`：墙体、立柱等静态障碍点；
- `/clicked_point`：RViz 目标点；起点通过 `map -> base_link` TF 获取；
- `/global_path`：DDDMR 输出的 `nav_msgs/Path`。

官方 launch 依赖 `mcl_3dl/pcl_publisher`，当前主机没有该包。因此当前工作区新增：

- `src/dddmr_pcd_publisher`：读取校平后的 PCD，按高度拆分地面和障碍并发布 DDDMR 输入；
- `tools/source_dddmr_workspace.sh`：加载工作区和 `/home/hjj/.local/lib` 中的 `small_gicp`。

先用离线 PCD 测试：

```bash
cd /home/hjj/Go2w_3D_Nav
source tools/source_dddmr_workspace.sh
ros2 launch dddmr_pcd_publisher dddmr_pcd_global_planner.launch.py \
  pcd_file:=/home/hjj/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_floor_level.pcd \
  frame_id:=map \
  robot_x:=0.0 robot_y:=0.0 robot_z:=0.0
```

节点参数的调节优先级：

```text
ground_z              校平后的地面高度，当前应为 0.0
ground_tolerance      地面点的上下容差，默认 0.12 m
obstacle_min_height   高于地面多少才作为障碍，默认 0.15 m
obstacle_max_height   参与规划的最大障碍高度，默认 1.8 m，用于排除高处天花板
map_voxel_size        障碍点降采样尺寸，默认 0.15 m
ground_voxel_size     地面点降采样尺寸，默认 0.20 m
```

PCT tomography 还使用 `map.ground_max_h` 限制地面候选的最高高度。当前 Go2w 校平地图设置为 `0.8 m`，防止只有天花板回波的 XY 栅格被误认为可行走地面；它不会删除原始 PCD，只限制 `layers_g`。

对于只有 Fast-LIO 最终 PCD、没有观测轨迹的情况，使用 `tools/structural_floor_filter.py` 作为离线回退方案。它按水平表面的高度峰值和上下净空推断每层地面/吊顶对，删除上方吊顶平面，并把推断地面写为 `normal_z=1` 标记：

```bash
python3 tools/structural_floor_filter.py \
  src/elevator_lio/PCD/mapv6_floor_level.pcd \
  src/elevator_lio/PCD/mapv6_structural_floor.pcd
```

当前 `mapv6` 的检测结果为地面约 `z=-0.03 m`、吊顶约 `z=2.47 m`。Tomography 仅允许带有 `normal_z >= 0.70` 标记的点成为地面候选，墙体和其他点仍会作为障碍高度输入。这是一种无轨迹的几何推断，跨楼层应使用同样的净空配对规则；拥有原始 Fast-LIO 轨迹时，应优先以实际观测方向统一法向。

DDDMR 适配目前只完成“PCD -> DDDMR 全局路径”的离线验证入口，尚未完成 `/global_path` 到 SCAN `initial_path` 的桥接，也尚未接入 Mid-360 在线障碍层。实际机器人测试前仍需确认 Elevator-LIO 的 TF、`base_link` 位置和 Go2w 控制接口。
