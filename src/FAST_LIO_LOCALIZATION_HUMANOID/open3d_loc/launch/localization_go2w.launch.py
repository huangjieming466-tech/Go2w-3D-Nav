from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    launch_fast_lio = LaunchConfiguration("launch_fast_lio")
    fast_lio_share = FindPackageShare("fast_lio")
    open3d_share = FindPackageShare("open3d_loc")

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "config_file": "go2w_mid360.yaml",
            "rviz": "false",
        }.items(),
        # 用 Super-LIO 供数据时设 launch_fast_lio:=false，避免两个 LIO 同时跑、
        # 抢 CPU 且互相覆盖话题。
        condition=IfCondition(launch_fast_lio),
    )

    localization = Node(
        package="open3d_loc",
        executable="global_localization_node",
        name="global_localization_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([open3d_share, "config", "loc_param_g1.yaml"]),
            {
                "path_map": map_file,
                "pcd_queue_maxsize": 10,

                # --- 几何 / 判据 -------------------------------------------------
                # 原先 voxelsize_fine 一个值同时充当"体素边长"和"ICP/fitness 距离"(x2/x4)，
                # 导致 speed 和 accuracy 绑死。现已拆成四个独立参数：
                #   voxelsize_map   目标地图降采样体素（只管点数）
                #   voxelsize_fine  扫描降采样体素（只管点数）
                #   icp_max_dist    ICP 最大对应距离（原 = voxelsize_fine*2 = 0.40）
                #   fitness_dist    fitness 评估距离（原 = voxelsize_fine*4 = 0.80）
                # 判据距离是"采信配准结果"的门槛：0.80 意味着偏 0.5 m 的位姿也能通过。
                "voxelsize_map": 0.15,
                "voxelsize_coarse": 0.30,   # 只用于 /map_points 可视化发布
                "voxelsize_fine": 0.10,
                # icp_max_dist 是 ICP 的"收敛盆地"，要够大才能跨过初始位姿的偏差。
                # 实测机器人上存在 ~6.7deg 的稳定偏航残差，6.7deg 在 1.7 m 处就是
                # 0.20 m 位移 —— 设 0.20 会把超过一半的点排除在对应之外，ICP 失去
                # 约束偏航的能力。fitness_dist 才是验收门槛，那个保持收紧。
                "icp_max_dist": 0.40,
                "fitness_dist": 0.25,
                # 诊断用: LocalizationInitialize() 里必须【连续两次】
                #   fitness_initial > threshold_fitness_init
                # 才 break 出初始化循环并置 loc_initialized_=true。
                # 达不到就死循环 -> 主循环永不运行 -> /baselink2map_kalman、
                # /localization_3d、/localization_3d_confidence 全部不发布。
                # 这里临时调低，让节点先能起来，再靠 confidence 看真实 fitness。
                # 定位调好之后应改回 0.5。
                "threshold_fitness_init": 0.50,
                "threshold_fitness": 0.65,

                # --- 频率与开销 ---------------------------------------------------
                # loc_frequence 单位是"秒"(定位间隔)，不是 Hz。
                # 实际更新率 = 1 / max(loc_frequence, 单次配准耗时 loc_cost)
                "loc_frequence": 0.3,
                "maxpoints_source": 25000,
                "maxpoints_target": 120000,
                "filter_odom2map": True,
            },
        ],
        # 内部约定: ICP 用 reg_matrix = mat_odom2map_ 变换 source，所以
        #   source 点云 和 里程计 必须【在同一个坐标系】。
        #   - FAST-LIO: /Odometry_loc (camera_init->body) + /cloud_registered_1 (camera_init)
        #   - Super-LIO: /lio/odom (world 系 IMU 位姿) + /lio/cloud_world (world 系当前帧)
        # 两对都是自洽的。用 lio_backend 切换即可。
        remappings=[
            ("/Odometry_loc", odom_topic),
            ("/cloud_registered_1", cloud_topic),
        ],
    )

    # The Go2W LIO configuration currently uses the IMU/body reference as its
    # navigation origin. Keep these explicit so open3d_loc can resolve its TFs.
    # FAST-LIO publishes camera_init -> body, while open3d_loc publishes
    # map -> odom.  camera_init is FAST-LIO's initial odometry frame, so this
    # identity transform closes the chain for RViz:
    # map -> odom -> camera_init -> body.
    odom_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "camera_init"],
    )
    imu_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # URDF: base_link -> imu_link = [-0.02557, 0, 0.04232], identity R.
        # Open3D Loc queries the inverse imu_link -> base_link transform.
        arguments=["0.02557", "0", "-0.04232", "0", "0", "0", "1", "imu_link", "base_link"],
    )
    base_to_motion = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "motion_link"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="open3d_localization_rviz",
        arguments=["-d", PathJoinSubstitution([open3d_share, "rviz_cfg", "localization_minimal.rviz"])],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_file"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/Odometry_loc",
            description="Odometry for localization; must share the frame of cloud_topic.",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/cloud_registered_1",
            description="Current-scan cloud; must share the frame of odom_topic.",
        ),
        DeclareLaunchArgument(
            "launch_fast_lio",
            default_value="true",
            description="Start FAST-LIO too. Set false when Super-LIO supplies odom+cloud.",
        ),
        fast_lio,
        odom_to_camera_init,
        imu_to_base,
        base_to_motion,
        localization,
        rviz_node,
    ])
