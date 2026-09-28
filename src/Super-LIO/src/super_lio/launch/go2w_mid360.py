from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 默认用机器狗那份配置; 台架直连雷达时传 config_file:=go2w_mid360_bench.yaml
    # (区别: imu_topic 用 /livox/imu, 外参取单位阵)。
    config = PathJoinSubstitution([
        FindPackageShare("super_lio"), "config", LaunchConfiguration("config_file")
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_3d_nav_viz"), "rviz", "super_lio.rviz"
    ])
    rviz = LaunchConfiguration("rviz")
    save_map = LaunchConfiguration("save_map")

    session_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_map_dir = f"../../PCD/map_{session_time}"
    map_name = f"go2w_mid360_map_{session_time}.pcd"

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "save_map",
            default_value="false",
            description="Save a timestamped PCD map when the node exits cleanly.",
        ),
        DeclareLaunchArgument("config_file", default_value="go2w_mid360.yaml"),
        LogInfo(
            condition=IfCondition(save_map),
            msg=f"Super-LIO map output: {save_map_dir}/{map_name}",
        ),
        Node(
            package="super_lio",
            executable="super_lio_node",
            name="super_lio_node",
            output="screen",
            parameters=[
                config,
                {
                    "lio.map.save_map": ParameterValue(save_map, value_type=bool),
                    "lio.map.save_map_dir": save_map_dir,
                    "lio.map.map_name": map_name,
                },
            ],
        ),
        Node(
            package="go2_3d_nav_viz",
            executable="history_cloud_accumulator",
            name="super_lio_history_map",
            output="screen",
            condition=IfCondition(save_map),
            parameters=[{
                "input_topic": "/lio/cloud_world",
                "output_topic": "/lio/history_map",
                "frame_id": "world",
                "voxel_size": 0.15,
                "publish_period": 2.0,
                "max_voxels": 1000000,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="super_lio_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
