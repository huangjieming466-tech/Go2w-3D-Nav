"""Start the host-side Go2W global/local navigation chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dddmr_launch = os.path.join(
        get_package_share_directory("dddmr_python_ros"),
        "launch",
        "dddmr_pcd_planner.launch.py",
    )
    scan_launch = os.path.join(
        get_package_share_directory("scan_planner"), "launch", "run.launch.py"
    )
    rviz_launch = os.path.join(
        get_package_share_directory("go2_3d_nav_viz"),
        "launch",
        "scan_planner_remote_rviz.launch.py",
    )

    arguments = [
        DeclareLaunchArgument(
            "pcd_file",
            default_value=(
                "/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/"
                "map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd"
            ),
        ),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("launch_dddmr", default_value="true"),
        DeclareLaunchArgument("launch_scan", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
    ]

    dddmr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dddmr_launch),
        condition=IfCondition(LaunchConfiguration("launch_dddmr")),
        launch_arguments={
            "pcd_file": LaunchConfiguration("pcd_file"),
            "frame_id": "map",
            "launch_rviz": "false",
            "use_odom_start": "true",
            "odom_topic": "/baselink2map_kalman",
            "use_source_normals": "true",
            "voxel_size": "0.10",
            "normal_k": "12",
            "normal_ground_min_z": "0.10",
            "max_slope_deg": "38.0",
            "max_roughness": "0.08",
            "robot_radius": "0.22",
            "robot_height": "0.50",
            "max_step": "0.30",
            "inflation_radius": "0.05",
            "connection_radius": "1.50",
            "transition_radius": "1.50",
            "transition_neighbors": "16",
            "max_neighbors": "32",
        }.items(),
    )

    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scan_launch),
        condition=IfCondition(LaunchConfiguration("launch_scan")),
        launch_arguments={
            "is_real_world": "true",
            "lio_backend": "super_global",
            "navi_mode": "3",
            "sensor_type": "lidar",
            "controller_mode": "closed_loop",
            "use_sim_time": "false",
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "launch_rviz": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        launch_arguments={"fixed_frame": "map"}.items(),
    )

    return LaunchDescription([*arguments, dddmr, scan, rviz])
