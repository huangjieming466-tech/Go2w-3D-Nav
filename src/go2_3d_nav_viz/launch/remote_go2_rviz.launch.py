import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fixed_frame = LaunchConfiguration("fixed_frame")
    dds_config = os.path.join(
        get_package_share_directory("go2_3d_nav_viz"),
        "config",
        "cyclonedds_unitree.xml",
    )
    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_3d_nav_viz"),
        "rviz",
        "remote_go2.rviz",
    ])

    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", "file://" + dds_config),
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="utlidar_lidar",
            description="RViz fixed frame. Change this to the frame in the remote messages if needed.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="remote_go2_rviz",
            output="screen",
            arguments=["-d", rviz_config, "-f", fixed_frame],
        ),
    ])
