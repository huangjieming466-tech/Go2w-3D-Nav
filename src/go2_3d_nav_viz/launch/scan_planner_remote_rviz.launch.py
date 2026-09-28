from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_3d_nav_viz"),
        "rviz",
        "scan_planner_remote.rviz",
    ])

    return LaunchDescription([
        # Inherit RMW_IMPLEMENTATION/CYCLONEDDS_URI from the invoking shell.
        # Interface names differ between the host and the Unitree endpoint;
        # forcing one here silently disconnects RViz from the ROS graph.
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="map",
            description="Global localization and DDDMR map frame.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="scan_planner_remote_rviz",
            output="screen",
            arguments=["-d", rviz_config, "-f", LaunchConfiguration("fixed_frame")],
        ),
    ])
