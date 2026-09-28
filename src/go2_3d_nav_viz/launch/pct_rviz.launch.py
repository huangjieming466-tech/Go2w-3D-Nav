from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parent.parent
    default_config = package_root / "rviz" / "pct_planner.rviz"

    fixed_frame = DeclareLaunchArgument("fixed_frame", default_value="map")
    tomogram_topic = DeclareLaunchArgument(
        "tomogram_topic", default_value="/pct_tomogram"
    )
    path_topic = DeclareLaunchArgument("path_topic", default_value="/pct_path")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="pct_rviz",
        output="screen",
        arguments=["-d", str(default_config)],
        parameters=[
            {
                "fixed_frame": LaunchConfiguration("fixed_frame"),
                "tomogram_topic": LaunchConfiguration("tomogram_topic"),
                "path_topic": LaunchConfiguration("path_topic"),
            }
        ],
    )

    return LaunchDescription([fixed_frame, tomogram_topic, path_topic, rviz])
