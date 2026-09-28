from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_3d_nav_viz"),
        "rviz",
        "elevator_lio_remote.rviz",
    ])

    return LaunchDescription([
        # Match the DDS setup used to receive Unitree/Go2 topics on this PC.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            "<CycloneDDS><Domain><General><Interfaces>"
            "<NetworkInterface name=\"eth0\" priority=\"default\" "
            "multicast=\"default\" />"
            "</Interfaces></General></Domain></CycloneDDS>",
        ),
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="world",
            description="LIO world frame used by registered clouds and map.",
        ),
        DeclareLaunchArgument(
            "publish_history_pcd",
            default_value="false",
            description="Publish a saved PCD locally for historical-map comparison.",
        ),
        DeclareLaunchArgument(
            "history_pcd_file",
            default_value="",
            description="Saved historical PCD in the same fixed_frame coordinates.",
        ),
        DeclareLaunchArgument(
            "history_pcd_topic",
            default_value="/lio/history_pcd_map",
        ),
        DeclareLaunchArgument(
            "history_pcd_voxel_size",
            default_value="0.15",
        ),
        Node(
            package="go2_3d_nav_viz",
            executable="pcd_map_publisher.py",
            name="lio_history_pcd_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_history_pcd")),
            parameters=[{
                "pcd_file": LaunchConfiguration("history_pcd_file"),
                "frame_id": LaunchConfiguration("fixed_frame"),
                "topic": LaunchConfiguration("history_pcd_topic"),
                "voxel_size": LaunchConfiguration("history_pcd_voxel_size"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="elevator_lio_remote_rviz",
            output="screen",
            arguments=["-d", rviz_config, "-f", LaunchConfiguration("fixed_frame")],
        ),
    ])
