from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    config_file = LaunchConfiguration("config_file")
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("fast_lio"), "launch", "mapping.launch.py"
        ])),
        launch_arguments={"config_file": config_file, "rviz": "false"}.items(),
    )
    locator = Node(
        package="open3d_loc_native",
        executable="global_localization_node",
        name="open3d_global_localization",
        output="screen",
        parameters=[{
            "map_file": map_file,
            "cloud_topic": "/cloud_registered_body_1",
            "odom_topic": "/Odometry_loc",
            "map_frame": "map",
            "base_frame": "base_link",
            "voxel_size": 0.15,
            "registration_distance": 1.5,
            "update_period": 0.10,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_file"),
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            FindPackageShare("fast_lio"), "config", "go2w_mid360.yaml"
        ])),
        fast_lio,
        locator,
    ])
