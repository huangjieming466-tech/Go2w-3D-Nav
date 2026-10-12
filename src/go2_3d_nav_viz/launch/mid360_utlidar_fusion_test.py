from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mid360_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('utlidar_topic', default_value='/utlidar/cloud'),
        DeclareLaunchArgument(
            'output_topic', default_value='/lio/mid360_utlidar_fused'
        ),
        DeclareLaunchArgument('utlidar_clock_offset_sec', default_value='402.228'),
        DeclareLaunchArgument('max_combined_scan_duration', default_value='0.22'),
        DeclareLaunchArgument('max_pair_end_skew_sec', default_value='0.10'),
        DeclareLaunchArgument('mid360_decimation', default_value='3'),
        Node(
            package='go2_3d_nav_viz',
            executable='mid360_utlidar_fusion.py',
            name='mid360_utlidar_fusion',
            output='screen',
            parameters=[{
                'mid360_topic': LaunchConfiguration('mid360_topic'),
                'utlidar_topic': LaunchConfiguration('utlidar_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'utlidar_clock_offset_sec': ParameterValue(
                    LaunchConfiguration('utlidar_clock_offset_sec'), value_type=float
                ),
                'mid360_decimation': ParameterValue(
                    LaunchConfiguration('mid360_decimation'), value_type=int
                ),
                'max_combined_scan_duration': ParameterValue(
                    LaunchConfiguration('max_combined_scan_duration'), value_type=float
                ),
                'max_pair_end_skew_sec': ParameterValue(
                    LaunchConfiguration('max_pair_end_skew_sec'), value_type=float
                ),
            }],
        ),
    ])
