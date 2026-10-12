import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('super_lio'), 'config', LaunchConfiguration('config_file')
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('go2_3d_nav_viz'), 'rviz', 'super_lio_fused.rviz'
    ])
    connect_to_lio = LaunchConfiguration('connect_to_lio')
    selected_lidar_topic = PythonExpression([
        "'/lio/mid360_utlidar_fused' if '", connect_to_lio,
        "' == 'true' else '/livox/lidar'",
    ])
    utlidar_clock_offset = LaunchConfiguration('utlidar_clock_offset_sec')
    auto_clock_offset = LaunchConfiguration('auto_estimate_utlidar_clock_offset')
    max_scan_duration = LaunchConfiguration('max_combined_scan_duration')
    max_pair_skew = LaunchConfiguration('max_pair_end_skew_sec')
    mid360_decimation = LaunchConfiguration('mid360_decimation')
    save_map = LaunchConfiguration('save_map')
    rviz = LaunchConfiguration('rviz')

    if os.environ.get('ROS_DISTRO') in ('foxy', 'galactic'):
        mid360_tf_args = [
            '0.1870', '0.0', '0.0803', '0.0', '0.2268928', '0.0',
            'imu', 'livox_frame',
        ]
        utlidar_tf_args = [
            '0.30773', '0.0', '-0.04232', '-1.01053', '-0.141324', '-2.92072',
            'imu', 'utlidar_lidar',
        ]
    else:
        mid360_tf_args = [
            '--x', '0.1870', '--y', '0.0', '--z', '0.0803',
            '--roll', '0.0', '--pitch', '0.2268928', '--yaw', '0.0',
            '--frame-id', 'imu', '--child-frame-id', 'livox_frame',
        ]
        utlidar_tf_args = [
            '--x', '0.30773', '--y', '0.0', '--z', '-0.04232',
            '--roll', '-2.92072', '--pitch', '-0.141324', '--yaw', '-1.01053',
            '--frame-id', 'imu', '--child-frame-id', 'utlidar_lidar',
        ]

    session_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    save_map_dir = f'../../PCD/map_{session_time}'
    map_name = f'go2w_mid360_fused_map_{session_time}.pcd'

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='go2w_mid360.yaml'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('save_map', default_value='false'),
        DeclareLaunchArgument(
            'connect_to_lio',
            default_value='true',
            description='Use fused scans as Super-LIO input; requires validated timing/extrinsics.',
        ),
        DeclareLaunchArgument(
            'utlidar_clock_offset_sec',
            default_value='402.228',
            description='Fallback UTLIDAR-to-MID-360 offset used only when auto estimation is disabled.',
        ),
        DeclareLaunchArgument('auto_estimate_utlidar_clock_offset', default_value='true'),
        DeclareLaunchArgument('clock_calibration_arrival_skew_sec', default_value='0.10'),
        DeclareLaunchArgument('clock_calibration_window_size', default_value='31'),
        DeclareLaunchArgument('clock_calibration_min_samples', default_value='7'),
        DeclareLaunchArgument('max_combined_scan_duration', default_value='0.22'),
        DeclareLaunchArgument('max_pair_end_skew_sec', default_value='0.10'),
        DeclareLaunchArgument('mid360_decimation', default_value='3'),
        DeclareLaunchArgument('utlidar_blind_range', default_value='0.5'),
        LogInfo(
            condition=IfCondition(connect_to_lio),
            msg='Super-LIO input is /lio/mid360_utlidar_fused; stop any other Super-LIO instance first.',
        ),
        Node(
            package='go2_3d_nav_viz',
            executable='mid360_utlidar_fusion.py',
            name='mid360_utlidar_fusion',
            output='screen',
            parameters=[{
                'mid360_topic': '/livox/lidar',
                'utlidar_topic': '/utlidar/cloud',
                'output_topic': '/lio/mid360_utlidar_fused',
                'max_combined_scan_duration': ParameterValue(
                    max_scan_duration, value_type=float
                ),
                'max_pair_end_skew_sec': ParameterValue(max_pair_skew, value_type=float),
                'mid360_decimation': ParameterValue(mid360_decimation, value_type=int),
                'utlidar_blind_range': ParameterValue(
                    LaunchConfiguration('utlidar_blind_range'), value_type=float
                ),
                'utlidar_clock_offset_sec': ParameterValue(
                    utlidar_clock_offset, value_type=float
                ),
                'auto_estimate_utlidar_clock_offset': ParameterValue(
                    auto_clock_offset, value_type=bool
                ),
                'clock_calibration_arrival_skew_sec': ParameterValue(
                    LaunchConfiguration('clock_calibration_arrival_skew_sec'),
                    value_type=float,
                ),
                'clock_calibration_window_size': ParameterValue(
                    LaunchConfiguration('clock_calibration_window_size'),
                    value_type=int,
                ),
                'clock_calibration_min_samples': ParameterValue(
                    LaunchConfiguration('clock_calibration_min_samples'),
                    value_type=int,
                ),
            }],
        ),
        Node(
            package='super_lio',
            executable='super_lio_node',
            name='super_lio_node',
            output='screen',
            parameters=[
                config,
                {
                    'lio.map.save_map': ParameterValue(save_map, value_type=bool),
                    'lio.map.save_map_dir': save_map_dir,
                    'lio.map.map_name': map_name,
                    'lio.ros.lidar_topic': ParameterValue(selected_lidar_topic, value_type=str),
                    'lio.sensor.rear_near_filter_enable': ParameterValue(
                        save_map, value_type=bool
                    ),
                    'lio.sensor.filter_rate': 1,
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_mid360_frame',
            arguments=mid360_tf_args,
            condition=IfCondition(rviz),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_utlidar_frame',
            arguments=utlidar_tf_args,
            condition=IfCondition(rviz),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='super_lio_fused_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
