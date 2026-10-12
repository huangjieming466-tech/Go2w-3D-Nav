from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    save_map = LaunchConfiguration('save_map')
    publish_history_map = LaunchConfiguration('publish_history_map')
    imu_topic = LaunchConfiguration('imu_topic')
    blind_range = LaunchConfiguration('blind_range')
    config = PathJoinSubstitution([
        FindPackageShare('super_lio'), 'config', 'go2w_mid360.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('super_lio'), 'rviz', 'super_lio_utlidar.rviz'
    ])

    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    save_dir = f'../../PCD/utlidar_l2_map_{stamp}'
    map_name = f'utlidar_l2_super_lio_{stamp}.pcd'

    if os.environ.get('ROS_DISTRO') in ('foxy', 'galactic'):
        tf_args = [
            # Foxy positional arguments are x y z yaw pitch roll.
            '0.30773', '0.0', '-0.04232', '-1.01053', '-0.141324', '-2.92072',
            'imu', 'utlidar_lidar',
        ]
    else:
        tf_args = [
            '--x', '0.30773', '--y', '0.0', '--z', '-0.04232',
            '--roll', '-2.92072', '--pitch', '-0.141324', '--yaw', '-1.01053',
            '--frame-id', 'imu', '--child-frame-id', 'utlidar_lidar',
        ]

    l2_imu_lidar_rotation = [
        0.526113848275, -0.810135821197, 0.258619739901,
        -0.838668205342, -0.544642673086, 0.000001615186,
        0.140854037932, -0.216897002900, -0.965979207919,
    ]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('save_map', default_value='false'),
        DeclareLaunchArgument('publish_history_map', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/go2/imu'),
        DeclareLaunchArgument('blind_range', default_value='0.5'),
        Node(
            package='super_lio',
            executable='utlidar_to_super_lio.py',
            name='utlidar_to_super_lio',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/cloud',
                'output_topic': '/lio/utlidar_super_lio',
                'blind_range': ParameterValue(blind_range, value_type=float),
                'fallback_frame_id': 'utlidar_lidar',
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
                    'lio.ros.lidar_topic': '/lio/utlidar_super_lio',
                    'lio.ros.imu_topic': imu_topic,
                    'lio.sensor.lidar_type': 8,
                    'lio.sensor.blind': ParameterValue(blind_range, value_type=float),
                    'lio.sensor.maxrange': 60.0,
                    'lio.sensor.filter_rate': 1,
                    'lio.sensor.rear_near_filter_enable': False,
                    'lio.sensor.voxel_fliter_size': 0.10,
                    'lio.map.ds_size': 0.10,
                    'lio.map.save_map': ParameterValue(save_map, value_type=bool),
                    'lio.map.save_map_dir': save_dir,
                    'lio.map.map_name': map_name,
                    'lio.extrinsic.lidar_imu': [
                        0.30773, 0.0, -0.04232, *l2_imu_lidar_rotation,
                    ],
                    'lio.extrinsic.odom_robo': [
                        0.30773, 0.0, -0.04232,
                        179.9999, -14.9882, -56.9997,
                    ],
                },
            ],
        ),
        Node(
            package='go2_3d_nav_viz',
            executable='history_cloud_accumulator',
            name='super_lio_history_map',
            output='screen',
            condition=IfCondition(publish_history_map),
            parameters=[{
                'input_topic': '/lio/cloud_world',
                'output_topic': '/lio/history_map',
                'frame_id': 'world',
                'voxel_size': 0.10,
                'publish_period': 1.0,
                'max_voxels': 1500000,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_utlidar_frame',
            arguments=tf_args,
            condition=IfCondition(rviz),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='super_lio_utlidar_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
