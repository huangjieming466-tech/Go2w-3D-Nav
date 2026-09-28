"""RViz view for live Super-LIO mapping or rosbag replay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('go2_3d_nav_viz')
    return LaunchDescription([
        DeclareLaunchArgument('fixed_frame', default_value='world'),
        DeclareLaunchArgument('publish_saved_map', default_value='false'),
        DeclareLaunchArgument('saved_map_file', default_value=''),
        DeclareLaunchArgument('saved_map_topic', default_value='/super_lio/saved_map'),
        DeclareLaunchArgument('saved_map_voxel_size', default_value='0.20'),
        Node(
            package='go2_3d_nav_viz',
            executable='pcd_map_publisher.py',
            name='super_lio_saved_map_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_saved_map')),
            parameters=[{
                'pcd_file': LaunchConfiguration('saved_map_file'),
                'frame_id': LaunchConfiguration('fixed_frame'),
                'topic': LaunchConfiguration('saved_map_topic'),
                'voxel_size': LaunchConfiguration('saved_map_voxel_size'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='super_lio_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                share, 'rviz', 'super_lio.rviz',
            ])],
            parameters=[{'fixed_frame': LaunchConfiguration('fixed_frame')}],
        ),
    ])
