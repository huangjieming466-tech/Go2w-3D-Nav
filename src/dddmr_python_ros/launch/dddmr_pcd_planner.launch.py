"""Launch DDMR Python PCD global planning with optional RViz."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('dddmr_python_ros')
    args = [
        DeclareLaunchArgument('pcd_file'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_z', default_value='0.0'),
        DeclareLaunchArgument('use_odom_start', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='/LIO/odom_vehicle'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('robot_radius', default_value='0.35'),
        DeclareLaunchArgument('robot_height', default_value='0.50'),
        DeclareLaunchArgument('max_slope_deg', default_value='30.0'),
        DeclareLaunchArgument('max_step', default_value='0.25'),
        DeclareLaunchArgument('max_roughness', default_value='0.05'),
        DeclareLaunchArgument('normal_k', default_value='16'),
        # A 0.05 m voxel map of a 900k-point Fast-LIO PCD creates almost one
        # million graph candidates.  These defaults keep startup interactive;
        # they can be reduced for smaller maps through launch arguments.
        DeclareLaunchArgument('voxel_size', default_value='0.15'),
        DeclareLaunchArgument('use_source_normals', default_value='true'),
        DeclareLaunchArgument('normal_ground_min_z', default_value='0.20'),
        DeclareLaunchArgument('inflation_radius', default_value='0.10'),
        DeclareLaunchArgument('connection_radius', default_value='0.45'),
        DeclareLaunchArgument('max_neighbors', default_value='12'),
        DeclareLaunchArgument('transition_neighbors', default_value='8'),
        DeclareLaunchArgument('transition_radius', default_value='0.45'),
        DeclareLaunchArgument('transition_min_height', default_value='0.03'),
        DeclareLaunchArgument('snap_radius', default_value='1.5'),
    ]
    planner = Node(
        package='dddmr_python_ros',
        executable='dddmr_global_planner_node',
        name='dddmr_global_planner',
        output='screen',
        parameters=[{
            'pcd_file': LaunchConfiguration('pcd_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'start_z': LaunchConfiguration('start_z'),
            'use_odom_start': LaunchConfiguration('use_odom_start'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'planner.robot_radius': LaunchConfiguration('robot_radius'),
            'planner.robot_height': LaunchConfiguration('robot_height'),
            'planner.max_slope_deg': LaunchConfiguration('max_slope_deg'),
            'planner.max_step': LaunchConfiguration('max_step'),
            'planner.max_roughness': LaunchConfiguration('max_roughness'),
            'planner.normal_k': LaunchConfiguration('normal_k'),
            'planner.voxel_size': LaunchConfiguration('voxel_size'),
            'planner.use_source_normals': LaunchConfiguration('use_source_normals'),
            'planner.normal_ground_min_z': LaunchConfiguration('normal_ground_min_z'),
            'planner.inflation_radius': LaunchConfiguration('inflation_radius'),
            'planner.connection_radius': LaunchConfiguration('connection_radius'),
            'planner.max_neighbors': LaunchConfiguration('max_neighbors'),
            'planner.transition_neighbors': LaunchConfiguration('transition_neighbors'),
            'planner.transition_radius': LaunchConfiguration('transition_radius'),
            'planner.transition_min_height': LaunchConfiguration('transition_min_height'),
            'planner.snap_radius': LaunchConfiguration('snap_radius'),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='dddmr_rviz', output='screen',
        arguments=[
            '-d', share + '/rviz/dddmr_pcd_planner.rviz',
            '-f', LaunchConfiguration('frame_id'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )
    return LaunchDescription([*args, planner, rviz])
