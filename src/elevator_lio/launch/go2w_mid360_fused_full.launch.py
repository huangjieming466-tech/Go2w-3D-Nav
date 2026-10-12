from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch',
                'rviz_MID360s_launch.py',
            ])
        )
    )

    imu_bridge = Node(
        package='lio',
        executable='go2_lowstate_imu_bridge',
        name='go2_lowstate_imu_bridge',
        output='screen',
    )

    super_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('super_lio'),
                'launch',
                'go2w_mid360_fused_super_lio.py',
            ])
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'save_map': LaunchConfiguration('save_map'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('save_map', default_value='false'),
        driver_launch,
        imu_bridge,
        super_lio_launch,
    ])
