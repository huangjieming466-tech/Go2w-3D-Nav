"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    viz_share = get_package_share_directory("go2_3d_nav_viz")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    planner_config = LaunchConfiguration("planner_config").perform(context)
    controller_config = LaunchConfiguration("controller_config").perform(context)
    planner_configs = [planner_yaml] + ([planner_config] if planner_config else [])
    controller_configs = [controllers_yaml] + ([controller_config] if controller_config else [])
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    lio_backend = LaunchConfiguration("lio_backend").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    humanoid_pose_topic = LaunchConfiguration("humanoid_pose_topic").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    planning_horizon = float(LaunchConfiguration("planning_horizon").perform(context))
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context))
    initial_path = "/initial_path"
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if lio_backend not in ("elevator", "super", "super_global", "humanoid"):
        raise RuntimeError("lio_backend must be 'elevator', 'super', 'super_global', or 'humanoid'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )

    if is_real:
        if lio_backend == "super":
            # Super-LIO's mapping node publishes its fused pose on /lio/odom.
            # /lio/robo/odom and /lio/imu/odom are declared by that node but
            # are not populated by its mapping implementation.
            body_pose = "/lio/base_odom"
            sensor_pose = "/lio/odom"
            cloud = "/lio/cloud_world"
            cloud_is_world = True
            need_extrinsic = False
        elif lio_backend == "super_global":
            # Keep local planning in Super-LIO's continuous world frame. The
            # DDDMR path is converted from map to world at the planner input.
            body_pose = "/lio/base_odom"
            sensor_pose = "/lio/odom"
            cloud = "/lio/cloud_world"
            cloud_is_world = True
            need_extrinsic = False
            initial_path = "/initial_path_world"
        elif lio_backend == "humanoid":
            # The cloud is from Fast-LIO Humanoid.  Use Open3D localization's
            # map-aligned pose by default so the DDDMR map and live scan share
            # the same frame.  /Odometry_loc remains available for local-only
            # tests through humanoid_pose_topic:=/Odometry_loc.
            body_pose = humanoid_pose_topic
            sensor_pose = humanoid_pose_topic
            cloud = "/cloud_registered_body_1"
            cloud_is_world = False
            need_extrinsic = False
        else:
            body_pose = "/LIO/odom_vehicle"
            sensor_pose = "/LIO/odom_imu"
            cloud = "/LIO/clouds_lidar"
            cloud_is_world = False
            need_extrinsic = True
        depth = "/camera/aligned_depth_to_color/image_raw"
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "fsm.planning_horizon": planning_horizon,
        "manager.planning_horizon": planning_horizon,
        "grid_map.frame_id": "world",
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=planner_configs + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", initial_path),
            ],
        )
    ]
    if is_real and lio_backend in ("super", "super_global"):
        actions.append(
            Node(
                package="go2_3d_nav_viz",
                executable="super_imu_to_base_odom.py",
                name="super_imu_to_base_odom",
                output="screen",
                parameters=[{
                    "input_topic": "/lio/odom",
                    "output_topic": "/lio/base_odom",
                    "base_frame": "base_link",
                    "base_to_imu_x": -0.02557,
                    "base_to_imu_y": 0.0,
                    "base_to_imu_z": 0.04232,
                }],
            )
        )
    if is_real and lio_backend == "super_global":
        actions.append(
            Node(
                package="go2_3d_nav_viz",
                executable="map_path_to_world_bridge.py",
                name="map_path_to_world_bridge",
                output="screen",
                parameters=[{
                    "input_path_topic": "/initial_path",
                    "map_world_topic": "/odom2map_kalman",
                    "output_path_topic": "/initial_path_world",
                    "world_frame": "world",
                }],
            )
        )
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                         " use_gazebo:=false"]
                    )
                },
            ],
        )
    )

    # The open-loop controller publishes a synthetic body pose.  It is useful
    # only in simulation and must never share a real localization topic.
    if controller_mode == "open_loop" and not is_real:
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=controller_configs + [common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=controller_configs + [common],
                remappings=[
                    ("body_pose", body_pose),
                    (
                        "cmd_vel",
                        LaunchConfiguration("cmd_vel_topic") if is_real else "/quad_0/cmd_vel",
                    ),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        *controller_configs,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=controller_configs + [common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    if launch_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="scan_planner_remote_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(viz_share, "rviz", "scan_planner_remote.rviz"),
                    "-f",
                    LaunchConfiguration("rviz_fixed_frame"),
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument(
                "lio_backend",
                default_value="elevator",
                description="Real-robot interface: elevator, super, super_global, or humanoid.",
            ),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument(
                "planning_horizon",
                default_value="5.5",
                description="Local planning lookahead distance in meters.",
            ),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Closed-loop velocity output for real hardware.",
            ),
            DeclareLaunchArgument(
                "humanoid_pose_topic",
                default_value="/baselink2map_kalman",
                description="Global pose for the Fast-LIO Humanoid backend.",
            ),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument(
                "planner_config",
                default_value="",
                description="Optional planner parameter YAML; defaults to the package config.",
            ),
            DeclareLaunchArgument(
                "controller_config",
                default_value="",
                description="Optional controller parameter YAML; defaults to the package config.",
            ),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Open the unified Scan Planner and DDDMR RViz view.",
            ),
            DeclareLaunchArgument(
                "rviz_fixed_frame",
                default_value="map",
                description="RViz fixed frame.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
