"""PCD-backed DDMR global planner ROS 2 interface."""

import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


DEFAULT_DDDMR_ROOT = str(
    Path.home() / 'Go2w_3D_Nav/src/dddmr_python_3d_global_planner/dddmr_py'
)


class DddmrGlobalPlannerNode(Node):
    """Expose dddmr_py PCD planning through standard ROS 2 messages."""

    def __init__(self):
        super().__init__('dddmr_global_planner')
        self.declare_parameter('dddmr_python_root', DEFAULT_DDDMR_ROOT)
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('use_odom_start', False)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)
        self.declare_parameter('odom_topic', '/LIO/odom_vehicle')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/move_base_simple/goal')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('initial_path_topic', '/initial_path')

        root = Path(self.get_parameter('dddmr_python_root').value).expanduser()
        if not (root / 'dddmr_py').is_dir():
            raise RuntimeError(
                f'dddmr_python_root is not a dddmr_py project root: {root}'
            )
        sys.path.insert(0, str(root))
        from dddmr_py import GlobalPlanner3D, PlannerConfig

        self._planner_type = GlobalPlanner3D
        self._config_type = PlannerConfig
        self.frame_id = self.get_parameter('frame_id').value
        self.use_odom_start = self.get_parameter('use_odom_start').value
        self.start = np.array([
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_z').value,
        ], dtype=float)
        self.odom_start = None

        config_values = {}
        for field in PlannerConfig.__dataclass_fields__.values():
            name = f'planner.{field.name}'
            self.declare_parameter(name, field.default)
            config_values[field.name] = self.get_parameter(name).value
        config = PlannerConfig(**config_values)

        pcd_file = self.get_parameter('pcd_file').value
        if not pcd_file:
            raise RuntimeError('pcd_file is required')
        pcd_path = Path(pcd_file).expanduser().resolve()
        if not pcd_path.is_file():
            raise RuntimeError(f'PCD does not exist: {pcd_path}')

        self.get_logger().info(f'Loading static PCD: {pcd_path}')
        self.planner = GlobalPlanner3D.from_pcd(str(pcd_path), config, verbose=True)
        self.get_logger().info(
            f'DDMR map ready: nodes={len(self.planner.ground.nodes)}, '
            f'free={int((~self.planner.ground.lethal).sum())}, '
            f'obstacles={len(self.planner.ground.obstacles)}'
        )

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.raw_pub = self.create_publisher(PointCloud2, '/dddmr/map_points', latched_qos)
        self.free_pub = self.create_publisher(PointCloud2, '/dddmr/traversable', latched_qos)
        self.lethal_pub = self.create_publisher(PointCloud2, '/dddmr/lethal', latched_qos)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/dddmr/obstacles', latched_qos)
        self.global_path_pub = self.create_publisher(
            RosPath, self.get_parameter('global_path_topic').value, latched_qos
        )
        self.initial_path_pub = self.create_publisher(
            RosPath, self.get_parameter('initial_path_topic').value, latched_qos
        )
        self.last_path = None

        self.create_subscription(
            PointStamped, self.get_parameter('clicked_point_topic').value,
            self.clicked_point_callback, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self.goal_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value,
            self.initialpose_callback, 10
        )
        if self.use_odom_start:
            self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value,
                self.odom_callback, 20
            )

        self.publish_map()

    def header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)

    def cloud_message(self, points):
        points = np.asarray(points, dtype=np.float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # The PCD export has no original LiDAR reflectance field.  Use
            # map-frame range as a subdued scan-like display intensity.
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        if len(points) == 0:
            return point_cloud2.create_cloud(self.header(), fields, [])
        ranges = np.linalg.norm(points, axis=1)
        records = [
            (float(point[0]), float(point[1]), float(point[2]), float(distance))
            for point, distance in zip(points, ranges)
        ]
        return point_cloud2.create_cloud(self.header(), fields, records)

    def publish_map(self):
        ground = self.planner.ground
        self.raw_pub.publish(self.cloud_message(self.planner.cloud))
        self.free_pub.publish(self.cloud_message(ground.nodes[~ground.lethal]))
        self.lethal_pub.publish(self.cloud_message(ground.nodes[ground.lethal]))
        self.obstacle_pub.publish(self.cloud_message(ground.obstacles))

    def odom_callback(self, message):
        pose = message.pose.pose.position
        self.odom_start = np.array([pose.x, pose.y, pose.z], dtype=float)

    def initialpose_callback(self, message):
        if self.use_odom_start:
            self.get_logger().info(
                'Ignoring /initialpose for planner start while use_odom_start=true'
            )
            return
        pose = message.pose.pose.position
        self.start = np.array([pose.x, pose.y, pose.z], dtype=float)
        self.get_logger().info(f'Planning start set to {self.start.tolist()} from /initialpose')

    def current_start(self):
        if self.use_odom_start:
            return self.odom_start
        return self.start

    def clicked_point_callback(self, message):
        self.plan_to(np.array([
            message.point.x, message.point.y, message.point.z,
        ], dtype=float))

    def goal_callback(self, message):
        self.plan_to(np.array([
            message.pose.position.x, message.pose.position.y, message.pose.position.z,
        ], dtype=float))

    def plan_to(self, goal):
        start = self.current_start()
        if start is None:
            self.get_logger().warning(
                'Cannot plan: waiting for odometry on the configured odom_topic'
            )
            return
        self.get_logger().info(
            f'Planning from {start.tolist()} to {goal.tolist()}'
        )
        path = self.planner.make_plan(start, goal)
        if not path.success:
            self.get_logger().warning(f'DDMR plan failed: {path.message}')
            return
        ros_path = RosPath(header=self.header())
        quaternions = path.quaternions()
        for point, quat in zip(path.points, quaternions):
            pose = PoseStamped(header=ros_path.header)
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            ros_path.poses.append(pose)
        self.last_path = ros_path
        self.publish_path(ros_path)
        self.get_logger().info(
            f'DDMR path published: poses={len(ros_path.poses)}, '
            f'length={path.length:.2f}m, climb={path.climb:.2f}m, '
            f'time={path.planning_time * 1000.0:.0f}ms'
        )

    def publish_path(self, path):
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in path.poses:
            pose.header = path.header
        self.global_path_pub.publish(path)
        self.initial_path_pub.publish(path)


def main():
    rclpy.init()
    node = DddmrGlobalPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
