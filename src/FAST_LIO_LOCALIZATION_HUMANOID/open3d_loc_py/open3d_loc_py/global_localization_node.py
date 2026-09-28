import math
import threading
from pathlib import Path

import numpy as np
import open3d as o3d

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def pose_matrix(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        w = 1.0
        x = y = z = 0.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return t


def matrix_pose(t, pose):
    pose.position.x = float(t[0, 3])
    pose.position.y = float(t[1, 3])
    pose.position.z = float(t[2, 3])
    r = t[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)


class Open3DLocalizer(Node):
    def __init__(self):
        super().__init__("open3d_global_localization")
        self.declare_parameter("map_file", "")
        self.declare_parameter("cloud_topic", "/cloud_registered_body_1")
        self.declare_parameter("odom_topic", "/Odometry_loc")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("voxel_size", 0.15)
        self.declare_parameter("scan_voxel_size", 0.10)
        self.declare_parameter("registration_distance", 1.5)
        self.declare_parameter("update_period", 0.10)

        map_file = str(self.get_parameter("map_file").value)
        map_path = Path(map_file)
        if map_path.is_dir():
            candidates = sorted(map_path.glob("*.pcd"))
            if not candidates:
                raise RuntimeError("map directory has no PCD file: " + map_file)
            preferred = [
                path for path in candidates
                if "oriented_normals" in path.name
            ]
            map_path = preferred[0] if preferred else candidates[0]
            map_file = str(map_path)
            self.get_logger().info("map_file is a directory; selected %s", map_file)
        if not map_path.is_file():
            raise RuntimeError("map_file does not exist: " + map_file)

        map_cloud = o3d.io.read_point_cloud(map_file)
        if map_cloud.is_empty():
            raise RuntimeError("map point cloud is empty: " + map_file)
        self.map_cloud = map_cloud.voxel_down_sample(
            float(self.get_parameter("voxel_size").value)
        )
        self.map_tree = o3d.geometry.KDTreeFlann(self.map_cloud)
        self.get_logger().info(
            "loaded map: %d points from %s",
            len(self.map_cloud.points),
            map_file,
        )

        self.lock = threading.Lock()
        self.latest_cloud = None
        self.latest_odom = None
        self.map_odom = np.eye(4, dtype=np.float64)
        self.initialized = False
        self.last_fitness = 0.0
        self.last_update_ns = 0

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.create_subscription(
            PoseStamped, "/initialpose", self.initialpose_callback, 10
        )

        self.pose_pub = self.create_publisher(Odometry, "/baselink2map_kalman", 10)
        self.pose_raw_pub = self.create_publisher(Odometry, "/baselink2map", 10)
        self.motion_pub = self.create_publisher(Odometry, "/motionlink2map", 10)
        self.odom_map_pub = self.create_publisher(Odometry, "/odom2map_kalman", 10)
        self.localization_pub = self.create_publisher(
            PoseStamped, "/localization_3d", 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, "/localization_3d_confidence", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(
            float(self.get_parameter("update_period").value),
            self.update,
        )

    def cloud_callback(self, msg):
        fields = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(fields):
            return
        points = []
        for item in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            points.append((float(item[0]), float(item[1]), float(item[2])))
        if len(points) < 30:
            return
        with self.lock:
            self.latest_cloud = np.asarray(points, dtype=np.float64)

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg

    def initialpose_callback(self, msg):
        with self.lock:
            if self.latest_odom is None:
                self.get_logger().warn("initialpose received before /Odometry_loc")
                return
            map_body = pose_matrix(msg.pose)
            odom_body = pose_matrix(self.latest_odom.pose.pose)
            self.map_odom = map_body @ np.linalg.inv(odom_body)
            self.initialized = True
        self.get_logger().info("updated map->odom from /initialpose")

    def update(self):
        with self.lock:
            cloud = None if self.latest_cloud is None else self.latest_cloud.copy()
            odom = self.latest_odom
            map_odom = self.map_odom.copy()
        if cloud is None or odom is None:
            return

        odom_body = pose_matrix(odom.pose.pose)
        predicted = map_odom @ odom_body
        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(cloud)
        scan = scan.voxel_down_sample(
            float(self.get_parameter("scan_voxel_size").value)
        )
        if len(scan.points) < 30:
            return

        result = o3d.pipelines.registration.registration_icp(
            scan,
            self.map_cloud,
            float(self.get_parameter("registration_distance").value),
            predicted,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-4,
                relative_rmse=1e-4,
                max_iteration=20,
            ),
        )
        if result.fitness >= 0.05:
            map_odom = result.transformation @ np.linalg.inv(odom_body)
            with self.lock:
                self.map_odom = map_odom
                self.initialized = True
        self.last_fitness = float(result.fitness)
        self.publish_outputs(odom.header, map_odom @ odom_body, map_odom)

    def publish_outputs(self, source_header, map_body, map_odom):
        stamp = source_header.stamp
        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)

        def make_odom(topic_frame):
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = map_frame
            msg.child_frame_id = topic_frame
            matrix_pose(map_body, msg.pose.pose)
            return msg

        pose_msg = make_odom(base_frame)
        self.pose_pub.publish(pose_msg)
        self.pose_raw_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = "odom"
        matrix_pose(map_odom, odom_msg.pose.pose)
        self.odom_map_pub.publish(odom_msg)
        self.motion_pub.publish(pose_msg)

        pose_stamped = PoseStamped()
        pose_stamped.header = pose_msg.header
        pose_stamped.pose = pose_msg.pose.pose
        self.localization_pub.publish(pose_stamped)
        confidence = Float32()
        confidence.data = self.last_fitness
        self.confidence_pub.publish(confidence)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = base_frame
        tf_msg.transform.translation.x = pose_msg.pose.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Open3DLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
