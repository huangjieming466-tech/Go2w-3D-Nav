#!/usr/bin/env python3
"""Transform Super-LIO's world-frame cloud into Open3D Loc's map frame."""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def rotation_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


class SuperMapCloudBridge(Node):
    def __init__(self):
        super().__init__('super_map_cloud_bridge')
        self.declare_parameter('input_cloud_topic', '/lio/cloud_world')
        self.declare_parameter('map_odom_topic', '/odom2map_kalman')
        self.declare_parameter('output_cloud_topic', '/lio/cloud_map')
        self.declare_parameter('map_frame', 'map')

        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.rotation = None
        self.translation = None
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_cloud_topic').value), reliable
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('map_odom_topic').value), self.odom_callback, reliable
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter('input_cloud_topic').value), self.cloud_callback, reliable
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.rotation = rotation_from_quaternion(pose.orientation)
        self.translation = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float32
        )

    def cloud_callback(self, msg):
        if self.rotation is None:
            self.get_logger().warn('Waiting for /odom2map_kalman before transforming cloud.')
            return
        raw_points = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        if raw_points.size == 0:
            return
        points = np.column_stack((
            raw_points['x'], raw_points['y'], raw_points['z']
        )).astype(np.float32, copy=False)
        points = points @ self.rotation.T + self.translation
        header = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        self.publisher.publish(pc2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = SuperMapCloudBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
