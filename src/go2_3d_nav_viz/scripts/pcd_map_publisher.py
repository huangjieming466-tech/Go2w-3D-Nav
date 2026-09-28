#!/usr/bin/env python3
"""Publish a PCD as PointCloud2 for optional RViz comparison."""

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


class PcdMapPublisher(Node):
    def __init__(self):
        super().__init__('pcd_map_publisher')
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/raw_pcd_map')
        self.declare_parameter('voxel_size', 0.10)

        pcd_file = self.get_parameter('pcd_file').value
        if not pcd_file:
            raise ValueError('pcd_file is required')
        frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        voxel_size = float(self.get_parameter('voxel_size').value)

        cloud = o3d.io.read_point_cloud(pcd_file)
        if cloud.is_empty():
            raise RuntimeError(f'PCD contains no points: {pcd_file}')
        if voxel_size > 0.0:
            cloud = cloud.voxel_down_sample(voxel_size)
        points = np.asarray(cloud.points, dtype=np.float32)
        self.get_logger().info(
            f'Loaded {points.shape[0]} points from {pcd_file} '
            f'(voxel_size={voxel_size:g})'
        )

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.msg = pc2.create_cloud_xyz32(
            Header(frame_id=frame_id), points
        )
        self.timer = self.create_timer(1.0, self.publish_cloud)
        self.publish_cloud()

    def publish_cloud(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcdMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
