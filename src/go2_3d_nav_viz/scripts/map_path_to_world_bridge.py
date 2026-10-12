#!/usr/bin/env python3
"""Convert a map-frame global path into Super-LIO's local world frame."""

import copy
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def pose_matrix(pose):
    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-12:
        x = y = z = 0.0
        w = 1.0
    else:
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def set_pose_from_matrix(pose, matrix):
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)


class MapPathToWorldBridge(Node):
    def __init__(self):
        super().__init__("map_path_to_world_bridge")
        self.declare_parameter("input_path_topic", "/initial_path")
        self.declare_parameter("map_world_topic", "/odom2map_kalman")
        self.declare_parameter("output_path_topic", "/initial_path_world")
        self.declare_parameter("world_frame", "world")

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.map_world = None
        self.pending_path = None
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter("output_path_topic").value), path_qos
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("map_world_topic").value),
            self.map_world_callback,
            odom_qos,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("input_path_topic").value),
            self.path_callback,
            path_qos,
        )

    def map_world_callback(self, msg):
        self.map_world = pose_matrix(msg.pose.pose)
        if self.pending_path is not None:
            path = self.pending_path
            self.pending_path = None
            self.publish_transformed(path)

    def path_callback(self, msg):
        if not msg.poses:
            return
        if self.map_world is None:
            self.pending_path = msg
            self.get_logger().warn("Waiting for /odom2map_kalman before converting global path")
            return
        self.publish_transformed(msg)

    def publish_transformed(self, msg):
        world_map = np.linalg.inv(self.map_world)
        output = Path()
        output.header = copy.deepcopy(msg.header)
        output.header.frame_id = self.world_frame
        output.poses = []
        for source in msg.poses:
            target = copy.deepcopy(source)
            set_pose_from_matrix(target.pose, world_map @ pose_matrix(source.pose))
            target.header = copy.deepcopy(source.header)
            target.header.frame_id = self.world_frame
            output.poses.append(target)
        self.publisher.publish(output)
        self.get_logger().info(
            f"Converted {len(output.poses)} path poses from map to {self.world_frame}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapPathToWorldBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
