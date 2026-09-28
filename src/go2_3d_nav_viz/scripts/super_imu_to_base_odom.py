#!/usr/bin/env python3
"""Convert Super-LIO's IMU pose odometry to the Go2W base_link pose."""

import copy

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def rotation_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


class SuperImuToBaseOdom(Node):
    def __init__(self):
        super().__init__("super_imu_to_base_odom")
        self.declare_parameter("input_topic", "/lio/odom")
        self.declare_parameter("output_topic", "/lio/base_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("base_to_imu_x", -0.02557)
        self.declare_parameter("base_to_imu_y", 0.0)
        self.declare_parameter("base_to_imu_z", 0.04232)

        base_to_imu = np.array([
            float(self.get_parameter("base_to_imu_x").value),
            float(self.get_parameter("base_to_imu_y").value),
            float(self.get_parameter("base_to_imu_z").value),
        ])
        # The supplied extrinsic has identity rotation, so T_imu_base has the
        # negated translation in IMU coordinates.
        self.imu_to_base = -base_to_imu
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self.odom_callback,
            qos_profile_sensor_data,
        )

    def odom_callback(self, msg):
        output = copy.deepcopy(msg)
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        position += rotation_from_quaternion(msg.pose.pose.orientation) @ self.imu_to_base
        output.pose.pose.position.x = float(position[0])
        output.pose.pose.position.y = float(position[1])
        output.pose.pose.position.z = float(position[2])
        output.child_frame_id = self.base_frame
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SuperImuToBaseOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
