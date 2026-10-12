#!/usr/bin/env python3
"""Print the inflated GridMap points responsible for a SCAN cmd_vel veto."""

import math
import struct

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class CollisionWatcher(Node):
    def __init__(self):
        super().__init__("scan_collision_watcher")
        self.odom = None
        self.inflate = None
        self.last_report_ns = 0
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/lio/base_odom", self.on_odom, sensor_qos)
        self.create_subscription(PointCloud2, "/grid_map/occupancy_inflate", self.on_inflate, sensor_qos)
        self.create_subscription(Bool, "/planning/cmd_collision", self.on_collision, 10)

    def on_odom(self, msg):
        self.odom = msg

    def on_inflate(self, msg):
        self.inflate = msg

    def on_collision(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if not msg.data or self.odom is None or self.inflate is None or now_ns - self.last_report_ns < 1_000_000_000:
            return
        self.last_report_ns = now_ns
        stats = self.footprint_stats()
        self.get_logger().warn(
            "cmd_collision: pose=(%.2f, %.2f, %.2f), footprint inflated=%d, "
            "relative_xyz=[(%.2f, %.2f, %.2f)..(%.2f, %.2f, %.2f)]"
            % stats
        )

    def footprint_stats(self):
        cloud = self.inflate
        offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return (0.0, 0.0, 0.0, 0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        p = self.odom.pose.pose.position
        inside = []
        for i in range(cloud.width * cloud.height):
            base = i * cloud.point_step
            x, y, z = struct.unpack_from("<fff", cloud.data, base + offsets["x"])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            dx, dy, dz = x - p.x, y - p.y, z - p.z
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if (bx - 0.13) ** 2 + by ** 2 <= 0.22 ** 2 or (bx + 0.13) ** 2 + by ** 2 <= 0.22 ** 2:
                inside.append((bx, by, dz))
        if not inside:
            return (p.x, p.y, p.z, 0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)
        xs, ys, zs = zip(*inside)
        return (p.x, p.y, p.z, len(inside), min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def main():
    rclpy.init()
    node = CollisionWatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
