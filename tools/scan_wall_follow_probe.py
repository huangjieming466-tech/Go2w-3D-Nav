#!/usr/bin/env python3
"""Measure a SCAN B-spline's XY/Z deviation from its Mode 3 reference path."""

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scan_planner_msgs.msg import Bspline
from scipy.interpolate import BSpline


class WallFollowProbe(Node):
    def __init__(self, args):
        super().__init__("scan_wall_follow_probe")
        self.args = args
        self.global_path = None
        self.odom = None
        self.metrics = []
        self.goal_sent = False
        self.started_at = time.monotonic()
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.create_subscription(Path, "/initial_path", self.path_callback, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.bspline_callback, 10)
        self.create_subscription(Odometry, "/quad_0/body_pose", self.odom_callback, 20)
        self.create_timer(0.2, self.tick)

    def path_callback(self, message):
        self.global_path = np.asarray(
            [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
             for pose in message.poses],
            dtype=float,
        )
        if len(self.global_path) > 1:
            length = np.linalg.norm(np.diff(self.global_path, axis=0), axis=1).sum()
            print(f"GLOBAL poses={len(self.global_path)} length={length:.3f}", flush=True)

    def odom_callback(self, message):
        position = message.pose.pose.position
        self.odom = np.asarray([position.x, position.y, position.z], dtype=float)

    def bspline_callback(self, message):
        if self.global_path is None or len(message.pos_pts) < message.order + 1:
            return

        control_points = np.asarray([[p.x, p.y, p.z] for p in message.pos_pts], dtype=float)
        knots = np.asarray(message.knots, dtype=float)
        degree = int(message.order)
        try:
            samples = np.linspace(knots[degree], knots[len(control_points)], 150)
            local_path = np.column_stack(
                [BSpline(knots, control_points[:, axis], degree)(samples) for axis in range(3)]
            )
        except ValueError as error:
            print(f"BSPLINE_ERROR {error}", flush=True)
            return

        xy_delta = local_path[:, None, :2] - self.global_path[None, :, :2]
        xy_nearest = np.linalg.norm(xy_delta, axis=2).min(axis=1)
        xyz_delta = local_path[:, None, :] - self.global_path[None, :, :]
        xyz_nearest = np.linalg.norm(xyz_delta, axis=2).min(axis=1)
        path_length = np.linalg.norm(np.diff(local_path, axis=0), axis=1).sum()
        metric = (
            float(xy_nearest.max()),
            float(xy_nearest.mean()),
            float(xyz_nearest.max()),
            float(path_length),
        )
        self.metrics.append(metric)
        print(
            f"LOCAL index={len(self.metrics)} max_xy={metric[0]:.3f} "
            f"mean_xy={metric[1]:.3f} max_xyz={metric[2]:.3f} length={metric[3]:.3f}",
            flush=True,
        )

    def tick(self):
        elapsed = time.monotonic() - self.started_at
        if self.odom is not None and not self.goal_sent and elapsed > 2.0:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.args.frame_id
            goal.pose.position.x = self.args.goal_x
            goal.pose.position.y = self.args.goal_y
            goal.pose.position.z = self.args.goal_z
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True
            print(f"GOAL sent odom={self.odom.tolist()}", flush=True)

        if elapsed >= self.args.duration:
            if self.metrics:
                data = np.asarray(self.metrics)
                print(
                    f"SUMMARY trajectories={len(data)} worst_max_xy={data[:, 0].max():.3f} "
                    f"mean_max_xy={data[:, 0].mean():.3f} "
                    f"worst_max_xyz={data[:, 2].max():.3f}",
                    flush=True,
                )
            else:
                print("SUMMARY trajectories=0", flush=True)
            raise SystemExit(0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=-2.5007)
    parser.add_argument("--goal-y", type=float, default=20.7320)
    parser.add_argument("--goal-z", type=float, default=3.6449)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--duration", type=float, default=75.0)
    return parser.parse_args()


def main():
    rclpy.init()
    node = WallFollowProbe(parse_args())
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
