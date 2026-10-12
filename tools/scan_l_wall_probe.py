#!/usr/bin/env python3
"""Assert that DDDMR, SCAN trajectories, and simulated motion avoid an L wall."""

import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scan_planner_msgs.msg import Bspline
from scipy.interpolate import BSpline
from std_msgs.msg import Bool


def distance_to_aabb(point, lower, upper):
    delta = np.maximum(np.maximum(lower - point, 0.0), point - upper)
    return float(np.linalg.norm(delta))


def wall_clearance(point):
    # Wall thickness is 0.10 m; clearance is point-to-wall-surface distance.
    walls = (
        (np.array([4.95, -0.05]), np.array([5.05, 6.05])),
        (np.array([4.95, 5.95]), np.array([10.05, 6.05])),
    )
    return min(distance_to_aabb(point[:2], lower, upper) for lower, upper in walls)


class LWallProbe(Node):
    def __init__(self, args):
        super().__init__("scan_l_wall_probe")
        self.args = args
        self.goal_sent = False
        self.reference_sent = False
        self.started_at = time.monotonic()
        self.global_path = None
        self.local_results = []
        self.odom_results = []
        self.final_odom = None
        self.odom_yaw = 0.0
        self.command_integrated_pose = None
        self.command_integrated_yaw = 0.0
        self.last_filtered_command_time = None
        self.command_integrated_clearances = []
        self.command_integrated_collision = None
        self.raw_cmd_wall_risks = []
        self.filtered_cmd_wall_risks = []
        self.cmd_collision_count = 0
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.path_pub = self.create_publisher(Path, "/initial_path", 10)
        self.create_subscription(Path, "/initial_path", self.path_callback, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.bspline_callback, 10)
        self.create_subscription(Odometry, "/quad_0/body_pose", self.odom_callback, 20)
        self.create_subscription(Twist, "/quad_0/cmd_vel", self.cmd_callback, 20)
        self.create_subscription(Twist, "/cmd_vel_raw", self.raw_cmd_callback, 20)
        self.create_subscription(Bool, "/planning/cmd_collision", self.collision_callback, 20)
        self.create_timer(0.1, self.tick)

    def path_callback(self, message):
        path = np.asarray([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                           for p in message.poses], dtype=float)
        if len(path) < 2:
            return
        self.global_path = path
        clearance = min(wall_clearance(point) for point in path)
        print(f"GLOBAL poses={len(path)} min_wall_clearance={clearance:.3f}", flush=True)
        print("GLOBAL points=" + "; ".join(
            f"({point[0]:.2f},{point[1]:.2f},{point[2]:.2f})" for point in path
        ), flush=True)

    def bspline_callback(self, message):
        if len(message.pos_pts) < message.order + 1:
            return
        control = np.asarray([[p.x, p.y, p.z] for p in message.pos_pts], dtype=float)
        knots = np.asarray(message.knots, dtype=float)
        try:
            times = np.linspace(knots[message.order], knots[len(control)], 200)
            points = np.column_stack([BSpline(knots, control[:, axis], message.order)(times)
                                      for axis in range(3)])
        except ValueError as error:
            print(f"BSPLINE_ERROR {error}", flush=True)
            return
        clearance = min(wall_clearance(point) for point in points)
        self.local_results.append(clearance)
        print(f"LOCAL index={len(self.local_results)} min_wall_clearance={clearance:.3f}", flush=True)

    def odom_callback(self, message):
        point = np.array([message.pose.pose.position.x, message.pose.pose.position.y,
                          message.pose.pose.position.z])
        self.final_odom = point
        orientation = message.pose.pose.orientation
        self.odom_yaw = np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        if self.command_integrated_pose is None:
            self.command_integrated_pose = point[:2].copy()
            self.command_integrated_yaw = self.odom_yaw
        self.odom_results.append(wall_clearance(point))

    def collision_callback(self, message):
        self.cmd_collision_count += int(message.data)

    def raw_cmd_callback(self, message):
        self.inspect_cmd(message, self.raw_cmd_wall_risks, "RAW")

    def cmd_callback(self, message):
        now = time.monotonic()
        if self.command_integrated_pose is not None:
            if self.last_filtered_command_time is not None:
                dt = now - self.last_filtered_command_time
                if 0.0 < dt <= 0.2:
                    c, s = np.cos(self.command_integrated_yaw), np.sin(self.command_integrated_yaw)
                    vx = c * message.linear.x - s * message.linear.y
                    vy = s * message.linear.x + c * message.linear.y
                    self.command_integrated_pose += np.array([vx, vy]) * dt
                    self.command_integrated_yaw += message.angular.z * dt
                    clearance = wall_clearance(self.command_integrated_pose)
                    self.command_integrated_clearances.append(clearance)
                    if clearance < self.args.radius and self.command_integrated_collision is None:
                        self.command_integrated_collision = (
                            now - self.started_at, self.command_integrated_pose.copy(), clearance
                        )
                        print("COMMAND_INTEGRATED_COLLISION "
                              f"t={self.command_integrated_collision[0]:.2f} "
                              f"pose=({self.command_integrated_pose[0]:.3f},"
                              f"{self.command_integrated_pose[1]:.3f}) "
                              f"clearance={clearance:.3f}", flush=True)
            self.last_filtered_command_time = now
        self.inspect_cmd(message, self.filtered_cmd_wall_risks, "FILTERED")

    def inspect_cmd(self, message, risks, label):
        if not self.args.detect_cmd_wall or self.final_odom is None:
            return
        c, s = np.cos(self.odom_yaw), np.sin(self.odom_yaw)
        velocity = np.array([
            c * message.linear.x - s * message.linear.y,
            s * message.linear.x + c * message.linear.y,
        ])
        speed = float(np.linalg.norm(velocity))
        if speed < 0.03:
            return
        # A 2 m command ray from the actual frozen odometry reaches a wall if
        # the time-driven controller is trying to chase a future point through it.
        distances = np.linspace(0.05, 2.0, 40)
        intersects_wall = any(
            wall_clearance(self.final_odom[:2] + velocity / speed * distance) < self.args.radius
            for distance in distances
        )
        if intersects_wall:
            risk = (time.monotonic() - self.started_at, self.final_odom.copy(), velocity)
            if not risks or risk[0] - risks[-1][0] > 0.5:
                risks.append(risk)
                print(f"{label}_CMD_WALL_RISK "
                      f"t={risk[0]:.2f} odom=({risk[1][0]:.2f},{risk[1][1]:.2f}) "
                      f"world_vel=({risk[2][0]:.3f},{risk[2][1]:.3f})", flush=True)

    def tick(self):
        elapsed = time.monotonic() - self.started_at
        if self.final_odom is not None and self.args.publish_reference and not self.reference_sent and elapsed > 1.0:
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "world"
            points = ((3.5, 4.8, 0.3), (3.5, 6.5, 0.3),
                      (4.5, 6.5, 0.3), (8.0, 8.0, 0.3)) if self.args.corner_start else (
                      (2.0, 2.0, 0.3), (3.5, 6.5, 0.3),
                      (4.5, 6.5, 0.3), (8.0, 8.0, 0.3))
            for x, y, z in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)
            self.reference_sent = True
            print("REFERENCE published with verified L-wall clearance and blocked direct chord", flush=True)
        if self.final_odom is not None and not self.goal_sent and elapsed > 2.0:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "world"
            goal.pose.position.x = self.args.goal_x
            goal.pose.position.y = self.args.goal_y
            goal.pose.position.z = self.args.goal_z
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True
            print(f"GOAL start={self.final_odom.tolist()} target={[self.args.goal_x, self.args.goal_y, self.args.goal_z]}", flush=True)

        if elapsed < self.args.duration:
            return
        radius = self.args.radius
        global_clearance = min(wall_clearance(point) for point in self.global_path) if self.global_path is not None else float("nan")
        local_clearance = min(self.local_results) if self.local_results else float("nan")
        odom_clearance = min(self.odom_results) if self.odom_results else float("nan")
        command_integrated_clearance = (
            min(self.command_integrated_clearances)
            if self.command_integrated_clearances else float("nan")
        )
        goal_error = (float(np.linalg.norm(self.final_odom[:2] - np.array([self.args.goal_x, self.args.goal_y])))
                      if self.final_odom is not None else float("nan"))
        print("SUMMARY "
              f"global_clearance={global_clearance:.3f} local_clearance={local_clearance:.3f} "
              f"odom_clearance={odom_clearance:.3f} robot_radius={radius:.3f} "
              f"command_integrated_clearance={command_integrated_clearance:.3f} "
              f"command_integrated_collision={self.command_integrated_collision is not None} "
              f"goal_error_xy={goal_error:.3f} local_trajectories={len(self.local_results)} "
              f"raw_cmd_wall_risks={len(self.raw_cmd_wall_risks)} "
              f"filtered_cmd_wall_risks={len(self.filtered_cmd_wall_risks)} "
              f"safety_collision_msgs={self.cmd_collision_count}", flush=True)
        if (not np.isfinite(global_clearance) or not np.isfinite(local_clearance) or
                not np.isfinite(odom_clearance) or not np.isfinite(command_integrated_clearance) or
                global_clearance < radius or
                local_clearance < radius or odom_clearance < radius or
                command_integrated_clearance < radius or
                self.command_integrated_collision is not None):
            raise SystemExit(2)
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=8.0)
    parser.add_argument("--goal-y", type=float, default=8.0)
    parser.add_argument("--goal-z", type=float, default=0.30)
    parser.add_argument("--radius", type=float, default=0.22)
    parser.add_argument("--duration", type=float, default=70.0)
    parser.add_argument("--publish-reference", action="store_true")
    parser.add_argument("--corner-start", action="store_true")
    parser.add_argument("--detect-cmd-wall", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = LWallProbe(args)
    try:
        rclpy.spin(node)
    except SystemExit as error:
        exit_code = int(error.code)
    else:
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
