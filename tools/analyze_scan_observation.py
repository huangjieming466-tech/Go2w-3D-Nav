#!/usr/bin/env python3
"""Summarize a recorded SCAN observation without replaying it."""

import math
import sys

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def point_text(point):
    return f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"


def main():
    bag_path = sys.argv[1]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(name) for topic, name in types.items()}
    records = {topic: [] for topic in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        records[topic].append((stamp, rclpy.serialization.deserialize_message(data, msg_types[topic])))

    for topic in ("/initial_path", "/initial_path_world"):
        if records.get(topic):
            _, path = records[topic][-1]
            print(f"{topic}: {len(path.poses)} points: " + "; ".join(
                point_text(pose.pose.position) for pose in path.poses
            ))

    odom = records.get("/lio/base_odom", [])
    if odom:
        _, first = odom[0]
        _, last = odom[-1]
        dx = last.pose.pose.position.x - first.pose.pose.position.x
        dy = last.pose.pose.position.y - first.pose.pose.position.y
        yaw_delta = yaw_from_quaternion(last.pose.pose.orientation) - yaw_from_quaternion(first.pose.pose.orientation)
        print("ODOM: first=" + point_text(first.pose.pose.position) +
              " last=" + point_text(last.pose.pose.position) +
              f" displacement_xy={math.hypot(dx, dy):.2f}m yaw_delta={yaw_delta:.2f}rad samples={len(odom)}")

    splines = records.get("/planning/bspline", [])
    print(f"BSPLINES: {len(splines)}")
    for index, (stamp, spline) in enumerate(splines):
        if not spline.pos_pts or not spline.knots:
            continue
        duration = spline.knots[len(spline.pos_pts)] - spline.knots[spline.order]
        first = spline.pos_pts[0]
        last = spline.pos_pts[-1]
        odom_at_plan = None
        if odom:
            odom_at_plan = min(odom, key=lambda sample: abs(sample[0] - stamp))[1]
        alignment = ""
        if odom_at_plan is not None:
            robot = odom_at_plan.pose.pose.position
            dx, dy = first.x - robot.x, first.y - robot.y
            start_error = math.hypot(dx, dy)
            if len(spline.pos_pts) > 1:
                direction = math.atan2(
                    spline.pos_pts[1].y - first.y,
                    spline.pos_pts[1].x - first.x,
                )
                yaw_error = math.atan2(
                    math.sin(direction - yaw_from_quaternion(odom_at_plan.pose.pose.orientation)),
                    math.cos(direction - yaw_from_quaternion(odom_at_plan.pose.pose.orientation)),
                )
                alignment = f" odom={point_text(robot)} start_err={start_error:.2f}m initial_yaw_err={yaw_error:.2f}rad"
        print(f"  {index + 1:02d} t={stamp / 1e9:.3f} id={spline.traj_id} duration={duration:.2f}s "
              f"ctrl={len(spline.pos_pts)} first={point_text(first)} last={point_text(last)}{alignment}")

    frozen = records.get("/planning/go2_execution_frozen", [])
    if frozen:
        frozen_count = sum(1 for _, msg in frozen if msg.data)
        print(f"EXECUTION_FROZEN: {frozen_count}/{len(frozen)} samples true")
        intervals = []
        begin = None
        for stamp, msg in frozen:
            if msg.data and begin is None:
                begin = stamp
            elif not msg.data and begin is not None:
                intervals.append((begin, stamp))
                begin = None
        if begin is not None:
            intervals.append((begin, frozen[-1][0]))
        longest = sorted(intervals, key=lambda span: span[1] - span[0], reverse=True)[:8]
        print("FROZEN_INTERVALS longest: " + "; ".join(
            f"{(end - begin) / 1e9:.2f}s@{begin / 1e9:.3f}" for begin, end in longest
        ))

    markers = records.get("/goal_point", [])
    if markers:
        _, marker = markers[-1]
        print("GOAL_MARKER: " + point_text(marker.pose.position) +
              f" type={marker.type} points={len(marker.points)}")

    reference_markers = records.get("/reference_path_segment", [])
    if reference_markers:
        latest_sphere = next(
            (marker for _, marker in reversed(reference_markers) if marker.points and marker.type == 7),
            None,
        )
        if latest_sphere is not None:
            print("REFERENCE_SEGMENT latest: " + "; ".join(
                point_text(point) for point in latest_sphere.points
            ))

    displays = records.get("/planning/data_display", [])
    if displays:
        _, display = displays[-1]
        print(f"DATA_DISPLAY last: a={display.a:.3f} b={display.b:.3f} c={display.c:.3f} d={display.d:.3f} e={display.e:.3f}")


if __name__ == "__main__":
    main()
