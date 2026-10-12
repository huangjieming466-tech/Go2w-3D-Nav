#!/usr/bin/env python3
"""Assert that a predicted collision blocks raw cmd_vel forwarding."""

import sys

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    messages = {topic: [] for topic in types}
    classes = {topic: get_message(msg_type) for topic, msg_type in types.items()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        messages[topic].append(rclpy.serialization.deserialize_message(data, classes[topic]))

    collision_true = sum(1 for message in messages.get("/planning/cmd_collision", []) if message.data)
    raw_forward = max((abs(message.linear.x) for message in messages.get("/cmd_vel_raw", [])), default=0.0)
    safe_forward = max((abs(message.linear.x) for message in messages.get("/quad_0/cmd_vel", [])), default=0.0)
    print(f"collision_true={collision_true} raw_max_vx={raw_forward:.3f} safe_max_vx={safe_forward:.3f}")
    if collision_true == 0 or raw_forward < 0.19 or safe_forward > 1e-3:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
