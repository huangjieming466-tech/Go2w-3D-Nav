#!/usr/bin/env python3
"""Create a signed-normal PCD from a Super-LIO map and its recording.

The final Super-LIO PCD stores XYZ and intensity only.  PCA recovers a plane
normal but not its sign.  When raw LiDAR scans are available, each map normal
is oriented toward the sensor poses that actually observed that map voxel.
This avoids associating a lower-floor ceiling with a geometrically closer
trajectory on the floor above.  Points without scan matches fall back to the
nearest-pose method.
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def read_trajectory(bag_path: Path, topic: str) -> np.ndarray:
    """Read positions from a nav_msgs/Odometry topic in a ROS 2 bag."""
    store = get_typestore(Stores.ROS2_HUMBLE)
    positions = []
    with Reader(bag_path) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            raise ValueError(f'Missing odometry topic {topic} in {bag_path}')
        for conn, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            pos = msg.pose.pose.position
            positions.append((pos.x, pos.y, pos.z))
    trajectory = np.asarray(positions, dtype=np.float64)
    if len(trajectory) < 2:
        raise ValueError(f'Only {len(trajectory)} poses found on {topic}')
    return trajectory


def read_poses(bag_path: Path, topic: str):
    """Read timestamped IMU poses from a nav_msgs/Odometry topic."""
    store = get_typestore(Stores.ROS2_HUMBLE)
    stamps = []
    positions = []
    quaternions = []
    with Reader(bag_path) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            raise ValueError(f'Missing odometry topic {topic} in {bag_path}')
        for conn, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            stamps.append(stamp_ns(msg))
            positions.append((pos.x, pos.y, pos.z))
            quaternions.append((quat.x, quat.y, quat.z, quat.w))
    if len(stamps) < 2:
        raise ValueError(f'Only {len(stamps)} poses found on {topic}')
    return (
        np.asarray(stamps, dtype=np.int64),
        np.asarray(positions, dtype=np.float64),
        Rotation.from_quat(np.asarray(quaternions, dtype=np.float64)).as_matrix(),
    )


def nearest_pose_indices(scan_stamps: np.ndarray, pose_stamps: np.ndarray) -> np.ndarray:
    right = np.searchsorted(pose_stamps, scan_stamps, side='left')
    right = np.clip(right, 0, len(pose_stamps) - 1)
    left = np.clip(right - 1, 0, len(pose_stamps) - 1)
    choose_left = np.abs(scan_stamps - pose_stamps[left]) <= np.abs(
        pose_stamps[right] - scan_stamps
    )
    return np.where(choose_left, left, right)


def pointcloud_xyz(msg) -> np.ndarray:
    """Return an XYZ view from a PointCloud2 message without copying fields."""
    offsets = {field.name: field.offset for field in msg.fields}
    missing = {'x', 'y', 'z'} - offsets.keys()
    if missing:
        raise ValueError(f'PointCloud2 is missing fields: {sorted(missing)}')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4'] * 3,
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': msg.point_step,
    })
    raw = np.asarray(msg.data, dtype=np.uint8)
    structured = np.frombuffer(raw, dtype=dtype, count=msg.width * msg.height)
    return np.column_stack((structured['x'], structured['y'], structured['z']))


def orient_from_scan_observations(
    cloud,
    bag_path: Path,
    odom_topic: str,
    lidar_topic: str,
    match_radius: float,
    point_stride: int,
    lidar_t_imu: np.ndarray,
    lidar_r_imu: np.ndarray,
):
    """Orient normals using votes from raw scans transformed by recorded poses."""
    pose_stamps, pose_positions, pose_rotations = read_poses(bag_path, odom_topic)
    points = np.asarray(cloud.points)
    normals = np.asarray(cloud.normals).copy()
    tree = cKDTree(points)
    votes = np.zeros(len(points), dtype=np.int32)
    observations = np.zeros(len(points), dtype=np.uint32)
    store = get_typestore(Stores.ROS2_HUMBLE)
    scan_count = 0
    matched_scans = 0
    max_time_error_ns = 200_000_000

    with Reader(bag_path) as reader:
        connections = [conn for conn in reader.connections if conn.topic == lidar_topic]
        if not connections:
            raise ValueError(f'Missing LiDAR topic {lidar_topic} in {bag_path}')
        for conn, _, raw in reader.messages(connections=connections):
            scan_count += 1
            msg = store.deserialize_cdr(raw, conn.msgtype)
            scan_stamp = stamp_ns(msg)
            pose_idx = nearest_pose_indices(
                np.asarray([scan_stamp], dtype=np.int64), pose_stamps
            )[0]
            if abs(int(scan_stamp) - int(pose_stamps[pose_idx])) > max_time_error_ns:
                continue

            xyz_lidar = pointcloud_xyz(msg)[::max(point_stride, 1)]
            finite = np.isfinite(xyz_lidar).all(axis=1)
            range2 = np.einsum('ij,ij->i', xyz_lidar, xyz_lidar)
            xyz_lidar = xyz_lidar[finite & (range2 >= 0.5 ** 2) & (range2 <= 60.0 ** 2)]
            if not len(xyz_lidar):
                continue

            rotation = pose_rotations[pose_idx]
            position = pose_positions[pose_idx]
            sensor_origin = rotation @ lidar_t_imu + position
            xyz_imu = xyz_lidar @ lidar_r_imu.T + lidar_t_imu
            xyz_world = xyz_imu @ rotation.T + position
            try:
                distance, map_idx = tree.query(xyz_world, k=1, workers=-1)
            except TypeError:
                distance, map_idx = tree.query(xyz_world, k=1)
            map_idx = np.unique(map_idx[distance <= match_radius])
            if not len(map_idx):
                continue
            matched_scans += 1
            toward_sensor = sensor_origin - points[map_idx]
            sign = np.where(
                np.einsum('ij,ij->i', normals[map_idx], toward_sensor) >= 0.0,
                1,
                -1,
            )
            np.add.at(votes, map_idx, sign)
            np.add.at(observations, map_idx, 1)

    voted = observations > 0
    flip = voted & (votes < 0)
    normals[flip] *= -1.0

    # Ties and points not recovered from raw scans use the legacy fallback.
    fallback = (~voted) | (votes == 0)
    distance, nearest = cKDTree(pose_positions).query(points[fallback], k=1, workers=-1)
    toward_sensor = pose_positions[nearest] - points[fallback]
    fallback_normals = normals[fallback]
    fallback_flip = np.einsum('ij,ij->i', fallback_normals, toward_sensor) < 0.0
    fallback_normals[fallback_flip] *= -1.0
    normals[fallback] = fallback_normals
    cloud.normals = o3d.utility.Vector3dVector(normals)

    print(f'scan_observations: scans={scan_count} matched_scans={matched_scans} '
          f'voted_points={int(voted.sum())}/{len(points)} '
          f'fallback_points={int(fallback.sum())}')
    if voted.any():
        print('votes_per_point: median='
              f'{np.median(observations[voted]):.1f} '
              f'p95={np.percentile(observations[voted], 95):.1f}')
    return normals, pose_positions, distance


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd', type=Path, help='Super-LIO final XYZ/I PCD')
    parser.add_argument('trajectory_bag', type=Path, help='rosbag containing /lio/odom')
    parser.add_argument('output', type=Path)
    parser.add_argument('--odom-topic', default='/lio/odom')
    parser.add_argument('--voxel', type=float, default=0.15)
    parser.add_argument('--normal-radius', type=float, default=0.30)
    parser.add_argument('--normal-max-nn', type=int, default=40)
    parser.add_argument('--lidar-topic', default='',
                        help='Use raw scan observations to orient normals')
    parser.add_argument('--match-radius', type=float, default=0.20)
    parser.add_argument('--scan-point-stride', type=int, default=3)
    parser.add_argument('--lidar-imu-translation', type=float, nargs=3,
                        default=(0.1870, 0.0, 0.0803))
    parser.add_argument('--lidar-imu-rotation', type=float, nargs=9,
                        default=(0.9743701, 0.0, 0.2249511,
                                 0.0, 1.0, 0.0,
                                 -0.2249511, 0.0, 0.9743701))
    args = parser.parse_args()

    trajectory = read_trajectory(args.trajectory_bag, args.odom_topic)
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    if not cloud.has_points():
        raise ValueError(f'No points read from {args.pcd}')
    raw_count = len(cloud.points)
    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(args.voxel)
    points = np.asarray(cloud.points)
    if len(points) < 10:
        raise RuntimeError('Too few points after voxel downsampling')

    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.normal_radius, max_nn=args.normal_max_nn
    ))
    normals = np.asarray(cloud.normals)
    if args.lidar_topic:
        normals, trajectory, distance = orient_from_scan_observations(
            cloud,
            args.trajectory_bag,
            args.odom_topic,
            args.lidar_topic,
            args.match_radius,
            args.scan_point_stride,
            np.asarray(args.lidar_imu_translation, dtype=np.float64),
            np.asarray(args.lidar_imu_rotation, dtype=np.float64).reshape(3, 3),
        )
    else:
        distance, nearest = cKDTree(trajectory).query(points, k=1, workers=-1)
        toward_sensor = trajectory[nearest] - points
        flip = np.einsum('ij,ij->i', normals, toward_sensor) < 0.0
        normals[flip] *= -1.0
        cloud.normals = o3d.utility.Vector3dVector(normals)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise RuntimeError(f'Failed to write {args.output}')
    print(f'input_points={raw_count} output_points={len(points)}')
    if not args.lidar_topic:
        print(f'nearest_pose_distance_m: median={np.median(distance):.3f} '
              f'p95={np.percentile(distance, 95):.3f}')
    print(f'normal_z: up={int((normals[:, 2] >= 0.70).sum())} '
          f'down={int((normals[:, 2] <= -0.70).sum())}')


if __name__ == '__main__':
    main()
