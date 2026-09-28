#!/usr/bin/env python3
"""Estimate signed map normals using an Elevator-LIO map and its odometry bag."""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def align_vectors(source, target):
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    source /= np.linalg.norm(source)
    target /= np.linalg.norm(target)
    cross = np.cross(source, target)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if np.linalg.norm(cross) < 1e-10:
        return np.eye(3) if dot > 0.0 else np.diag([-1.0, -1.0, 1.0])
    skew = np.array([
        [0.0, -cross[2], cross[1]],
        [cross[2], 0.0, -cross[0]],
        [-cross[1], cross[0], 0.0],
    ])
    return np.eye(3) + skew + skew @ skew * ((1.0 - dot) / np.dot(cross, cross))


def gravity_rotation(bag, topic, samples):
    store = get_typestore(Stores.ROS2_HUMBLE)
    values = []
    with Reader(bag) as reader:
        conns = [item for item in reader.connections if item.topic == topic]
        if not conns:
            raise ValueError(f'Missing IMU topic {topic} in {bag}')
        for conn, _, raw in reader.messages(connections=conns):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            values.append([msg.linear_acceleration.x, msg.linear_acceleration.y,
                           msg.linear_acceleration.z])
            if len(values) >= samples:
                break
    mean = np.mean(values, axis=0)
    if len(values) < 20 or np.linalg.norm(mean) < 1e-6:
        raise ValueError('Insufficient stationary IMU samples for gravity alignment')
    up = mean / np.linalg.norm(mean)
    print(f'gravity samples={len(values)} up={up} tilt_deg='
          f'{np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))):.3f}')
    return align_vectors(up, np.array([0.0, 0.0, 1.0]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--odom-topic', default='/LIO/odom_vehicle')
    parser.add_argument('--imu-topic', default='/go2/imu')
    parser.add_argument('--voxel', type=float, default=0.10)
    parser.add_argument('--normal-radius', type=float, default=0.25)
    parser.add_argument('--normal-max-nn', type=int, default=40)
    parser.add_argument('--gravity-samples', type=int, default=1000)
    parser.add_argument('--lidar-to-body-translation', nargs=3, type=float,
                        default=(-0.1641436, 0.0, -0.1203078))
    parser.add_argument('--lidar-to-body-rotation', nargs=9, type=float,
                        default=(0.9743701, 0.0, -0.2249511,
                                 0.0, 1.0, 0.0,
                                 0.2249511, 0.0, 0.9743701))
    args = parser.parse_args()

    store = get_typestore(Stores.ROS2_HUMBLE)
    rotations, translations = [], []
    with Reader(args.bag) as reader:
        conns = [item for item in reader.connections if item.topic == args.odom_topic]
        if not conns:
            raise ValueError(f'Missing odometry topic {args.odom_topic} in {args.bag}')
        for conn, _, raw in reader.messages(connections=conns):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            pose = msg.pose.pose
            rotations.append(rotation_matrix(pose.orientation))
            translations.append([pose.position.x, pose.position.y, pose.position.z])
    if len(rotations) < 2:
        raise ValueError('At least two odometry poses are required')

    level = gravity_rotation(args.bag, args.imu_topic, args.gravity_samples)
    r_bl = np.asarray(args.lidar_to_body_rotation, dtype=np.float64).reshape(3, 3)
    t_bl = np.asarray(args.lidar_to_body_translation, dtype=np.float64)
    trajectory = np.asarray([
        rotation @ t_bl + translation
        for rotation, translation in zip(rotations, translations)
    ]) @ level.T

    cloud = o3d.io.read_point_cloud(str(args.pcd))
    if len(cloud.points) == 0:
        raise ValueError(f'No points read from {args.pcd}')
    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(args.voxel)
    points = np.asarray(cloud.points) @ level.T
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.normal_radius, max_nn=args.normal_max_nn
    ))

    normals = np.asarray(cloud.normals)
    _, nearest = cKDTree(trajectory).query(points, k=1, workers=-1)
    toward_sensor = trajectory[nearest] - points
    flip = np.einsum('ij,ij->i', normals, toward_sensor) < 0.0
    normals[flip] *= -1.0
    cloud.normals = o3d.utility.Vector3dVector(normals)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise RuntimeError(f'Failed to write {args.output}')
    print(f'odom_poses={len(trajectory)} output_points={len(points)}')
    print(f'normal_z: up={int((normals[:, 2] >= 0.70).sum())} '
          f'down={int((normals[:, 2] <= -0.70).sum())}')


if __name__ == '__main__':
    main()
