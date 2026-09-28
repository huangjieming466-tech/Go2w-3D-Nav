#!/usr/bin/env python3
"""Build a gravity-aligned, normal-oriented PCD from a LIO result bag.

Normals are estimated in each body-frame scan and oriented toward the sensor
origin. A floor observed from above therefore has +Z normal in the world map;
a ceiling or slab underside observed from below has -Z normal. This preserves
the information a final XYZ/I PCD normally loses.
"""

import argparse
import bisect
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


POINTFIELD_DTYPES = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def cloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    required = ('x', 'y', 'z')
    if any(name not in fields for name in required):
        raise ValueError(f'PointCloud2 missing XYZ fields: {list(fields)}')
    dtype = np.dtype({
        'names': required,
        'formats': [POINTFIELD_DTYPES[fields[name].datatype] for name in required],
        'offsets': [fields[name].offset for name in required],
        'itemsize': msg.point_step,
    })
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    points = np.column_stack((data['x'], data['y'], data['z'])).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def nearest_pose(stamps, poses, target):
    index = bisect.bisect_left(stamps, target)
    choices = [item for item in (index - 1, index) if 0 <= item < len(stamps)]
    best = min(choices, key=lambda item: abs(stamps[item] - target))
    if abs(stamps[best] - target) > 50_000_000:
        return None
    return poses[best]


def align_vectors(source, target):
    """Return the proper rotation that maps one unit vector to another."""
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


def initial_imu_up(imu_bag, topic, samples):
    """Estimate the upward direction from a stationary beginning of a bag."""
    store = get_typestore(Stores.ROS2_HUMBLE)
    values = []
    with Reader(imu_bag) as reader:
        connections = [item for item in reader.connections if item.topic == topic]
        if not connections:
            raise ValueError(f'Missing IMU topic {topic} in {imu_bag}')
        for connection, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_cdr(raw, connection.msgtype)
            values.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ])
            if len(values) >= samples:
                break
    if len(values) < 20:
        raise ValueError(f'Only {len(values)} IMU samples available for gravity alignment')
    mean = np.mean(np.asarray(values, dtype=np.float64), axis=0)
    norm = np.linalg.norm(mean)
    if norm < 1e-6:
        raise ValueError('Initial IMU acceleration has zero magnitude')
    return mean / norm, mean, len(values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--frame-voxel', type=float, default=0.10)
    parser.add_argument('--global-voxel', type=float, default=0.10)
    parser.add_argument('--normal-radius', type=float, default=0.25)
    parser.add_argument('--cloud-topic', default='/cloud_registered_body',
                        help='Deskewed PointCloud2 in the LiDAR frame')
    parser.add_argument('--odom-topic', default='/Odometry',
                        help='Odometry describing the body pose in the map frame')
    parser.add_argument('--imu-bag', type=Path,
                        help='Original input bag used to align Fast-LIO map Z with gravity')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--gravity-samples', type=int, default=1000)
    parser.add_argument('--max-position-norm', type=float,
                        help='Skip scans whose matched odometry position norm exceeds this many metres')
    parser.add_argument('--lidar-to-body-translation', nargs=3, type=float,
                        metavar=('X', 'Y', 'Z'), default=(0.0, 0.0, 0.0),
                        help='LiDAR origin expressed in body frame, in metres')
    parser.add_argument('--lidar-to-body-rotation', nargs=9, type=float,
                        metavar=('R00', 'R01', 'R02', 'R10', 'R11', 'R12', 'R20', 'R21', 'R22'),
                        default=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                        help='Row-major 3x3 rotation that maps LiDAR vectors into body frame')
    args = parser.parse_args()

    store = get_typestore(Stores.ROS2_HUMBLE)
    leveling_rotation = np.eye(3)
    if args.imu_bag:
        up, mean_acc, n_imu = initial_imu_up(
            args.imu_bag, args.imu_topic, args.gravity_samples
        )
        # Fast-LIO's map starts in the initial IMU frame. At rest, specific
        # force points upward in that frame; rotate it onto the map +Z axis.
        leveling_rotation = align_vectors(up, np.array([0.0, 0.0, 1.0]))
        tilt = np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0)))
        print(f'gravity alignment: samples={n_imu} mean_acc={mean_acc} '
              f'up={up} tilt_deg={tilt:.3f}')
    with Reader(args.bag) as reader:
        connections = {connection.topic: connection for connection in reader.connections}
        for topic in (args.odom_topic, args.cloud_topic):
            if topic not in connections:
                raise ValueError(f'Missing required topic: {topic}')

        stamps, poses = [], []
        for connection, _, raw in reader.messages(connections=[connections[args.odom_topic]]):
            msg = store.deserialize_cdr(raw, connection.msgtype)
            pose = msg.pose.pose
            stamps.append(stamp_ns(msg.header))
            poses.append((rotation_matrix(pose.orientation), np.array([
                pose.position.x, pose.position.y, pose.position.z,
            ], dtype=np.float64)))

        chunks = []
        used = skipped = 0
        lidar_to_body_rotation = np.asarray(
            args.lidar_to_body_rotation, dtype=np.float64
        ).reshape(3, 3)
        lidar_to_body_translation = np.asarray(
            args.lidar_to_body_translation, dtype=np.float64
        )
        for connection, _, raw in reader.messages(connections=[connections[args.cloud_topic]]):
            msg = store.deserialize_cdr(raw, connection.msgtype)
            transform = nearest_pose(stamps, poses, stamp_ns(msg.header))
            if transform is None:
                skipped += 1
                continue
            if (args.max_position_norm is not None and
                    np.linalg.norm(transform[1]) > args.max_position_norm):
                skipped += 1
                continue
            points = cloud_xyz(msg)
            if len(points) < 30:
                skipped += 1
                continue
            scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            scan = scan.voxel_down_sample(args.frame_voxel)
            scan.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
                radius=args.normal_radius, max_nn=40))
            scan.orient_normals_towards_camera_location(np.zeros(3))
            local_points = np.asarray(scan.points)
            local_normals = np.asarray(scan.normals)
            rotation, translation = transform
            # The cloud is in the LiDAR frame while /LIO/odom_vehicle is the
            # body pose.  Apply LiDAR -> body before body -> world.
            points_body = local_points @ lidar_to_body_rotation.T + lidar_to_body_translation
            normals_body = local_normals @ lidar_to_body_rotation.T
            chunks.append((points_body @ rotation.T + translation,
                           normals_body @ rotation.T))
            used += 1
            if used % 100 == 0:
                print(f'processed {used} scans')

    if not chunks:
        raise RuntimeError('No synchronized point clouds were processed')
    points = np.vstack([chunk[0] for chunk in chunks])
    normals = np.vstack([chunk[1] for chunk in chunks])
    # Apply the same world-frame rotation to positions and normal directions.
    points = points @ leveling_rotation.T
    normals = normals @ leveling_rotation.T
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.normals = o3d.utility.Vector3dVector(normals)
    cloud = cloud.voxel_down_sample(args.global_voxel)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise RuntimeError(f'Failed to write {args.output}')
    output_normals = np.asarray(cloud.normals)
    print(f'used_scans={used} skipped_scans={skipped} output_points={len(cloud.points)}')
    print(f'normal_z: up={int((output_normals[:, 2] >= 0.70).sum())} '
          f'down={int((output_normals[:, 2] <= -0.70).sum())}')


if __name__ == '__main__':
    main()
