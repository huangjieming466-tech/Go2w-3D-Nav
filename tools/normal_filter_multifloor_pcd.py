#!/usr/bin/env python3
"""Build a leveled PCD that excludes downward-facing horizontal surfaces.

The PCD exported by Elevator-LIO contains normal fields but they are zeroed.
This tool estimates normals after voxel downsampling and orients each normal
towards the nearest recorded LiDAR pose.  A horizontal surface observed from
above is floor-like (normal points up); one observed from below is ceiling-like
(normal points down).
"""

import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from level_pcd_from_floor import rotation_align


def load_tum(path: Path) -> np.ndarray:
    poses = np.loadtxt(path, usecols=(1, 2, 3), dtype=np.float64)
    if poses.ndim != 2 or poses.shape[0] == 0:
        raise RuntimeError(f"No positions found in {path}")
    return poses


def find_floor_transform(points: np.ndarray, sample_points: int, distance: float, iterations: int):
    rng = np.random.default_rng(42)
    sample = points[rng.choice(len(points), min(len(points), sample_points), replace=False)]
    remaining = sample
    candidates = []
    for _ in range(16):
        if len(remaining) < 1000:
            break
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(remaining))
        plane, inliers = cloud.segment_plane(distance, 3, iterations)
        normal = np.asarray(plane[:3], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        if abs(normal[2]) >= 0.95:
            center_xy = np.mean(sample[:, :2], axis=0)
            height = -(plane[0] * center_xy[0] + plane[1] * center_xy[1] + plane[3]) / plane[2]
            candidates.append((height, np.asarray(plane, dtype=np.float64)))
        keep = np.ones(len(remaining), dtype=bool)
        keep[np.asarray(inliers)] = False
        remaining = remaining[keep]
    if not candidates:
        raise RuntimeError("Could not find a near-horizontal floor plane")
    _, plane = min(candidates, key=lambda item: item[0])
    normal = plane[:3] / np.linalg.norm(plane[:3])
    if normal[2] < 0.0:
        normal = -normal
        plane = np.r_[-normal, -plane[3]]
    rotation = rotation_align(normal, np.array([0.0, 0.0, 1.0]))
    signed = sample @ normal + plane[3]
    inliers = np.abs(signed) <= distance
    floor_z = float(np.median((sample @ rotation.T)[inliers, 2]))
    return rotation, np.array([0.0, 0.0, floor_z]), plane


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="Elevator-LIO source PCD in its original map frame")
    parser.add_argument("trajectory", type=Path, help="Elevator-LIO traj.tum from the same map run")
    parser.add_argument("output", type=Path, help="Output leveled, ceiling-filtered PCD")
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument("--normal-radius", type=float, default=0.30)
    parser.add_argument("--normal-max-nn", type=int, default=40)
    parser.add_argument("--horizontal-normal-z", type=float, default=0.70)
    parser.add_argument("--sample-points", type=int, default=160000)
    parser.add_argument("--plane-distance", type=float, default=0.04)
    parser.add_argument("--plane-iterations", type=int, default=1000)
    args = parser.parse_args()

    if args.voxel_size <= 0 or args.normal_radius <= 0:
        raise ValueError("voxel-size and normal-radius must be positive")
    if not 0 < args.horizontal_normal_z < 1:
        raise ValueError("horizontal-normal-z must be in (0, 1)")

    print(f"Loading {args.input}")
    source = o3d.io.read_point_cloud(str(args.input))
    source_points = np.asarray(source.points)
    if len(source_points) == 0:
        raise RuntimeError("Input PCD contains no points")
    print(f"Source points: {len(source_points)}")

    rotation, translation, plane = find_floor_transform(
        source_points, args.sample_points, args.plane_distance, args.plane_iterations
    )
    poses = load_tum(args.trajectory)
    poses = poses @ rotation.T - translation
    print("Selected floor plane:", " ".join(f"{v:.8f}" for v in plane))
    print(f"Trajectory poses: {len(poses)}")

    cloud = source.voxel_down_sample(args.voxel_size)
    points = np.asarray(cloud.points)
    points[:] = points @ rotation.T - translation
    print(f"Voxel points: {len(points)} at {args.voxel_size:.2f} m")

    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=args.normal_radius, max_nn=args.normal_max_nn)
    )
    normals = np.asarray(cloud.normals)
    nearest_pose = poses[cKDTree(poses).query(points, k=1)[1]]
    view_direction = nearest_pose - points
    flip = np.einsum("ij,ij->i", normals, view_direction) < 0.0
    normals[flip] *= -1.0

    ceiling = normals[:, 2] < -args.horizontal_normal_z
    floor = normals[:, 2] > args.horizontal_normal_z
    keep = ~ceiling
    filtered = cloud.select_by_index(np.flatnonzero(keep))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), filtered, write_ascii=False, compressed=False):
        raise RuntimeError(f"Failed to write {args.output}")

    print(f"Upward horizontal points: {int(floor.sum())}")
    print(f"Downward horizontal points removed: {int(ceiling.sum())}")
    print(f"Retained points: {int(keep.sum())}")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
