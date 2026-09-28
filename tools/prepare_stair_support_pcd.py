#!/usr/bin/env python3
"""Prepare a PCT input that separates horizontal support candidates from obstacles."""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def detect_floor_z(points, normals, bin_size):
    horizontal = np.abs(normals[:, 2]) >= 0.88
    z = points[horizontal, 2]
    start = np.floor(z.min() / bin_size) * bin_size
    bins = np.arange(start, z.max() + 2 * bin_size, bin_size)
    counts, edges = np.histogram(z, bins=bins)
    # Choose the lowest well-observed horizontal surface, not a lone return.
    eligible = np.flatnonzero(counts >= max(40, int(counts.max() * 0.20)))
    if not len(eligible):
        raise RuntimeError("No dense horizontal plane found for floor alignment")
    index = int(eligible[0])
    return float((edges[index] + edges[index + 1]) / 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel", type=float, default=0.08)
    parser.add_argument("--floor-z", type=float)
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(str(args.input))
    cloud = cloud.voxel_down_sample(args.voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=40))
    points = np.asarray(cloud.points)
    normals = np.asarray(cloud.normals)
    floor_z = args.floor_z if args.floor_z is not None else detect_floor_z(points, normals, 0.05)

    points[:, 2] -= floor_z
    horizontal = np.abs(normals[:, 2]) >= 0.88
    # Tomography reads normal.z as a support-surface label. Non-support
    # points remain in the cloud and therefore still contribute to collision.
    normals[:] = 0.0
    normals[horizontal, 2] = 1.0
    cloud.normals = o3d.utility.Vector3dVector(normals)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise RuntimeError(f"Failed to write {args.output}")
    print(f"floor_z={floor_z:.3f} m, points={len(points)}, support_candidates={int(horizontal.sum())}")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
