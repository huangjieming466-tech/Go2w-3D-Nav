#!/usr/bin/env python3
"""Merge Super-LIO scans_*.pcd fragments into one voxelized map."""

import argparse
import re
from pathlib import Path

import numpy as np
import open3d as o3d


def natural_key(path: Path):
    return [int(part) if part.isdigit() else part for part in re.split(r'(\d+)', path.name)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('fragment_dir', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--voxel', type=float, default=0.08)
    args = parser.parse_args()

    files = sorted(args.fragment_dir.glob('scans_*.pcd'), key=natural_key)
    if not files:
        raise ValueError(f'No scans_*.pcd files in {args.fragment_dir}')

    chunks = []
    total = 0
    for path in files:
        cloud = o3d.io.read_point_cloud(str(path))
        points = np.asarray(cloud.points)
        if len(points):
            chunks.append(points.copy())
            total += len(points)
        print(f'{path.name}: {len(points)} points')

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(np.concatenate(chunks, axis=0))
    if args.voxel > 0:
        merged = merged.voxel_down_sample(args.voxel)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), merged, write_ascii=False):
        raise RuntimeError(f'Failed to write {args.output}')
    print(f'input_points={total} output_points={len(merged.points)} voxel={args.voxel}')


if __name__ == '__main__':
    main()
