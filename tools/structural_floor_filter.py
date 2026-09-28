#!/usr/bin/env python3
"""Infer floor planes from a PCD without LiDAR poses.

This is a map-only fallback. Horizontal height modes are paired with the next
mode above them. The lower member of a pair with usable clearance is marked as
floor; the upper member is treated as ceiling and removed. Output normals are
metadata: only inferred floor points receive an upward normal.
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument("--normal-radius", type=float, default=0.30)
    parser.add_argument("--normal-max-nn", type=int, default=40)
    parser.add_argument("--horizontal-normal-z", type=float, default=0.70)
    parser.add_argument("--height-bin", type=float, default=0.05)
    parser.add_argument("--plane-band", type=float, default=0.12)
    parser.add_argument("--min-clearance", type=float, default=1.0)
    parser.add_argument("--max-clearance", type=float, default=3.5)
    parser.add_argument("--peak-prominence-ratio", type=float, default=0.05)
    args = parser.parse_args()

    print(f"Loading {args.input}")
    cloud = o3d.io.read_point_cloud(str(args.input))
    if len(cloud.points) == 0:
        raise RuntimeError("Input PCD contains no points")
    cloud = cloud.voxel_down_sample(args.voxel_size)
    points = np.asarray(cloud.points)
    print(f"Voxel points: {len(points)} at {args.voxel_size:.2f} m")
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.normal_radius, max_nn=args.normal_max_nn
    ))
    normals = np.asarray(cloud.normals)
    horizontal = np.abs(normals[:, 2]) >= args.horizontal_normal_z
    heights = points[horizontal, 2]
    if len(heights) == 0:
        raise RuntimeError("No near-horizontal surfaces found")

    lo = np.floor(heights.min() / args.height_bin) * args.height_bin
    hi = np.ceil(heights.max() / args.height_bin) * args.height_bin
    edges = np.arange(lo, hi + args.height_bin, args.height_bin)
    histogram, edges = np.histogram(heights, bins=edges)
    smooth = gaussian_filter1d(histogram.astype(np.float64), sigma=1.0)
    peaks, _ = find_peaks(smooth, prominence=max(1.0, smooth.max() * args.peak_prominence_ratio))
    modes = (edges[peaks] + edges[peaks + 1]) * 0.5
    if len(modes) < 2:
        raise RuntimeError("Fewer than two horizontal surface height modes found")

    floors, ceilings = [], []
    for i, lower in enumerate(modes[:-1]):
        above = modes[i + 1:]
        clearance = above - lower
        candidates = above[(clearance >= args.min_clearance) & (clearance <= args.max_clearance)]
        if len(candidates):
            floors.append(lower)
            ceilings.append(float(candidates[0]))
    if not floors:
        raise RuntimeError("No floor/ceiling pairs satisfy the clearance limits")
    floors = np.asarray(floors)
    ceilings = np.asarray(ceilings)
    print("Detected floor/ceiling pairs:")
    for floor, ceiling in zip(floors, ceilings):
        print(f"  floor={floor:.2f} ceiling={ceiling:.2f} clearance={ceiling - floor:.2f}")

    dist_mode = np.abs(points[:, 2, None] - modes[None, :]).min(axis=1)
    on_plane = horizontal & (dist_mode <= args.plane_band)
    dist_floor = np.abs(points[:, 2, None] - floors[None, :]).min(axis=1)
    floor_mask = horizontal & (dist_floor <= args.plane_band)
    keep = ~on_plane | floor_mask
    filtered = cloud.select_by_index(np.flatnonzero(keep))
    output_normals = np.zeros((int(keep.sum()), 3), dtype=np.float64)
    output_normals[floor_mask[keep], 2] = 1.0
    filtered.normals = o3d.utility.Vector3dVector(output_normals)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), filtered, write_ascii=False, compressed=False):
        raise RuntimeError(f"Failed to write {args.output}")
    print(f"Floor-marked points: {int(floor_mask.sum())}")
    print(f"Removed ceiling-plane points: {int((~keep).sum())}")
    print(f"Retained points: {int(keep.sum())}")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
