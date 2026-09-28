#!/usr/bin/env python3
"""Level a binary XYZ-first PCD using a selected horizontal floor plane."""

import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d


FIELDS = 8
RECORD_BYTES = FIELDS * 4


def read_header(stream):
    lines = []
    while True:
        line = stream.readline()
        if not line:
            raise RuntimeError("PCD header ended before DATA")
        lines.append(line)
        if line.strip().lower().startswith(b"data"):
            return lines, stream.tell()


def rotation_align(a, b):
    """Return a Rodrigues rotation matrix that maps unit vector a to b."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        # 180-degree rotation around any axis perpendicular to a.
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis -= a * np.dot(a, axis)
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sample-points", type=int, default=160000)
    parser.add_argument("--distance", type=float, default=0.04)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument(
        "--floor-plane",
        nargs=4,
        type=float,
        metavar=("A", "B", "C", "D"),
        help="Use a*x+b*y+c*z+d=0 instead of detecting the lowest near-horizontal plane.",
    )
    args = parser.parse_args()

    with args.input.open("rb") as source:
        header, data_offset = read_header(source)
        points = None
        for line in header:
            if line.startswith(b"POINTS "):
                points = int(line.split()[1])
        if points is None:
            raise RuntimeError("PCD header has no POINTS field")
        raw = np.memmap(
            args.input,
            mode="r",
            offset=data_offset,
            dtype="<f4",
            shape=(points, FIELDS),
        )

    rng = np.random.default_rng(42)
    sample_count = min(points, args.sample_points)
    sample = np.asarray(raw[rng.choice(points, sample_count, replace=False), :3], dtype=np.float64)

    if args.floor_plane is not None:
        plane = np.asarray(args.floor_plane, dtype=np.float64)
    else:
        # Remove candidate planes until the lowest near-horizontal plane is found.
        remaining = sample
        candidates = []
        for _ in range(16):
            if len(remaining) < 1000:
                break
            cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(remaining))
            candidate, inliers = cloud.segment_plane(
                distance_threshold=args.distance,
                ransac_n=3,
                num_iterations=args.iterations,
            )
            normal = np.asarray(candidate[:3], dtype=np.float64)
            normal /= np.linalg.norm(normal)
            if abs(normal[2]) >= 0.95:
                # Use the plane height at the sample XY center for ranking.
                cx, cy = np.mean(sample[:, :2], axis=0)
                height = -(candidate[0] * cx + candidate[1] * cy + candidate[3]) / candidate[2]
                candidates.append((height, np.asarray(candidate, dtype=np.float64), len(inliers)))
            mask = np.ones(len(remaining), dtype=bool)
            mask[np.asarray(inliers)] = False
            remaining = remaining[mask]
        if not candidates:
            raise RuntimeError("Could not find a near-horizontal plane")
        # The lowest broad horizontal plane is the floor; the upper one is normally the ceiling.
        candidates.sort(key=lambda item: item[0])
        _, plane, _ = candidates[0]

    normal = np.asarray(plane[:3], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
        plane = np.r_[-normal, -float(plane[3])]
    rotation = rotation_align(normal, np.array([0.0, 0.0, 1.0]))

    # Estimate the plane height after rotation and translate it to z=0.
    plane_sample = sample @ rotation.T
    signed = (sample @ normal + float(plane[3]))
    inlier_mask = np.abs(signed) <= args.distance
    if not np.any(inlier_mask):
        raise RuntimeError("Selected floor plane has no inliers in the sample")
    floor_z = float(np.median(plane_sample[inlier_mask, 2]))
    translation = np.array([0.0, 0.0, floor_z], dtype=np.float64)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as target:
        target.writelines(header)
        chunk = 250000
        for start in range(0, points, chunk):
            end = min(start + chunk, points)
            block = np.array(raw[start:end], dtype=np.float32, copy=True)
            xyz = block[:, :3].astype(np.float64)
            block[:, :3] = ((xyz @ rotation.T) - translation).astype(np.float32)
            block.astype("<f4", copy=False).tofile(target)

    print("selected floor plane:", " ".join(f"{v:.8f}" for v in plane))
    print("floor normal:", " ".join(f"{v:.8f}" for v in normal))
    print(f"rotation angle: {math.degrees(math.acos(np.clip(normal[2], -1.0, 1.0))):.4f} deg")
    print(f"floor z before translation: {floor_z:.6f} m")
    print(f"wrote {points} points to {args.output}")


if __name__ == "__main__":
    main()
