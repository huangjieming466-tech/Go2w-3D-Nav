#!/usr/bin/env python3
"""Generate a deterministic floor and L-wall PCD for SCAN Mode 3 tests."""

import argparse
from pathlib import Path

import numpy as np


def add_horizontal(points, normals, xmin, xmax, ymin, ymax, z, spacing):
    xs = np.arange(xmin, xmax + spacing * 0.5, spacing)
    ys = np.arange(ymin, ymax + spacing * 0.5, spacing)
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    plane = np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, z)))
    points.append(plane)
    normals.append(np.tile((0.0, 0.0, 1.0), (len(plane), 1)))


def add_vertical_x(points, normals, x, ymin, ymax, zmin, zmax, spacing):
    ys = np.arange(ymin, ymax + spacing * 0.5, spacing)
    zs = np.arange(zmin, zmax + spacing * 0.5, spacing)
    yy, zz = np.meshgrid(ys, zs, indexing="xy")
    wall = np.column_stack((np.full(yy.size, x), yy.ravel(), zz.ravel()))
    points.append(wall)
    normals.append(np.tile((1.0, 0.0, 0.0), (len(wall), 1)))


def add_vertical_y(points, normals, y, xmin, xmax, zmin, zmax, spacing):
    xs = np.arange(xmin, xmax + spacing * 0.5, spacing)
    zs = np.arange(zmin, zmax + spacing * 0.5, spacing)
    xx, zz = np.meshgrid(xs, zs, indexing="xy")
    wall = np.column_stack((xx.ravel(), np.full(xx.size, y), zz.ravel()))
    points.append(wall)
    normals.append(np.tile((0.0, 1.0, 0.0), (len(wall), 1)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="tools/test_maps/l_wall_mode3.pcd")
    parser.add_argument("--spacing", type=float, default=0.05)
    args = parser.parse_args()

    points, normals = [], []
    # Traversable floor, with an L wall that blocks the direct (2,2)->(8,8) chord.
    add_horizontal(points, normals, 0.0, 10.0, 0.0, 10.0, 0.0, args.spacing)
    add_vertical_x(points, normals, 5.0, 0.0, 6.0, 0.0, 1.6, args.spacing)
    add_vertical_y(points, normals, 6.0, 5.0, 10.0, 0.0, 1.6, args.spacing)

    data = np.hstack((np.vstack(points), np.vstack(normals)))
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z normal_x normal_y normal_z\n")
        handle.write("SIZE 4 4 4 4 4 4\n")
        handle.write("TYPE F F F F F F\n")
        handle.write("COUNT 1 1 1 1 1 1\n")
        handle.write(f"WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(data)}\nDATA ascii\n")
        np.savetxt(handle, data, fmt="%.4f")
    print(f"Wrote {output} with {len(data)} points")


if __name__ == "__main__":
    main()
