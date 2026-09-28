#!/usr/bin/env python3
"""Rotate the XYZ fields of a binary float32 PCD while preserving other fields."""

import argparse
import math
from pathlib import Path

import numpy as np


def read_header(stream):
    lines = []
    while True:
        line = stream.readline()
        if not line:
            raise RuntimeError("PCD header ended before DATA")
        lines.append(line)
        if line.strip().lower().startswith(b"data"):
            return lines


def header_value(header, key):
    prefix = key.encode() + b" "
    for line in header:
        if line.startswith(prefix):
            return line.split()[1:]
    raise RuntimeError(f"missing PCD header field: {key}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--degrees", type=float, default=25.7)
    args = parser.parse_args()

    with args.input.open("rb") as source:
        header = read_header(source)
        fields = [item.decode() for item in header_value(header, "FIELDS")]
        sizes = [int(item) for item in header_value(header, "SIZE")]
        types = [item.decode() for item in header_value(header, "TYPE")]
        counts = [int(item) for item in header_value(header, "COUNT")]
        points = int(header_value(header, "POINTS")[0])
        data_kind = header[-1].split()[1].lower()
        if data_kind != b"binary":
            raise RuntimeError("only binary PCD is supported")
        if fields[:3] != ["x", "y", "z"]:
            raise RuntimeError("the first three PCD fields must be x y z")
        if sizes[:3] != [4, 4, 4] or types[:3] != ["F", "F", "F"]:
            raise RuntimeError("x/y/z must be float32")
        if counts[:3] != [1, 1, 1]:
            raise RuntimeError("x/y/z must have COUNT 1")

        record_width = sum(size * count for size, count in zip(sizes, counts))
        if record_width != 32 or len(fields) != 8:
            raise RuntimeError(
                f"expected the current 8-float PCD layout, got {len(fields)} fields and {record_width} bytes"
            )
        raw = np.fromfile(source, dtype="<f4", count=points * 8)
        if raw.size != points * 8:
            raise RuntimeError(f"expected {points} points, read {raw.size // 8}")
        cloud = raw.reshape(points, 8)

    angle = math.radians(args.degrees)
    c = math.cos(angle)
    s = math.sin(angle)
    x = cloud[:, 0].copy()
    z = cloud[:, 2].copy()
    cloud[:, 0] = c * x + s * z
    cloud[:, 2] = -s * x + c * z

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as target:
        target.writelines(header)
        cloud.astype("<f4", copy=False).tofile(target)

    print(f"rotated {points} points by +{args.degrees:.4f} degrees around Y")
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
