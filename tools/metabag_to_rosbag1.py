#!/usr/bin/env python3
"""Create a ROS1-bag compatibility copy of a MetaBag V2 recording.

MetaCam writes ROS1 bag records after a one-byte-longer `#METABAG` magic.
This converter changes the magic and every absolute file offset stored in the
bag index. It is an inspection aid; the source recording is never modified.
"""

import argparse
import struct
from pathlib import Path


META_MAGIC = b"#METABAG V2.0\n"
ROS_MAGIC = b"#ROSBAG V2.0\n"


def shift_offset(header: bytearray, field: bytes) -> None:
    """Adjust an absolute ROS bag offset after shortening the magic by 1 byte."""
    position = header.find(field)
    if position < 0:
        return
    offset_pos = position + len(field)
    value = struct.unpack_from("<Q", header, offset_pos)[0]
    if value == 0:
        return
    struct.pack_into("<Q", header, offset_pos, value - 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    with args.input.open("rb") as source:
        if source.read(len(META_MAGIC)) != META_MAGIC:
            raise RuntimeError("Input is not a MetaBag V2 file")
        header_len = struct.unpack("<I", source.read(4))[0]
        header = bytearray(source.read(header_len))
        if b"index_pos=" not in header:
            raise RuntimeError("Bag header does not contain index_pos")
        shift_offset(header, b"index_pos=")
        data_len = source.read(4)
        if len(data_len) != 4:
            raise RuntimeError("Truncated bag header data length")
        header_data_size = struct.unpack("<I", data_len)[0]
        header_data = source.read(header_data_size)
        if len(header_data) != header_data_size:
            raise RuntimeError("Truncated bag header data")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("wb") as target:
            target.write(ROS_MAGIC)
            target.write(struct.pack("<I", header_len))
            target.write(header)
            target.write(data_len)
            target.write(header_data)
            while record_header_len := source.read(4):
                if len(record_header_len) != 4:
                    raise RuntimeError("Truncated record header length")
                record_header_size = struct.unpack("<I", record_header_len)[0]
                record_header = bytearray(source.read(record_header_size))
                if len(record_header) != record_header_size:
                    raise RuntimeError("Truncated record header")

                # Chunk-info records in the index carry absolute chunk offsets.
                # Every later byte moves back by one in the ROS-compatible copy.
                shift_offset(record_header, b"chunk_pos=")

                record_data_len = source.read(4)
                if len(record_data_len) != 4:
                    raise RuntimeError("Truncated record data length")
                record_data_size = struct.unpack("<I", record_data_len)[0]
                record_data = source.read(record_data_size)
                if len(record_data) != record_data_size:
                    raise RuntimeError("Truncated record data")

                target.write(record_header_len)
                target.write(record_header)
                target.write(record_data_len)
                target.write(record_data)

    print(f"Converted compatibility copy: {args.output}")


if __name__ == "__main__":
    main()
