#!/usr/bin/env python3
"""
Convert per-point timestamps in sensor_msgs/PointCloud2 from nanoseconds -> seconds, and
rewrite to a Hesai-style PointCloud2 layout, into a NEW rosbag using LZ4 compression.

Only reads the selected input topic and writes the selected output topic.

Output point layout (Hesai-style):
  float32 x, y, z
  float32 intensity
  float64 timestamp   # seconds
  uint16  ring

Input (your Livox example) fields:
  x,y,z,float32
  intensity,float32
  tag,uint8
  line,uint8          # used as ring if 'ring' doesn't exist
  timestamp,float64   # nanoseconds (epoch ns) in your sample

Additional requirements implemented:
- Output bag is written with LZ4 compression
- The sensor header is preserved by default. --header-time bag explicitly selects
  the old record-time convention; record time is not necessarily acquisition time.

Accuracy notes:
- If the input timestamp is stored as float64 at ~1e18 magnitude, float64 cannot represent
  every nanosecond at that magnitude. This script preserves maximum available precision by:
  (1) converting to int64 nanoseconds (rounded to nearest representable integer), then
  (2) forming seconds as integer seconds + remainder/1e9.
"""

import sys
import os
import glob
import argparse
from pathlib import Path
import rosbag
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from box_auto.utils.atomic_file import atomic_output


# ---------------------------
# HARDCODED PARAMETERS
# ---------------------------
BAG_IN_DIR = "/home/tutuna/ijrr_paper/src/test_tests/arc_2"
BAG_IN_GLOB_PATTERN = "*_livox.bag"  # non-recursive match within BAG_IN_DIR

# Output bag will be written in BAG_IN_DIR with this suffix appended to the input stem
BAG_OUT_SUFFIX = "_hesai_ts_sec_lz4.bag"

IN_TOPIC = "/boxi/livox/points"
OUT_TOPIC = "/boxi/livox/points_converted"

PRINT_PROGRESS = True
PROGRESS_EVERY_N_MSG = 200
# ---------------------------


# PointField datatype -> numpy dtype kind
_PF_TO_NUMPY_KIND = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def _endian_char(is_bigendian: bool) -> str:
    return ">" if is_bigendian else "<"


def _numpy_dtype_for_pointfield(datatype: int, is_bigendian: bool) -> np.dtype:
    if datatype not in _PF_TO_NUMPY_KIND:
        raise ValueError(f"Unsupported PointField.datatype={datatype}")
    dt = np.dtype(_PF_TO_NUMPY_KIND[datatype])
    if dt.itemsize > 1:
        dt = dt.newbyteorder(_endian_char(is_bigendian))
    return dt


def _validate_fields(msg: PointCloud2):
    if getattr(msg, "_type", None) != "sensor_msgs/PointCloud2":
        raise ValueError("Expected sensor_msgs/PointCloud2")
    names = {f.name for f in msg.fields}
    if len(names) != len(msg.fields):
        raise ValueError("Duplicate point field names")
    required = {"x", "y", "z", "intensity", "timestamp"}
    missing = sorted(required - names)
    if missing:
        raise ValueError(f"Missing required fields in input cloud: {missing}")
    if "ring" not in names and "line" not in names:
        raise ValueError("Missing ring source field: expected 'ring' or 'line' in input cloud")
    scalar_fields = required | {"ring" if "ring" in names else "line"}
    if any(f.count != 1 for f in msg.fields if f.name in scalar_fields):
        raise ValueError("Required point fields must be scalar")
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError("Invalid point or row stride")
    if len(msg.data) != msg.row_step * msg.height:
        raise ValueError("PointCloud2 data length disagrees with its dimensions")


def _build_struct_dtype_from_pc2(msg: PointCloud2) -> np.dtype:
    """
    Build a numpy structured dtype that matches msg.fields + offsets + point_step padding.
    """
    names, formats, offsets = [], [], []
    for f in msg.fields:
        dt = _numpy_dtype_for_pointfield(f.datatype, msg.is_bigendian)
        names.append(f.name)
        if f.count == 1:
            formats.append(dt)
        else:
            formats.append((dt, (f.count,)))
        offsets.append(f.offset)

    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})


def _pc2_to_struct_array(msg: PointCloud2, struct_dtype: np.dtype) -> np.ndarray:
    """
    Convert PointCloud2 binary data into a flat numpy structured array (N points).
    Handles row padding if row_step != width * point_step.
    """
    npts = int(msg.width) * int(msg.height)
    if npts == 0:
        return np.empty((0,), dtype=struct_dtype)

    expected_row = int(msg.width) * int(msg.point_step)

    # Fast path: no row padding
    if int(msg.row_step) == expected_row:
        return np.frombuffer(msg.data, dtype=struct_dtype, count=npts)

    # Row padding: parse row-by-row and skip padding bytes
    mv = memoryview(msg.data)
    rows = []
    for r in range(int(msg.height)):
        start = r * int(msg.row_step)
        row_bytes = mv[start : start + expected_row]
        rows.append(np.frombuffer(row_bytes, dtype=struct_dtype, count=int(msg.width)))
    return np.concatenate(rows, axis=0)


def _ns_float_to_int64(ns_vals: np.ndarray) -> np.ndarray:
    """
    Convert ns values to int64 nanoseconds. For float inputs, round to nearest integer.
    """
    # Comparing against the exclusive upper bound avoids float64 rounding
    # INT64_MAX up to 2**63 and silently overflowing during the cast.
    if not np.all(np.isfinite(ns_vals)) or np.any(ns_vals < 0) or np.any(ns_vals >= 2**63):
        raise ValueError("Point timestamps must be finite, nonnegative int64 nanoseconds")
    if np.issubdtype(ns_vals.dtype, np.integer):
        return ns_vals.astype(np.int64, copy=False)
    return np.rint(ns_vals).astype(np.int64)


def _ns_int_to_seconds_float64(ns_int: np.ndarray) -> np.ndarray:
    """
    Convert int64 nanoseconds to float64 seconds using integer seconds + remainder nanoseconds.
    """
    sec_int = ns_int // 1_000_000_000
    rem_ns = ns_int - sec_int * 1_000_000_000
    return sec_int.astype(np.float64) + (rem_ns.astype(np.float64) / 1_000_000_000.0)


def convert_cloud_to_hesai(msg: PointCloud2, bag_stamp, header_time="input") -> PointCloud2:
    """
    Convert field layout/units, retaining acquisition time unless explicitly overridden.
    """
    _validate_fields(msg)

    in_dtype = _build_struct_dtype_from_pc2(msg)
    in_arr = _pc2_to_struct_array(msg, in_dtype)
    npts = in_arr.shape[0]

    out_endian = _endian_char(msg.is_bigendian)
    out_dtype = np.dtype(
        [
            ("x", out_endian + "f4"),
            ("y", out_endian + "f4"),
            ("z", out_endian + "f4"),
            ("intensity", out_endian + "f4"),
            ("timestamp", out_endian + "f8"),  # seconds
            ("ring", out_endian + "u2"),
        ]
    )

    out_arr = np.empty((npts,), dtype=out_dtype)

    # xyz + intensity
    out_arr["x"] = in_arr["x"].astype(out_dtype["x"], copy=False)
    out_arr["y"] = in_arr["y"].astype(out_dtype["y"], copy=False)
    out_arr["z"] = in_arr["z"].astype(out_dtype["z"], copy=False)
    out_arr["intensity"] = in_arr["intensity"].astype(out_dtype["intensity"], copy=False)

    # ring source: prefer 'ring' else use 'line'
    ring_src = in_arr["ring"] if "ring" in in_arr.dtype.names else in_arr["line"]
    if (
        not np.isfinite(ring_src).all()
        or np.any(ring_src < 0)
        or np.any(ring_src > 65535)
        or np.any(ring_src != np.floor(ring_src))
    ):
        raise ValueError("Ring values must be integers in [0, 65535]")
    out_arr["ring"] = ring_src.astype(out_dtype["ring"], copy=False)

    # timestamp: ns -> seconds
    ts_ns = in_arr["timestamp"]
    ns_int = _ns_float_to_int64(ts_ns)
    ts_sec = _ns_int_to_seconds_float64(ns_int)

    out_arr["timestamp"] = ts_sec.astype(out_dtype["timestamp"], copy=False)

    # output message
    out_msg = PointCloud2()

    out_hdr = Header()
    if getattr(msg, "header", None) is not None:
        out_hdr.seq = msg.header.seq
        out_hdr.frame_id = msg.header.frame_id
    if header_time not in ("input", "bag"):
        raise ValueError("header_time must be 'input' or 'bag'")
    out_hdr.stamp = msg.header.stamp if header_time == "input" else bag_stamp
    out_msg.header = out_hdr

    out_msg.height = msg.height
    out_msg.width = msg.width
    out_msg.is_bigendian = msg.is_bigendian
    out_msg.is_dense = msg.is_dense

    out_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name="ring", offset=24, datatype=PointField.UINT16, count=1),
    ]

    out_msg.point_step = out_dtype.itemsize  # 26 bytes
    out_msg.row_step = out_msg.point_step * out_msg.width
    out_msg.data = out_arr.tobytes(order="C")

    return out_msg


def resolve_input_bag_path(bag_dir: str, pattern: str) -> str:
    """
    Non-recursively find exactly one bag in bag_dir matching pattern.
    """
    bag_dir = os.path.expanduser(str(bag_dir))
    pattern = str(pattern)

    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f"Input bag directory does not exist: {bag_dir}")

    glob_pat = os.path.join(bag_dir, pattern)
    matches = sorted([p for p in glob.glob(glob_pat) if os.path.isfile(p)])

    if len(matches) == 1:
        return os.path.abspath(matches[0])
    if len(matches) == 0:
        raise FileNotFoundError(f"No bag files match pattern '{pattern}' in directory: {bag_dir}")
    raise RuntimeError("Multiple bag files matched; please disambiguate:\n" + "\n".join(f"  {m}" for m in matches))


def derive_output_bag_path(input_bag_path: str, out_dir: str, suffix: str) -> str:
    out_dir = os.path.expanduser(str(out_dir))
    stem = os.path.splitext(os.path.basename(str(input_bag_path)))[0]
    return os.path.join(out_dir, stem + str(suffix))


def convert_bag(
    input_bag, output_bag, input_topic=IN_TOPIC, output_topic=OUT_TOPIC, *, header_time="input", overwrite=False
):
    if Path(input_bag).resolve() == Path(output_bag).resolve():
        raise ValueError("Conversion output must differ from the input bag")
    converted = 0
    points = 0
    with atomic_output(output_bag, overwrite=overwrite) as temporary:
        with rosbag.Bag(str(input_bag), "r") as source, rosbag.Bag(str(temporary), "w", compression="lz4") as output:
            for _, msg, stamp in source.read_messages(topics=[input_topic]):
                cloud = convert_cloud_to_hesai(msg, stamp, header_time=header_time)
                output.write(output_topic, cloud, stamp)
                converted += 1
                points += cloud.width * cloud.height
            if not converted or not points:
                raise ValueError(f"No usable points on {input_topic}; existing output is preserved")
    return converted, points


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--input-topic", default=IN_TOPIC)
    parser.add_argument("--output-topic", default=OUT_TOPIC)
    parser.add_argument("--header-time", choices=("input", "bag"), default="input")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    output = args.output or args.input.with_name(args.input.stem + BAG_OUT_SUFFIX)
    try:
        messages, points = convert_bag(
            args.input,
            output,
            args.input_topic,
            args.output_topic,
            header_time=args.header_time,
            overwrite=args.overwrite,
        )
    except Exception as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 1
    print(f"Converted {messages} clouds / {points} points: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
