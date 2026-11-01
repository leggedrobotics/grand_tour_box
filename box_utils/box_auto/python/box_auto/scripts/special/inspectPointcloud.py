#!/usr/bin/env python3
"""
Inspect sensor_msgs/PointCloud2 topic in a ROS1 .bag with dtype-aware field printing.

Improvements vs original:
- Centralized numeric formatting with adjustable precision controls.
- No scientific notation for floats (no "e18" etc). Uses positional float formatting.
- Float printing defaults to the maximum round-trip significant digits implied by the stored dtype
  (float16/32/64). You can clamp it via CLI or constants.
- Reads PointCloud2 data via numpy structured dtypes (preserves float32 vs float64 storage),
  including correct handling of row_step padding for organized clouds.

What it does:
- Prints a one-line summary for every PointCloud2 message on TOPIC (can be toggled)
- Prints the first N point clouds on TOPIC, and for each of those clouds prints:
  - field table (name/offset/count/ROS dtype/numpy dtype)
  - numpy structured dtype (verifies offsets + point_step padding)
  - first M points, printing all fields and their values per point (and Python value types)
"""

import sys
import math
import argparse
import os
import glob
import rosbag
import numpy as np
from sensor_msgs.msg import PointField


# ---------------------------
# DEFAULT PARAMETERS (can be overridden by CLI)
# ---------------------------
BAG_GLOB_PATTERN = "*_velodyne.bag"
TOPIC = "/anymal/velodyne/points"

NUM_CLOUDS_TO_PRINT = 1
NUM_POINTS_PER_CLOUD = 20000

# If True, skip any point that contains non-finite values in any float field.
SKIP_NANS = False

# If True, print one-line summary per message
PRINT_SUMMARY_EACH_MSG = False

# Timestamp formatting (bag_time + header_stamp) in fixed decimals (never scientific)
TIME_DECIMALS = 9

# Float formatting:
# - By default, prints up to the stored dtype's max round-trip significant digits:
#     float16 -> 5, float32 -> 9, float64 -> 17
# - If FLOAT_SIGDIGS_OVERRIDE is not None, it clamps to min(override, dtype_max_digits)
FLOAT_SIGDIGS_OVERRIDE = None

# If True, print value types alongside values
SHOW_VALUE_TYPES = True
# ---------------------------


# Map ROS PointField datatype -> (human name, numpy dtype)
_ROS_DTYPE_MAP = {
    PointField.INT8: ("INT8", np.dtype(np.int8)),
    PointField.UINT8: ("UINT8", np.dtype(np.uint8)),
    PointField.INT16: ("INT16", np.dtype(np.int16)),
    PointField.UINT16: ("UINT16", np.dtype(np.uint16)),
    PointField.INT32: ("INT32", np.dtype(np.int32)),
    PointField.UINT32: ("UINT32", np.dtype(np.uint32)),
    PointField.FLOAT32: ("FLOAT32", np.dtype(np.float32)),
    PointField.FLOAT64: ("FLOAT64", np.dtype(np.float64)),
}


def ros_dtype_name_and_numpy(datatype: int):
    name, ndt = _ROS_DTYPE_MAP.get(datatype, (f"UNKNOWN({datatype})", None))
    return name, ndt


def _float_max_roundtrip_sigdigs(dtype: np.dtype) -> int:
    """
    IEEE-754 round-trip significant digits (aka max_digits10):
      p = (# mantissa bits) + 1 implicit leading bit
      max_digits10 = ceil(p * log10(2) + 1)
    """
    dt = np.dtype(dtype)
    if dt == np.dtype(np.float16) or dt == np.dtype(np.float32) or dt == np.dtype(np.float64):
        fi = np.finfo(dt)
        p = int(fi.nmant) + 1
        return int(math.ceil(p * math.log10(2.0) + 1.0))
    # fallback for unusual float dtypes
    return 17


class NumFormatter:
    def __init__(self, time_decimals: int, float_sigdigs_override=None):
        self.time_decimals = int(time_decimals)
        self.float_sigdigs_override = float_sigdigs_override

    def format_time_seconds(self, t: float) -> str:
        if t is None:
            return "N/A"
        # fixed-point, never scientific
        return format(float(t), f".{self.time_decimals}f")

    def _float_sigdigs_for_dtype(self, dtype: np.dtype) -> int:
        max_sd = _float_max_roundtrip_sigdigs(dtype)
        if self.float_sigdigs_override is None:
            return max_sd
        try:
            ov = int(self.float_sigdigs_override)
        except Exception:
            return max_sd
        return max(1, min(ov, max_sd))

    def format_float(self, x: float, dtype: np.dtype) -> str:
        # positional formatting (no exponent), trim trailing zeros/dot
        sd = self._float_sigdigs_for_dtype(dtype)
        return np.format_float_positional(
            np.dtype(dtype).type(x),
            precision=sd,
            unique=False,
            fractional=False,
            trim="-",
        )

    def format_scalar(self, v):
        # numpy scalar?
        if isinstance(v, np.generic):
            dt = v.dtype
            kind = dt.kind
            if kind == "f":
                return self.format_float(float(v), dt)
            if kind in ("i", "u"):
                return str(int(v))
            if kind == "b":
                return "True" if bool(v) else "False"
            return str(v)

        # python scalar?
        if isinstance(v, float):
            return self.format_float(v, np.dtype(np.float64))
        if isinstance(v, int):
            return str(v)
        if isinstance(v, bool):
            return "True" if v else "False"

        return str(v)

    def format_value(self, v):
        # arrays for count>1 fields (or nested)
        if isinstance(v, np.ndarray):
            return "[" + ", ".join(self.format_value(x) for x in v.tolist()) + "]"
        if isinstance(v, (tuple, list)):
            return "[" + ", ".join(self.format_value(x) for x in v) + "]"
        return self.format_scalar(v)


def format_fields_table(fields):
    """
    Return a multi-line aligned table describing fields and their storage dtypes.
    """
    rows = []
    for f in fields:
        ros_name, ndt = ros_dtype_name_and_numpy(f.datatype)
        ndt_str = str(ndt) if ndt is not None else "?"
        rows.append((f.name, f.offset, f.count, ros_name, ndt_str))

    name_w = max(len(r[0]) for r in rows) if rows else 4
    header = f"{'name':<{name_w}}  {'off':>4}  {'cnt':>3}  {'ros_type':<10}  {'numpy_dtype'}"
    line = "-" * len(header)
    body = "\n".join(f"{n:<{name_w}}  {off:>4}  {cnt:>3}  {rt:<10}  {nd}" for (n, off, cnt, rt, nd) in rows)
    return f"{header}\n{line}\n{body}"


def as_structured_numpy_dtype(fields, point_step: int, is_bigendian: bool):
    """
    Construct a numpy structured dtype matching the PointCloud2 layout.
    Includes itemsize=point_step to preserve padding between points.

    For count>1 fields, dtype entry is shaped (count,).
    Respects msg.is_bigendian by setting dtype byteorder.
    """
    endian = ">" if is_bigendian else "<"

    names = []
    formats = []
    offsets = []

    for f in fields:
        _, base = ros_dtype_name_and_numpy(f.datatype)
        if base is None:
            raise ValueError(f"Unknown PointField datatype={f.datatype} for field '{f.name}'")

        ndt = np.dtype(base).newbyteorder(endian)

        names.append(f.name)
        if f.count == 1:
            formats.append(ndt)
        else:
            formats.append((ndt, (f.count,)))
        offsets.append(int(f.offset))

    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": int(point_step)})


def python_type_str(v):
    if isinstance(v, np.ndarray):
        return f"ndarray[{v.dtype}, shape={tuple(v.shape)}]"
    if isinstance(v, np.generic):
        return f"{type(v).__name__}[{v.dtype}]"
    if isinstance(v, (tuple, list)):
        inner = ", ".join(type(x).__name__ for x in v)
        return f"{type(v).__name__}[{inner}]"
    return type(v).__name__


def pc2_summary(msg, bag_time, msg_idx: int, fmt: NumFormatter):
    n_points = int(msg.width) * int(msg.height)
    frame_id = ""
    header_stamp_sec = None

    if getattr(msg, "header", None) is not None:
        frame_id = getattr(msg.header, "frame_id", "") or ""
        try:
            header_stamp_sec = msg.header.stamp.to_sec()
        except Exception:
            header_stamp_sec = None

    return (
        f"[{msg_idx:06d}] "
        f"bag_time={fmt.format_time_seconds(bag_time.to_sec())}  "
        f"header_stamp={fmt.format_time_seconds(header_stamp_sec)}  "
        f"frame_id='{frame_id}'  "
        f"w={msg.width} h={msg.height} points={n_points}  "
        f"point_step={msg.point_step} row_step={msg.row_step}  "
        f"is_bigendian={msg.is_bigendian} is_dense={msg.is_dense}"
    )


def pc2_to_structured_array(msg):
    """
    Convert PointCloud2 binary blob into a 1D numpy structured array of length width*height.

    Correctly handles row padding (row_step may be larger than width*point_step).
    """
    sdtype = as_structured_numpy_dtype(msg.fields, msg.point_step, msg.is_bigendian)

    w = int(msg.width)
    h = int(msg.height)
    ps = int(msg.point_step)
    rs = int(msg.row_step)

    data = memoryview(msg.data)
    rows = []
    for r in range(h):
        start = r * rs
        # Only take the meaningful points region in this row (ignore any row padding beyond w*ps)
        row_bytes = data[start : start + w * ps]
        row_arr = np.frombuffer(row_bytes, dtype=sdtype, count=w)
        rows.append(row_arr)

    if h == 1:
        return rows[0], sdtype
    return np.concatenate(rows, axis=0), sdtype


def _point_is_finite(point_struct, float_field_names):
    for name in float_field_names:
        v = point_struct[name]
        if isinstance(v, np.ndarray):
            if not np.all(np.isfinite(v)):
                return False
        else:
            if not np.isfinite(v):
                return False
    return True


def print_cloud_points(msg, cloud_print_index: int, max_points: int, fmt: NumFormatter):
    """
    Print field layout and first max_points points for the given PointCloud2 msg.
    """
    print("-" * 120)
    print(f"Cloud #{cloud_print_index} details")
    print("Fields (byte layout + storage dtype):")
    print(format_fields_table(msg.fields))

    field_names = [f.name for f in msg.fields]
    float_field_names = []
    for f in msg.fields:
        _, ndt = ros_dtype_name_and_numpy(f.datatype)
        if ndt is not None and np.dtype(ndt).kind == "f":
            float_field_names.append(f.name)

    try:
        arr, sdtype = pc2_to_structured_array(msg)
        print("\nNumpy structured dtype (verifies offsets/itemsize):")
        print(f"  {sdtype}")
        source = "numpy"
    except Exception as e:
        print(f"\nERROR: Failed to parse PointCloud2 binary data into numpy structured array:\n  {e}", file=sys.stderr)
        return

    print(f"\nFirst {max_points} points (all fields) [{source}]:")

    printed = 0
    for i in range(arr.shape[0]):
        p = arr[i]

        if SKIP_NANS and float_field_names:
            if not _point_is_finite(p, float_field_names):
                continue

        parts = []
        for name in field_names:
            val = p[name]
            sval = fmt.format_value(val)
            if SHOW_VALUE_TYPES:
                parts.append(f"{name}={sval}({python_type_str(val)})")
            else:
                parts.append(f"{name}={sval}")

        print(f"{printed:03d}: " + ", ".join(parts))
        printed += 1
        if printed >= max_points:
            break

    if printed == 0:
        print("  (no points printed; all points filtered or cloud empty)")


def parse_args():
    ap = argparse.ArgumentParser(
        description="Inspect sensor_msgs/PointCloud2 messages in a ROS1 bag with dtype-aware printing (no scientific notation)."
    )
    ap.add_argument(
        "--bag", required=True, help="Path to .bag file OR directory containing a matching bag (non-recursive)"
    )
    ap.add_argument("--topic", default=TOPIC, help="PointCloud2 topic name")
    ap.add_argument("--clouds", type=int, default=NUM_CLOUDS_TO_PRINT, help="Number of clouds to print details for")
    ap.add_argument("--points", type=int, default=NUM_POINTS_PER_CLOUD, help="Number of points to print per cloud")
    ap.add_argument(
        "--skip-nans",
        action="store_true",
        default=SKIP_NANS,
        help="Skip points with non-finite values in any float field",
    )
    ap.add_argument(
        "--summary",
        action="store_true",
        default=PRINT_SUMMARY_EACH_MSG,
        help="Print one-line summary for every message",
    )
    ap.add_argument(
        "--time-decimals", type=int, default=TIME_DECIMALS, help="Fixed decimals for timestamps (bag_time/header_stamp)"
    )
    ap.add_argument(
        "--float-sigdigs",
        type=int,
        default=FLOAT_SIGDIGS_OVERRIDE if FLOAT_SIGDIGS_OVERRIDE is not None else None,
        help="Clamp float significant digits (no scientific notation). Default: dtype max (float32->9, float64->17).",
    )
    ap.add_argument("--no-types", action="store_true", help="Do not print value types")
    return ap.parse_args()


def resolve_bag_path(bag_arg: str) -> str:
    """
    Resolve a bag file path.
    - If bag_arg is a file path, use it.
    - If bag_arg is a directory, pick exactly one match for BAG_GLOB_PATTERN in that directory (non-recursive).
    - Otherwise, treat bag_arg as a glob pattern and pick exactly one match (non-recursive unless user uses **).
    """
    if bag_arg is None:
        raise ValueError("bag path is None")

    bag_arg = os.path.expanduser(str(bag_arg))

    if os.path.isfile(bag_arg):
        return os.path.abspath(bag_arg)

    # Directory: search non-recursively for pattern in that directory
    if os.path.isdir(bag_arg):
        pattern = os.path.join(bag_arg, BAG_GLOB_PATTERN)
        matches = sorted([p for p in glob.glob(pattern) if os.path.isfile(p)])
        if len(matches) == 1:
            return os.path.abspath(matches[0])
        if len(matches) == 0:
            raise FileNotFoundError(f"No bag files match pattern '{BAG_GLOB_PATTERN}' in directory: {bag_arg}")
        raise RuntimeError("Multiple bag files matched; please disambiguate:\n" + "\n".join(f"  {m}" for m in matches))

    # Glob: search as given
    matches = sorted([p for p in glob.glob(bag_arg) if os.path.isfile(p)])
    if len(matches) == 1:
        return os.path.abspath(matches[0])
    if len(matches) == 0:
        raise FileNotFoundError(f"No bag files matched: {bag_arg}")
    raise RuntimeError("Multiple bag files matched; please disambiguate:\n" + "\n".join(f"  {m}" for m in matches))


def main():
    args = parse_args()

    global SKIP_NANS, SHOW_VALUE_TYPES
    SKIP_NANS = bool(args.skip_nans)
    SHOW_VALUE_TYPES = not bool(args.no_types)

    fmt = NumFormatter(time_decimals=args.time_decimals, float_sigdigs_override=args.float_sigdigs)

    try:
        bag_path = resolve_bag_path(args.bag)
        bag = rosbag.Bag(bag_path, "r")
    except Exception as e:
        print(f"ERROR: Failed to open bag: {args.bag}\n{e}", file=sys.stderr)
        return 2

    try:
        info = bag.get_type_and_topic_info()
        if args.topic not in info.topics:
            print(f"ERROR: Topic not found in bag: {args.topic}", file=sys.stderr)
            print("Available topics:", file=sys.stderr)
            for tname, tinfo in sorted(info.topics.items()):
                print(f"  {tname}  (type={tinfo.msg_type}, count={tinfo.message_count})", file=sys.stderr)
            return 3

        tinfo = info.topics[args.topic]
        print(f"Bag:   {bag_path}")
        print(f"Topic: {args.topic}")
        print(f"Type:  {tinfo.msg_type}")
        print(f"Count: {tinfo.message_count}")
        print("=" * 120)

        msg_idx = 0
        clouds_printed = 0

        for _topic, msg, bag_time in bag.read_messages(topics=[args.topic]):
            msg_idx += 1

            if args.summary:
                print(pc2_summary(msg, bag_time, msg_idx, fmt))

            if clouds_printed < int(args.clouds):
                clouds_printed += 1
                print_cloud_points(msg, clouds_printed, int(args.points), fmt)

        print("=" * 120)
        print(f"Done. Read {msg_idx} messages on {args.topic}.")
        print(
            f"Printed samples for {min(clouds_printed, int(args.clouds))} point clouds "
            f"({int(args.points)} points each)."
        )
        return 0

    finally:
        bag.close()


if __name__ == "__main__":
    raise SystemExit(main())
