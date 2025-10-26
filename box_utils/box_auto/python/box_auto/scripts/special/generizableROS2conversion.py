#!/usr/bin/env python3
# ROS1 .bag  ->  MCAP (ROS 2 profile, CDR-serialized ROS 2 messages)
# Supports: sensor_msgs/{Image,CompressedImage,CameraInfo,PointCloud2,Imu,Temperature,FluidPressure,MagneticField},
#           nav_msgs/Odometry,
#           geometry_msgs/{TwistStamped,PoseWithCovarianceStamped,PointStamped},
#           grid_map_msgs/GridMap,
#           tf2_msgs/TFMessage
# Optional: --decompress-images to turn CompressedImage into Image on the raw topic.
# Optional: --print-random-sample [--sample-prefer KIND] to print one ROS1 vs ROS2 message (header + data)

import sys
import argparse
import random
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple

import numpy as np
import cv2

from rosbags.highlevel import AnyReader

# MCAP generic writer (explicit ROS 2 schemas + CDR payloads)
from mcap.writer import Writer as McapWriter, CompressionType, IndexType

# ROS 2 type system & CDR serialization
from importlib.metadata import version
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

print("rosbags:", version("rosbags"))

from tqdm import tqdm
def _make_pbar(total: int):
    return tqdm(total=total, unit="msg", desc="Converting → ROS2/CDR")

# -------------------- time handling --------------------
_NSEC = 10**9

class RosTimeError(ValueError):
    pass

def _extract_sec_nsec(t) -> Tuple[int, int]:
    """Return (sec, nsec) from common ROS1/ROS2 representations."""
    if hasattr(t, "secs") and hasattr(t, "nsecs"):
        return int(t.secs), int(t.nsecs)
    if hasattr(t, "sec") and hasattr(t, "nanosec"):
        return int(t.sec), int(t.nanosec)
    if hasattr(t, "sec") and hasattr(t, "nsec"):
        return int(t.sec), int(t.nsec)
    raise RosTimeError(f"Unsupported time object: {type(t)}")

def time_to_nsec(t) -> int:
    """Robust time → nanoseconds."""
    if isinstance(t, (int, np.integer)):
        return int(t)
    if hasattr(t, "to_nsec"):
        return int(t.to_nsec())
    sec, nsec = _extract_sec_nsec(t)
    if not (0 <= nsec < _NSEC):
        raise RosTimeError(f"nsec out of range [0,1e9): {nsec}")
    return sec * _NSEC + nsec

def _stamp_is_zero(stamp) -> bool:
    try:
        s, ns = _extract_sec_nsec(stamp)
        return (s == 0 and ns == 0)
    except RosTimeError:
        return False

def extract_publish_and_log_ns(msg, bag_time_ns: int, *, allow_zero_header: bool = False) -> Tuple[int, int]:
    """Return (publish_ns, log_ns). Prefer top-level header; fallback to msg.info.header (GridMap in ROS1)."""
    log_ns = int(bag_time_ns)

    hdr = getattr(msg, "header", None)
    if hdr is not None and hasattr(hdr, "stamp"):
        st = hdr.stamp
        if _stamp_is_zero(st):
            if allow_zero_header:
                return (log_ns, log_ns)
            raise RosTimeError("header.stamp is zero (unset).")
        return time_to_nsec(st), log_ns

    # Fallback for ROS1 grid_map_msgs/GridMap: header lives in msg.info.header
    info = getattr(msg, "info", None)
    if info is not None:
        ihdr = getattr(info, "header", None)
        if ihdr is not None and hasattr(ihdr, "stamp"):
            st = ihdr.stamp
            if _stamp_is_zero(st):
                if allow_zero_header:
                    return (log_ns, log_ns)
                raise RosTimeError("info.header.stamp is zero (unset).")
            return time_to_nsec(st), log_ns

    return (log_ns, log_ns)

# -------------------- helpers --------------------
def _norm_type(t: str | None) -> str | None:
    """Normalize ROS type strings across ROS1/ROS2 forms."""
    if t is None:
        return None
    s = str(t).strip()
    s = s.replace("/msg/", "/").replace("::", "/")
    return s

def _datatype(entry):
    s = getattr(entry, "datatype", None)
    if s is None:
        try:
            s = entry[0]
        except Exception:
            return None
    return _norm_type(s)

def _is_type(entry, want: str) -> bool:
    return _datatype(entry) == _norm_type(want)

def _msg_count(entry):
    for attr in ("message_count", "messages", "count"):
        if hasattr(entry, attr):
            return int(getattr(entry, attr))
    try:
        return int(entry[1])
    except Exception:
        return 0

def _print_progress(processed, total, width=40):
    if total <= 0:
        sys.stderr.write(f"\rprocessed {processed} msgs"); sys.stderr.flush(); return
    ratio = processed / total if total else 0.0
    filled = int(ratio * width)
    bar = "#" * filled + "-" * (width - filled)
    sys.stderr.write(f"\r[{bar}] {processed}/{total} ({ratio*100:.1f}%)"); sys.stderr.flush()
    if processed == total:
        sys.stderr.write("\n")

def _raw_topic_from_compressed(topic: str) -> str:
    if topic.endswith("/compressed"):
        return topic[:-len("/compressed")]
    if topic.endswith("/compressedDepth"):
        return topic[:-len("/compressedDepth")]
    return topic

def _encoding_for_cv(img: np.ndarray) -> str:
    """Map NumPy image array to ROS encoding string."""
    if img.ndim == 2:
        ch = 1
    elif img.ndim == 3:
        ch = img.shape[2]
        if ch not in (1, 2, 3, 4):
            raise ValueError(f"Unsupported channel count: {ch}")
    else:
        raise ValueError(f"Unsupported ndim: {img.ndim}")

    dt = img.dtype
    if ch == 1:
        if dt == np.uint8:   return "mono8"
        if dt == np.uint16:  return "16UC1"
        if dt == np.int16:   return "16SC1"
        if dt == np.float32: return "32FC1"
        if dt == np.float64: return "64FC1"
    if ch == 2:
        if dt == np.uint8:   return "8UC2"
        if dt == np.uint16:  return "16UC2"
        if dt == np.int16:   return "16SC2"
        if dt == np.float32: return "32FC2"
        if dt == np.float64: return "64FC2"
    if ch == 3:
        if dt == np.uint8:   return "bgr8"
        if dt == np.uint16:  return "bgr16"
        if dt == np.int16:   return "16SC3"
        if dt == np.float32: return "32FC3"
        if dt == np.float64: return "64FC3"
    if ch == 4:
        if dt == np.uint8:   return "bgra8"
        if dt == np.uint16:  return "bgra16"
        if dt == np.int16:   return "16SC4"
        if dt == np.float32: return "32FC4"
        if dt == np.float64: return "64FC4"
    kind = dt.kind
    if kind == "u": tcode = "U"
    elif kind == "i": tcode = "S"
    elif kind == "f": tcode = "F"
    else:
        raise ValueError(f"Unsupported dtype kind: {kind} (dtype={dt})")
    return f"{dt.itemsize*8}{tcode}C{ch}"

def _sec_nsec_from_ns(ns: int):
    return int(ns // _NSEC), int(ns % _NSEC)

# ---- dump helpers for sample printing ----
def _stamp_dict(stamp):
    try:
        s, ns = _extract_sec_nsec(stamp)
        return {"sec": s, "nsec": ns}
    except Exception:
        return {"sec": None, "nsec": None}

def _header_dict(h):
    if h is None:
        return None
    return {"stamp": _stamp_dict(getattr(h, "stamp", None)),
            "frame_id": getattr(h, "frame_id", None)}

def _dump_ros1_vs_ros2(topic, kind, ros1_msg, ros2_msg, publish_ns, log_ns):
    # Minimal fix: print logical ROS 2 schema name instead of Python class.
    def _ros2_typename(obj, kind):
        if hasattr(obj, "format"):
            return "sensor_msgs/msg/CompressedImage"
        if hasattr(obj, "encoding"):
            return "sensor_msgs/msg/Image"
        return {
            "cinfo": "sensor_msgs/msg/CameraInfo",
            "imu":   "sensor_msgs/msg/Imu",
            "pc2":   "sensor_msgs/msg/PointCloud2",
            "odom":  "nav_msgs/msg/Odometry",
            "twist": "geometry_msgs/msg/TwistStamped",
            "pwcs":  "geometry_msgs/msg/PoseWithCovarianceStamped",
            "pnt":   "geometry_msgs/msg/PointStamped",
            "temp":  "sensor_msgs/msg/Temperature",
            "press": "sensor_msgs/msg/FluidPressure",
            "mag":   "sensor_msgs/msg/MagneticField",
            "gmap":  "grid_map_msgs/msg/GridMap",
            "tf":    "tf2_msgs/msg/TFMessage",
            "tfs":   "tf2_msgs/msg/TFMessage",
        }.get(kind, str(type(obj)))

    # For ROS1, try top-level header; if None, try info.header (GridMap ROS1)
    ros1_hdr = getattr(ros1_msg, "header", None)
    if ros1_hdr is None and hasattr(ros1_msg, "info"):
        ros1_hdr = getattr(ros1_msg.info, "header", None)

    sys.stderr.write("\n=== SAMPLE MESSAGE COMPARISON ===\n")
    sys.stderr.write(f"topic: {topic}\nkind: {kind}\n")
    sys.stderr.write(f"publish_ns: {publish_ns}\nlog_ns: {log_ns}\n")

    # ROS1
    sys.stderr.write("\n[ROS1]\n")
    sys.stderr.write(f"type: {type(ros1_msg)}\n")
    sys.stderr.write(f"header: { _header_dict(ros1_hdr) }\n")
    if hasattr(ros1_msg, "format"):
        sys.stderr.write(f"format: {getattr(ros1_msg, 'format', None)}\n")

    # ROS2
    sys.stderr.write("\n[ROS2]\n")
    sys.stderr.write(f"type: {_ros2_typename(ros2_msg, kind)}\n")
    sys.stderr.write(f"header: { _header_dict(getattr(ros2_msg, 'header', None)) }\n")
    if hasattr(ros2_msg, "format"):
        sys.stderr.write(f"format: {getattr(ros2_msg, 'format', None)}\n")
    sys.stderr.write("=== END SAMPLE ===\n\n")

# -------------------- ROS 2 message definitions (explicit) --------------------
TIME_DEF = """int32 sec
uint32 nanosec
"""
HEADER_DEF = """builtin_interfaces/Time stamp
string frame_id
"""
ROI_DEF = """uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
"""
CAMERAINFO_DEF = """std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
"""
IMAGE_DEF = """std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
"""
CIMAGE_DEF = """std_msgs/Header header
string format
uint8[] data
"""
QUAT_DEF = """float64 x
float64 y
float64 z
float64 w
"""
VEC3_DEF = """float64 x
float64 y
float64 z
"""
IMU_DEF = """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
"""
PFIELD_DEF = """string name
uint32 offset
uint8 datatype
uint32 count
"""
PC2_DEF = """std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
"""
POINT_DEF = """float64 x
float64 y
float64 z
"""
POINTSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/Point point
"""
POSE_DEF = """geometry_msgs/Point position
geometry_msgs/Quaternion orientation
"""
POSECOV_DEF = """geometry_msgs/Pose pose
float64[36] covariance
"""
POSECOVSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
"""
TWIST_DEF = """geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
"""
TWISTCOV_DEF = """geometry_msgs/Twist twist
float64[36] covariance
"""
ODOM_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
"""
TWISTSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/Twist twist
"""

# Temperature
TEMP_DEF = """std_msgs/Header header
float64 temperature
float64 variance
"""

# FluidPressure
FLUID_DEF = """std_msgs/Header header
float64 fluid_pressure
float64 variance
"""

# MagneticField
MAG_DEF = """std_msgs/Header header
geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance
"""

# ----- tf2_msgs / geometry_msgs for TF -----
TRANS_DEF = """geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
"""
TRANSSTAMPED_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
"""
TFMSG_DEF = """geometry_msgs/TransformStamped[] transforms
"""

# ----- grid_map_msgs (ROS 2 Jazzy) + std_msgs arrays -----
MADIM_DEF = """string label
uint32 size
uint32 stride
"""
MALAYOUT_DEF = """std_msgs/MultiArrayDimension[] dim
uint32 data_offset
"""
F32MA_DEF = """std_msgs/MultiArrayLayout layout
float32[] data
"""
GRIDMAPINFO_DEF = """float32 resolution
float32 length_x
float32 length_y
geometry_msgs/Pose pose
"""
GRIDMAP_DEF = """std_msgs/Header header
grid_map_msgs/GridMapInfo info
string[] layers
string[] basic_layers
std_msgs/Float32MultiArray[] data
uint16 outer_start_index
uint16 inner_start_index
"""

# -------------------- conversion script --------------------
def main():
    ap = argparse.ArgumentParser(description="ROS1 .bag → MCAP (ROS 2 profile, CDR) with optional image decompression")
    ap.add_argument("--inbag", required=False, help="input ROS1 .bag", default="/media/tutuna/LaCie/to_elliot/dlio_replayed.bag")

    ap.add_argument("--decompress-images", type=int, choices=[0, 1], default=1,
                   help="If set, decode CompressedImage to Image on the raw topic (default: keep compressed).")

    ap.add_argument("--print-random-sample", type=int, choices=[0, 1], default=1,
                   help="Print full header and data of one random message (ROS1 vs ROS2).")

    ap.add_argument("--sample-prefer",
                    choices=["cimg","img","cinfo","imu","pc2","odom","twist","pwcs","pnt","temp","press","mag","gmap","tf","tfs"],
                    default=None,
                    help="Prefer sampling from this kind if present. Defaults to cimg if available.")

    args = ap.parse_args()

    # Normalize paths and validate
    bag_path = Path(args.inbag).expanduser()
    if not bag_path.is_file():
        sys.exit(f"Input bag not found: {bag_path}")

    # Replace .bag extension with .mcap for output
    if bag_path.suffix.lower() == '.bag':
        out_path = Path(str(bag_path.with_suffix('.mcap')))
    else:
        out_path = bag_path.with_suffix('.mcap')

    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Typestore + explicit definitions (for writing)
    ts = get_typestore(Stores.ROS2_JAZZY)
    ts.register(get_types_from_msg(TIME_DEF,      "builtin_interfaces/msg/Time"))
    ts.register(get_types_from_msg(HEADER_DEF,    "std_msgs/msg/Header"))
    ts.register(get_types_from_msg(ROI_DEF,       "sensor_msgs/msg/RegionOfInterest"))
    ts.register(get_types_from_msg(CAMERAINFO_DEF,"sensor_msgs/msg/CameraInfo"))
    ts.register(get_types_from_msg(IMAGE_DEF,     "sensor_msgs/msg/Image"))
    ts.register(get_types_from_msg(CIMAGE_DEF,    "sensor_msgs/msg/CompressedImage"))
    ts.register(get_types_from_msg(QUAT_DEF,      "geometry_msgs/msg/Quaternion"))
    ts.register(get_types_from_msg(VEC3_DEF,      "geometry_msgs/msg/Vector3"))
    ts.register(get_types_from_msg(IMU_DEF,       "sensor_msgs/msg/Imu"))
    ts.register(get_types_from_msg(PFIELD_DEF,    "sensor_msgs/msg/PointField"))
    ts.register(get_types_from_msg(PC2_DEF,       "sensor_msgs/msg/PointCloud2"))
    ts.register(get_types_from_msg(POINT_DEF,     "geometry_msgs/msg/Point"))
    ts.register(get_types_from_msg(POINTSTAMPED_DEF, "geometry_msgs/msg/PointStamped"))
    ts.register(get_types_from_msg(POSE_DEF,      "geometry_msgs/msg/Pose"))
    ts.register(get_types_from_msg(POSECOV_DEF,   "geometry_msgs/msg/PoseWithCovariance"))
    ts.register(get_types_from_msg(POSECOVSTAMPED_DEF, "geometry_msgs/msg/PoseWithCovarianceStamped"))
    ts.register(get_types_from_msg(TWIST_DEF,     "geometry_msgs/msg/Twist"))
    ts.register(get_types_from_msg(TWISTCOV_DEF,  "geometry_msgs/msg/TwistWithCovariance"))
    ts.register(get_types_from_msg(ODOM_DEF,      "nav_msgs/msg/Odometry"))
    ts.register(get_types_from_msg(TWISTSTAMPED_DEF, "geometry_msgs/msg/TwistStamped"))

    # Temperature / FluidPressure / MagneticField
    ts.register(get_types_from_msg(TEMP_DEF,      "sensor_msgs/msg/Temperature"))
    ts.register(get_types_from_msg(FLUID_DEF,     "sensor_msgs/msg/FluidPressure"))
    ts.register(get_types_from_msg(MAG_DEF,       "sensor_msgs/msg/MagneticField"))

    # tf2 / geometry for TF
    ts.register(get_types_from_msg(TRANS_DEF,        "geometry_msgs/msg/Transform"))
    ts.register(get_types_from_msg(TRANSSTAMPED_DEF, "geometry_msgs/msg/TransformStamped"))
    ts.register(get_types_from_msg(TFMSG_DEF,        "tf2_msgs/msg/TFMessage"))

    # std_msgs arrays + grid_map_msgs
    ts.register(get_types_from_msg(MADIM_DEF,         "std_msgs/msg/MultiArrayDimension"))
    ts.register(get_types_from_msg(MALAYOUT_DEF,      "std_msgs/msg/MultiArrayLayout"))
    ts.register(get_types_from_msg(F32MA_DEF,         "std_msgs/msg/Float32MultiArray"))
    ts.register(get_types_from_msg(GRIDMAPINFO_DEF,   "grid_map_msgs/msg/GridMapInfo"))
    ts.register(get_types_from_msg(GRIDMAP_DEF,       "grid_map_msgs/msg/GridMap"))

    # Type aliases (for writing)
    Time                 = ts.types["builtin_interfaces/msg/Time"]
    Header               = ts.types["std_msgs/msg/Header"]
    RegionOfInterest     = ts.types["sensor_msgs/msg/RegionOfInterest"]
    CameraInfo           = ts.types["sensor_msgs/msg/CameraInfo"]
    Image                = ts.types["sensor_msgs/msg/Image"]
    CompressedImage      = ts.types["sensor_msgs/msg/CompressedImage"]
    Quaternion           = ts.types["geometry_msgs/msg/Quaternion"]
    Vector3              = ts.types["geometry_msgs/msg/Vector3"]
    Imu                  = ts.types["sensor_msgs/msg/Imu"]
    PointField           = ts.types["sensor_msgs/msg/PointField"]
    Point                = ts.types["geometry_msgs/msg/Point"]
    PointStamped         = ts.types["geometry_msgs/msg/PointStamped"]
    Pose                 = ts.types["geometry_msgs/msg/Pose"]
    PoseWithCov          = ts.types["geometry_msgs/msg/PoseWithCovariance"]
    PoseWithCovStamped   = ts.types["geometry_msgs/msg/PoseWithCovarianceStamped"]
    Twist                = ts.types["geometry_msgs/msg/Twist"]
    TwistWithCov         = ts.types["geometry_msgs/msg/TwistWithCovariance"]
    Odometry             = ts.types["nav_msgs/msg/Odometry"]
    TwistStamped         = ts.types["geometry_msgs/msg/TwistStamped"]

    Temperature          = ts.types["sensor_msgs/msg/Temperature"]
    FluidPressure        = ts.types["sensor_msgs/msg/FluidPressure"]
    MagneticField        = ts.types["sensor_msgs/msg/MagneticField"]

    # TF types
    Transform            = ts.types["geometry_msgs/msg/Transform"]
    TransformStamped     = ts.types["geometry_msgs/msg/TransformStamped"]
    TFMessage            = ts.types["tf2_msgs/msg/TFMessage"]

    # New: array + grid_map types
    MultiArrayDimension  = ts.types["std_msgs/msg/MultiArrayDimension"]
    MultiArrayLayout     = ts.types["std_msgs/msg/MultiArrayLayout"]
    Float32MultiArray    = ts.types["std_msgs/msg/Float32MultiArray"]
    GridMapInfo          = ts.types["grid_map_msgs/msg/GridMapInfo"]
    GridMap              = ts.types["grid_map_msgs/msg/GridMap"]

    with AnyReader([bag_path]) as bag, open(out_path, "wb") as fh:
        conns = list(bag.connections)
        topics_info = {
            c.topic: SimpleNamespace(
                datatype=c.msgtype,
                message_count=getattr(c, "msgcount", getattr(c, "count", 0)),
            )
            for c in conns
        }

        # Helper for TF/TFMessage type detection (ROS1 'tf/tfMessage' or tf2)
        def _is_tf_datatype(ti):
            return _is_type(ti, "tf2_msgs/TFMessage") or _is_type(ti, "tf/tfMessage")

        # Discover topics present (normalized)
        img_raw_topics        = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/Image")]
        img_compressed_topics = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/CompressedImage")]
        cam_info_topics       = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/CameraInfo")]
        imu_topics            = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/Imu")]
        pc2_topics            = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/PointCloud2")]
        odom_topics           = [t for t, ti in topics_info.items() if _is_type(ti, "nav_msgs/Odometry")]
        twist_topics          = [t for t, ti in topics_info.items() if _is_type(ti, "geometry_msgs/TwistStamped")]
        pwcs_topics           = [t for t, ti in topics_info.items() if _is_type(ti, "geometry_msgs/PoseWithCovarianceStamped")]
        pnt_topics            = [t for t, ti in topics_info.items() if _is_type(ti, "geometry_msgs/PointStamped")]
        temp_topics           = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/Temperature")]
        press_topics          = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/FluidPressure")]
        mag_topics            = [t for t, ti in topics_info.items() if _is_type(ti, "sensor_msgs/MagneticField")]
        gmap_topics           = [t for t, ti in topics_info.items() if _is_type(ti, "grid_map_msgs/GridMap")]

        # TF topics: split dynamic vs static by name suffix to preserve QoS semantics
        tf_static_topics      = [t for t, ti in topics_info.items() if _is_tf_datatype(ti) and t.endswith("/tf_static")]
        tf_topics             = [t for t, ti in topics_info.items() if _is_tf_datatype(ti) and t.endswith("/tf") and t not in tf_static_topics]

        if not (img_raw_topics or img_compressed_topics or cam_info_topics or imu_topics or pc2_topics or
                odom_topics or twist_topics or pwcs_topics or pnt_topics or temp_topics or press_topics or mag_topics
                or gmap_topics or tf_topics or tf_static_topics):
            seen = sorted({ _datatype(ti) for ti in topics_info.values() if _datatype(ti) })
            sys.stderr.write("No supported topics found in bag.\n")
            sys.stderr.write("Seen datatypes:\n  " + "\n  ".join(seen) + "\n")
            sys.exit(2)

        topic_kind = {}
        topic_kind.update({t: "img"   for t in img_raw_topics})
        topic_kind.update({t: "cimg"  for t in img_compressed_topics})
        topic_kind.update({t: "cinfo" for t in cam_info_topics})
        topic_kind.update({t: "imu"   for t in imu_topics})
        topic_kind.update({t: "pc2"   for t in pc2_topics})
        topic_kind.update({t: "odom"  for t in odom_topics})
        topic_kind.update({t: "twist" for t in twist_topics})
        topic_kind.update({t: "pwcs"  for t in pwcs_topics})
        topic_kind.update({t: "pnt"   for t in pnt_topics})
        topic_kind.update({t: "temp"  for t in temp_topics})
        topic_kind.update({t: "press" for t in press_topics})
        topic_kind.update({t: "mag"   for t in mag_topics})
        topic_kind.update({t: "gmap"  for t in gmap_topics})
        topic_kind.update({t: "tf"    for t in tf_topics})
        topic_kind.update({t: "tfs"   for t in tf_static_topics})   # <-- static TF

        wanted = tuple(topic_kind.keys())
        total_msgs = sum(_msg_count(ti) for t, ti in topics_info.items() if t in topic_kind)

        # ---- sampling setup ----
        sample_enabled = bool(args.print_random_sample)
        prefer_order = ["cimg", "img", "cinfo", "imu", "pc2", "odom", "twist", "pwcs", "pnt", "temp", "press", "mag", "gmap", "tf", "tfs"]
        if args.sample_prefer:
            prefer_order = [args.sample_prefer] + [k for k in prefer_order if k != args.sample_prefer]

        chosen_topic = None
        if sample_enabled:
            for k in prefer_order:
                candidates = [t for t in wanted if topic_kind[t] == k]
                if candidates:
                    chosen_topic = random.choice(candidates)
                    break
            if chosen_topic is None:
                chosen_topic = random.choice(wanted)

        per_topic_seen = defaultdict(int)
        sample_target_index = None
        if sample_enabled and chosen_topic in topics_info:
            n = topics_info[chosen_topic].message_count or 1
            sample_target_index = random.randint(0, max(0, n - 1))

        raw_topics_set = set(img_raw_topics)

        processed = 0
        pbar = _make_pbar(total_msgs) if total_msgs > 0 else None

        mw = McapWriter(
            fh,
            chunk_size=4*1024*1024,
            compression=CompressionType.ZSTD,
            index_types=IndexType.ALL,
            repeat_channels=True,
            repeat_schemas=True,
            use_chunking=True,
            use_statistics=True,
            use_summary_offsets=True,
            enable_crcs=True,
            enable_data_crcs=False,
        )
        mw.start("ros2")

        def _reg(name, defn):
            return mw.register_schema(name=name, encoding="ros2msg", data=defn.encode())

        schema_ids = {
            "sensor_msgs/msg/CameraInfo":                  _reg("sensor_msgs/msg/CameraInfo", CAMERAINFO_DEF),
            "sensor_msgs/msg/Image":                       _reg("sensor_msgs/msg/Image", IMAGE_DEF),
            "sensor_msgs/msg/CompressedImage":             _reg("sensor_msgs/msg/CompressedImage", CIMAGE_DEF),
            "sensor_msgs/msg/Imu":                         _reg("sensor_msgs/msg/Imu", IMU_DEF),
            "sensor_msgs/msg/PointCloud2":                 _reg("sensor_msgs/msg/PointCloud2", PC2_DEF),
            "nav_msgs/msg/Odometry":                       _reg("nav_msgs/msg/Odometry", ODOM_DEF),
            "geometry_msgs/msg/TwistStamped":              _reg("geometry_msgs/msg/TwistStamped", TWISTSTAMPED_DEF),
            "geometry_msgs/msg/PoseWithCovarianceStamped": _reg("geometry_msgs/msg/PoseWithCovarianceStamped", POSECOVSTAMPED_DEF),
            "geometry_msgs/msg/PointStamped":              _reg("geometry_msgs/msg/PointStamped", POINTSTAMPED_DEF),
            "sensor_msgs/msg/Temperature":                 _reg("sensor_msgs/msg/Temperature", TEMP_DEF),
            "sensor_msgs/msg/FluidPressure":               _reg("sensor_msgs/msg/FluidPressure", FLUID_DEF),
            "sensor_msgs/msg/MagneticField":               _reg("sensor_msgs/msg/MagneticField", MAG_DEF),
            "grid_map_msgs/msg/GridMap":                   _reg("grid_map_msgs/msg/GridMap", GRIDMAP_DEF),
            "tf2_msgs/msg/TFMessage":                      _reg("tf2_msgs/msg/TFMessage", TFMSG_DEF),
        }

        # QoS metadata helper so rosbag2 treats TF topics with expected QoS
        def _qos_meta(topic: str) -> dict:
            if topic.endswith("/tf_static"):
                return {
                    "offered_qos_profiles": (
                        "[{history: KEEP_LAST, depth: 1, reliability: RELIABLE, "
                        "durability: TRANSIENT_LOCAL, deadline: {sec: 2147483647, nsec: 2147483647}, "
                        "liveliness: AUTOMATIC, liveliness_lease_duration: {sec: 2147483647, nsec: 2147483647}}]"
                    )
                }
            if topic.endswith("/tf"):
                return {
                    "offered_qos_profiles": (
                        "[{history: KEEP_LAST, depth: 100, reliability: BEST_EFFORT, "
                        "durability: VOLATILE, deadline: {sec: 2147483647, nsec: 2147483647}, "
                        "liveliness: AUTOMATIC, liveliness_lease_duration: {sec: 2147483647, nsec: 2147483647}}]"
                    )
                }
            return {}

        topic_to_channel = {}
        def get_channel(topic: str, fulltypename: str) -> int:
            if topic in topic_to_channel:
                return topic_to_channel[topic]
            cid = mw.register_channel(
                topic=topic,
                message_encoding="cdr",
                schema_id=schema_ids[fulltypename],
                metadata=_qos_meta(topic),   # <-- attach QoS metadata for TF(/_static)
            )
            topic_to_channel[topic] = cid
            return cid

        sample_printed = False

        want_conns = [c for c in conns if c.topic in wanted]
        for conn, log_ns, raw in bag.messages(connections=want_conns):
            topic = conn.topic
            kind = topic_kind[topic]
            msg = bag.deserialize(raw, conn.msgtype)
            publish_ns, log_ns = extract_publish_and_log_ns(msg, log_ns, allow_zero_header=False)

            # per-topic index tracking for sampling
            idx_this = per_topic_seen[topic]
            per_topic_seen[topic] += 1
            sample_hit = (sample_enabled and not sample_printed and topic == chosen_topic and idx_this == sample_target_index)

            if kind == "cinfo":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                D  = np.asarray(msg.D, dtype=np.float64)
                K  = np.asarray(msg.K, dtype=np.float64)
                Rm = np.asarray(msg.R, dtype=np.float64)
                Pm = np.asarray(msg.P, dtype=np.float64)
                roi = msg.roi
                ROI = RegionOfInterest(int(roi.x_offset), int(roi.y_offset),
                                       int(roi.height), int(roi.width), bool(roi.do_rectify))
                ros2_obj = CameraInfo(
                    hdr,
                    int(msg.height), int(msg.width),
                    msg.distortion_model,
                    D, K, Rm, Pm,
                    int(msg.binning_x), int(msg.binning_y),
                    ROI,
                )
                chan = get_channel(topic, "sensor_msgs/msg/CameraInfo")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CameraInfo")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "img":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                data = np.frombuffer(msg.data, dtype=np.uint8)
                ros2_obj = Image(
                    hdr,
                    int(msg.height), int(msg.width),
                    msg.encoding,
                    int(msg.is_bigendian),
                    int(msg.step),
                    data,
                )
                chan = get_channel(topic, "sensor_msgs/msg/Image")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Image")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "cimg":
                raw_topic = _raw_topic_from_compressed(topic)
                if args.decompress_images and raw_topic not in raw_topics_set:
                    try:
                        buf = np.frombuffer(msg.data, dtype=np.uint8)
                        if buf.size == 0:
                            raise ValueError("compressed payload is empty")
                        cv_img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                        if cv_img is None:
                            raise ValueError("cv2.imdecode returned None")
                        cv_img = np.ascontiguousarray(cv_img)

                        enc = _encoding_for_cv(cv_img)
                        hh, ww = int(cv_img.shape[0]), int(cv_img.shape[1])
                        ch = 1 if cv_img.ndim == 2 else int(cv_img.shape[2])
                        bytes_per_pixel = int(cv_img.dtype.itemsize * ch)
                        step = int(ww * bytes_per_pixel)

                        data = cv_img.ravel().view(np.uint8)
                        expected_len = int(hh * step)
                        if data.size != expected_len:
                            cv_img = np.ascontiguousarray(cv_img)
                            data = cv_img.ravel().view(np.uint8)
                            if data.size != expected_len:
                                raise ValueError(
                                    f"payload size mismatch for {enc}: data_len={data.size}, expected={expected_len}"
                                )

                        fmt = getattr(msg, "format", "") or ""
                        if "compressedDepth" in fmt and not (enc in ("16UC1", "32FC1") and ch == 1):
                            sys.stderr.write(
                                f"\n[WARN] compressedDepth format but decoded as enc={enc}, ch={ch}, dtype={cv_img.dtype}\n"
                            )

                        hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                        ros2_obj = Image(hdr, hh, ww, enc, 0, step, data)
                        chan = get_channel(raw_topic, "sensor_msgs/msg/Image")
                        payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Image")
                        mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)

                        if sample_hit:
                            _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                            sample_printed = True

                    except Exception as e:
                        sys.stderr.write(f"\n[WARN] decompress failed on {topic} @ {publish_ns}: {e}\n")
                        hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                        data = np.frombuffer(msg.data, dtype=np.uint8)
                        ros2_obj = CompressedImage(hdr, getattr(msg, "format", ""), data)
                        chan = get_channel(topic, "sensor_msgs/msg/CompressedImage")
                        payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CompressedImage")
                        mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                        if sample_hit:
                            _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                            sample_printed = True
                else:
                    hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                    data = np.frombuffer(msg.data, dtype=np.uint8)
                    ros2_obj = CompressedImage(hdr, getattr(msg, "format", ""), data)
                    chan = get_channel(topic, "sensor_msgs/msg/CompressedImage")
                    payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CompressedImage")
                    mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                    if sample_hit:
                        _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                        sample_printed = True

            elif kind == "imu":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                ori = msg.orientation
                av  = msg.angular_velocity
                la  = msg.linear_acceleration
                ros2_obj = Imu(
                    hdr,
                    Quaternion(ori.x, ori.y, ori.z, ori.w),
                    np.asarray(msg.orientation_covariance, dtype=np.float64),
                    Vector3(av.x, av.y, av.z),
                    np.asarray(msg.angular_velocity_covariance, dtype=np.float64),
                    Vector3(la.x, la.y, la.z),
                    np.asarray(msg.linear_acceleration_covariance, dtype=np.float64),
                )
                chan = get_channel(topic, "sensor_msgs/msg/Imu")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Imu")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pc2":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                fields = [PointField(f.name, int(f.offset), int(f.datatype), int(f.count)) for f in msg.fields]
                data = np.frombuffer(msg.data, dtype=np.uint8)
                ros2_obj = ts.types["sensor_msgs/msg/PointCloud2"](
                    hdr,
                    int(msg.height), int(msg.width),
                    fields,
                    bool(msg.is_bigendian),
                    int(msg.point_step), int(msg.row_step),
                    data,
                    bool(msg.is_dense),
                )
                chan = get_channel(topic, "sensor_msgs/msg/PointCloud2")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/PointCloud2")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "odom":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                tlin = msg.twist.twist.linear
                tang = msg.twist.twist.angular
                ros2_obj = Odometry(
                    hdr,
                    msg.child_frame_id,
                    PoseWithCov(
                        Pose(Point(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w)),
                        np.asarray(msg.pose.covariance, dtype=np.float64),
                    ),
                    TwistWithCov(
                        Twist(Vector3(tlin.x, tlin.y, tlin.z), Vector3(tang.x, tang.y, tang.z)),
                        np.asarray(msg.twist.covariance, dtype=np.float64),
                    ),
                )
                chan = get_channel(topic, "nav_msgs/msg/Odometry")
                payload = ts.serialize_cdr(ros2_obj, "nav_msgs/msg/Odometry")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "twist":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                lin = msg.twist.linear
                ang = msg.twist.angular
                ros2_obj = TwistStamped(
                    hdr,
                    Twist(Vector3(lin.x, lin.y, lin.z), Vector3(ang.x, ang.y, ang.z)),
                )
                chan = get_channel(topic, "geometry_msgs/msg/TwistStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/TwistStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pwcs":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                ros2_obj = PoseWithCovStamped(
                    hdr,
                    PoseWithCov(
                        Pose(Point(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w)),
                        np.asarray(msg.pose.covariance, dtype=np.float64),
                    ),
                )
                chan = get_channel(topic, "geometry_msgs/msg/PoseWithCovarianceStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/PoseWithCovarianceStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pnt":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                pt = msg.point
                ros2_obj = PointStamped(hdr, Point(pt.x, pt.y, pt.z))
                chan = get_channel(topic, "geometry_msgs/msg/PointStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/PointStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "temp":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                ros2_obj = Temperature(hdr, float(msg.temperature), float(msg.variance))
                chan = get_channel(topic, "sensor_msgs/msg/Temperature")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Temperature")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "press":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                ros2_obj = FluidPressure(hdr, float(msg.fluid_pressure), float(msg.variance))
                chan = get_channel(topic, "sensor_msgs/msg/FluidPressure")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/FluidPressure")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "mag":
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                mf = msg.magnetic_field
                ros2_obj = MagneticField(
                    hdr,
                    Vector3(float(mf.x), float(mf.y), float(mf.z)),
                    np.asarray(msg.magnetic_field_covariance, dtype=np.float64),
                )
                chan = get_channel(topic, "sensor_msgs/msg/MagneticField")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/MagneticField")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "gmap":
                # ROS1 GridMap: header lives in msg.info.header
                ihdr = msg.info.header
                hdr = Header(Time(*_sec_nsec_from_ns(publish_ns)), ihdr.frame_id)

                # info (ROS2 has no header inside GridMapInfo)
                pose_p = msg.info.pose.position
                pose_q = msg.info.pose.orientation
                info2 = GridMapInfo(
                    float(msg.info.resolution),
                    float(msg.info.length_x),
                    float(msg.info.length_y),
                    Pose(Point(pose_p.x, pose_p.y, pose_p.z),
                         Quaternion(pose_q.x, pose_q.y, pose_q.z, pose_q.w)),
                )

                layers = list(msg.layers)
                basic_layers = list(msg.basic_layers)

                # Convert each Float32MultiArray with layout
                data_arr = []
                for ma in msg.data:
                    dims = [MultiArrayDimension(d.label, int(d.size), int(d.stride)) for d in ma.layout.dim]
                    layout = MultiArrayLayout(dims, int(ma.layout.data_offset))
                    arr = np.asarray(ma.data, dtype=np.float32)
                    data_arr.append(Float32MultiArray(layout, arr))

                ros2_obj = GridMap(
                    hdr,
                    info2,
                    layers,
                    basic_layers,
                    data_arr,
                    int(msg.outer_start_index),
                    int(msg.inner_start_index),
                )

                chan = get_channel(topic, "grid_map_msgs/msg/GridMap")
                payload = ts.serialize_cdr(ros2_obj, "grid_map_msgs/msg/GridMap")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind in ("tf", "tfs"):
                # Use bag's log time as publish_time (exactly match ROS1 bag timing)
                publish_ns_tf = log_ns

                # Build ROS 2 TFMessage from array of TransformStamped
                tlist = []
                for tr in getattr(msg, "transforms", []):
                    try:
                        t_stamp_ns = time_to_nsec(tr.header.stamp)
                        t_hdr = Header(Time(*_sec_nsec_from_ns(t_stamp_ns)), tr.header.frame_id)
                        trans = tr.transform
                        t_trans = Transform(
                            Vector3(trans.translation.x, trans.translation.y, trans.translation.z),
                            Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w),
                        )
                        tlist.append(TransformStamped(t_hdr, tr.child_frame_id, t_trans))
                    except Exception as e:
                        sys.stderr.write(f"\n[WARN] skipping malformed TransformStamped in {topic}: {e}\n")

                if not tlist:
                    continue

                ros2_obj = TFMessage(tlist)
                chan = get_channel(topic, "tf2_msgs/msg/TFMessage")
                payload = ts.serialize_cdr(ros2_obj, "tf2_msgs/msg/TFMessage")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns_tf, data=payload)

                if sample_hit:
                    _dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns_tf, log_ns)
                    sample_printed = True

            else:
                raise RuntimeError(f"Unknown kind for topic {topic}")

            processed += 1
            if pbar is not None:
                pbar.update(1)
            else:
                _print_progress(processed, total_msgs)

        mw.finish()

    if 'pbar' in locals() and pbar is not None:
        pbar.close()
    else:
        _print_progress(processed, total_msgs)

if __name__ == "__main__":
    main()
