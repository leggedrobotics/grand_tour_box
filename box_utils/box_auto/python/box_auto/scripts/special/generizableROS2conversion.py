#!/usr/bin/env python3
# ROS1 .bag → MCAP (ROS 2 profile, CDR-serialized ROS 2 messages).
# Standard topics + splitting anymal_msgs/AnymalState into standard ROS 2 topics.
# Extended: split series_elastic_actuator_msgs/SeActuatorReadings into standard ROS 2 messages.
# IMU in SEA readings is skipped; everything else is converted per joint.

import sys
import argparse
import random
import re
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple, Optional

import numpy as np
import cv2

from rosbags.highlevel import AnyReader
from mcap.writer import Writer as McapWriter, CompressionType, IndexType
from importlib.metadata import version
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

print("rosbags:", version("rosbags"))

from tqdm import tqdm
def make_pbar(total: int):
    return tqdm(total=total, unit="msg", desc="Converting → ROS2/CDR")

# -------------------- hard-coded batch option --------------------
# If True, iterate over all .bag files in a folder.
BATCH_MODE: bool = True
# If None, uses parent directory of --inbag if it is a file,
# or uses --inbag itself if it’s a directory.
BATCH_DIR: Optional[str] = ""
# Glob used to pick bag files from BATCH_DIR
BATCH_GLOB: str = "*.bag"

# -------------------- time handling --------------------
_NSEC = 10**9

class RosTimeError(ValueError):
    pass

def extract_sec_nsec(t) -> Tuple[int, int]:
    if hasattr(t, "secs") and hasattr(t, "nsecs"):
        return int(t.secs), int(t.nsecs)
    if hasattr(t, "sec") and hasattr(t, "nanosec"):
        return int(t.sec), int(t.nanosec)
    if hasattr(t, "sec") and hasattr(t, "nsec"):
        return int(t.sec), int(t.nsec)
    raise RosTimeError(f"Unsupported time object: {type(t)}")

def time_to_nsec(t) -> int:
    if isinstance(t, (int, np.integer)):
        return int(t)
    if hasattr(t, "to_nsec"):
        return int(t.to_nsec())
    sec, nsec = extract_sec_nsec(t)
    if not (0 <= nsec < _NSEC):
        raise RosTimeError(f"nsec out of range [0,1e9): {nsec}")
    return sec * _NSEC + nsec

def stamp_is_zero(stamp) -> bool:
    try:
        s, ns = extract_sec_nsec(stamp)
        return (s == 0 and ns == 0)
    except RosTimeError:
        return False

def extract_publish_and_log_ns(msg, bag_time_ns: int, *, allow_zero_header: bool = False) -> Tuple[int, int]:
    log_ns = int(bag_time_ns)
    hdr = getattr(msg, "header", None)
    if hdr is not None and hasattr(hdr, "stamp"):
        st = hdr.stamp
        if stamp_is_zero(st):
            if allow_zero_header:
                return (log_ns, log_ns)
            raise RosTimeError("header.stamp is zero (unset).")
        return time_to_nsec(st), log_ns
    info = getattr(msg, "info", None)
    if info is not None:
        ihdr = getattr(info, "header", None)
        if ihdr is not None and hasattr(ihdr, "stamp"):
            st = ihdr.stamp
            if stamp_is_zero(st):
                if allow_zero_header:
                    return (log_ns, log_ns)
                raise RosTimeError("info.header.stamp is zero (unset).")
            return time_to_nsec(st), log_ns
    return (log_ns, log_ns)

def sec_nsec_from_ns(ns: int):
    return int(ns // _NSEC), int(ns % _NSEC)

def hdr_stamp_ns(hdr, *, allow_zero: bool = False) -> Optional[int]:
    if hdr is None or not hasattr(hdr, "stamp"):
        return None
    try:
        s, ns = extract_sec_nsec(hdr.stamp)
        if s == 0 and ns == 0 and not allow_zero:
            return None
        return s * _NSEC + ns
    except Exception:
        return None

# -------------------- helpers --------------------
def norm_type(t: Optional[str]) -> Optional[str]:
    if t is None:
        return None
    s = str(t).strip()
    s = s.replace("/msg/", "/").replace("::", "/")
    return s

def datatype(entry):
    s = getattr(entry, "datatype", None)
    if s is None:
        try:
            s = entry[0]
        except Exception:
            return None
    return norm_type(s)

def is_type(entry, want: str) -> bool:
    return datatype(entry) == norm_type(want)

def is_anymal_state(entry) -> bool:
    return datatype(entry) in ("anymal_msgs/AnymalState",)

def msg_count(entry):
    for attr in ("message_count", "messages", "count"):
        if hasattr(entry, attr):
            return int(getattr(entry, attr))
    try:
        return int(entry[1])
    except Exception:
        return 0

def print_progress(processed, total, width=40):
    if total <= 0:
        sys.stderr.write(f"\rprocessed {processed} msgs"); sys.stderr.flush(); return
    ratio = processed / total if total else 0.0
    filled = int(ratio * width)
    bar = "#" * filled + "-" * (width - filled)
    sys.stderr.write(f"\r[{bar}] {processed}/{total} ({ratio*100:.1f}%)"); sys.stderr.flush()
    if processed == total:
        sys.stderr.write("\n")

def raw_topic_from_compressed(topic: str) -> str:
    if topic.endswith("/compressed"):
        return topic[:-len("/compressed")]
    if topic.endswith("/compressedDepth"):
        return topic[:-len("/compressedDepth")]
    return topic

def encoding_for_cv(img: np.ndarray) -> str:
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
        if dt == np.uint16:  return "16UC3"
        if dt == np.int16:   return "16SC3"
        if dt == np.float32: return "32FC3"
        if dt == np.float64: return "64FC3"
    if ch == 4:
        if dt == np.uint8:   return "bgra8"
        if dt == np.uint16:  return "16UC4"
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

# ---- debug helpers (for one sample print) ----
def stamp_dict(stamp):
    try:
        s, ns = extract_sec_nsec(stamp)
        return {"sec": s, "nsec": ns}
    except Exception:
        return {"sec": None, "nsec": None}

def header_dict(h):
    if h is None:
        return None
    return {"stamp": stamp_dict(getattr(h, "stamp", None)),
            "frame_id": getattr(h, "frame_id", None)}

def ros2_typename_for_kind(obj, kind: str):
    if hasattr(obj, "format"):
        return "sensor_msgs/msg/CompressedImage"
    if hasattr(obj, "encoding"):
        return "sensor_msgs/msg/Image"
    mapping = {
        "cinfo": "sensor_msgs/msg/CameraInfo",
        "imu":   "sensor_msgs/msg/Imu",
        "pc2":   "sensor_msgs/msg/PointCloud2",
        "odom":  "nav_msgs/msg/Odometry",
        "twist": "geometry_msgs/msg/TwistStamped",
        "twistcovs": "geometry_msgs/msg/TwistWithCovarianceStamped",
        "pwcs":  "geometry_msgs/msg/PoseWithCovarianceStamped",
        "pnt":   "geometry_msgs/msg/PointStamped",
        "temp":  "sensor_msgs/msg/Temperature",
        "press": "sensor_msgs/msg/FluidPressure",
        "mag":   "sensor_msgs/msg/MagneticField",
        "gmap":  "grid_map_msgs/msg/GridMap",
        "path":  "nav_msgs/msg/Path",
        "marker": "visualization_msgs/msg/Marker",
        "marray": "visualization_msgs/msg/MarkerArray",
        "tf":    "tf2_msgs/msg/TFMessage",
        "tfs":   "tf2_msgs/msg/TFMessage",
        "astate_pose":   "geometry_msgs/msg/PoseStamped",
        "astate_twist":  "geometry_msgs/msg/TwistStamped",
        "astate_joint":  "sensor_msgs/msg/JointState",
        "astate_state":  "std_msgs/msg/Int8",
        "astate_tf":     "tf2_msgs/msg/TFMessage",
        "astate_c_wrench":  "geometry_msgs/msg/WrenchStamped",
        "astate_c_pos":     "geometry_msgs/msg/PointStamped",
        "astate_c_norm":    "geometry_msgs/msg/Vector3Stamped",
        "sear_js": "sensor_msgs/msg/JointState",
        "sear_djs": "control_msgs/msg/DynamicJointState",
        "sear_status": "std_msgs/msg/UInt32",
    }
    return mapping.get(kind, str(type(obj)))

def dump_ros1_vs_ros2(topic, kind, ros1_msg, ros2_msg, publish_ns, log_ns):
    sys.stderr.write("\n=== SAMPLE MESSAGE COMPARISON ===\n")
    sys.stderr.write(f"topic: {topic}\nkind: {kind}\n")
    sys.stderr.write(f"publish_ns: {publish_ns}\nlog_ns: {log_ns}\n")
    ros1_hdr = getattr(ros1_msg, "header", None)
    if ros1_hdr is None and hasattr(ros1_msg, "info"):
        ros1_hdr = getattr(ros1_msg.info, "header", None)
    sys.stderr.write("\n[ROS1]\n")
    sys.stderr.write(f"type: {type(ros1_msg)}\n")
    sys.stderr.write(f"header: { header_dict(ros1_hdr) }\n")
    if hasattr(ros1_msg, "format"):
        sys.stderr.write(f"format: {getattr(ros1_msg, 'format', None)}\n")
    sys.stderr.write("\n[ROS2]\n")
    sys.stderr.write(f"type: {ros2_typename_for_kind(ros2_msg, kind)}\n")
    sys.stderr.write(f"header: { header_dict(getattr(ros2_msg, 'header', None)) }\n")
    if hasattr(ros2_msg, "format"):
        sys.stderr.write(f"format: {getattr(ros2_msg, 'format', None)}\n")
    sys.stderr.write("=== END SAMPLE ===\n\n")

# -------------------- ROS 2 message definitions (explicit, ROS2-style names) --------------------
TIME_DEF = """int32 sec
uint32 nanosec
"""
DURATION_DEF = """int32 sec
uint32 nanosec
"""
HEADER_DEF = """builtin_interfaces/msg/Time stamp
string frame_id
"""
ROI_DEF = """uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
"""
CAMERAINFO_DEF = """std_msgs/msg/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/msg/RegionOfInterest roi
"""
IMAGE_DEF = """std_msgs/msg/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
"""
CIMAGE_DEF = """std_msgs/msg/Header header
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
IMU_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/msg/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/msg/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
"""
PFIELD_DEF = """string name
uint32 offset
uint8 datatype
uint32 count
"""
PC2_DEF = """std_msgs/msg/Header header
uint32 height
uint32 width
sensor_msgs/msg/PointField[] fields
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
POINTSTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Point point
"""
POSE_DEF = """geometry_msgs/msg/Point position
geometry_msgs/msg/Quaternion orientation
"""
POSECOV_DEF = """geometry_msgs/msg/Pose pose
float64[36] covariance
"""
POSECOVSTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/PoseWithCovariance pose
"""
POSESTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Pose pose
"""
TWIST_DEF = """geometry_msgs/msg/Vector3 linear
geometry_msgs/msg/Vector3 angular
"""
TWISTSTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Twist twist
"""
TWISTCOV_DEF = """geometry_msgs/msg/Twist twist
float64[36] covariance
"""
TWISTCOVSTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/TwistWithCovariance twist
"""
ODOM_DEF = """std_msgs/msg/Header header
string child_frame_id
geometry_msgs/msg/PoseWithCovariance pose
geometry_msgs/msg/TwistWithCovariance twist
"""
PATH_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/PoseStamped[] poses
"""
TEMP_DEF = """std_msgs/msg/Header header
float64 temperature
float64 variance
"""
FLUID_DEF = """std_msgs/msg/Header header
float64 fluid_pressure
float64 variance
"""
MAG_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Vector3 magnetic_field
float64[9] magnetic_field_covariance
"""
TRANS_DEF = """geometry_msgs/msg/Vector3 translation
geometry_msgs/msg/Quaternion rotation
"""
TRANSSTAMPED_DEF = """std_msgs/msg/Header header
string child_frame_id
geometry_msgs/msg/Transform transform
"""
TFMSG_DEF = """geometry_msgs/msg/TransformStamped[] transforms
"""
MADIM_DEF = """string label
uint32 size
uint32 stride
"""
MALAYOUT_DEF = """std_msgs/msg/MultiArrayLayout layout
float32[] data
"""
F32MA_DEF = """std_msgs/msg/MultiArrayLayout layout
float32[] data
"""
GRIDMAPINFO_DEF = """float32 resolution
float32 length_x
float32 length_y
geometry_msgs/msg/Pose pose
"""
GRIDMAP_DEF = """std_msgs/msg/Header header
grid_map_msgs/msg/GridMapInfo info
string[] layers
string[] basic_layers
std_msgs/msg/Float32MultiArray[] data
uint16 outer_start_index
uint16 inner_start_index
"""
VEC3STAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Vector3 vector
"""
WRENCH_DEF = """geometry_msgs/msg/Vector3 force
geometry_msgs/msg/Vector3 torque
"""
WRENCHSTAMPED_DEF = """std_msgs/msg/Header header
geometry_msgs/msg/Wrench wrench
"""
JOINTSTATE_DEF = """std_msgs/msg/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
"""
INT8_DEF = """int8 data
"""
UINT8_DEF = """uint8 data
"""
UINT32_DEF = """uint32 data
"""
FLOAT64_DEF = """float64 data
"""
COLOR_DEF = """float32 r
float32 g
float32 b
float32 a
"""
# visualization_msgs
MARKER_DEF = """int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11
int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3
std_msgs/msg/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/msg/Pose pose
geometry_msgs/msg/Vector3 scale
std_msgs/msg/ColorRGBA color
builtin_interfaces/msg/Duration lifetime
bool frame_locked
geometry_msgs/msg/Point[] points
std_msgs/msg/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
"""
MARKERARRAY_DEF = """visualization_msgs/msg/Marker[] markers
"""

# ---- control_msgs (for DynamicJointState) ----
INTERFACEVALUE_DEF = """string interface_name
float64[] values
"""
DYNJOINTSTATE_DEF = """std_msgs/msg/Header header
string[] joint_names
string[] interface_names
control_msgs/msg/InterfaceValue[] interface_values
"""

# ==================== ros2msg schema bundling (fix health-check errors) ====================
_SEP_LINE = "=" * 78 + "\n"

def _canon(name: str) -> str:
    name = name.strip().replace("::", "/")
    if "/msg/" not in name and name.count("/") == 1:
        pkg, typ = name.split("/")
        return f"{pkg}/msg/{typ}"
    return name

def _strip_arrays(tok: str) -> str:
    return re.sub(r"(\[[^\]]*\])+$", "", tok.strip())

DEFS = {
    "builtin_interfaces/msg/Time": TIME_DEF,
    "builtin_interfaces/msg/Duration": DURATION_DEF,
    "std_msgs/msg/Header": HEADER_DEF,
    "std_msgs/msg/ColorRGBA": COLOR_DEF,
    "std_msgs/msg/MultiArrayDimension": MADIM_DEF,
    "std_msgs/msg/MultiArrayLayout": MALAYOUT_DEF,
    "std_msgs/msg/Float32MultiArray": F32MA_DEF,
    "std_msgs/msg/Int8": INT8_DEF,
    "std_msgs/msg/UInt8": UINT8_DEF,
    "std_msgs/msg/UInt32": UINT32_DEF,
    "std_msgs/msg/Float64": FLOAT64_DEF,
    "geometry_msgs/msg/Point": POINT_DEF,
    "geometry_msgs/msg/PointStamped": POINTSTAMPED_DEF,
    "geometry_msgs/msg/Quaternion": QUAT_DEF,
    "geometry_msgs/msg/Vector3": VEC3_DEF,
    "geometry_msgs/msg/Vector3Stamped": VEC3STAMPED_DEF,
    "geometry_msgs/msg/Pose": POSE_DEF,
    "geometry_msgs/msg/PoseWithCovariance": POSECOV_DEF,
    "geometry_msgs/msg/PoseWithCovarianceStamped": POSECOVSTAMPED_DEF,
    "geometry_msgs/msg/PoseStamped": POSESTAMPED_DEF,
    "geometry_msgs/msg/Twist": TWIST_DEF,
    "geometry_msgs/msg/TwistStamped": TWISTSTAMPED_DEF,
    "geometry_msgs/msg/TwistWithCovariance": TWISTCOV_DEF,
    "geometry_msgs/msg/TwistWithCovarianceStamped": TWISTCOVSTAMPED_DEF,
    "geometry_msgs/msg/Wrench": WRENCH_DEF,
    "geometry_msgs/msg/WrenchStamped": WRENCHSTAMPED_DEF,
    "geometry_msgs/msg/Transform": TRANS_DEF,
    "geometry_msgs/msg/TransformStamped": TRANSSTAMPED_DEF,
    "sensor_msgs/msg/RegionOfInterest": ROI_DEF,
    "sensor_msgs/msg/CameraInfo": CAMERAINFO_DEF,
    "sensor_msgs/msg/Image": IMAGE_DEF,
    "sensor_msgs/msg/CompressedImage": CIMAGE_DEF,
    "sensor_msgs/msg/Imu": IMU_DEF,
    "sensor_msgs/msg/PointField": PFIELD_DEF,
    "sensor_msgs/msg/PointCloud2": PC2_DEF,
    "sensor_msgs/msg/Temperature": TEMP_DEF,
    "sensor_msgs/msg/FluidPressure": FLUID_DEF,
    "sensor_msgs/msg/MagneticField": MAG_DEF,
    "sensor_msgs/msg/JointState": JOINTSTATE_DEF,
    "nav_msgs/msg/Odometry": ODOM_DEF,
    "nav_msgs/msg/Path": PATH_DEF,
    "tf2_msgs/msg/TFMessage": TFMSG_DEF,
    "grid_map_msgs/msg/GridMapInfo": GRIDMAPINFO_DEF,
    "grid_map_msgs/msg/GridMap": GRIDMAP_DEF,
    "visualization_msgs/msg/Marker": MARKER_DEF,
    "visualization_msgs/msg/MarkerArray": MARKERARRAY_DEF,
    "control_msgs/msg/InterfaceValue": INTERFACEVALUE_DEF,
    "control_msgs/msg/DynamicJointState": DYNJOINTSTATE_DEF,
}

_BUILTINS = {
    "bool","byte","char","int8","uint8","int16","uint16","int32","uint32",
    "int64","uint64","float32","float64","string","wstring","time","duration",
}

def _deps_from_def(def_text: str) -> list:
    out = []
    seen = set()
    for raw in def_text.replace("\r\n","\n").replace("\r","\n").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        ty = _strip_arrays(parts[0])
        if ty in _BUILTINS:
            continue
        if "/" in ty:
            name = _canon(ty)
            if name not in seen:
                seen.add(name)
                out.append(name)
    return out

def make_ros2msg_bundle(top_name: str) -> str:
    top = _canon(top_name)
    if top not in DEFS:
        raise KeyError(f"Definition not found for '{top}'")
    stack = [top]
    order = []
    visited = set()
    while stack:
        cur = stack.pop()
        if cur in visited:
            continue
        visited.add(cur)
        order.append(cur)
        for dep in _deps_from_def(DEFS[cur]):
            if dep not in visited and dep in DEFS:
                stack.append(dep)
    pieces = [DEFS[top].rstrip() + "\n"]
    for dep in order[1:]:
        pieces.append(_SEP_LINE)
        pieces.append(f"MSG: {dep}\n")
        pieces.append(DEFS[dep].rstrip() + "\n")
    return "".join(pieces)

# -------------------- ANYmal naming (index → names/frames) --------------------
ANYMAL_JOINT_NAMES = [
    "LF_HAA", "LF_HFE", "LF_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
]
ANYMAL_MEAS_FRAMES = [f"{n}_driver" for n in ANYMAL_JOINT_NAMES]

def canon_joint_name(idx: int, fallback: str) -> str:
    return ANYMAL_JOINT_NAMES[idx] if 0 <= idx < len(ANYMAL_JOINT_NAMES) else fallback

def canon_measurement_frame(idx: int, fallback: str) -> str:
    return ANYMAL_MEAS_FRAMES[idx] if 0 <= idx < len(ANYMAL_MEAS_FRAMES) else fallback

# -------------------- per-bag conversion --------------------
def convert_single_bag(bag_path: Path, args):
    if not bag_path.is_file():
        sys.exit(f"Input bag not found: {bag_path}")

    out_path = bag_path.with_suffix('.mcap')
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ts = get_typestore(Stores.ROS2_JAZZY)

    def ensure_type(name: str, defn: str):
        if name not in ts.types:
            ts.register(get_types_from_msg(defn, name))

    # Ensure required types
    ensure_type("builtin_interfaces/msg/Time", TIME_DEF)
    ensure_type("builtin_interfaces/msg/Duration", DURATION_DEF)
    ensure_type("std_msgs/msg/Header", HEADER_DEF)
    ensure_type("std_msgs/msg/ColorRGBA", COLOR_DEF)
    ensure_type("sensor_msgs/msg/RegionOfInterest", ROI_DEF)
    ensure_type("sensor_msgs/msg/CameraInfo", CAMERAINFO_DEF)
    ensure_type("sensor_msgs/msg/Image", IMAGE_DEF)
    ensure_type("sensor_msgs/msg/CompressedImage", CIMAGE_DEF)
    ensure_type("geometry_msgs/msg/Quaternion", QUAT_DEF)
    ensure_type("geometry_msgs/msg/Vector3", VEC3_DEF)
    ensure_type("sensor_msgs/msg/Imu", IMU_DEF)
    ensure_type("sensor_msgs/msg/PointField", PFIELD_DEF)
    ensure_type("sensor_msgs/msg/PointCloud2", PC2_DEF)
    ensure_type("geometry_msgs/msg/Point", POINT_DEF)
    ensure_type("geometry_msgs/msg/PointStamped", POINTSTAMPED_DEF)
    ensure_type("geometry_msgs/msg/Pose", POSE_DEF)
    ensure_type("geometry_msgs/msg/PoseWithCovariance", POSECOV_DEF)
    ensure_type("geometry_msgs/msg/PoseWithCovarianceStamped", POSECOVSTAMPED_DEF)
    ensure_type("geometry_msgs/msg/PoseStamped", POSESTAMPED_DEF)
    ensure_type("geometry_msgs/msg/Twist", TWIST_DEF)
    ensure_type("geometry_msgs/msg/TwistWithCovariance", TWISTCOV_DEF)
    ensure_type("geometry_msgs/msg/TwistWithCovarianceStamped", TWISTCOVSTAMPED_DEF)
    ensure_type("nav_msgs/msg/Odometry", ODOM_DEF)
    ensure_type("nav_msgs/msg/Path", PATH_DEF)
    ensure_type("geometry_msgs/msg/TwistStamped", TWISTSTAMPED_DEF)
    ensure_type("sensor_msgs/msg/Temperature", TEMP_DEF)
    ensure_type("sensor_msgs/msg/FluidPressure", FLUID_DEF)
    ensure_type("sensor_msgs/msg/MagneticField", MAG_DEF)
    ensure_type("geometry_msgs/msg/Transform", TRANS_DEF)
    ensure_type("geometry_msgs/msg/TransformStamped", TRANSSTAMPED_DEF)
    ensure_type("tf2_msgs/msg/TFMessage", TFMSG_DEF)
    ensure_type("std_msgs/msg/MultiArrayDimension", MADIM_DEF)
    ensure_type("std_msgs/msg/MultiArrayLayout", MALAYOUT_DEF)
    ensure_type("std_msgs/msg/Float32MultiArray", F32MA_DEF)
    ensure_type("grid_map_msgs/msg/GridMapInfo", GRIDMAPINFO_DEF)
    ensure_type("grid_map_msgs/msg/GridMap", GRIDMAP_DEF)
    ensure_type("geometry_msgs/msg/Vector3Stamped", VEC3STAMPED_DEF)
    ensure_type("geometry_msgs/msg/Wrench", WRENCH_DEF)
    ensure_type("geometry_msgs/msg/WrenchStamped", WRENCHSTAMPED_DEF)
    ensure_type("sensor_msgs/msg/JointState", JOINTSTATE_DEF)
    ensure_type("std_msgs/msg/Int8", INT8_DEF)
    ensure_type("std_msgs/msg/UInt8", UINT8_DEF)
    ensure_type("std_msgs/msg/UInt32", UINT32_DEF)
    ensure_type("std_msgs/msg/Float64", FLOAT64_DEF)
    ensure_type("visualization_msgs/msg/Marker", MARKER_DEF)
    ensure_type("visualization_msgs/msg/MarkerArray", MARKERARRAY_DEF)
    ensure_type("control_msgs/msg/InterfaceValue", INTERFACEVALUE_DEF)
    ensure_type("control_msgs/msg/DynamicJointState", DYNJOINTSTATE_DEF)

    # Type aliases
    Time               = ts.types["builtin_interfaces/msg/Time"]
    Duration           = ts.types["builtin_interfaces/msg/Duration"]
    Header             = ts.types["std_msgs/msg/Header"]
    ColorRGBA          = ts.types["std_msgs/msg/ColorRGBA"]
    CameraInfo         = ts.types["sensor_msgs/msg/CameraInfo"]
    Image              = ts.types["sensor_msgs/msg/Image"]
    CompressedImage    = ts.types["sensor_msgs/msg/CompressedImage"]
    Quaternion         = ts.types["geometry_msgs/msg/Quaternion"]
    Vector3            = ts.types["geometry_msgs/msg/Vector3"]
    Imu                = ts.types["sensor_msgs/msg/Imu"]
    PointField         = ts.types["sensor_msgs/msg/PointField"]
    Point              = ts.types["geometry_msgs/msg/Point"]
    PointStamped       = ts.types["geometry_msgs/msg/PointStamped"]
    Pose               = ts.types["geometry_msgs/msg/Pose"]
    PoseWithCov        = ts.types["geometry_msgs/msg/PoseWithCovariance"]
    PoseWithCovStamped = ts.types["geometry_msgs/msg/PoseWithCovarianceStamped"]
    PoseStamped        = ts.types["geometry_msgs/msg/PoseStamped"]
    Twist              = ts.types["geometry_msgs/msg/Twist"]
    TwistWithCov       = ts.types["geometry_msgs/msg/TwistWithCovariance"]
    TwistWithCovStamped= ts.types["geometry_msgs/msg/TwistWithCovarianceStamped"]
    Odometry           = ts.types["nav_msgs/msg/Odometry"]
    PathMsg            = ts.types["nav_msgs/msg/Path"]
    TwistStamped       = ts.types["geometry_msgs/msg/TwistStamped"]
    Temperature        = ts.types["sensor_msgs/msg/Temperature"]
    FluidPressure      = ts.types["sensor_msgs/msg/FluidPressure"]
    MagneticField      = ts.types["sensor_msgs/msg/MagneticField"]
    Transform          = ts.types["geometry_msgs/msg/Transform"]
    TransformStamped   = ts.types["geometry_msgs/msg/TransformStamped"]
    TFMessage          = ts.types["tf2_msgs/msg/TFMessage"]
    Vector3Stamped     = ts.types["geometry_msgs/msg/Vector3Stamped"]
    Wrench             = ts.types["geometry_msgs/msg/Wrench"]
    WrenchStamped      = ts.types["geometry_msgs/msg/WrenchStamped"]
    JointState         = ts.types["sensor_msgs/msg/JointState"]
    Int8Msg            = ts.types["std_msgs/msg/Int8"]
    UInt8Msg           = ts.types["std_msgs/msg/UInt8"]
    UInt32Msg          = ts.types["std_msgs/msg/UInt32"]
    Float64Msg         = ts.types["std_msgs/msg/Float64"]
    InterfaceValue     = ts.types["control_msgs/msg/InterfaceValue"]
    DynamicJointState  = ts.types["control_msgs/msg/DynamicJointState"]
    MarkerMsg          = ts.types["visualization_msgs/msg/Marker"]
    MarkerArrayMsg     = ts.types["visualization_msgs/msg/MarkerArray"]

    with AnyReader([bag_path]) as bag, open(out_path, "wb") as fh:
        conns = list(bag.connections)
        topics_info = {
            c.topic: SimpleNamespace(
                datatype=c.msgtype,
                message_count=getattr(c, "msgcount", getattr(c, "count", 0)),
            )
            for c in conns
        }

        def is_tf_datatype(ti):
            return is_type(ti, "tf2_msgs/TFMessage") or is_type(ti, "tf/tfMessage")

        img_raw_topics        = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/Image")]
        img_compressed_topics = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/CompressedImage")]
        cam_info_topics       = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/CameraInfo")]
        imu_topics            = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/Imu")]
        pc2_topics            = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/PointCloud2")]
        odom_topics           = [t for t, ti in topics_info.items() if is_type(ti, "nav_msgs/Odometry")]
        twist_topics          = [t for t, ti in topics_info.items() if is_type(ti, "geometry_msgs/TwistStamped")]
        twistcovs_topics      = [t for t, ti in topics_info.items() if is_type(ti, "geometry_msgs/TwistWithCovarianceStamped")]
        pwcs_topics           = [t for t, ti in topics_info.items() if is_type(ti, "geometry_msgs/PoseWithCovarianceStamped")]
        path_topics           = [t for t, ti in topics_info.items() if is_type(ti, "nav_msgs/Path")]
        marker_topics         = [t for t, ti in topics_info.items() if is_type(ti, "visualization_msgs/Marker")]
        marker_array_topics   = [t for t, ti in topics_info.items() if is_type(ti, "visualization_msgs/MarkerArray")]
        pnt_topics            = [t for t, ti in topics_info.items() if is_type(ti, "geometry_msgs/PointStamped")]
        temp_topics           = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/Temperature")]
        press_topics          = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/FluidPressure")]
        mag_topics            = [t for t, ti in topics_info.items() if is_type(ti, "sensor_msgs/MagneticField")]
        gmap_topics           = [t for t, ti in topics_info.items() if is_type(ti, "grid_map_msgs/GridMap")]
        sea_topics            = [t for t, ti in topics_info.items() if is_type(ti, "series_elastic_actuator_msgs/SeActuatorReadings")]

        tf_static_topics      = [t for t, ti in topics_info.items() if is_tf_datatype(ti) and t.endswith("/tf_static")]
        tf_topics             = [t for t, ti in topics_info.items() if is_tf_datatype(ti) and t.endswith("/tf") and t not in tf_static_topics]

        anymal_state_topics   = [t for t, ti in topics_info.items() if is_anymal_state(ti)]

        if not (img_raw_topics or img_compressed_topics or cam_info_topics or imu_topics or pc2_topics or
                odom_topics or twist_topics or twistcovs_topics or pwcs_topics or pnt_topics or temp_topics or press_topics or mag_topics
                or path_topics or marker_topics or marker_array_topics
                or gmap_topics or tf_topics or tf_static_topics or anymal_state_topics or sea_topics):
            seen = sorted({ datatype(ti) for ti in topics_info.values() if datatype(ti) })
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
        topic_kind.update({t: "twistcovs" for t in twistcovs_topics})
        topic_kind.update({t: "pwcs"  for t in pwcs_topics})
        topic_kind.update({t: "path"  for t in path_topics})
        topic_kind.update({t: "marker" for t in marker_topics})
        topic_kind.update({t: "marray" for t in marker_array_topics})
        topic_kind.update({t: "pnt"   for t in pnt_topics})
        topic_kind.update({t: "temp"  for t in temp_topics})
        topic_kind.update({t: "press" for t in press_topics})
        topic_kind.update({t: "mag"   for t in mag_topics})
        topic_kind.update({t: "gmap"  for t in gmap_topics})
        topic_kind.update({t: "tf"    for t in tf_topics})
        topic_kind.update({t: "tfs"   for t in tf_static_topics})
        topic_kind.update({t: "astate" for t in anymal_state_topics})
        topic_kind.update({t: "sear"  for t in sea_topics})

        wanted = tuple(topic_kind.keys())
        total_msgs = sum(msg_count(ti) for t, ti in topics_info.items() if t in topic_kind)

        # ---- Print identified topics BEFORE parsing any messages ----
        def print_identified_topics(topics_info, topic_kind):
            sys.stdout.write("\n=== Identified topics (grouped by kind) ===\n")
            grouped = defaultdict(list)
            for t, ti in topics_info.items():
                if t in topic_kind:
                    grouped[topic_kind[t]].append((t, datatype(ti), msg_count(ti)))
            order = ["cimg","img","cinfo","imu","pc2","odom","twist","twistcovs","pwcs","pnt","temp","press","mag","gmap","path","marker","marray","tf","tfs","astate","sear"]
            for kind in [k for k in order if k in grouped] + sorted(k for k in grouped if k not in order):
                sys.stdout.write(f"[{kind}] ({len(grouped[kind])} topics)\n")
                for t, dt, cnt in sorted(grouped[kind], key=lambda x: x[0]):
                    sys.stdout.write(f"  {t}  type={dt}  count={cnt}\n")
            sys.stdout.write(f"Total messages to process: {total_msgs}\n")
            sys.stdout.write("=== End of topic list ===\n\n")
            sys.stdout.flush()

        print_identified_topics(topics_info, topic_kind)
        # ------------------------------------------------------------

        sample_enabled = bool(args.print_random_sample)
        prefer_order = ["cimg", "img", "astate", "sear", "cinfo", "imu", "pc2", "odom",
                        "twist", "twistcovs", "pwcs", "pnt", "temp", "press", "mag", "gmap",
                        "path", "marker", "marray", "tf", "tfs"]
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
        pbar = make_pbar(total_msgs) if total_msgs > 0 else None

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

        def _reg_bundled(name: str) -> int:
            blob = make_ros2msg_bundle(name).encode("utf-8")
            return mw.register_schema(name=name, encoding="ros2msg", data=blob)

        def dual_reg(fullname: str) -> int:
            cid = _reg_bundled(fullname)
            if "/msg/" in fullname:
                alias = fullname.replace("/msg/", "/")
            else:
                parts = fullname.split("/")
                alias = f"{parts[0]}/msg/{parts[1]}" if len(parts) == 2 else fullname
            mw.register_schema(name=alias, encoding="ros2msg", data=make_ros2msg_bundle(fullname).encode("utf-8"))
            return cid

        schema_ids = {
            "builtin_interfaces/msg/Time":                 dual_reg("builtin_interfaces/msg/Time"),
            "builtin_interfaces/msg/Duration":             dual_reg("builtin_interfaces/msg/Duration"),
            "std_msgs/msg/Header":                         dual_reg("std_msgs/msg/Header"),
            "std_msgs/msg/ColorRGBA":                      dual_reg("std_msgs/msg/ColorRGBA"),
            "geometry_msgs/msg/Point":                     dual_reg("geometry_msgs/msg/Point"),
            "geometry_msgs/msg/Quaternion":                dual_reg("geometry_msgs/msg/Quaternion"),
            "geometry_msgs/msg/Vector3":                   dual_reg("geometry_msgs/msg/Vector3"),
            "geometry_msgs/msg/Pose":                      dual_reg("geometry_msgs/msg/Pose"),
            "geometry_msgs/msg/Wrench":                    dual_reg("geometry_msgs/msg/Wrench"),
            "geometry_msgs/msg/Transform":                 dual_reg("geometry_msgs/msg/Transform"),
            "geometry_msgs/msg/TransformStamped":          dual_reg("geometry_msgs/msg/TransformStamped"),
            "geometry_msgs/msg/PointStamped":              dual_reg("geometry_msgs/msg/PointStamped"),
            "geometry_msgs/msg/PoseStamped":               dual_reg("geometry_msgs/msg/PoseStamped"),
            "geometry_msgs/msg/Twist":                     dual_reg("geometry_msgs/msg/Twist"),
            "geometry_msgs/msg/TwistStamped":              dual_reg("geometry_msgs/msg/TwistStamped"),
            "geometry_msgs/msg/TwistWithCovariance":       dual_reg("geometry_msgs/msg/TwistWithCovariance"),
            "geometry_msgs/msg/TwistWithCovarianceStamped":dual_reg("geometry_msgs/msg/TwistWithCovarianceStamped"),
            "geometry_msgs/msg/Vector3Stamped":            dual_reg("geometry_msgs/msg/Vector3Stamped"),
            "geometry_msgs/msg/WrenchStamped":             dual_reg("geometry_msgs/msg/WrenchStamped"),
            "geometry_msgs/msg/PoseWithCovariance":        dual_reg("geometry_msgs/msg/PoseWithCovariance"),
            "geometry_msgs/msg/PoseWithCovarianceStamped": dual_reg("geometry_msgs/msg/PoseWithCovarianceStamped"),
            "sensor_msgs/msg/RegionOfInterest":            dual_reg("sensor_msgs/msg/RegionOfInterest"),
            "sensor_msgs/msg/CameraInfo":                  dual_reg("sensor_msgs/msg/CameraInfo"),
            "sensor_msgs/msg/Image":                       dual_reg("sensor_msgs/msg/Image"),
            "sensor_msgs/msg/CompressedImage":             dual_reg("sensor_msgs/msg/CompressedImage"),
            "sensor_msgs/msg/Imu":                         dual_reg("sensor_msgs/msg/Imu"),
            "sensor_msgs/msg/PointField":                  dual_reg("sensor_msgs/msg/PointField"),
            "sensor_msgs/msg/PointCloud2":                 dual_reg("sensor_msgs/msg/PointCloud2"),
            "sensor_msgs/msg/Temperature":                 dual_reg("sensor_msgs/msg/Temperature"),
            "sensor_msgs/msg/FluidPressure":               dual_reg("sensor_msgs/msg/FluidPressure"),
            "sensor_msgs/msg/MagneticField":               dual_reg("sensor_msgs/msg/MagneticField"),
            "sensor_msgs/msg/JointState":                  dual_reg("sensor_msgs/msg/JointState"),
            "nav_msgs/msg/Odometry":                       dual_reg("nav_msgs/msg/Odometry"),
            "nav_msgs/msg/Path":                           dual_reg("nav_msgs/msg/Path"),
            "tf2_msgs/msg/TFMessage":                      dual_reg("tf2_msgs/msg/TFMessage"),
            "std_msgs/msg/MultiArrayDimension":            dual_reg("std_msgs/msg/MultiArrayDimension"),
            "std_msgs/msg/MultiArrayLayout":               dual_reg("std_msgs/msg/MultiArrayLayout"),
            "std_msgs/msg/Float32MultiArray":              dual_reg("std_msgs/msg/Float32MultiArray"),
            "grid_map_msgs/msg/GridMapInfo":               dual_reg("grid_map_msgs/msg/GridMapInfo"),
            "grid_map_msgs/msg/GridMap":                   dual_reg("grid_map_msgs/msg/GridMap"),
            "std_msgs/msg/Int8":                           dual_reg("std_msgs/msg/Int8"),
            "std_msgs/msg/UInt8":                          dual_reg("std_msgs/msg/UInt8"),
            "std_msgs/msg/UInt32":                         dual_reg("std_msgs/msg/UInt32"),
            "std_msgs/msg/Float64":                        dual_reg("std_msgs/msg/Float64"),
            "visualization_msgs/msg/Marker":               dual_reg("visualization_msgs/msg/Marker"),
            "visualization_msgs/msg/MarkerArray":          dual_reg("visualization_msgs/msg/MarkerArray"),
            "control_msgs/msg/InterfaceValue":             dual_reg("control_msgs/msg/InterfaceValue"),
            "control_msgs/msg/DynamicJointState":          dual_reg("control_msgs/msg/DynamicJointState"),
        }

        def qos_meta(_topic: str) -> dict:
            if _topic.endswith("/tf_static"):
                return {
                    "offered_qos_profiles": (
                        "[{history: keep_last, depth: 1, reliability: reliable, "
                        "durability: transient_local, deadline: {sec: 2147483647, nsec: 2147483647}, "
                        "lifespan: {sec: 2147483647, nsec: 2147483647}, liveliness: automatic, "
                        "liveliness_lease_duration: {sec: 2147483647, nsec: 2147483647}, "
                        "avoid_ros_namespace_conventions: false}]"
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
                metadata=qos_meta(topic),
            )
            topic_to_channel[topic] = cid
            return cid

        def frame_id(*candidates: Optional[str]) -> str:
            for f in candidates:
                if isinstance(f, str) and len(f):
                    return f
            return ""

        def sanitize(seg: str) -> str:
            seg = (seg or "").strip().replace(" ", "_")
            seg = seg.replace("/", "_")
            seg = re.sub(r"[^A-Za-z0-9_.-]", "_", seg)
            seg = re.sub(r"_+", "_", seg).strip("_")
            return seg or "contact"

        sample_printed = False

        want_conns = [c for c in conns if c.topic in wanted]
        for conn, log_ns, raw in bag.messages(connections=want_conns):
            topic = conn.topic
            kind = topic_kind[topic]
            msg = bag.deserialize(raw, conn.msgtype)

            # --- AnymalState split path ---
            if kind == "astate":
                base = topic.rstrip("/")
                top_hdr = getattr(msg, "header", None)

                # PoseStamped
                if hasattr(msg, "pose"):
                    ps = msg.pose
                    ps_ns = hdr_stamp_ns(getattr(ps, "header", None)) or hdr_stamp_ns(top_hdr) or int(log_ns)
                    pose = Pose(
                        Point(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
                        Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w),
                    )
                    ps_out = PoseStamped(
                        Header(Time(*sec_nsec_from_ns(ps_ns)),
                               frame_id(getattr(ps.header, "frame_id", None), getattr(top_hdr, "frame_id", None))),
                        pose,
                    )
                    payload = ts.serialize_cdr(ps_out, "geometry_msgs/msg/PoseStamped")
                    mw.add_message(channel_id=get_channel(f"{base}/pose", "geometry_msgs/msg/PoseStamped"),
                                   log_time=int(log_ns), publish_time=int(ps_ns), data=payload)

                # TwistStamped
                if hasattr(msg, "twist"):
                    tsrc = msg.twist
                    t_ns = hdr_stamp_ns(getattr(tsrc, "header", None)) or hdr_stamp_ns(top_hdr) or int(log_ns)
                    tw = Twist(
                        Vector3(tsrc.twist.linear.x, tsrc.twist.linear.y, tsrc.twist.linear.z),
                        Vector3(tsrc.twist.angular.x, tsrc.twist.angular.y, tsrc.twist.angular.z),
                    )
                    tw_out = TwistStamped(
                        Header(Time(*sec_nsec_from_ns(t_ns)),
                               frame_id(getattr(tsrc.header, "frame_id", None), getattr(top_hdr, "frame_id", None))),
                        tw,
                    )
                    payload = ts.serialize_cdr(tw_out, "geometry_msgs/msg/TwistStamped")
                    mw.add_message(channel_id=get_channel(f"{base}/twist", "geometry_msgs/msg/TwistStamped"),
                                   log_time=int(log_ns), publish_time=int(t_ns), data=payload)

                # JointState (+ DynamicJointState for acceleration)
                if hasattr(msg, "joints"):
                    js = msg.joints
                    j_ns = hdr_stamp_ns(getattr(js, "header", None)) or hdr_stamp_ns(top_hdr) or int(log_ns)
                    names = list(getattr(js, "name", []))
                    pos   = np.asarray(getattr(js, "position", []), dtype=np.float64)
                    vel   = np.asarray(getattr(js, "velocity", []), dtype=np.float64)
                    eff   = np.asarray(getattr(js, "effort", []),   dtype=np.float64)
                    acc   = np.asarray(getattr(js, "acceleration", []), dtype=np.float64)

                    n = min(len(names), len(pos), len(vel), len(eff))
                    if n < len(names) or n < len(pos) or n < len(vel) or n < len(eff):
                        sys.stderr.write(f"[WARN] JointState arrays mismatch on {base}; truncating to {n}\n")
                    names = names[:n]
                    pos, vel, eff = pos[:n], vel[:n], eff[:n]

                    for i in range(n):
                        names[i] = canon_joint_name(i, names[i])

                    js_out = JointState(
                        Header(Time(*sec_nsec_from_ns(j_ns)),
                               frame_id(getattr(js.header, "frame_id", None), getattr(top_hdr, "frame_id", None))),
                        names, pos, vel, eff
                    )
                    payload = ts.serialize_cdr(js_out, "sensor_msgs/msg/JointState")
                    mw.add_message(channel_id=get_channel(f"{base}/joint_states", "sensor_msgs/msg/JointState"),
                                   log_time=int(log_ns), publish_time=int(j_ns), data=payload)

                    iface_names = ["position","velocity","acceleration","effort"]
                    value_arrays = [pos, vel, acc, eff]
                    iface_vals = []
                    for iface, arr in zip(iface_names, value_arrays):
                        vals_i = [float(arr[j]) if j < len(arr) else float("nan") for j in range(n)]
                        iface_vals.append(InterfaceValue(str(iface), np.asarray(vals_i, dtype=np.float64)))
                    djs_out = DynamicJointState(
                        Header(Time(*sec_nsec_from_ns(j_ns)),
                               frame_id(getattr(js.header, "frame_id", None), getattr(top_hdr, "frame_id", None))),
                        list(names), list(iface_names), iface_vals
                    )
                    payload = ts.serialize_cdr(djs_out, "control_msgs/msg/DynamicJointState")
                    mw.add_message(channel_id=get_channel(f"{base}/dynamic_joint_state", "control_msgs/msg/DynamicJointState"),
                                   log_time=int(log_ns), publish_time=int(j_ns), data=payload)

                # State (std_msgs/Int8)
                if hasattr(msg, "state"):
                    st_ns = hdr_stamp_ns(top_hdr) or int(log_ns)
                    st_out = Int8Msg(int(getattr(msg, "state", 0)))
                    payload = ts.serialize_cdr(st_out, "std_msgs/msg/Int8")
                    mw.add_message(channel_id=get_channel(f"{base}/state", "std_msgs/msg/Int8"),
                                   log_time=int(log_ns), publish_time=int(st_ns), data=payload)

                # Contacts (+ friction & restitution)
                if hasattr(msg, "contacts") and isinstance(msg.contacts, (list, tuple)):
                    for c in msg.contacts:
                        cname = sanitize(getattr(c, "name", "contact"))
                        chdr = getattr(c, "header", None)
                        c_ns = hdr_stamp_ns(chdr) or hdr_stamp_ns(top_hdr) or int(log_ns)
                        fid = frame_id(getattr(chdr, "frame_id", None), getattr(top_hdr, "frame_id", None))

                        if hasattr(c, "wrench"):
                            w = c.wrench
                            w_out = WrenchStamped(
                                Header(Time(*sec_nsec_from_ns(c_ns)), fid),
                                Wrench(Vector3(w.force.x, w.force.y, w.force.z),
                                       Vector3(w.torque.x, w.torque.y, w.torque.z))
                            )
                            payload = ts.serialize_cdr(w_out, "geometry_msgs/msg/WrenchStamped")
                            mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/wrench", "geometry_msgs/msg/WrenchStamped"),
                                           log_time=int(log_ns), publish_time=int(c_ns), data=payload)

                        if hasattr(c, "position"):
                            p = c.position
                            p_out = PointStamped(Header(Time(*sec_nsec_from_ns(c_ns)), fid),
                                                 Point(p.x, p.y, p.z))
                            payload = ts.serialize_cdr(p_out, "geometry_msgs/msg/PointStamped")
                            mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/position", "geometry_msgs/msg/PointStamped"),
                                           log_time=int(log_ns), publish_time=int(c_ns), data=payload)

                        if hasattr(c, "normal"):
                            n = c.normal
                            n_out = Vector3Stamped(Header(Time(*sec_nsec_from_ns(c_ns)), fid),
                                                   Vector3(n.x, n.y, n.z))
                            payload = ts.serialize_cdr(n_out, "geometry_msgs/msg/Vector3Stamped")
                            mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/normal", "geometry_msgs/msg/Vector3Stamped"),
                                           log_time=int(log_ns), publish_time=int(c_ns), data=payload)

                        if hasattr(c, "state"):
                            cs_out = UInt8Msg(int(getattr(c, "state", 0)))
                            payload = ts.serialize_cdr(cs_out, "std_msgs/msg/UInt8")
                            mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/state", "std_msgs/msg/UInt8"),
                                           log_time=int(log_ns), publish_time=int(c_ns), data=payload)

                        if hasattr(c, "frictionCoefficient"):
                            try:
                                fc_msg = Float64Msg(float(getattr(c, "frictionCoefficient")))
                                payload = ts.serialize_cdr(fc_msg, "std_msgs/msg/Float64")
                                mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/friction_coefficient", "std_msgs/msg/Float64"),
                                               log_time=int(log_ns), publish_time=int(c_ns), data=payload)
                            except Exception:
                                pass
                        if hasattr(c, "restitutionCoefficient"):
                            try:
                                rc_msg = Float64Msg(float(getattr(c, "restitutionCoefficient")))
                                payload = ts.serialize_cdr(rc_msg, "std_msgs/msg/Float64")
                                mw.add_message(channel_id=get_channel(f"{base}/contacts/{cname}/restitution_coefficient", "std_msgs/msg/Float64"),
                                               log_time=int(log_ns), publish_time=int(c_ns), data=payload)
                            except Exception:
                                pass

                processed += 1
                if pbar is not None:
                    pbar.update(1)
                else:
                    print_progress(processed, total_msgs)
                continue

            # --- SEA readings path -> split into standard messages (IMU skipped) ---
            if kind == "sear":
                base = topic.rstrip("/")

                pub_ns = None
                frame_guess = ""
                if hasattr(msg, "readings") and msg.readings:
                    for rd in msg.readings:
                        for cand in (
                            getattr(rd, "header", None),
                            getattr(getattr(rd, "state", None), "header", None),
                            getattr(getattr(rd, "commanded", None), "header", None),
                        ):
                            pub_ns = hdr_stamp_ns(cand) or pub_ns
                            if frame_guess == "" and cand is not None:
                                frame_guess = getattr(cand, "frame_id", "") or frame_guess
                        if pub_ns:
                            break
                if pub_ns is None:
                    pub_ns = int(log_ns)

                names = []
                pos, vel, eff = [], [], []
                cur, gear_pos, gear_vel, jacc = [], [], [], []

                cmd_pos, cmd_vel, cmd_eff = [], [], []
                cmd_cur, cmd_pid_p, cmd_pid_i, cmd_pid_d, cmd_mode = [], [], [], [], []

                status_vals = []

                readings = getattr(msg, "readings", []) or []
                for idx, rd in enumerate(readings):
                    st = getattr(rd, "state", None)
                    cmd = getattr(rd, "commanded", None)

                    name = ""
                    if st is not None:
                        name = str(getattr(st, "name", "") or "")
                    if not name and cmd is not None:
                        name = str(getattr(cmd, "name", "") or "")
                    if not name:
                        name = f"joint_{idx}"
                    names.append(name)

                    def fget(obj, attr, default=np.nan):
                        try:
                            return float(getattr(obj, attr, default))
                        except Exception:
                            return float(default)

                    if st is None:
                        pos.append(np.nan); vel.append(np.nan); eff.append(np.nan)
                        cur.append(np.nan); gear_pos.append(np.nan); gear_vel.append(np.nan); jacc.append(np.nan)
                        status_vals.append(0)
                    else:
                        pos.append(fget(st, "joint_position"))
                        vel.append(fget(st, "joint_velocity"))
                        eff.append(fget(st, "joint_torque"))
                        cur.append(fget(st, "current"))
                        gear_pos.append(fget(st, "gear_position"))
                        gear_vel.append(fget(st, "gear_velocity"))
                        jacc.append(fget(st, "joint_acceleration"))
                        status_vals.append(int(getattr(st, "statusword", 0)))

                    if cmd is None:
                        cmd_pos.append(np.nan); cmd_vel.append(np.nan); cmd_eff.append(np.nan)
                        cmd_cur.append(np.nan)
                        cmd_pid_p.append(0.0); cmd_pid_i.append(0.0); cmd_pid_d.append(0.0)
                        cmd_mode.append(np.nan)
                    else:
                        cmd_pos.append(fget(cmd, "position"))
                        cmd_vel.append(fget(cmd, "velocity"))
                        cmd_eff.append(fget(cmd, "joint_torque"))
                        cmd_cur.append(fget(cmd, "current"))
                        cmd_pid_p.append(fget(cmd, "pid_gains_p", 0.0))
                        cmd_pid_i.append(fget(cmd, "pid_gains_i", 0.0))
                        cmd_pid_d.append(fget(cmd, "pid_gains_d", 0.0))
                        try:
                            cmd_mode.append(float(getattr(cmd, "mode", np.nan)))
                        except Exception:
                            cmd_mode.append(np.nan)

                N = len(names)
                canon_names = list(names)
                for i in range(N):
                    canon_names[i] = canon_joint_name(i, canon_names[i])

                js_hdr = Header(Time(*sec_nsec_from_ns(pub_ns)), frame_guess)
                js_out = JointState(
                    js_hdr,
                    canon_names,
                    np.asarray(pos, dtype=np.float64),
                    np.asarray(vel, dtype=np.float64),
                    np.asarray(eff, dtype=np.float64),
                )
                payload = ts.serialize_cdr(js_out, "sensor_msgs/msg/JointState")
                mw.add_message(channel_id=get_channel(f"{base}/joint_states", "sensor_msgs/msg/JointState"),
                               log_time=int(log_ns), publish_time=int(pub_ns), data=payload)

                djs_hdr = Header(Time(*sec_nsec_from_ns(pub_ns)), frame_guess)
                iface_names = [
                    "position","velocity","effort","current",
                    "gear_position","gear_velocity","joint_acceleration",
                    "cmd_position","cmd_velocity","cmd_effort","cmd_current",
                    "cmd_pid_p","cmd_pid_i","cmd_pid_d","cmd_mode",
                ]
                value_arrays = [
                    pos, vel, eff, cur, gear_pos, gear_vel, jacc,
                    cmd_pos, cmd_vel, cmd_eff, cmd_cur, cmd_pid_p, cmd_pid_i, cmd_pid_d, cmd_mode,
                ]
                iface_list = []
                for iface, arr in zip(iface_names, value_arrays):
                    vals_i = [float(arr[j]) if j < len(arr) else float("nan") for j in range(N)]
                    iface_list.append(InterfaceValue(str(iface), np.asarray(vals_i, dtype=np.float64)))
                djs_out = DynamicJointState(djs_hdr, canon_names, list(iface_names), iface_list)
                payload = ts.serialize_cdr(djs_out, "control_msgs/msg/DynamicJointState")
                mw.add_message(channel_id=get_channel(f"{base}/dynamic_joint_state", "control_msgs/msg/DynamicJointState"),
                               log_time=int(log_ns), publish_time=int(pub_ns), data=payload)

                js_cmd = JointState(
                    js_hdr,
                    canon_names,
                    np.asarray(cmd_pos, dtype=np.float64),
                    np.asarray(cmd_vel, dtype=np.float64),
                    np.asarray(cmd_eff, dtype=np.float64),
                )
                payload = ts.serialize_cdr(js_cmd, "sensor_msgs/msg/JointState")
                mw.add_message(channel_id=get_channel(f"{base}/joint_states_commanded", "sensor_msgs/msg/JointState"),
                               log_time=int(log_ns), publish_time=int(pub_ns), data=payload)

                for idx, (n, sw) in enumerate(zip(names, status_vals)):
                    frame_name = canon_measurement_frame(idx, n)
                    san = sanitize(frame_name)
                    sw_msg = UInt32Msg(int(sw))
                    payload = ts.serialize_cdr(sw_msg, "std_msgs/msg/UInt32")
                    mw.add_message(channel_id=get_channel(f"{base}/{san}/statusword", "std_msgs/msg/UInt32"),
                                   log_time=int(log_ns), publish_time=int(pub_ns), data=payload)

                processed += 1
                if pbar is not None: pbar.update(1)
                else: print_progress(processed, total_msgs)
                continue

            # --- Standard messages path ---
            publish_ns, log_ns = extract_publish_and_log_ns(msg, log_ns, allow_zero_header=True)

            idx_this = per_topic_seen[topic]
            per_topic_seen[topic] += 1
            sample_hit = (sample_enabled and not sample_printed and topic == chosen_topic and idx_this == sample_target_index)

            if kind == "cinfo":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                D  = np.asarray(msg.D, dtype=np.float64)
                K  = np.asarray(msg.K, dtype=np.float64)
                Rm = np.asarray(msg.R, dtype=np.float64)
                Pm = np.asarray(msg.P, dtype=np.float64)
                roi = msg.roi
                ROI = ts.types["sensor_msgs/msg/RegionOfInterest"](int(roi.x_offset), int(roi.y_offset),
                                                                   int(roi.height), int(roi.width), bool(roi.do_rectify))
                ros2_obj = CameraInfo(
                    hdr, int(msg.height), int(msg.width), msg.distortion_model, D, K, Rm, Pm,
                    int(msg.binning_x), int(msg.binning_y), ROI,
                )
                chan = get_channel(topic, "sensor_msgs/msg/CameraInfo")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CameraInfo")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "img":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                data = np.frombuffer(msg.data, dtype=np.uint8)
                ros2_obj = Image(hdr, int(msg.height), int(msg.width), msg.encoding, int(msg.is_bigendian), int(msg.step), data)
                chan = get_channel(topic, "sensor_msgs/msg/Image")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Image")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "cimg":
                raw_topic = raw_topic_from_compressed(topic)
                if args.decompress_images and raw_topic not in raw_topics_set:
                    try:
                        buf = np.frombuffer(msg.data, dtype=np.uint8)
                        if buf.size == 0:
                            raise ValueError("compressed payload is empty")
                        cv_img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                        if cv_img is None:
                            raise ValueError("cv2.imdecode returned None")
                        cv_img = np.ascontiguousarray(cv_img)

                        enc = encoding_for_cv(cv_img)
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
                                raise ValueError(f"payload size mismatch for {enc}: data_len={data.size}, expected={expected_len}")

                        fmt = getattr(msg, "format", "") or ""
                        if "compressedDepth" in fmt and not (enc in ("16UC1", "32FC1") and ch == 1):
                            sys.stderr.write(f"\n[WARN] compressedDepth format but decoded as enc={enc}, ch={ch}, dtype={cv_img.dtype}\n")

                        hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                        ros2_obj = Image(hdr, hh, ww, enc, 0, step, data)
                        chan = get_channel(raw_topic, "sensor_msgs/msg/Image")
                        payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Image")
                        mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)

                        if sample_hit:
                            dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                            sample_printed = True

                    except Exception as e:
                        sys.stderr.write(f"\n[WARN] decompress failed on {topic} @ {publish_ns}: {e}\n")
                        hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                        data = np.frombuffer(msg.data, dtype=np.uint8)
                        ros2_obj = CompressedImage(hdr, getattr(msg, "format", ""), data)
                        chan = get_channel(topic, "sensor_msgs/msg/CompressedImage")
                        payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CompressedImage")
                        mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                        if sample_hit:
                            dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                            sample_printed = True
                else:
                    hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                    data = np.frombuffer(msg.data, dtype=np.uint8)
                    ros2_obj = CompressedImage(hdr, getattr(msg, "format", ""), data)
                    chan = get_channel(topic, "sensor_msgs/msg/CompressedImage")
                    payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/CompressedImage")
                    mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                    if sample_hit:
                        dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                        sample_printed = True

            elif kind == "imu":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pc2":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "odom":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "twist":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                lin = msg.twist.linear
                ang = msg.twist.angular
                ros2_obj = TwistStamped(hdr, Twist(Vector3(lin.x, lin.y, lin.z), Vector3(ang.x, ang.y, ang.z)))
                chan = get_channel(topic, "geometry_msgs/msg/TwistStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/TwistStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "twistcovs":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                lin = msg.twist.twist.linear
                ang = msg.twist.twist.angular
                twcov = TwistWithCov(
                    Twist(Vector3(lin.x, lin.y, lin.z), Vector3(ang.x, ang.y, ang.z)),
                    np.asarray(msg.twist.covariance, dtype=np.float64),
                )
                ros2_obj = TwistWithCovStamped(hdr, twcov)
                chan = get_channel(topic, "geometry_msgs/msg/TwistWithCovarianceStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/TwistWithCovarianceStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pwcs":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "path":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                poses_out = []
                for ps in getattr(msg, "poses", []) or []:
                    p_hdr_ns = hdr_stamp_ns(getattr(ps, "header", None), allow_zero=True)
                    if p_hdr_ns is None:
                        p_hdr_ns = publish_ns
                    p_hdr = Header(Time(*sec_nsec_from_ns(p_hdr_ns)), getattr(ps.header, "frame_id", ""))
                    pos = ps.pose.position
                    ori = ps.pose.orientation
                    poses_out.append(PoseStamped(p_hdr, Pose(Point(pos.x, pos.y, pos.z),
                                                             Quaternion(ori.x, ori.y, ori.z, ori.w))))
                ros2_obj = PathMsg(hdr, poses_out)
                chan = get_channel(topic, "nav_msgs/msg/Path")
                payload = ts.serialize_cdr(ros2_obj, "nav_msgs/msg/Path")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "marker":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                pose = Pose(Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                            Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
                scale = Vector3(float(msg.scale.x), float(msg.scale.y), float(msg.scale.z))
                color = ColorRGBA(float(msg.color.r), float(msg.color.g), float(msg.color.b), float(msg.color.a))
                lt_s, lt_ns = extract_sec_nsec(getattr(msg, "lifetime", SimpleNamespace(sec=0, nsec=0)))
                lifetime = Duration(lt_s, lt_ns)
                pts = [Point(p.x, p.y, p.z) for p in getattr(msg, "points", []) or []]
                cols = [ColorRGBA(float(c.r), float(c.g), float(c.b), float(c.a)) for c in getattr(msg, "colors", []) or []]
                ros2_obj = MarkerMsg(
                    hdr,
                    str(getattr(msg, "ns", "")),
                    int(getattr(msg, "id", 0)),
                    int(getattr(msg, "type", 0)),
                    int(getattr(msg, "action", 0)),
                    pose,
                    scale,
                    color,
                    lifetime,
                    bool(getattr(msg, "frame_locked", False)),
                    pts,
                    cols,
                    str(getattr(msg, "text", "")),
                    str(getattr(msg, "mesh_resource", "")),
                    bool(getattr(msg, "mesh_use_embedded_materials", False)),
                )
                chan = get_channel(topic, "visualization_msgs/msg/Marker")
                payload = ts.serialize_cdr(ros2_obj, "visualization_msgs/msg/Marker")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "marray":
                markers = []
                for m in getattr(msg, "markers", []) or []:
                    m_hdr_ns = hdr_stamp_ns(getattr(m, "header", None), allow_zero=True) or publish_ns
                    m_hdr = Header(Time(*sec_nsec_from_ns(m_hdr_ns)), getattr(m.header, "frame_id", ""))
                    pose = Pose(Point(m.pose.position.x, m.pose.position.y, m.pose.position.z),
                                Quaternion(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w))
                    scale = Vector3(float(m.scale.x), float(m.scale.y), float(m.scale.z))
                    color = ColorRGBA(float(m.color.r), float(m.color.g), float(m.color.b), float(m.color.a))
                    lt_s, lt_ns = extract_sec_nsec(getattr(m, "lifetime", SimpleNamespace(sec=0, nsec=0)))
                    lifetime = Duration(lt_s, lt_ns)
                    pts = [Point(p.x, p.y, p.z) for p in getattr(m, "points", []) or []]
                    cols = [ColorRGBA(float(c.r), float(c.g), float(c.b), float(c.a)) for c in getattr(m, "colors", []) or []]
                    markers.append(MarkerMsg(
                        m_hdr,
                        str(getattr(m, "ns", "")),
                        int(getattr(m, "id", 0)),
                        int(getattr(m, "type", 0)),
                        int(getattr(m, "action", 0)),
                        pose,
                        scale,
                        color,
                        lifetime,
                        bool(getattr(m, "frame_locked", False)),
                        pts,
                        cols,
                        str(getattr(m, "text", "")),
                        str(getattr(m, "mesh_resource", "")),
                        bool(getattr(m, "mesh_use_embedded_materials", False)),
                    ))
                ros2_obj = MarkerArrayMsg(markers)
                chan = get_channel(topic, "visualization_msgs/msg/MarkerArray")
                payload = ts.serialize_cdr(ros2_obj, "visualization_msgs/msg/MarkerArray")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "pnt":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                pt = msg.point
                ros2_obj = PointStamped(hdr, Point(pt.x, pt.y, pt.z))
                chan = get_channel(topic, "geometry_msgs/msg/PointStamped")
                payload = ts.serialize_cdr(ros2_obj, "geometry_msgs/msg/PointStamped")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "temp":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                ros2_obj = Temperature(hdr, float(msg.temperature), float(msg.variance))
                chan = get_channel(topic, "sensor_msgs/msg/Temperature")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/Temperature")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "press":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
                ros2_obj = FluidPressure(hdr, float(msg.fluid_pressure), float(msg.variance))
                chan = get_channel(topic, "sensor_msgs/msg/FluidPressure")
                payload = ts.serialize_cdr(ros2_obj, "sensor_msgs/msg/FluidPressure")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=publish_ns, data=payload)
                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "mag":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind == "gmap":
                hdr = Header(Time(*sec_nsec_from_ns(publish_ns)), msg.header.frame_id)

                pose_p = msg.info.pose.position
                pose_q = msg.info.pose.orientation
                info2 = ts.types["grid_map_msgs/msg/GridMapInfo"](
                    float(msg.info.resolution),
                    float(msg.info.length_x),
                    float(msg.info.length_y),
                    Pose(Point(pose_p.x, pose_p.y, pose_p.z),
                         Quaternion(pose_q.x, pose_q.y, pose_q.z, pose_q.w)),
                )

                layers = list(msg.layers)
                basic_layers = list(msg.basic_layers)

                data_arr = []
                for ma in msg.data:
                    dims = [ts.types["std_msgs/msg/MultiArrayDimension"](d.label, int(d.size), int(d.stride)) for d in ma.layout.dim]
                    layout = ts.types["std_msgs/msg/MultiArrayLayout"](dims, int(ma.layout.data_offset))
                    arr = np.asarray(ma.data, dtype=np.float32)
                    data_arr.append(ts.types["std_msgs/msg/Float32MultiArray"](layout, arr))

                ros2_obj = ts.types["grid_map_msgs/msg/GridMap"](
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
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, publish_ns, log_ns)
                    sample_printed = True

            elif kind in ("tf", "tfs"):
                tlist = []
                for tr in getattr(msg, "transforms", []):
                    try:
                        t_stamp_ns = time_to_nsec(tr.header.stamp)
                        t_hdr = Header(Time(*sec_nsec_from_ns(t_stamp_ns)), tr.header.frame_id)
                        trans = tr.transform
                        t_trans = Transform(
                            Vector3(trans.translation.x, trans.translation.y, trans.translation.z),
                            Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w),
                        )
                        tlist.append(TransformStamped(t_hdr, tr.child_frame_id, t_trans))
                    except Exception as e:
                        sys.stderr.write(f"\n[WARN] skipping malformed TransformStamped in {topic}: {e}\n")

                if not tlist:
                    processed += 1
                    if pbar is not None:
                        pbar.update(1)
                    else:
                        print_progress(processed, total_msgs)
                    continue

                ros2_obj = TFMessage(tlist)
                chan = get_channel(topic, "tf2_msgs/msg/TFMessage")
                payload = ts.serialize_cdr(ros2_obj, "tf2_msgs/msg/TFMessage")
                mw.add_message(channel_id=chan, log_time=log_ns, publish_time=int(log_ns), data=payload)

                if sample_hit:
                    dump_ros1_vs_ros2(topic, kind, msg, ros2_obj, int(log_ns), log_ns)
                    sample_printed = True

            else:
                raise RuntimeError(f"Unknown kind for topic {topic}")

            processed += 1
            if pbar is not None:
                pbar.update(1)
            else:
                print_progress(processed, total_msgs)

        mw.finish()

    if pbar is not None:
        pbar.close()
    else:
        print_progress(processed, total_msgs)

# -------------------- CLI --------------------
def main():
    ap = argparse.ArgumentParser(description="ROS1 .bag → MCAP (ROS 2 profile, CDR) with optional image decompression and message splitting")
    ap.add_argument("--inbag", required=False, help="input ROS1 .bag or a directory (used if BATCH_MODE=True)", default="2024-11-14-13-45-37_anymal_state.bag")
    ap.add_argument("--decompress-images", type=int, choices=[0, 1], default=0)
    ap.add_argument("--print-random-sample", type=int, choices=[0, 1], default=1)
    ap.add_argument("--sample-prefer",
                    choices=["cimg","img","cinfo","imu","pc2","odom","twist","twistcovs","pwcs","pnt","temp","press","mag","gmap","path","marker","marray","tf","tfs","astate","sear"],
                    default=None)
    args = ap.parse_args()

    root = Path(args.inbag).expanduser()
    if BATCH_MODE:
        if BATCH_DIR is not None:
            base_dir = Path(BATCH_DIR).expanduser()
        else:
            base_dir = root if root.is_dir() else root.parent

        if not base_dir.is_dir():
            sys.exit(f"BATCH_MODE: directory not found: {base_dir}")

        bag_files = sorted(p for p in base_dir.glob(BATCH_GLOB) if p.is_file())
        if not bag_files:
            sys.exit(f"BATCH_MODE: no files matching '{BATCH_GLOB}' in {base_dir}")

        sys.stdout.write(f"\n=== BATCH_MODE: converting {len(bag_files)} bag(s) in {base_dir} ===\n")
        sys.stdout.flush()
        failures = 0
        for bp in bag_files:
            sys.stdout.write(f"\n--- Converting: {bp} ---\n"); sys.stdout.flush()
            try:
                convert_single_bag(bp, args)
            except SystemExit as se:
                failures += 1
                sys.stderr.write(f"[ERROR] {bp}: {se}\n")
            except Exception as e:
                failures += 1
                sys.stderr.write(f"[ERROR] {bp}: {e}\n")
        if failures:
            sys.stderr.write(f"\nBATCH_MODE finished with {failures} failure(s).\n")
        else:
            sys.stdout.write("\nBATCH_MODE finished successfully.\n")
        return

    if root.is_dir():
        sys.exit(f"--inbag points to a directory but BATCH_MODE is False: {root}")
    convert_single_bag(root, args)

if __name__ == "__main__":
    main()
