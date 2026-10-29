#!/usr/bin/env python3
# MCAP (ROS 2 profile, CDR-serialized ROS 2 messages) → ROS1 .bag
# Converts topics containing standard ROS2 messages (as written by the inverse script)
# into a ROS1 bag using rosbags' ROS1 serializer.
#
# Notes:
# - ROS1 std_msgs/Header includes seq; ROS2 does not. This script fills seq per-topic.
# - Message timestamps in the output bag use MCAP message.log_time (nanoseconds).

import sys
import argparse
import re
from collections import defaultdict
from pathlib import Path
from typing import Tuple, Optional

import numpy as np

from importlib.metadata import version
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.rosbag1 import Writer as Ros1BagWriter

# MCAP reader (same 'mcap' library as used for writing in the original script)
try:
    from mcap.reader import make_reader
except Exception:
    # Older MCAP python package layout
    from mcap.mcap0.reader import make_reader  # type: ignore

print("rosbags:", version("rosbags"))

from tqdm import tqdm


def make_pbar(total: int):
    return tqdm(total=total, unit="msg", desc="Converting → ROS1/eibag")


# -------------------- batch mode (kept consistent with original script) --------------------
BATCH_GLOB: str = "*.mcap"

# -------------------- time handling --------------------
_NSEC = 10**9


def sec_nsec_from_ns(ns: int) -> Tuple[int, int]:
    return int(ns // _NSEC), int(ns % _NSEC)


# -------------------- helpers --------------------
def _canon(name: str) -> str:
    name = str(name).strip().replace("::", "/")
    if "/msg/" not in name and name.count("/") == 1:
        pkg, typ = name.split("/")
        return f"{pkg}/msg/{typ}"
    return name


def norm_type(t: Optional[str]) -> Optional[str]:
    if t is None:
        return None
    s = str(t).strip()
    s = s.replace("/msg/", "/").replace("::", "/")
    return s


def ensure_type(ts, name: str, defn: str):
    if name not in ts.types:
        ts.register(get_types_from_msg(defn, name))


# -------------------- ROS 2 message definitions (from original script; needed for CDR decode) --------------------
TIME_DEF = """int32 sec
uint32 nanosec
"""
DURATION_DEF = """int32 sec
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
POSESTAMPED_DEF = """std_msgs/Header header
geometry_msgs/Pose pose
"""
TWIST_DEF = """geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
"""
TWISTSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/Twist twist
"""
TWISTCOV_DEF = """geometry_msgs/Twist twist
float64[36] covariance
"""
TWISTCOVSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/TwistWithCovariance twist
"""
ODOM_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
"""
PATH_DEF = """std_msgs/Header header
geometry_msgs/PoseStamped[] poses
"""
TEMP_DEF = """std_msgs/Header header
float64 temperature
float64 variance
"""
FLUID_DEF = """std_msgs/Header header
float64 fluid_pressure
float64 variance
"""
MAG_DEF = """std_msgs/Header header
geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance
"""
TRANS_DEF = """geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
"""
TRANSSTAMPED_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
"""
TFMSG_DEF = """geometry_msgs/TransformStamped[] transforms
"""
MADIM_DEF = """string label
uint32 size
uint32 stride
"""
MALAYOUT_DEF = """std_msgs/MultiArrayLayout layout
float32[] data
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
VEC3STAMPED_DEF = """std_msgs/Header header
geometry_msgs/Vector3 vector
"""
WRENCH_DEF = """geometry_msgs/Vector3 force
geometry_msgs/Vector3 torque
"""
WRENCHSTAMPED_DEF = """std_msgs/Header header
geometry_msgs/Wrench wrench
"""
JOINTSTATE_DEF = """std_msgs/Header header
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
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
"""
MARKERARRAY_DEF = """visualization_msgs/Marker[] markers
"""
INTERFACEVALUE_DEF = """string interface_name
float64[] values
"""
DYNJOINTSTATE_DEF = """std_msgs/Header header
string[] joint_names
string[] interface_names
control_msgs/InterfaceValue[] interface_values
"""


# -------------------- per-file conversion --------------------
def convert_single_mcap(mcap_path: Path, args):
    if not mcap_path.is_file():
        sys.exit(f"Input mcap not found: {mcap_path}")

    out_path = mcap_path.with_suffix(".bag")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Typestores
    ts2 = get_typestore(Stores.ROS2_JAZZY)
    ts1 = get_typestore(Stores.ROS1_NOETIC)

    # Ensure ROS2 types needed for CDR decode (same set as original script)
    ensure_type(ts2, "builtin_interfaces/msg/Time", TIME_DEF)
    ensure_type(ts2, "builtin_interfaces/msg/Duration", DURATION_DEF)
    ensure_type(ts2, "std_msgs/msg/Header", HEADER_DEF)
    ensure_type(ts2, "std_msgs/msg/ColorRGBA", COLOR_DEF)
    ensure_type(ts2, "sensor_msgs/msg/RegionOfInterest", ROI_DEF)
    ensure_type(ts2, "sensor_msgs/msg/CameraInfo", CAMERAINFO_DEF)
    ensure_type(ts2, "sensor_msgs/msg/Image", IMAGE_DEF)
    ensure_type(ts2, "sensor_msgs/msg/CompressedImage", CIMAGE_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Quaternion", QUAT_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Vector3", VEC3_DEF)
    ensure_type(ts2, "sensor_msgs/msg/Imu", IMU_DEF)
    ensure_type(ts2, "sensor_msgs/msg/PointField", PFIELD_DEF)
    ensure_type(ts2, "sensor_msgs/msg/PointCloud2", PC2_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Point", POINT_DEF)
    ensure_type(ts2, "geometry_msgs/msg/PointStamped", POINTSTAMPED_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Pose", POSE_DEF)
    ensure_type(ts2, "geometry_msgs/msg/PoseWithCovariance", POSECOV_DEF)
    ensure_type(ts2, "geometry_msgs/msg/PoseWithCovarianceStamped", POSECOVSTAMPED_DEF)
    ensure_type(ts2, "geometry_msgs/msg/PoseStamped", POSESTAMPED_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Twist", TWIST_DEF)
    ensure_type(ts2, "geometry_msgs/msg/TwistWithCovariance", TWISTCOV_DEF)
    ensure_type(ts2, "geometry_msgs/msg/TwistWithCovarianceStamped", TWISTCOVSTAMPED_DEF)
    ensure_type(ts2, "nav_msgs/msg/Odometry", ODOM_DEF)
    ensure_type(ts2, "nav_msgs/msg/Path", PATH_DEF)
    ensure_type(ts2, "geometry_msgs/msg/TwistStamped", TWISTSTAMPED_DEF)
    ensure_type(ts2, "sensor_msgs/msg/Temperature", TEMP_DEF)
    ensure_type(ts2, "sensor_msgs/msg/FluidPressure", FLUID_DEF)
    ensure_type(ts2, "sensor_msgs/msg/MagneticField", MAG_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Transform", TRANS_DEF)
    ensure_type(ts2, "geometry_msgs/msg/TransformStamped", TRANSSTAMPED_DEF)
    ensure_type(ts2, "tf2_msgs/msg/TFMessage", TFMSG_DEF)
    ensure_type(ts2, "std_msgs/msg/MultiArrayDimension", MADIM_DEF)
    ensure_type(ts2, "std_msgs/msg/MultiArrayLayout", MALAYOUT_DEF)
    ensure_type(ts2, "std_msgs/msg/Float32MultiArray", F32MA_DEF)
    ensure_type(ts2, "grid_map_msgs/msg/GridMapInfo", GRIDMAPINFO_DEF)
    ensure_type(ts2, "grid_map_msgs/msg/GridMap", GRIDMAP_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Vector3Stamped", VEC3STAMPED_DEF)
    ensure_type(ts2, "geometry_msgs/msg/Wrench", WRENCH_DEF)
    ensure_type(ts2, "geometry_msgs/msg/WrenchStamped", WRENCHSTAMPED_DEF)
    ensure_type(ts2, "sensor_msgs/msg/JointState", JOINTSTATE_DEF)
    ensure_type(ts2, "std_msgs/msg/Int8", INT8_DEF)
    ensure_type(ts2, "std_msgs/msg/UInt8", UINT8_DEF)
    ensure_type(ts2, "std_msgs/msg/UInt32", UINT32_DEF)
    ensure_type(ts2, "std_msgs/msg/Float64", FLOAT64_DEF)
    ensure_type(ts2, "visualization_msgs/msg/Marker", MARKER_DEF)
    ensure_type(ts2, "visualization_msgs/msg/MarkerArray", MARKERARRAY_DEF)
    ensure_type(ts2, "control_msgs/msg/InterfaceValue", INTERFACEVALUE_DEF)
    ensure_type(ts2, "control_msgs/msg/DynamicJointState", DYNJOINTSTATE_DEF)

    # Ensure custom ROS1 types (only if missing)
    ensure_type(ts1, "builtin_interfaces/msg/Time", TIME_DEF)
    ensure_type(ts1, "builtin_interfaces/msg/Duration", DURATION_DEF)
    ensure_type(ts1, "std_msgs/msg/Header", HEADER_DEF)
    ensure_type(ts1, "std_msgs/msg/ColorRGBA", COLOR_DEF)
    ensure_type(ts1, "sensor_msgs/msg/RegionOfInterest", ROI_DEF)
    ensure_type(ts1, "sensor_msgs/msg/CameraInfo", CAMERAINFO_DEF)
    ensure_type(ts1, "sensor_msgs/msg/Image", IMAGE_DEF)
    ensure_type(ts1, "sensor_msgs/msg/CompressedImage", CIMAGE_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Quaternion", QUAT_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Vector3", VEC3_DEF)
    ensure_type(ts1, "sensor_msgs/msg/Imu", IMU_DEF)
    ensure_type(ts1, "sensor_msgs/msg/PointField", PFIELD_DEF)
    ensure_type(ts1, "sensor_msgs/msg/PointCloud2", PC2_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Point", POINT_DEF)
    ensure_type(ts1, "geometry_msgs/msg/PointStamped", POINTSTAMPED_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Pose", POSE_DEF)
    ensure_type(ts1, "geometry_msgs/msg/PoseWithCovariance", POSECOV_DEF)
    ensure_type(ts1, "geometry_msgs/msg/PoseWithCovarianceStamped", POSECOVSTAMPED_DEF)
    ensure_type(ts1, "geometry_msgs/msg/PoseStamped", POSESTAMPED_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Twist", TWIST_DEF)
    ensure_type(ts1, "geometry_msgs/msg/TwistWithCovariance", TWISTCOV_DEF)
    ensure_type(ts1, "geometry_msgs/msg/TwistWithCovarianceStamped", TWISTCOVSTAMPED_DEF)
    ensure_type(ts1, "nav_msgs/msg/Odometry", ODOM_DEF)
    ensure_type(ts1, "nav_msgs/msg/Path", PATH_DEF)
    ensure_type(ts1, "geometry_msgs/msg/TwistStamped", TWISTSTAMPED_DEF)
    ensure_type(ts1, "sensor_msgs/msg/Temperature", TEMP_DEF)
    ensure_type(ts1, "sensor_msgs/msg/FluidPressure", FLUID_DEF)
    ensure_type(ts1, "sensor_msgs/msg/MagneticField", MAG_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Transform", TRANS_DEF)
    ensure_type(ts1, "geometry_msgs/msg/TransformStamped", TRANSSTAMPED_DEF)
    ensure_type(ts1, "tf2_msgs/msg/TFMessage", TFMSG_DEF)
    ensure_type(ts1, "std_msgs/msg/MultiArrayDimension", MADIM_DEF)
    ensure_type(ts1, "std_msgs/msg/MultiArrayLayout", MALAYOUT_DEF)
    ensure_type(ts1, "std_msgs/msg/Float32MultiArray", F32MA_DEF)
    ensure_type(ts1, "grid_map_msgs/msg/GridMapInfo", GRIDMAPINFO_DEF)
    ensure_type(ts1, "grid_map_msgs/msg/GridMap", GRIDMAP_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Vector3Stamped", VEC3STAMPED_DEF)
    ensure_type(ts1, "geometry_msgs/msg/Wrench", WRENCH_DEF)
    ensure_type(ts1, "geometry_msgs/msg/WrenchStamped", WRENCHSTAMPED_DEF)
    ensure_type(ts1, "sensor_msgs/msg/JointState", JOINTSTATE_DEF)
    ensure_type(ts1, "std_msgs/msg/Int8", INT8_DEF)
    ensure_type(ts1, "std_msgs/msg/UInt8", UINT8_DEF)
    ensure_type(ts1, "std_msgs/msg/UInt32", UINT32_DEF)
    ensure_type(ts1, "std_msgs/msg/Float64", FLOAT64_DEF)
    ensure_type(ts1, "visualization_msgs/msg/Marker", MARKER_DEF)
    ensure_type(ts1, "visualization_msgs/msg/MarkerArray", MARKERARRAY_DEF)
    ensure_type(ts1, "control_msgs/msg/InterfaceValue", INTERFACEVALUE_DEF)
    ensure_type(ts1, "control_msgs/msg/DynamicJointState", DYNJOINTSTATE_DEF)

    # ROS1 type aliases
    Time1 = ts1.types["builtin_interfaces/msg/Time"]
    Duration1 = ts1.types["builtin_interfaces/msg/Duration"]
    Header1 = ts1.types["std_msgs/msg/Header"]
    ColorRGBA1 = ts1.types["std_msgs/msg/ColorRGBA"]
    ROI1 = ts1.types["sensor_msgs/msg/RegionOfInterest"]
    CameraInfo1 = ts1.types["sensor_msgs/msg/CameraInfo"]
    Image1 = ts1.types["sensor_msgs/msg/Image"]
    CompressedImage1 = ts1.types["sensor_msgs/msg/CompressedImage"]
    Quaternion1 = ts1.types["geometry_msgs/msg/Quaternion"]
    Vector31 = ts1.types["geometry_msgs/msg/Vector3"]
    Imu1 = ts1.types["sensor_msgs/msg/Imu"]
    PointField1 = ts1.types["sensor_msgs/msg/PointField"]
    PointCloud21 = ts1.types["sensor_msgs/msg/PointCloud2"]
    Point1 = ts1.types["geometry_msgs/msg/Point"]
    PointStamped1 = ts1.types["geometry_msgs/msg/PointStamped"]
    Pose1 = ts1.types["geometry_msgs/msg/Pose"]
    PoseWithCov1 = ts1.types["geometry_msgs/msg/PoseWithCovariance"]
    PoseWithCovStamped1 = ts1.types["geometry_msgs/msg/PoseWithCovarianceStamped"]
    PoseStamped1 = ts1.types["geometry_msgs/msg/PoseStamped"]
    Twist1 = ts1.types["geometry_msgs/msg/Twist"]
    TwistWithCov1 = ts1.types["geometry_msgs/msg/TwistWithCovariance"]
    TwistWithCovStamped1 = ts1.types["geometry_msgs/msg/TwistWithCovarianceStamped"]
    Odometry1 = ts1.types["nav_msgs/msg/Odometry"]
    Path1 = ts1.types["nav_msgs/msg/Path"]
    TwistStamped1 = ts1.types["geometry_msgs/msg/TwistStamped"]
    TimeReference1 = ts1.types["sensor_msgs/msg/TimeReference"]
    Temperature1 = ts1.types["sensor_msgs/msg/Temperature"]
    FluidPressure1 = ts1.types["sensor_msgs/msg/FluidPressure"]
    MagneticField1 = ts1.types["sensor_msgs/msg/MagneticField"]
    Transform1 = ts1.types["geometry_msgs/msg/Transform"]
    TransformStamped1 = ts1.types["geometry_msgs/msg/TransformStamped"]
    TFMessage1 = ts1.types["tf2_msgs/msg/TFMessage"]
    MultiArrayDimension1 = ts1.types["std_msgs/msg/MultiArrayDimension"]
    MultiArrayLayout1 = ts1.types["std_msgs/msg/MultiArrayLayout"]
    Float32MultiArray1 = ts1.types["std_msgs/msg/Float32MultiArray"]
    GridMapInfo1 = ts1.types["grid_map_msgs/msg/GridMapInfo"]
    GridMap1 = ts1.types["grid_map_msgs/msg/GridMap"]
    Vector3Stamped1 = ts1.types["geometry_msgs/msg/Vector3Stamped"]
    Wrench1 = ts1.types["geometry_msgs/msg/Wrench"]
    WrenchStamped1 = ts1.types["geometry_msgs/msg/WrenchStamped"]
    JointState1 = ts1.types["sensor_msgs/msg/JointState"]
    Int8Msg1 = ts1.types["std_msgs/msg/Int8"]
    UInt8Msg1 = ts1.types["std_msgs/msg/UInt8"]
    UInt32Msg1 = ts1.types["std_msgs/msg/UInt32"]
    Float64Msg1 = ts1.types["std_msgs/msg/Float64"]
    Marker1 = ts1.types["visualization_msgs/msg/Marker"]
    MarkerArray1 = ts1.types["visualization_msgs/msg/MarkerArray"]
    InterfaceValue1 = ts1.types["control_msgs/msg/InterfaceValue"]
    DynamicJointState1 = ts1.types["control_msgs/msg/DynamicJointState"]

    TF_CANON = "tf2_msgs/msg/TFMessage"
    TF_TOPICS = ("/tf", "/tf_static")

    # Supported types (canonical)
    supported = {
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/PointCloud2",
        "nav_msgs/msg/Odometry",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/TwistWithCovarianceStamped",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        "nav_msgs/msg/Path",
        "visualization_msgs/msg/Marker",
        "visualization_msgs/msg/MarkerArray",
        "geometry_msgs/msg/PointStamped",
        "sensor_msgs/msg/TimeReference",
        "sensor_msgs/msg/Temperature",
        "sensor_msgs/msg/FluidPressure",
        "sensor_msgs/msg/MagneticField",
        "grid_map_msgs/msg/GridMap",
        "tf2_msgs/msg/TFMessage",
        "sensor_msgs/msg/JointState",
        "std_msgs/msg/Int8",
        "std_msgs/msg/UInt8",
        "std_msgs/msg/UInt32",
        "std_msgs/msg/Float64",
        "control_msgs/msg/InterfaceValue",
        "control_msgs/msg/DynamicJointState",
    }

    # Per-topic seq counter (ROS1 Header.seq)
    seq_by_topic = defaultdict(int)

    def t1_from_t2(t2):
        return Time1(int(getattr(t2, "sec", 0)), int(getattr(t2, "nanosec", 0)))

    def d1_from_d2(d2):
        return Duration1(int(getattr(d2, "sec", 0)), int(getattr(d2, "nanosec", 0)))

    def h1_from_h2(topic: str, h2, seq: int):
        st2 = getattr(h2, "stamp", None)
        return Header1(
            seq=int(seq),
            stamp=t1_from_t2(st2) if st2 is not None else Time1(0, 0),
            frame_id=str(getattr(h2, "frame_id", "")),
        )

    # Writer connections cache
    conn_by_topic_type = {}

    def get_conn(writer: Ros1BagWriter, topic: str, msgtype: str):
        key = (topic, msgtype)
        if key in conn_by_topic_type:
            return conn_by_topic_type[key]
        conn = writer.add_connection(topic, msgtype, typestore=ts1)
        conn_by_topic_type[key] = conn
        return conn

    # ---- track TF frames seen during conversion (requested) ----
    tf_unique_frames = set()

    # ---- FIX: buffer /tf by TransformStamped.header.stamp (bundle identical stamps only) ----
    tf_buffers = {}  # topic -> {"stamp_ns": int, "transforms": list[TransformStamped1]}

    def _stamp_ns_from_ros2_stamp(st) -> int:
        if st is None:
            return 0
        if hasattr(st, "sec") and hasattr(st, "nanosec"):
            return int(st.sec) * _NSEC + int(st.nanosec)
        if hasattr(st, "secs") and hasattr(st, "nsecs"):
            return int(st.secs) * _NSEC + int(st.nsecs)
        if hasattr(st, "sec") and hasattr(st, "nsec"):
            return int(st.sec) * _NSEC + int(st.nsec)
        return 0

    def flush_tf_buffer(writer: Ros1BagWriter, topic: str):
        buf = tf_buffers.get(topic)
        if not buf:
            return
        tlist = buf["transforms"]
        stamp_ns = int(buf["stamp_ns"])
        if tlist:
            obj_tf = TFMessage1(list(tlist))
            raw_tf = ts1.serialize_ros1(obj_tf, TF_CANON)
            conn_tf = get_conn(writer, topic, TF_CANON)
            writer.write(conn_tf, stamp_ns, raw_tf)
        tf_buffers.pop(topic, None)

    # ---- safer TFMessage decoding ----
    def _looks_like_cdr_encapsulation(b: bytes) -> bool:
        if len(b) < 4:
            return False
        return (b[:4] in (b"\x00\x01\x00\x00", b"\x00\x00\x00\x00", b"\x00\x03\x00\x00", b"\x00\x05\x00\x00"))

    def _tf_obj_valid(o) -> bool:
        try:
            trs = o.transforms
        except Exception:
            return False
        if trs is None:
            return False
        if len(trs) == 0:
            return True
        if len(trs) > 5000:
            return False
        for tr in trs[: min(len(trs), 10)]:
            h = getattr(tr, "header", None)
            fid = getattr(h, "frame_id", "")
            cid = getattr(tr, "child_frame_id", "")
            if not isinstance(fid, str) or not isinstance(cid, str):
                return False
            if len(fid) == 0 or len(cid) == 0:
                return False
            if len(fid) > 512 or len(cid) > 512:
                return False
            if (not fid.isprintable()) or (not cid.isprintable()):
                return False
        return True

    def deserialize_cdr_auto(data: bytes, msgtype: str):
        if msgtype != TF_CANON:
            return ts2.deserialize_cdr(data, msgtype)

        candidates = []
        if _looks_like_cdr_encapsulation(data):
            candidates.append(data)
        else:
            candidates.append(b"\x00\x01\x00\x00" + data)
            candidates.append(data)

        if len(data) >= 4:
            candidates.append(data[4:])
            if not _looks_like_cdr_encapsulation(data):
                candidates.append((b"\x00\x01\x00\x00" + data)[4:])

        last_err = None
        best_obj = None
        best_score = -1

        def _score(o) -> int:
            try:
                n = len(o.transforms or [])
            except Exception:
                return -1
            return (n * 10) + (5 if _tf_obj_valid(o) else 0)

        for payload in candidates:
            try:
                o = ts2.deserialize_cdr(payload, msgtype)
            except Exception as e:
                last_err = e
                continue
            sc = _score(o)
            if sc > best_score:
                best_obj, best_score = o, sc

        if best_obj is not None and _tf_obj_valid(best_obj):
            return best_obj
        if best_obj is not None:
            return best_obj
        if last_err is not None:
            raise last_err
        return ts2.deserialize_cdr(data, msgtype)

    # Convert ROS2-object (decoded from CDR) into ROS1-object for the same canonical msgtype
    def to_ros1_obj(topic: str, msgtype: str, obj2):
        seq = seq_by_topic[topic]
        seq_by_topic[topic] += 1

        if msgtype == "sensor_msgs/msg/CameraInfo":
            h = h1_from_h2(topic, obj2.header, seq)
            roi2 = obj2.roi
            roi = ROI1(
                int(roi2.x_offset),
                int(roi2.y_offset),
                int(roi2.height),
                int(roi2.width),
                bool(roi2.do_rectify),
            )
            return CameraInfo1(
                h,
                int(obj2.height),
                int(obj2.width),
                str(obj2.distortion_model),
                np.asarray(obj2.d, dtype=np.float64),
                np.asarray(obj2.k, dtype=np.float64),
                np.asarray(obj2.r, dtype=np.float64),
                np.asarray(obj2.p, dtype=np.float64),
                int(obj2.binning_x),
                int(obj2.binning_y),
                roi,
            )

        if msgtype == "sensor_msgs/msg/Image":
            h = h1_from_h2(topic, obj2.header, seq)
            return Image1(
                h,
                int(obj2.height),
                int(obj2.width),
                str(obj2.encoding),
                int(obj2.is_bigendian),
                int(obj2.step),
                np.asarray(obj2.data, dtype=np.uint8),
            )

        if msgtype == "sensor_msgs/msg/CompressedImage":
            h = h1_from_h2(topic, obj2.header, seq)
            return CompressedImage1(
                h,
                str(getattr(obj2, "format", "")),
                np.asarray(obj2.data, dtype=np.uint8),
            )

        if msgtype == "sensor_msgs/msg/Imu":
            h = h1_from_h2(topic, obj2.header, seq)
            o = obj2.orientation
            av = obj2.angular_velocity
            la = obj2.linear_acceleration
            return Imu1(
                h,
                Quaternion1(float(o.x), float(o.y), float(o.z), float(o.w)),
                np.asarray(obj2.orientation_covariance, dtype=np.float64),
                Vector31(float(av.x), float(av.y), float(av.z)),
                np.asarray(obj2.angular_velocity_covariance, dtype=np.float64),
                Vector31(float(la.x), float(la.y), float(la.z)),
                np.asarray(obj2.linear_acceleration_covariance, dtype=np.float64),
            )

        if msgtype == "sensor_msgs/msg/PointCloud2":
            h = h1_from_h2(topic, obj2.header, seq)
            fields = [
                PointField1(str(f.name), int(f.offset), int(f.datatype), int(f.count))
                for f in obj2.fields
            ]
            return PointCloud21(
                h,
                int(obj2.height),
                int(obj2.width),
                fields,
                bool(obj2.is_bigendian),
                int(obj2.point_step),
                int(obj2.row_step),
                np.asarray(obj2.data, dtype=np.uint8),
                bool(obj2.is_dense),
            )

        if msgtype == "nav_msgs/msg/Odometry":
            h = h1_from_h2(topic, obj2.header, seq)
            p = obj2.pose.pose.position
            q = obj2.pose.pose.orientation
            tlin = obj2.twist.twist.linear
            tang = obj2.twist.twist.angular
            pose = PoseWithCov1(
                Pose1(Point1(float(p.x), float(p.y), float(p.z)),
                      Quaternion1(float(q.x), float(q.y), float(q.z), float(q.w))),
                np.asarray(obj2.pose.covariance, dtype=np.float64),
            )
            twist = TwistWithCov1(
                Twist1(Vector31(float(tlin.x), float(tlin.y), float(tlin.z)),
                       Vector31(float(tang.x), float(tang.y), float(tang.z))),
                np.asarray(obj2.twist.covariance, dtype=np.float64),
            )
            return Odometry1(h, str(obj2.child_frame_id), pose, twist)

        if msgtype == "geometry_msgs/msg/TwistStamped":
            h = h1_from_h2(topic, obj2.header, seq)
            lin = obj2.twist.linear
            ang = obj2.twist.angular
            return TwistStamped1(
                h,
                Twist1(Vector31(float(lin.x), float(lin.y), float(lin.z)),
                       Vector31(float(ang.x), float(ang.y), float(ang.z))),
            )

        if msgtype == "geometry_msgs/msg/TwistWithCovarianceStamped":
            h = h1_from_h2(topic, obj2.header, seq)
            lin = obj2.twist.twist.linear
            ang = obj2.twist.twist.angular
            twcov = TwistWithCov1(
                Twist1(Vector31(float(lin.x), float(lin.y), float(lin.z)),
                       Vector31(float(ang.x), float(ang.y), float(ang.z))),
                np.asarray(obj2.twist.covariance, dtype=np.float64),
            )
            return TwistWithCovStamped1(h, twcov)

        if msgtype == "geometry_msgs/msg/PoseWithCovarianceStamped":
            h = h1_from_h2(topic, obj2.header, seq)
            p = obj2.pose.pose.position
            q = obj2.pose.pose.orientation
            pwc = PoseWithCov1(
                Pose1(Point1(float(p.x), float(p.y), float(p.z)),
                      Quaternion1(float(q.x), float(q.y), float(q.z), float(q.w))),
                np.asarray(obj2.pose.covariance, dtype=np.float64),
            )
            return PoseWithCovStamped1(h, pwc)

        if msgtype == "nav_msgs/msg/Path":
            h = h1_from_h2(topic, obj2.header, seq)
            poses = []
            for ps2 in obj2.poses:
                hps = h1_from_h2(topic, ps2.header, seq)  # keep same seq within record
                p = ps2.pose.position
                q = ps2.pose.orientation
                poses.append(
                    PoseStamped1(
                        hps,
                        Pose1(Point1(float(p.x), float(p.y), float(p.z)),
                              Quaternion1(float(q.x), float(q.y), float(q.z), float(q.w))),
                    )
                )
            return Path1(h, poses)

        if msgtype == "visualization_msgs/msg/Marker":
            h = h1_from_h2(topic, obj2.header, seq)
            p = obj2.pose.position
            q = obj2.pose.orientation
            pose = Pose1(Point1(float(p.x), float(p.y), float(p.z)),
                         Quaternion1(float(q.x), float(q.y), float(q.z), float(q.w)))
            s = obj2.scale
            scale = Vector31(float(s.x), float(s.y), float(s.z))
            c = obj2.color
            color = ColorRGBA1(float(c.r), float(c.g), float(c.b), float(c.a))
            lifetime = d1_from_d2(obj2.lifetime)
            pts = [Point1(float(pp.x), float(pp.y), float(pp.z)) for pp in (obj2.points or [])]
            cols = [ColorRGBA1(float(cc.r), float(cc.g), float(cc.b), float(cc.a)) for cc in (obj2.colors or [])]
            return Marker1(
                h,
                str(obj2.ns),
                int(obj2.id),
                int(obj2.type),
                int(obj2.action),
                pose,
                scale,
                color,
                lifetime,
                bool(obj2.frame_locked),
                pts,
                cols,
                str(obj2.text),
                str(obj2.mesh_resource),
                bool(obj2.mesh_use_embedded_materials),
            )

        if msgtype == "visualization_msgs/msg/MarkerArray":
            markers = []
            for m2 in (obj2.markers or []):
                hm = h1_from_h2(topic, m2.header, seq)
                p = m2.pose.position
                q = m2.pose.orientation
                pose = Pose1(Point1(float(p.x), float(p.y), float(p.z)),
                             Quaternion1(float(q.x), float(q.y), float(q.z), float(q.w)))
                s = m2.scale
                scale = Vector31(float(s.x), float(s.y), float(s.z))
                c = m2.color
                color = ColorRGBA1(float(c.r), float(c.g), float(c.b), float(c.a))
                lifetime = d1_from_d2(m2.lifetime)
                pts = [Point1(float(pp.x), float(pp.y), float(pp.z)) for pp in (m2.points or [])]
                cols = [ColorRGBA1(float(cc.r), float(cc.g), float(cc.b), float(cc.a)) for cc in (m2.colors or [])]
                markers.append(
                    Marker1(
                        hm,
                        str(m2.ns),
                        int(m2.id),
                        int(m2.type),
                        int(m2.action),
                        pose,
                        scale,
                        color,
                        lifetime,
                        bool(m2.frame_locked),
                        pts,
                        cols,
                        str(m2.text),
                        str(m2.mesh_resource),
                        bool(m2.mesh_use_embedded_materials),
                    )
                )
            return MarkerArray1(markers)

        if msgtype == "geometry_msgs/msg/PointStamped":
            h = h1_from_h2(topic, obj2.header, seq)
            pt = obj2.point
            return PointStamped1(h, Point1(float(pt.x), float(pt.y), float(pt.z)))

        if msgtype == "sensor_msgs/msg/TimeReference":
            return TimeReference1(h1_from_h2(topic, obj2.header, seq), t1_from_t2(obj2.time_ref), str(obj2.source))

        if msgtype == "sensor_msgs/msg/Temperature":
            h = h1_from_h2(topic, obj2.header, seq)
            return Temperature1(h, float(obj2.temperature), float(obj2.variance))

        if msgtype == "sensor_msgs/msg/FluidPressure":
            h = h1_from_h2(topic, obj2.header, seq)
            return FluidPressure1(h, float(obj2.fluid_pressure), float(obj2.variance))

        if msgtype == "sensor_msgs/msg/MagneticField":
            h = h1_from_h2(topic, obj2.header, seq)
            mf = obj2.magnetic_field
            return MagneticField1(
                h,
                Vector31(float(mf.x), float(mf.y), float(mf.z)),
                np.asarray(obj2.magnetic_field_covariance, dtype=np.float64),
            )

        if msgtype == "grid_map_msgs/msg/GridMap":
            h = h1_from_h2(topic, obj2.header, seq)
            ip = obj2.info.pose.position
            iq = obj2.info.pose.orientation
            info = GridMapInfo1(
                float(obj2.info.resolution),
                float(obj2.info.length_x),
                float(obj2.info.length_y),
                Pose1(Point1(float(ip.x), float(ip.y), float(ip.z)),
                      Quaternion1(float(iq.x), float(iq.y), float(iq.z), float(iq.w))),
            )
            data_arr = []
            for ma2 in (obj2.data or []):
                dims = [MultiArrayDimension1(str(d.label), int(d.size), int(d.stride)) for d in ma2.layout.dim]
                layout = MultiArrayLayout1(dims, int(ma2.layout.data_offset))
                arr = np.asarray(ma2.data, dtype=np.float32)
                data_arr.append(Float32MultiArray1(layout, arr))
            return GridMap1(
                h,
                info,
                list(obj2.layers),
                list(obj2.basic_layers),
                data_arr,
                int(obj2.outer_start_index),
                int(obj2.inner_start_index),
            )

        if msgtype == TF_CANON:
            tlist = []
            for tr2 in (obj2.transforms or []):
                ht = h1_from_h2(topic, tr2.header, seq)
                trans2 = tr2.transform
                t = Transform1(
                    Vector31(float(trans2.translation.x), float(trans2.translation.y), float(trans2.translation.z)),
                    Quaternion1(float(trans2.rotation.x), float(trans2.rotation.y), float(trans2.rotation.z), float(trans2.rotation.w)),
                )
                tlist.append(TransformStamped1(ht, str(tr2.child_frame_id), t))
            return TFMessage1(tlist)

        if msgtype == "sensor_msgs/msg/JointState":
            h = h1_from_h2(topic, obj2.header, seq)
            return JointState1(
                h,
                list(obj2.name),
                np.asarray(obj2.position, dtype=np.float64),
                np.asarray(obj2.velocity, dtype=np.float64),
                np.asarray(obj2.effort, dtype=np.float64),
            )

        if msgtype == "std_msgs/msg/Int8":
            return Int8Msg1(int(obj2.data))

        if msgtype == "std_msgs/msg/UInt8":
            return UInt8Msg1(int(obj2.data))

        if msgtype == "std_msgs/msg/UInt32":
            return UInt32Msg1(int(obj2.data))

        if msgtype == "std_msgs/msg/Float64":
            return Float64Msg1(float(obj2.data))

        if msgtype == "control_msgs/msg/InterfaceValue":
            return InterfaceValue1(str(obj2.interface_name), np.asarray(obj2.values, dtype=np.float64))

        if msgtype == "control_msgs/msg/DynamicJointState":
            h = h1_from_h2(topic, obj2.header, seq)
            ivs = []
            for iv2 in (obj2.interface_values or []):
                ivs.append(InterfaceValue1(str(iv2.interface_name), np.asarray(iv2.values, dtype=np.float64)))
            return DynamicJointState1(h, list(obj2.joint_names), list(obj2.interface_names), ivs)

        raise RuntimeError(f"Unsupported msgtype: {msgtype}")

    # Pre-scan to get total messages to process (progress bar like the forward converter)
    total_msgs = 0
    unsupported = set()
    with open(mcap_path, "rb") as inf_count:
        reader_count = make_reader(inf_count)
        for schema, channel, message in reader_count.iter_messages(log_time_order=True):
            msgtype = _canon(schema.name) if schema is not None else "<missing schema>"
            if msgtype not in supported:
                unsupported.add((channel.topic, msgtype))
            else:
                total_msgs += 1
    if unsupported:
        details = ", ".join(f"{topic} ({kind})" for topic, kind in sorted(unsupported))
        raise ValueError(f"Cannot convert all messages; unsupported topics: {details}")
    if total_msgs == 0:
        raise ValueError("Input contains no messages to convert")

    # Run conversion
    with open(mcap_path, "rb") as inf:
        writer = Ros1BagWriter(str(out_path))
        with writer:
            reader = make_reader(inf)

            pbar = make_pbar(total_msgs)

            for schema, channel, message in reader.iter_messages(log_time_order=True):
                if schema is None:
                    continue

                topic = channel.topic
                msgtype = _canon(schema.name)

                if msgtype not in supported:
                    continue

                # Decode ROS2 CDR payload
                obj2 = deserialize_cdr_auto(message.data, msgtype)

                # record TF frames during conversion (requested)
                if msgtype == TF_CANON and topic in TF_TOPICS:
                    for tr in (getattr(obj2, "transforms", None) or []):
                        h = getattr(tr, "header", None)
                        fid = str(getattr(h, "frame_id", "") or "")
                        cid = str(getattr(tr, "child_frame_id", "") or "")
                        if fid:
                            tf_unique_frames.add(fid)
                        if cid:
                            tf_unique_frames.add(cid)

                # Convert to ROS1 object
                obj1 = to_ros1_obj(topic, msgtype, obj2)

                # ---- FIX: write /tf using TransformStamped.header.stamp and bundle identical stamps ----
                if msgtype == TF_CANON and topic in TF_TOPICS:
                    ros2_trs = list(getattr(obj2, "transforms", []) or [])
                    ros1_trs = list(getattr(obj1, "transforms", []) or [])
                    n = min(len(ros2_trs), len(ros1_trs))

                    for i in range(n):
                        tr2 = ros2_trs[i]
                        tr1 = ros1_trs[i]

                        h2 = getattr(tr2, "header", None)
                        st2 = getattr(h2, "stamp", None)
                        stamp_ns = _stamp_ns_from_ros2_stamp(st2)
                        if stamp_ns == 0:
                            stamp_ns = int(message.log_time)

                        buf = tf_buffers.get(topic)
                        if buf is None or int(buf["stamp_ns"]) != int(stamp_ns):
                            if buf is not None:
                                flush_tf_buffer(writer, topic)
                            tf_buffers[topic] = {"stamp_ns": int(stamp_ns), "transforms": []}

                        tf_buffers[topic]["transforms"].append(tr1)

                    pbar.update(1)
                    continue

                # Normal write path for non-TF topics/types
                raw1 = ts1.serialize_ros1(obj1, msgtype)
                conn = get_conn(writer, topic, msgtype)
                writer.write(conn, int(message.log_time), raw1)
                pbar.update(1)

            # flush remaining TF buffers at end
            for tpc in list(tf_buffers.keys()):
                flush_tf_buffer(writer, tpc)

            pbar.close()

    # print TF unique frames after conversion (requested)
    if tf_unique_frames:
        frames_sorted = sorted(tf_unique_frames)
        sys.stdout.write(f"\n[tf] unique frames seen on /tf(/tf_static): {len(frames_sorted)}\n")
        for fr in frames_sorted:
            sys.stdout.write(f"  - {fr}\n")
        sys.stdout.flush()


# -------------------- CLI --------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="Convert ROS2/CDR .mcap; a directory selects batch processing.")
    ap.add_argument("--inmcap", required=True, help="Input file or directory")
    ap.add_argument("--glob", default=BATCH_GLOB, help="File pattern when input is a directory")
    args = ap.parse_args(argv)
    root = Path(args.inmcap).expanduser()
    if not root.exists():
        ap.error(f"Input does not exist: {root}")
    inputs = sorted(p for p in root.glob(args.glob) if p.is_file()) if root.is_dir() else [root]
    if not inputs:
        ap.error(f"No files matching '{args.glob}' in {root}")
    failures = 0
    for source in inputs:
        try:
            convert_single_mcap(source, args)
        except (Exception, SystemExit) as exc:
            failures += 1
            print(f"[ERROR] {source}: {exc}", file=sys.stderr)
    if failures:
        print(f"Conversion finished with {failures} failure(s).", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
