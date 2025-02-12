from __future__ import annotations

from typing import Any
from typing import Callable
from typing import Dict
from typing import Optional
from typing import Tuple
from typing import Union

import mcap.records
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ros_numpy import numpify
from roslib.message import get_message_class  # type: ignore
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

BasicType = Union[np.ndarray, int, float, str, bool]


CV_BRIDGE = CvBridge()


def parse_compressed_image(
    msg: CompressedImage,
) -> Dict[str, BasicType]:
    image = CV_BRIDGE.compressed_imgmsg_to_cv2(msg)
    if image is None:
        return {}
    return {"image": image}


def load_image_from_message(msg: CompressedImage) -> Optional[np.ndarray]:
    return CV_BRIDGE.compressed_imgmsg_to_cv2(msg)


MAX_POINTS = 64000


def pad_point_cloud(points: np.ndarray) -> np.ndarray:
    shape = points.shape
    new_shape = (MAX_POINTS,) + shape[1:]
    ret = np.full(new_shape, 0, dtype=points.dtype)
    ret[: shape[0], ...] = points
    return ret


def fix_dlio_point_cloud2_msg(msg: PointCloud2) -> PointCloud2:
    # dlio topics have duplicate timestamp fields that are not supported by
    # ros_numpy so we need to remove then
    field_names = [f.name for f in msg.fields]
    if "t" in field_names and "time" in field_names:
        for idx in sorted((-2, -3)):
            msg.fields.pop(idx)  # type: ignore
    return msg


def parse_point_cloud2(msg: PointCloud2) -> Dict[str, BasicType]:
    msg = fix_dlio_point_cloud2_msg(msg)
    structured_array = numpify(msg)
    assert structured_array is not None

    coordinates = ["x", "y", "z"]
    points = np.array([structured_array[c] for c in coordinates])
    ret = {"point_cloud_points": pad_point_cloud(points.transpose(1, 0))}

    for name in structured_array.dtype.names:
        if name in coordinates:
            continue
        ret[f"point_cloud_{name}"] = pad_point_cloud(np.array(structured_array[name]))
    return ret  # type: ignore


def parse_nav_sat_fix(
    msg: NavSatFix,
) -> Dict[str, BasicType]:
    data = {
        "lat": msg.latitude,
        "long": msg.longitude,
        "alt": msg.altitude,
        "cov": np.array(msg.position_covariance).reshape(3, 3),
        "cov_type": msg.position_covariance_type,
    }
    return data


def parse_camera_info(
    msg: CameraInfo,
) -> Dict[str, BasicType]:
    # currently roi is always trivial and therefore not parsed
    data = {
        "height": msg.height,
        "width": msg.width,
        "distortion_model": msg.distortion_model,
        "D": np.array(msg.D),  # type: ignore
        "K": np.array(msg.K).reshape(3, 3),  # type: ignore
        "R": np.array(msg.R).reshape(3, 3),  # type: ignore
        "P": np.array(msg.P).reshape(3, 4),  # type: ignore
    }
    return data


def _parse_quaternion(msg: Quaternion) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z, msg.w])


def _parse_vector3(msg: Union[Point, Vector3]) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z])


def _parse_covariance(arr: Union[np.ndarray, Tuple[float, ...]], n: int) -> np.ndarray:
    assert len(arr) == n * n
    return np.array(arr).reshape(n, n)


def parse_magnetic_field(msg: MagneticField) -> Dict[str, BasicType]:
    data = {
        "b_field": _parse_vector3(msg.magnetic_field),
        "b_field_cov": _parse_covariance(msg.magnetic_field_covariance, 3),
    }
    return data  # type: ignore


def parse_imu(msg: Imu) -> Dict[str, BasicType]:
    return {
        "orien": _parse_quaternion(msg.orientation),
        "orien_cov": _parse_covariance(msg.orientation_covariance, 3),
        "ang_vel": _parse_vector3(msg.angular_velocity),
        "ang_vel_cov": _parse_covariance(msg.angular_velocity_covariance, 3),
        "lin_acc": _parse_vector3(msg.linear_acceleration),
        "lin_acc_cov": _parse_covariance(msg.linear_acceleration_covariance, 3),
    }


def parse_illuminance(msg: Illuminance) -> Dict[str, BasicType]:
    return {
        "illum": msg.illuminance,
        "illum_var": msg.variance,
    }


def parse_odometry(msg: Odometry) -> Dict[str, BasicType]:
    return {
        "pose_pos": _parse_vector3(msg.pose.pose.position),
        "pose_orien": _parse_quaternion(msg.pose.pose.orientation),
        "pose_cov": _parse_covariance(msg.pose.covariance, 6),
        "twist_lin": _parse_vector3(msg.twist.twist.linear),
        "twist_ang": _parse_vector3(msg.twist.twist.angular),
        "twist_cov": _parse_covariance(msg.twist.covariance, 6),
    }


def parse_pose(
    msg: PoseStamped,
) -> Dict[str, BasicType]:
    return {
        "pose_pos": _parse_vector3(msg.pose.position),
        "pose_orien": _parse_quaternion(msg.pose.orientation),
    }


def parse_pose_with_covar(
    msg: PoseWithCovarianceStamped,
) -> Dict[str, BasicType]:
    return {
        "pose_pos": _parse_vector3(msg.pose.pose.position),
        "pose_orien": _parse_quaternion(msg.pose.pose.orientation),
        "pose_cov": _parse_covariance(msg.pose.covariance, 6),
    }


def parse_point(
    msg: PointStamped,
) -> Dict[str, BasicType]:
    return {
        "point": _parse_vector3(msg.point),
    }


def parse_tf2_singleton_message(msg: TFMessage) -> Dict[str, BasicType]:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return {
        "translation": _parse_vector3(transform.transform.translation),
        "rotation": _parse_quaternion(transform.transform.rotation),
    }


def parse_tf2_header_message(msg: TFMessage) -> Dict[str, BasicType]:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return {
        "timestamp": transform.header.stamp.to_nsec(),
        "sequence_id": transform.header.seq,
    }


MESSAGE_PARSERS: Dict[str, Callable[[Any], Dict[str, BasicType]]] = {
    "sensor_msgs/CompressedImage": lambda _: {},
    "sensor_msgs/NavSatFix": parse_nav_sat_fix,
    "sensor_msgs/PointCloud2": parse_point_cloud2,
    "sensor_msgs/MagneticField": parse_magnetic_field,
    "sensor_msgs/Imu": parse_imu,
    "sensor_msgs/Illuminance": parse_illuminance,
    "nav_msgs/Odometry": parse_odometry,
    "geometry_msgs/PoseWithCovarianceStamped": parse_pose_with_covar,
    "geometry_msgs/PoseStamped": parse_pose,
    "geometry_msgs/PointStamped": parse_point,
    "tf2_msgs/TFMessage": parse_tf2_singleton_message,
}


SPECIAL_HEADER_MESSAGES = {
    "tf2_msgs/TFMessage": parse_tf2_header_message,
}

SKIP_MESSAGES = [
    "sensor_msgs/CameraInfo",  # metadata
    "std_msgs/Header",  # metadata
    "std_msgs/String",  # metadata
    "tf/tfMessage",  # not used
    "nav_msgs/Path",  # this is just an aggregate of past poses
    "sensor_msgs/FluidPressure",  # not useful
    "sensor_msgs/Temperature",  # not useful
]


def extract_default_header_info(msg: Any) -> Dict[str, BasicType]:
    header: Header = msg.header
    return {
        "timestamp": header.stamp.to_nsec(),  # type: ignore
        "sequence_id": header.seq,  # type: ignore
    }


def deserialize_message(
    schema: Optional[mcap.records.Schema], data: bytes
) -> Optional[Any]:
    if schema is None:
        return
    message_cls = get_message_class(schema.name)
    if message_cls is None:
        return None
    message = message_cls().deserialize(data)
    if message._type in SKIP_MESSAGES:
        return None
    return message


def parse_message_data(msg: Any, message_type: str) -> Dict[str, BasicType]:
    return MESSAGE_PARSERS[message_type](msg)


def parse_deserialized_message(msg: Any) -> Dict[str, BasicType]:
    message_data = parse_message_data(msg, msg._type)
    if msg._type in SPECIAL_HEADER_MESSAGES:
        extract_func = SPECIAL_HEADER_MESSAGES[msg._type]
        header_data = extract_func(msg)
    else:
        header_data = extract_default_header_info(msg)
    message_data.update(header_data)
    return message_data
