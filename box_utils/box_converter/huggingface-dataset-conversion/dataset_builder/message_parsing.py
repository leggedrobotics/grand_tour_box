from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Dict
from typing import Tuple
from typing import Union
from typing import cast

import cv2
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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from dataset_builder.dataset_config import ImageTopic
from dataset_builder.dataset_config import ImuTopic
from dataset_builder.dataset_config import LidarTopic
from dataset_builder.dataset_config import MagneticFieldTopic
from dataset_builder.dataset_config import NavSatFixTopic
from dataset_builder.dataset_config import OdometryTopic
from dataset_builder.dataset_config import PointTopic
from dataset_builder.dataset_config import PoseTopic
from dataset_builder.dataset_config import SingletonTransformTopic
from dataset_builder.dataset_config import Topic

BasicType = Union[np.ndarray, int, float, str, bool]


def extract_header_metadata_from_deserialized_message(msg: Any) -> Dict[str, Any]:
    header = _extract_header(msg)
    return {
        "frame_id": header.frame_id,
    }


def _extract_region_of_interest_metadata(msg: RegionOfInterest) -> Dict[str, Any]:
    return {
        "x_offset": msg.x_offset,
        "y_offset": msg.y_offset,
        "height": msg.height,
        "width": msg.width,
        "do_rectify": msg.do_rectify,
    }


def extract_camera_info_metadata_from_deserialized_message(
    msg: CameraInfo,
) -> Dict[str, Any]:
    ret = extract_header_metadata_from_deserialized_message(msg)
    ret["distortion_model"] = msg.distortion_model
    ret["width"] = msg.width
    ret["height"] = msg.height
    ret["D"] = list(msg.D)  # type: ignore
    ret["K"] = list(msg.K)  # type: ignore
    ret["R"] = list(msg.R)  # type: ignore
    ret["P"] = list(msg.P)  # type: ignore
    ret["binning_x"] = msg.binning_x
    ret["binning_y"] = msg.binning_y
    ret["roi"] = _extract_region_of_interest_metadata(msg.roi)
    return ret


CV_BRIDGE = CvBridge()


def extract_and_save_image_from_message(
    msg: CompressedImage, image_index: int, *, topic_desc: ImageTopic, image_dir: Path
) -> None:
    if topic_desc.compressed:
        image = CV_BRIDGE.compressed_imgmsg_to_cv2(msg)
    else:
        image = CV_BRIDGE.imgmsg_to_cv2(msg)
    file_path = image_dir / f"{image_index:06d}.{topic_desc.format}"
    cv2.imwrite(str(file_path), image)


def _pad_point_cloud(points: np.ndarray, max_points: int) -> np.ndarray:
    shape = points.shape
    new_shape = (max_points,) + shape[1:]
    ret = np.full(new_shape, 0, dtype=points.dtype)
    ret[: shape[0], ...] = points
    return ret


def _fix_dlio_point_cloud2_msg(msg: PointCloud2) -> PointCloud2:
    # dlio topics have duplicate timestamp fields that are not supported by
    # ros_numpy so we need to remove then
    field_names = [f.name for f in msg.fields]
    if "t" in field_names and "time" in field_names:
        for idx in sorted((-2, -3)):
            msg.fields.pop(idx)  # type: ignore
    return msg


def _parse_point_cloud2(
    msg: PointCloud2, topic_desc: LidarTopic
) -> Dict[str, BasicType]:
    msg = _fix_dlio_point_cloud2_msg(msg)
    structured_array = numpify(msg)
    assert structured_array is not None

    coordinates = ["x", "y", "z"]
    points = np.array([structured_array[c] for c in coordinates])
    ret = {
        "point_cloud_points": _pad_point_cloud(
            points.transpose(1, 0), topic_desc.max_points
        )
    }

    for name in structured_array.dtype.names:
        if name in coordinates:
            continue
        ret[f"point_cloud_{name}"] = _pad_point_cloud(
            np.array(structured_array[name]), topic_desc.max_points
        )
    return ret  # type: ignore


def _parse_nav_sat_fix(msg: NavSatFix) -> Dict[str, BasicType]:
    data = {
        "lat": msg.latitude,
        "long": msg.longitude,
        "alt": msg.altitude,
        "cov": np.array(msg.position_covariance).reshape(3, 3),
        "cov_type": msg.position_covariance_type,
    }
    return data


def _parse_quaternion(msg: Quaternion) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z, msg.w])


def _parse_vector3(msg: Union[Point, Vector3]) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z])


def _parse_covariance(arr: Union[np.ndarray, Tuple[float, ...]], n: int) -> np.ndarray:
    assert len(arr) == n * n
    return np.array(arr).reshape(n, n)


def _parse_magnetic_field(msg: MagneticField) -> Dict[str, BasicType]:
    data = {
        "b_field": _parse_vector3(msg.magnetic_field),
        "b_field_cov": _parse_covariance(msg.magnetic_field_covariance, 3),
    }
    return data  # type: ignore


def _parse_imu(msg: Imu) -> Dict[str, BasicType]:
    return {
        "orien": _parse_quaternion(msg.orientation),
        "orien_cov": _parse_covariance(msg.orientation_covariance, 3),
        "ang_vel": _parse_vector3(msg.angular_velocity),
        "ang_vel_cov": _parse_covariance(msg.angular_velocity_covariance, 3),
        "lin_acc": _parse_vector3(msg.linear_acceleration),
        "lin_acc_cov": _parse_covariance(msg.linear_acceleration_covariance, 3),
    }


def _parse_odometry(msg: Odometry) -> Dict[str, BasicType]:
    return {
        "pose_pos": _parse_vector3(msg.pose.pose.position),
        "pose_orien": _parse_quaternion(msg.pose.pose.orientation),
        "pose_cov": _parse_covariance(msg.pose.covariance, 6),
        "twist_lin": _parse_vector3(msg.twist.twist.linear),
        "twist_ang": _parse_vector3(msg.twist.twist.angular),
        "twist_cov": _parse_covariance(msg.twist.covariance, 6),
    }


def _parse_pose(
    msg: Union[PoseStamped, PoseWithCovarianceStamped], topic_desc: PoseTopic
) -> Dict[str, BasicType]:
    ret = {}
    if topic_desc.covariance:
        msg = cast(PoseWithCovarianceStamped, msg)
        raw_pose = msg.pose.pose
        ret["pose_cov"] = _parse_covariance(msg.pose.covariance, 6)
    else:
        msg = cast(PoseStamped, msg)
        raw_pose = msg.pose

    return {
        "pose_pos": _parse_vector3(raw_pose.position),
        "pose_orien": _parse_quaternion(raw_pose.orientation),
        **ret,
    }


def _parse_point(
    msg: PointStamped,
) -> Dict[str, BasicType]:
    return {
        "point": _parse_vector3(msg.point),
    }


def _parse_tf2_singleton_message(msg: TFMessage) -> Dict[str, BasicType]:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return {
        "translation": _parse_vector3(transform.transform.translation),
        "rotation": _parse_quaternion(transform.transform.rotation),
    }


def _extract_default_header(msg: Any) -> Header:
    return msg.header  # type: ignore


def _extract_tf2_message_header(msg: TFMessage) -> Header:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return transform.header  # type: ignore


SPECIAL_HEADER_MESSAGES_EXTRACT_FUNCTIONS = {
    "tf2_msgs/TFMessage": _extract_tf2_message_header,
}


def _extract_header(msg: Any) -> Header:
    extract_func = SPECIAL_HEADER_MESSAGES_EXTRACT_FUNCTIONS.get(msg._type)
    if extract_func is not None:
        header = extract_func(msg)
    else:
        header = _extract_default_header(msg)
    return header


def _extract_header_data_from_deserialized_message(msg: Any) -> Dict[str, BasicType]:
    header = _extract_header(msg)
    return {
        "timestamp": header.stamp.to_sec(),  # type: ignore
        "sequence_id": header.seq,  # type: ignore
    }


def _parse_message_data_from_deserialized_message(
    msg: Any, topic_desc: Topic
) -> Dict[str, BasicType]:
    if isinstance(topic_desc, LidarTopic):
        return _parse_point_cloud2(msg, topic_desc)
    elif isinstance(topic_desc, NavSatFixTopic):
        return _parse_nav_sat_fix(msg)
    elif isinstance(topic_desc, MagneticFieldTopic):
        return _parse_magnetic_field(msg)
    elif isinstance(topic_desc, ImuTopic):
        return _parse_imu(msg)
    elif isinstance(topic_desc, PoseTopic):
        return _parse_pose(msg, topic_desc)
    elif isinstance(topic_desc, PointTopic):
        return _parse_point(msg)
    elif isinstance(topic_desc, SingletonTransformTopic):
        return _parse_tf2_singleton_message(msg)
    elif isinstance(topic_desc, OdometryTopic):
        return _parse_odometry(msg)
    else:
        return {}


def parse_deserialized_message(msg: Any, topic_desc: Topic) -> Dict[str, BasicType]:
    header_data = _extract_header_data_from_deserialized_message(msg)
    message_data = _parse_message_data_from_deserialized_message(msg, topic_desc)
    return {**header_data, **message_data}


def deserialize_message(schema: mcap.records.Schema, data: bytes) -> Any:
    message_cls = get_message_class(schema.name)
    assert message_cls is not None, f"unknown message type: {schema.name}"
    return message_cls().deserialize(data)
