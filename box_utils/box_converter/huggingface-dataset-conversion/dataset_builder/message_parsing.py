from __future__ import annotations

from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple
from typing import Union
from typing import cast

import numpy as np
from anymal_msgs.msg import AnymalState  # type: ignore
from anymal_msgs.msg import Contact      # type: ignore
from anymal_msgs.msg import ExtendedJointState  # type: ignore
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from gnss_msgs.msg import GnssRaw  # type: ignore
from gps_common.msg import GPSFix  # type: ignore
from nav_msgs.msg import Odometry
from ros_numpy import numpify
from anymal_msgs.msg import SeActuatorReadings  # type: ignore
from anymal_msgs.msg import SeActuatorReading   # type: ignore
from anymal_msgs.msg import SeActuatorCommand   # type: ignore
from anymal_msgs.msg import SeActuatorState     # type: ignore
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import Temperature
from anymal_msgs.msg import BatteryState  # type: ignore
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from dataset_builder.dataset_config import AnymalStateTopic, GnssRawTopic
from dataset_builder.dataset_config import FluidPressureTopic
from dataset_builder.dataset_config import GPSFixTopic
from dataset_builder.dataset_config import ImuTopic
from dataset_builder.dataset_config import LidarTopic
from dataset_builder.dataset_config import MagneticFieldTopic
from dataset_builder.dataset_config import NavSatFixTopic
from dataset_builder.dataset_config import OdometryTopic
from dataset_builder.dataset_config import PointTopic
from dataset_builder.dataset_config import PoseTopic
from dataset_builder.dataset_config import SingletonTransformTopic
from dataset_builder.dataset_config import TemperatureTopic
from dataset_builder.dataset_config import Topic

from dataset_builder.dataset_config import (
    TwistTopic,
    AnymalTwistTopic,
    BatteryStateTopic,
    ActuatorReadingsTopic,
    ImuVector,
    AnymalDebugTopic,
)

BasicType = Union[np.ndarray, int, float, str, bool]


def _parse_quaternion(msg: Quaternion) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z, msg.w])


def _parse_vector3(msg: Union[Point, Vector3]) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.z])


def _parse_covariance(arr: Union[np.ndarray, Tuple[float, ...]], n: int) -> np.ndarray:
    assert len(arr) == n * n
    return np.array(arr).reshape(n, n)


def _parse_actuator_command(msg: SeActuatorCommand) -> Dict[str, BasicType]:
    """\
    for some reason the anymal actuator readings dont have a name,
    they all have `msg.name == ''`
    """
    return {
        "mode": msg.mode,
        "current": msg.current,
        "position": msg.position,
        "velocity": msg.velocity,
        "joint_torque": msg.joint_torque,
        "pid_gains_p": msg.pid_gains_p,
        "pid_gains_i": msg.pid_gains_i,
        "pid_gains_d": msg.pid_gains_d,
    }


def _parse_actuator_state(msg: SeActuatorState) -> Dict[str, BasicType]:
    """\
    for some reason the anymal actuator readings dont have a name,
    they all have `msg.name == ''`
    """
    ret: Dict[str, BasicType] = {
        "statusword": msg.statusword,
        "current": msg.current,
        "gear_position": msg.gear_position,
        "gear_velocity": msg.gear_velocity,
        "joint_position": msg.joint_position,
        "joint_velocity": msg.joint_velocity,
        "joint_acceleration": msg.joint_acceleration,
        "joint_torque": msg.joint_torque,
    }

    state_imu_data = _parse_imu(msg.imu, None)
    ret.update({f"imu_{k}": v for k, v in state_imu_data.items()})

    return ret


def _parse_single_actuator_reading(
    msg: SeActuatorReading, idx: int
) -> Dict[str, BasicType]:
    cdata = _parse_actuator_command(msg.commanded)
    sdata = _parse_actuator_state(msg.state)

    ret: Dict[str, BasicType] = {}
    ret.update({f"{idx:02d}_command_{k}": v for k, v in cdata.items()})
    ret.update({f"{idx:02d}_state_{k}": v for k, v in sdata.items()})

    return ret


def _parse_actuator_readings(
    msg: SeActuatorReadings, topic_desc: ActuatorReadingsTopic
) -> Dict[str, BasicType]:
    assert len(msg.readings) == topic_desc.number_of_actuators

    ret: Dict[str, BasicType] = {}
    for idx, reading in enumerate(msg.readings):
        actuator_data = _parse_single_actuator_reading(reading, idx)
        ret.update(actuator_data)
    return ret


def _parse_battery_state(msg: BatteryState, _: Any) -> Dict[str, BasicType]:
    return {
        "is_connected": msg.is_connected,
        "cell_temperature": msg.cell_temperature,
        "fet_temperature": msg.fet_temperature,
        "bms_temperature": msg.bms_temperature,
        "voltage": msg.voltage,
        "current": msg.current,
        "state_of_charge": msg.state_of_charge,
        "humidity": msg.humidity,
        "pressure": msg.pressure,
        "status": msg.status,
        "health_status": msg.health_status,
        "battery_status": msg.battery_status,
        "safety_status": msg.safety_status,
    }


def _parse_gnss_raw(msg: GnssRaw, _: Any) -> Dict[str, BasicType]:
    return {
        "position_ecef": _parse_vector3(msg.position_ecef),
        "position_ecef_std": _parse_vector3(msg.position_ecef_std),
        "orientation_hrp": _parse_vector3(msg.orientation_hrp),
        "orientation_hrp_std": _parse_vector3(msg.orientation_hrp_std),
    }


def _parse_extended_joint_state(msg: ExtendedJointState) -> Dict[str, BasicType]:
    return {
        "joint_positions": np.array(msg.position),
        "joint_velocities": np.array(msg.velocity),
        "joint_accelerations": np.array(msg.acceleration),
        "joint_efforts": np.array(msg.effort),
    }


def _parse_contact(msg: Optional[Contact]) -> Tuple[Dict[str, BasicType], str]:
    data = {
        "wrench_force": (_parse_vector3(msg.wrench.force) if msg else np.zeros(3)),
        "wrench_torque": (_parse_vector3(msg.wrench.torque) if msg else np.zeros(3)),
        "normal": _parse_vector3(msg.normal) if msg else np.zeros(3),
        "friction_coef": msg.frictionCoefficient if msg else 0,
        "restitution_coef": msg.restitutionCoefficient if msg else 0,
        "state": msg.state if msg else 0,
        "contact": 1 if msg else 0,
    }
    return data, msg.name if msg else ""


def _parse_contacts(
    contacts: List[Contact], topic_desc: AnymalStateTopic
) -> Dict[str, BasicType]:
    # extract all existing contacts
    contacts_by_foot_name: Dict[str, Dict[str, BasicType]] = {}
    for contact in contacts:
        contact_data, foot_name = _parse_contact(contact)
        assert foot_name in topic_desc.feet
        contacts_by_foot_name[foot_name] = contact_data

    # fill the missing contacts with default values
    for foot_name in topic_desc.feet:
        if foot_name not in contacts_by_foot_name:
            contacts_by_foot_name[foot_name] = _parse_contact(None)[0]

    # create the record by prefixing the contact attributes with the foot name
    ret: Dict[str, BasicType] = {}
    for foot_name, contact_data in contacts_by_foot_name.items():
        ret.update({f"{foot_name}_{k}": v for k, v in contact_data.items()})
    return ret


def _parse_anymal_state(
    msg: AnymalState, topic_desc: AnymalStateTopic
) -> Dict[str, BasicType]:
    ret: Dict[str, BasicType] = {
        "pose_pos": _parse_vector3(msg.pose.pose.position),
        "pose_orien": _parse_quaternion(msg.pose.pose.orientation),
        "twist_lin": _parse_vector3(msg.twist.twist.linear),
        "twist_ang": _parse_vector3(msg.twist.twist.angular),
    }
    ret.update(_parse_extended_joint_state(msg.joints))
    ret.update(_parse_contacts(msg.contacts, topic_desc))
    return ret


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


def _parse_nav_sat_fix(msg: NavSatFix, _: Any) -> Dict[str, BasicType]:
    data = {
        "lat": msg.latitude,
        "long": msg.longitude,
        "alt": msg.altitude,
        "cov": np.array(msg.position_covariance).reshape(3, 3),
        "cov_type": msg.position_covariance_type,
    }
    return data


def _parse_magnetic_field(msg: MagneticField, _: Any) -> Dict[str, BasicType]:
    data = {
        "b_field": _parse_vector3(msg.magnetic_field),
        "b_field_cov": _parse_covariance(msg.magnetic_field_covariance, 3),
    }
    return data  # type: ignore


def _parse_imu(msg: Imu, _: Any) -> Dict[str, BasicType]:
    return {
        "orien": _parse_quaternion(msg.orientation),
        "orien_cov": _parse_covariance(msg.orientation_covariance, 3),
        "ang_vel": _parse_vector3(msg.angular_velocity),
        "ang_vel_cov": _parse_covariance(msg.angular_velocity_covariance, 3),
        "lin_acc": _parse_vector3(msg.linear_acceleration),
        "lin_acc_cov": _parse_covariance(msg.linear_acceleration_covariance, 3),
    }


def _parse_odometry(msg: Odometry, _: Any) -> Dict[str, BasicType]:
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
    _: Any,
) -> Dict[str, BasicType]:
    return {
        "point": _parse_vector3(msg.point),
    }


def _parse_tf2_singleton_message(msg: TFMessage, _: Any) -> Dict[str, BasicType]:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return {
        "translation": _parse_vector3(transform.transform.translation),
        "rotation": _parse_quaternion(transform.transform.rotation),
    }


def _parse_temperature_message(msg: Temperature, _: Any) -> Dict[str, BasicType]:
    return {
        "temp": msg.temperature,
        "var": msg.variance,
    }


def _parse_fluid_pressure_message(msg: FluidPressure, _: Any) -> Dict[str, BasicType]:
    return {
        "pressure": msg.fluid_pressure,
        "var": msg.variance,
    }


def _parse_twist_message(msg: Twist, _: Any) -> Dict[str, BasicType]:
    return {
        "twist_lin": _parse_vector3(msg.linear),
        "twist_ang": _parse_vector3(msg.angular),
    }


def _parse_anymal_twist_message(msg: TwistStamped, _: Any) -> Dict[str, BasicType]:
    return {
        "linear": _parse_vector3(msg.twist.linear),
        "angular": _parse_vector3(msg.twist.angular),
    }


def _parse_gps_fix_message(msg: GPSFix, _: Any) -> Dict[str, BasicType]:
    return {
        "long": msg.longitude,
        "lat": msg.latitude,
        "alt": msg.altitude,
        "track": msg.track,
        "speed": msg.speed,
        "climb": msg.climb,
        "pitch": msg.pitch,
        "roll": msg.roll,
        "dip": msg.dip,
        "time": msg.time,
        "gdop": msg.gdop,
        "pdop": msg.pdop,
        "hdop": msg.hdop,
        "vdop": msg.vdop,
        "tdop": msg.tdop,
        "err": msg.err,
        "err_hor": msg.err_hor,
        "err_ver": msg.err_ver,
        "err_track": msg.err_track,
        "err_speed": msg.err_speed,
        "err_climb": msg.err_climb,
        "err_time": msg.err_time,
        "err_pitch": msg.err_pitch,
        "err_roll": msg.err_roll,
        "err_dip": msg.err_dip,
        "pos_cov": np.array(msg.position_covariance).reshape(3, 3),
        "pos_cov_type": msg.position_covariance_type,
    }


def _parse_imu_vecor(msg: Vector3, _: Any) -> Dict[str, BasicType]:
    return {
        "vector": _parse_vector3(msg.vector),
    }


def _parse_float32_multi_array(msg: Float32MultiArray, _: Any) -> Dict[str, BasicType]:
    dim_info = [
        {
            "label": dim.label,
            "size": dim.size,
            "stride": dim.stride,
        }
        for dim in msg.layout.dim
    ]
    return {
        "data": np.array(msg.data),
        "data_offset": msg.layout.data_offset,
        "dimensions": dim_info,
    }

def _extract_default_header(msg: Any) -> Header:
    return msg.header  # type: ignore


def _extract_tf2_message_header(msg: TFMessage) -> Header:
    assert len(msg.transforms) == 1
    transform = msg.transforms[0]  # type: ignore
    return transform.header  # type: ignore


def _extract_actuator_readings_header(msg: SeActuatorReadings) -> Header:
    return msg.readings[0].header


SPECIAL_HEADER_MESSAGES_EXTRACT_FUNCTIONS = [
    (SingletonTransformTopic, _extract_tf2_message_header),
    (ActuatorReadingsTopic, _extract_actuator_readings_header),
]


def _extract_header(msg: Any, topic_desc: Topic) -> Header:
    for topic_tp, func in SPECIAL_HEADER_MESSAGES_EXTRACT_FUNCTIONS:
        if isinstance(topic_desc, topic_tp):
            return func(msg)
    return _extract_default_header(msg)


def _extract_header_data_from_deserialized_message(
    msg: Any, topic_desc: Topic
) -> Dict[str, BasicType]:
    header = _extract_header(msg, topic_desc)
    if isinstance(header, list):  # since there can be multiple transforms in a TFMessage
        return {
            f"frame_id_{i}": h.frame_id for i, h in enumerate(header)
        }
    return {
        "timestamp": header.stamp.to_sec(),  # type: ignore
        "sequence_id": header.seq,  # type: ignore
    }


MESSAGE_PARSING_FUNCTIONS = [
    (LidarTopic, _parse_point_cloud2),
    (NavSatFixTopic, _parse_nav_sat_fix),
    (MagneticFieldTopic, _parse_magnetic_field),
    (ImuTopic, _parse_imu),
    (PoseTopic, _parse_pose),
    (PointTopic, _parse_point),
    (SingletonTransformTopic, _parse_tf2_singleton_message),
    (OdometryTopic, _parse_odometry),
    (AnymalStateTopic, _parse_anymal_state),
    (TemperatureTopic, _parse_temperature_message),
    (FluidPressureTopic, _parse_fluid_pressure_message),
    (TwistTopic, _parse_twist_message),
    (AnymalTwistTopic, _parse_anymal_twist_message),
    (GPSFixTopic, _parse_gps_fix_message),
    (GnssRawTopic, _parse_gnss_raw),
    (ImuVector, _parse_imu_vecor),
    (BatteryStateTopic, _parse_battery_state),
    (ActuatorReadingsTopic, _parse_actuator_readings),
    (AnymalDebugTopic, _parse_float32_multi_array),
]


def _parse_message_data_from_deserialized_message(
    msg: Any, topic_desc: Topic
) -> Dict[str, BasicType]:
    for topic_type, parser in MESSAGE_PARSING_FUNCTIONS:
        if isinstance(topic_desc, topic_type):
            return parser(msg, topic_desc)
    return {}


def parse_deserialized_message(msg: Any, topic_desc: Topic) -> Dict[str, BasicType]:
    header_data = _extract_header_data_from_deserialized_message(msg, topic_desc)
    message_data = _parse_message_data_from_deserialized_message(msg, topic_desc)
    return {**header_data, **message_data}


def _extract_region_of_interest_metadata(msg: RegionOfInterest) -> Dict[str, Any]:
    return {
        "x_offset": msg.x_offset,
        "y_offset": msg.y_offset,
        "height": msg.height,
        "width": msg.width,
        "do_rectify": msg.do_rectify,
    }


def extract_header_metadata_from_deserialized_message(
    msg: Any, topic_desc: Topic
) -> Dict[str, Any]:
    header = _extract_header(msg, topic_desc)
    return {
        "frame_id": header.frame_id,
    }


def extract_camera_info_metadata_from_deserialized_message(
    msg: CameraInfo, topic_desc: Topic
) -> Dict[str, Any]:
    ret = extract_header_metadata_from_deserialized_message(msg, topic_desc)
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
