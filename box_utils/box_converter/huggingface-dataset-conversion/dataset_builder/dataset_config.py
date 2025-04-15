from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Mapping
from typing import Sequence
from typing import Tuple

import numpy as np
import numpy.typing
import yaml

METADATA_KEY = "metadata"
DATA_KEY = "data"

CAMERA_INTRISICS_KEY = "camera_intrinsics"
FRAME_TRANSFORMS_KEY = "frame_transforms"

#TODO: add missing topics according to mission


@dataclass(frozen=True)
class ArrayType:
    shape: Tuple[int, ...]
    dtype: numpy.typing.DTypeLike


@dataclass
class Topic:
    alias: str
    topic: str
    file: str


@dataclass
class ImageTopic(Topic):
    format: str = "jpeg"
    compressed: bool = True


@dataclass
class LidarTopic(Topic):
    max_points: int
    attributes: List[str] = field(default_factory=list)


@dataclass
class PoseTopic(Topic):
    covariance: bool


@dataclass
class ImuTopic(Topic): ...


@dataclass
class OdometryTopic(Topic): ...


@dataclass
class AnymalStateTopic(Topic):
    number_of_joints: int
    feet: List[str] = field(default_factory=list)


@dataclass
class NavSatFixTopic(Topic): ...


@dataclass
class MagneticFieldTopic(Topic): ...


@dataclass
class PointTopic(Topic): ...


@dataclass
class SingletonTransformTopic(Topic): ...


@dataclass
class TemperatureTopic(Topic): ...


@dataclass
class FluidPressureTopic(Topic): ...


@dataclass
class TwistTopic(Topic): ...


@dataclass
class GPSFixTopic(Topic): ...


@dataclass
class GnssRawTopic(Topic): ...


@dataclass
class ActuatorReadingsTopic(Topic):
    number_of_actuators: int


@dataclass
class BatteryStateTopic(Topic): ...


@dataclass
class CameraInfoTopic(Topic): ...


@dataclass
class FrameTransformConfig:
    topic: str
    file: str
    base_frame: str = "base" #TODO rename to child_frame_id


@dataclass
class MetadataConfig:
    frame_transforms: FrameTransformConfig
    camera_intrinsics: List[CameraInfoTopic] = field(default_factory=list) # set a default value to []


AttributeTypes = Dict[str, ArrayType]
TopicRegistry = Dict[str, Tuple[AttributeTypes, Topic]]


def _build_single_actuator_reading(idx: int) -> Dict[str, ArrayType]:
    fmt = f"{idx:02d}"
    return {
        f"{fmt}_state_statusword": ArrayType(tuple(), np.uint32),
        f"{fmt}_state_current": ArrayType(tuple(), np.float64),
        f"{fmt}_state_gear_position": ArrayType(tuple(), np.float64),
        f"{fmt}_state_gear_velocity": ArrayType(tuple(), np.float64),
        f"{fmt}_state_joint_position": ArrayType(tuple(), np.float64),
        f"{fmt}_state_joint_velocity": ArrayType(tuple(), np.float64),
        f"{fmt}_state_joint_acceleration": ArrayType(tuple(), np.float64),
        f"{fmt}_state_joint_torque": ArrayType(tuple(), np.float64),
        f"{fmt}_state_imu_orien_cov": ArrayType((3, 3), np.float64),
        f"{fmt}_state_imu_ang_vel_cov": ArrayType((3, 3), np.float64),
        f"{fmt}_state_imu_lin_acc_cov": ArrayType((3, 3), np.float64),
        f"{fmt}_state_imu_orien": ArrayType((4,), np.float64),
        f"{fmt}_state_imu_ang_vel": ArrayType((3,), np.float64),
        f"{fmt}_state_imu_lin_acc": ArrayType((3,), np.float64),
        f"{fmt}_command_mode": ArrayType(tuple(), np.uint8),
        f"{fmt}_command_current": ArrayType(tuple(), np.float64),
        f"{fmt}_command_position": ArrayType(tuple(), np.float64),
        f"{fmt}_command_velocity": ArrayType(tuple(), np.float64),
        f"{fmt}_command_joint_torque": ArrayType(tuple(), np.float64),
        f"{fmt}_command_pid_gains_p": ArrayType(tuple(), np.float32),
        f"{fmt}_command_pid_gains_i": ArrayType(tuple(), np.float32),
        f"{fmt}_command_pid_gains_d": ArrayType(tuple(), np.float32),
    }


def _build_actuator_readings_single_topic_attributes(topic_desc: Any) -> AttributeTypes:
    ret = {}
    for idx in range(topic_desc.number_of_actuators):
        ret.update(_build_single_actuator_reading(idx))
    return ret


def _build_actuator_readings_topics_attributes(
    actuator_readings_topics: Sequence[ActuatorReadingsTopic],
) -> TopicRegistry:
    return {
        topic.alias: (_build_actuator_readings_single_topic_attributes(topic), topic)
        for topic in actuator_readings_topics
    }


def _build_battery_state_topics_attributes(
    battery_state_topics: Sequence[BatteryStateTopic],
) -> TopicRegistry:
    topic_tp = {
        "is_connected": ArrayType(tuple(), np.uint8),
        "cell_temperature": ArrayType(tuple(), np.float32),
        "fet_temperature": ArrayType(tuple(), np.float32),
        "bms_temperature": ArrayType(tuple(), np.float32),
        "voltage": ArrayType(tuple(), np.float32),
        "current": ArrayType(tuple(), np.float32),
        "state_of_charge": ArrayType(tuple(), np.float32),
        "humidity": ArrayType(tuple(), np.float32),
        "pressure": ArrayType(tuple(), np.float32),
        "status": ArrayType(tuple(), np.uint8),
        "health_status": ArrayType(tuple(), np.uint8),
        "battery_status": ArrayType(tuple(), np.uint16),
        "safety_status": ArrayType(tuple(), np.uint32),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in battery_state_topics}


def _build_gnss_raw_topics_attributes(
    gnss_raw_topics: Sequence[GnssRawTopic],
) -> TopicRegistry:
    topic_tp = {
        "position_ecef": ArrayType((3,), np.float64),
        "position_ecef_std": ArrayType((3,), np.float64),
        "orientation_hrp": ArrayType((3,), np.float64),
        "orientation_hrp_std": ArrayType((3,), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in gnss_raw_topics}


def _build_point_cloud_attr_types(
    properties: Sequence[str], max_points: int
) -> AttributeTypes:
    POINT_CLOUD_PROPERTIES: Dict[str, ArrayType] = {
        "intensity": ArrayType((max_points,), np.float32),
        "timestamp": ArrayType((max_points,), np.uint64),
        "tag": ArrayType((max_points,), np.uint8),
        "line": ArrayType((max_points,), np.uint16),
        "ring": ArrayType((max_points,), np.uint16),
    }

    column_types: Dict[str, ArrayType] = {
        "point_cloud_points": ArrayType((max_points, 3), np.float32),
    }
    for prop in properties:
        column_types[f"point_cloud_{prop}"] = POINT_CLOUD_PROPERTIES[prop]
    return column_types


def _build_pose_attr_types(covar: bool) -> AttributeTypes:
    column_types = {
        "pose_pos": ArrayType((3,), np.float64),
        "pose_orien": ArrayType((4,), np.float64),
    }

    if covar:
        column_types["pose_cov"] = ArrayType((6, 6), np.float64)
    return column_types


def _build_image_topics_attributes(
    image_topics: Sequence[ImageTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in image_topics:
        ret[topic.alias] = ({}, topic)
    return ret


def _build_lidar_topics_attributes(
    lidar_topics: Sequence[LidarTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in lidar_topics:
        ret[topic.alias] = (
            _build_point_cloud_attr_types(topic.attributes, topic.max_points),
            topic,
        )
    return ret


def _build_pose_topics_attributes(
    pose_topics: Sequence[PoseTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in pose_topics:
        ret[topic.alias] = (_build_pose_attr_types(topic.covariance), topic)
    return ret


def _build_imu_topics_attributes(
    imu_topics: Sequence[ImuTopic],
) -> TopicRegistry:
    topic_tp = {
        "orien_cov": ArrayType((3, 3), np.float64),
        "ang_vel_cov": ArrayType((3, 3), np.float64),
        "lin_acc_cov": ArrayType((3, 3), np.float64),
        "orien": ArrayType((4,), np.float64),
        "ang_vel": ArrayType((3,), np.float64),
        "lin_acc": ArrayType((3,), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in imu_topics}


def _build_temperature_topics_attributes(
    temperature_topics: Sequence[TemperatureTopic],
) -> TopicRegistry:
    topic_tp = {
        "temp": ArrayType(tuple(), np.float64),
        "var": ArrayType(tuple(), np.float64),
    }
    return {topic.alias: (topic_tp.copy(), topic) for topic in temperature_topics}


def _build_fluid_pressure_topics_attributes(
    fluid_pressure_topics: Sequence[FluidPressureTopic],
) -> TopicRegistry:
    topic_tp = {
        "pressure": ArrayType(tuple(), np.float64),
        "var": ArrayType(tuple(), np.float64),
    }
    return {topic.alias: (topic_tp.copy(), topic) for topic in fluid_pressure_topics}


def _build_anymal_state_single_topic_attributes(
    anymal_state_topic: AnymalStateTopic,
) -> AttributeTypes:
    ANYMAL_FOOT_CONTACT_MSG = {
        "wrench_force": ArrayType((3,), np.float64),
        "wrench_torque": ArrayType((3,), np.float64),
        "normal": ArrayType((3,), np.float64),
        "friction_coef": ArrayType(tuple(), np.float64),
        "restitution_coef": ArrayType(tuple(), np.float64),
        "state": ArrayType(tuple(), np.uint8),
        "contact": ArrayType(tuple(), np.uint8),  # 0: no contact, 1: contact
    }

    n_joints = anymal_state_topic.number_of_joints
    ret = {
        "pose_pos": ArrayType((3,), np.float64),
        "pose_orien": ArrayType((4,), np.float64),
        "twist_lin": ArrayType((3,), np.float64),
        "twist_ang": ArrayType((3,), np.float64),
        "joint_positions": ArrayType((n_joints,), np.float64),
        "joint_velocities": ArrayType((n_joints,), np.float64),
        "joint_accelerations": ArrayType((n_joints,), np.float64),
        "joint_efforts": ArrayType((n_joints,), np.float64),
    }

    for foot in anymal_state_topic.feet:
        ret.update(
            {f"{foot}_{key}": value for key, value in ANYMAL_FOOT_CONTACT_MSG.items()}
        )
    return ret


def _build_anymal_state_topics_attributes(
    anymal_state_topics: Sequence[AnymalStateTopic],
) -> TopicRegistry:

    return {
        topic.alias: (_build_anymal_state_single_topic_attributes(topic), topic)
        for topic in anymal_state_topics
    }


def _build_odometry_topics_attributes(
    odometry_topics: Sequence[OdometryTopic],
) -> TopicRegistry:
    topic_tp = {
        "pose_cov": ArrayType((6, 6), np.float64),
        "twist_cov": ArrayType((6, 6), np.float64),
        "pose_pos": ArrayType((3,), np.float64),
        "pose_orien": ArrayType((4,), np.float64),
        "twist_lin": ArrayType((3,), np.float64),
        "twist_ang": ArrayType((3,), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in odometry_topics}


def _build_nav_sat_fix_topics_attributes(
    nav_sat_fix_topics: Sequence[NavSatFixTopic],
) -> TopicRegistry:
    topic_tp = {
        "lat": ArrayType(tuple(), np.float64),
        "long": ArrayType(tuple(), np.float64),
        "alt": ArrayType(tuple(), np.float64),
        "cov": ArrayType((3, 3), np.float64),
        "cov_type": ArrayType(tuple(), np.uint8),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in nav_sat_fix_topics}


def _build_magnetic_field_topics_attributes(
    magnetic_field_topics: Sequence[MagneticFieldTopic],
) -> TopicRegistry:

    topic_tp = {
        "b_field": ArrayType((3,), np.float64),
        "b_field_cov": ArrayType((3, 3), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in magnetic_field_topics}


def _build_point_topics_attributes(
    point_topics: Sequence[PointTopic],
) -> TopicRegistry:
    topic_tp = {
        "point": ArrayType((3,), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in point_topics}


def _build_singleton_transform_topics_attributes(
    singleton_transform_topics: Sequence[SingletonTransformTopic],
) -> TopicRegistry:
    topic_tp = {
        "translation": ArrayType((3,), np.float64),
        "rotation": ArrayType((4,), np.float64),
    }

    return {
        topic.alias: (topic_tp.copy(), topic) for topic in singleton_transform_topics
    }


def _build_gps_fix_topics_attributes(
    gps_fix_topics: Sequence[GPSFixTopic],
):
    topic_tp = {
        "lat": ArrayType(tuple(), np.float64),
        "long": ArrayType(tuple(), np.float64),
        "alt": ArrayType(tuple(), np.float64),
        "track": ArrayType(tuple(), np.float64),
        "speed": ArrayType(tuple(), np.float64),
        "climb": ArrayType(tuple(), np.float64),
        "pitch": ArrayType(tuple(), np.float64),
        "roll": ArrayType(tuple(), np.float64),
        "dip": ArrayType(tuple(), np.float64),
        "time": ArrayType(tuple(), np.float64),
        "gdop": ArrayType(tuple(), np.float64),
        "pdop": ArrayType(tuple(), np.float64),
        "hdop": ArrayType(tuple(), np.float64),
        "vdop": ArrayType(tuple(), np.float64),
        "tdop": ArrayType(tuple(), np.float64),
        "err": ArrayType(tuple(), np.float64),
        "err_hor": ArrayType(tuple(), np.float64),
        "err_ver": ArrayType(tuple(), np.float64),
        "err_track": ArrayType(tuple(), np.float64),
        "err_speed": ArrayType(tuple(), np.float64),
        "err_climb": ArrayType(tuple(), np.float64),
        "err_time": ArrayType(tuple(), np.float64),
        "err_pitch": ArrayType(tuple(), np.float64),
        "err_roll": ArrayType(tuple(), np.float64),
        "err_dip": ArrayType(tuple(), np.float64),
        "pos_cov": ArrayType((3, 3), np.float64),
        "pos_cov_type": ArrayType(tuple(), np.uint8),
    }
    return {topic.alias: (topic_tp.copy(), topic) for topic in gps_fix_topics}


def _build_twist_topics_attributes(
    twist_topics: Sequence[TwistTopic],
):
    topic_tp = {
        "twist_lin": ArrayType((3,), np.float64),
        "twist_ang": ArrayType((3,), np.float64),
    }
    return {topic.alias: (topic_tp.copy(), topic) for topic in twist_topics}


UNIVERSAL_ATTRIBUTES: Dict[str, ArrayType] = {
    "timestamp": ArrayType(tuple(), np.uint64),
    "sequence_id": ArrayType(tuple(), np.uint64),
}


def _add_universal_attributes(
    registry: TopicRegistry,
) -> TopicRegistry:
    return {
        topic: ({**UNIVERSAL_ATTRIBUTES, **attributes}, topic_obj)
        for topic, (attributes, topic_obj) in registry.items()
    }


TOPIC_TYPES = {
    "image_topics": (ImageTopic, _build_image_topics_attributes),
    "lidar_topics": (LidarTopic, _build_lidar_topics_attributes),
    "pose_topics": (PoseTopic, _build_pose_topics_attributes),
    "imu_topics": (ImuTopic, _build_imu_topics_attributes),
    "odometry_topics": (OdometryTopic, _build_odometry_topics_attributes),
    "anymal_state_topics": (AnymalStateTopic, _build_anymal_state_topics_attributes),
    "nav_sat_fix_topics": (NavSatFixTopic, _build_nav_sat_fix_topics_attributes),
    "magnetic_field_topics": (
        MagneticFieldTopic,
        _build_magnetic_field_topics_attributes,
    ),
    "point_topics": (PointTopic, _build_point_topics_attributes),
    "singleton_transform_topics": (
        SingletonTransformTopic,
        _build_singleton_transform_topics_attributes,
    ),
    "temperature_topics": (TemperatureTopic, _build_temperature_topics_attributes),
    "fluid_pressure_topics": (
        FluidPressureTopic,
        _build_fluid_pressure_topics_attributes,
    ),
    "gps_fix_topics": (GPSFixTopic, _build_gps_fix_topics_attributes),
    "twist_topics": (TwistTopic, _build_twist_topics_attributes),
    "gnss_raw_topics": (GnssRawTopic, _build_gnss_raw_topics_attributes),
    "battery_state_topics": (BatteryStateTopic, _build_battery_state_topics_attributes),
    "actuator_readings_topics": (
        ActuatorReadingsTopic,
        _build_actuator_readings_topics_attributes,
    ),
}


def _load_topic_registry_from_config(
    data_config_object: Mapping[str, List[Mapping[str, Any]]], mission_name: str
) -> TopicRegistry:
    
    """
    Load the topic registry from the data part of the config file
    """

    registry: TopicRegistry = {}
    assert isinstance(
        data_config_object, Mapping
    ), "data part of config file must be a mapping"

    for key in data_config_object.keys():
        assert key in TOPIC_TYPES.keys(), f"unknown key in config: {key}"

    try:
        for key, (topic_type, _build_fn) in TOPIC_TYPES.items():
            if key in data_config_object:
                topics = [
                    topic_type(**topic_obj)
                    for topic_obj in data_config_object.get(key, [])
                ]
                registry.update(_build_fn(topics))

        for _, topic_obj in registry.values():
            topic_obj.file = topic_obj.file.format(mission_name)

    except Exception as e:
        # TODO: remove this breakpoint
        breakpoint()
        raise ValueError(f"error parsing data part of config file: {e}") from e
    
    return _add_universal_attributes(registry)

def _load_metadata_config(
    metadata_config_object: Mapping[str, Any], mission_name: str
) -> MetadataConfig:
    """
    Load the metadata config from the metadata part of the config file
    """
    
    camera_intrinsics_object = metadata_config_object.get(CAMERA_INTRISICS_KEY, [])

    try:
        camera_intrinsics = [
            CameraInfoTopic(**topic_obj) for topic_obj in camera_intrinsics_object
        ]
    except Exception as e:
        
        # TODO: remove this breakpoint
        breakpoint()
        raise ValueError(
            f"error parsing {CAMERA_INTRISICS_KEY!r} part of config file: {e}"
        ) from e

    try:
        frame_transforms_object = metadata_config_object.get(FRAME_TRANSFORMS_KEY, {})
        frame_transforms = FrameTransformConfig(**frame_transforms_object)
    except Exception as e:

        # TODO: remove this breakpoint
        breakpoint()

        raise ValueError(
            f"error parsing {FRAME_TRANSFORMS_KEY!r} part of config file: {e}"
        ) from e

    # format file names
    frame_transforms.file = frame_transforms.file.format(mission_name)
    for camera in camera_intrinsics:
        camera.file = camera.file.format(mission_name)

    return MetadataConfig(frame_transforms, camera_intrinsics)

def load_config(
    config_path: Path, mission_name: str
) -> Tuple[TopicRegistry, MetadataConfig]:
    
    # load config --> data and metadata shapes
    with open(config_path, "r") as f:
        config_object = yaml.safe_load(f)

    try:
        data_config_object = config_object[DATA_KEY]
        metadata_config_object = config_object[METADATA_KEY]
    except KeyError as e:

        raise ValueError(
            f"config {config_path!r} does not contain keys {DATA_KEY!r} or {METADATA_KEY!r}"
        ) from e

    return (
        _load_topic_registry_from_config(data_config_object, mission_name),
        _load_metadata_config(metadata_config_object, mission_name),
    )

