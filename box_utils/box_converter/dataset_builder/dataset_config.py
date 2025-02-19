from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict
from typing import List
from typing import Mapping
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np
import numpy.typing
import yaml

IMAGE_TOPICS_KEY = "image_topics"
LIDAR_TOPICS_KEY = "lidar_topics"
POSE_TOPICS_KEY = "pose_topics"
IMU_TOPICS_KEY = "imu_topics"
ODOMETRY_TOPICS_KEY = "odometry_topics"
NAV_SAT_FIX_TOPICS_KEY = "nav_sat_fix_topics"
MAGNETIC_FIELD_TOPICS_KEY = "magnetic_field_topics"
POINT_TOPICS_KEY = "point_topics"
SINGLETON_TRANSFORM_TOPICS_KEY = "singleton_transform_topics"


ALLOWED_KEYS = [
    IMAGE_TOPICS_KEY,
    LIDAR_TOPICS_KEY,
    POSE_TOPICS_KEY,
    IMU_TOPICS_KEY,
    ODOMETRY_TOPICS_KEY,
    NAV_SAT_FIX_TOPICS_KEY,
    MAGNETIC_FIELD_TOPICS_KEY,
    POINT_TOPICS_KEY,
    SINGLETON_TRANSFORM_TOPICS_KEY,
]


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
    format: str


@dataclass
class LidarTopic(Topic):
    attributes: List[str]
    max_points: int


@dataclass
class PoseTopic(Topic):
    covariance: bool


@dataclass
class ImuTopic(Topic): ...


@dataclass
class OdometryTopic(Topic): ...


@dataclass
class NavSatFixTopic(Topic): ...


@dataclass
class MagneticFieldTopic(Topic): ...


@dataclass
class PointTopic(Topic): ...


@dataclass
class SingletonTransformTopic(Topic): ...


AttributeTypes = Dict[str, ArrayType]
TopicRegistry = Dict[str, Tuple[AttributeTypes, Topic]]


def build_point_cloud_attr_types(
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


def build_pose_attr_types(covar: bool) -> AttributeTypes:
    column_types = {
        "pose_pos": ArrayType((3,), np.float64),
        "pose_orien": ArrayType((4,), np.float64),
    }

    if covar:
        column_types["pose_cov"] = ArrayType((6, 6), np.float64)
    return column_types


def extract_image_topic_attributes(
    image_topics: Sequence[ImageTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in image_topics:
        ret[topic.alias] = ({}, topic)
    return ret


def extract_lidar_topic_attributes(
    lidar_topics: Sequence[LidarTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in lidar_topics:
        ret[topic.alias] = (
            build_point_cloud_attr_types(topic.attributes, topic.max_points),
            topic,
        )
    return ret


def extract_pose_topic_attributes(
    pose_topics: Sequence[PoseTopic],
) -> TopicRegistry:
    ret: TopicRegistry = {}
    for topic in pose_topics:
        ret[topic.alias] = (build_pose_attr_types(topic.covariance), topic)
    return ret


def extract_imu_topic_attributes(
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


def extract_odometry_topic_attributes(
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


def extract_nav_sat_fix_topic_attributes(
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


def extract_magnetic_field_topic_attributes(
    magnetic_field_topics: Sequence[MagneticFieldTopic],
) -> TopicRegistry:

    topic_tp = {
        "b_field": ArrayType((3,), np.float64),
        "b_field_cov": ArrayType((3, 3), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in magnetic_field_topics}


def extract_point_topic_attributes(
    point_topics: Sequence[PointTopic],
) -> TopicRegistry:
    topic_tp = {
        "point": ArrayType((3,), np.float64),
    }

    return {topic.alias: (topic_tp.copy(), topic) for topic in point_topics}


def extract_singleton_transform_topic_attributes(
    singleton_transform_topics: Sequence[SingletonTransformTopic],
) -> TopicRegistry:
    topic_tp = {
        "translation": ArrayType((3,), np.float64),
        "rotation": ArrayType((4,), np.float64),
    }

    return {
        topic.alias: (topic_tp.copy(), topic) for topic in singleton_transform_topics
    }


UNIVERSAL_ATTRIBUTES: Dict[str, ArrayType] = {
    "timestamp": ArrayType(tuple(), np.uint64),
    "sequence_id": ArrayType(tuple(), np.uint64),
}


def add_universal_attributes(
    registry: TopicRegistry,
) -> TopicRegistry:
    return {
        topic: ({**UNIVERSAL_ATTRIBUTES, **attributes}, topic_obj)
        for topic, (attributes, topic_obj) in registry.items()
    }


def load_topic_registry_from_config(
    config: Path,
) -> TopicRegistry:
    registry = {}

    with open(config, "r") as f:
        config_object = yaml.safe_load(f)

    assert isinstance(config_object, Mapping), "config file must be a mapping"
    for key in config_object.keys():
        assert key in ALLOWED_KEYS, f"unknown key in config: {key}"

    try:
        # image topics
        image_topics = [
            ImageTopic(**topic_obj)
            for topic_obj in config_object.get(IMAGE_TOPICS_KEY, [])
        ]
        registry.update(extract_image_topic_attributes(image_topics))

        # lidar topics
        lidar_topics = [
            LidarTopic(**topic_obj)
            for topic_obj in config_object.get(LIDAR_TOPICS_KEY, [])
        ]
        registry.update(extract_lidar_topic_attributes(lidar_topics))

        # pose topics
        pose_topics = [
            PoseTopic(**topic_obj)
            for topic_obj in config_object.get(POSE_TOPICS_KEY, [])
        ]
        registry.update(extract_pose_topic_attributes(pose_topics))

        # imu topics
        imu_topics = [
            ImuTopic(**topic_obj) for topic_obj in config_object.get(IMU_TOPICS_KEY, [])
        ]
        registry.update(extract_imu_topic_attributes(imu_topics))

        # odometry topics
        odometry_topics = [
            OdometryTopic(**topic_obj)
            for topic_obj in config_object.get(ODOMETRY_TOPICS_KEY, [])
        ]
        registry.update(extract_odometry_topic_attributes(odometry_topics))

        # nav_sat_fix topics
        nav_sat_fix_topics = [
            NavSatFixTopic(**topic_obj)
            for topic_obj in config_object.get(NAV_SAT_FIX_TOPICS_KEY, [])
        ]
        registry.update(extract_nav_sat_fix_topic_attributes(nav_sat_fix_topics))

        # magnetic_field topics
        magnetic_field_topics = [
            MagneticFieldTopic(**topic_obj)
            for topic_obj in config_object.get(MAGNETIC_FIELD_TOPICS_KEY, [])
        ]
        registry.update(extract_magnetic_field_topic_attributes(magnetic_field_topics))

        # point topics
        point_topics = [
            PointTopic(**topic_obj)
            for topic_obj in config_object.get(POINT_TOPICS_KEY, [])
        ]
        registry.update(extract_point_topic_attributes(point_topics))

        # singleton_transform topics
        singleton_transform_topics = [
            SingletonTransformTopic(**topic_obj)
            for topic_obj in config_object.get(SINGLETON_TRANSFORM_TOPICS_KEY, [])
        ]
        registry.update(
            extract_singleton_transform_topic_attributes(singleton_transform_topics)
        )
    except Exception as e:
        raise ValueError(f"error parsing config file: {e}") from e
    return add_universal_attributes(registry)


if __name__ == "__main__":
    print("hello")

    config = Path(__file__).parent / "configs" / "config.yaml"
    reg = load_attributes_registry_from_config(config)
