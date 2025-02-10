from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
from typing import Mapping
from typing import Optional
from typing import Sequence
from typing import Tuple

import numpy as np
import numpy.typing


@dataclass
class ArrayType:
    shape: Tuple[int, ...]
    dtype: numpy.typing.DTypeLike


AttributeTypes = Mapping[str, ArrayType]


IMAGE_TOPIC_RESOLUTIONS = [
    ("/gt_box/zed2i/zed_node/depth_mono/depth_registered/compressed", (1080, 1920)),
    ("/gt_box/zed2i/zed_node/confidence_mono/confidence_map/compressed", (1080, 1920)),
    # ("/gt_box/zed2i/zed_node/confidence/confidence_map/compressed", (1080, 1920)), TODO: currently not working
    # ("/gt_box/zed2i/zed_node/depth/depth_registered/compressed", (1080, 1920)), TODO: currently not working
    (
        "/gt_box/alphasense_driver_node/cam1/color_corrected/image/compressed",
        (1080, 1440, 3),
    ),
    (
        "/gt_box/alphasense_driver_node/cam2/color_corrected/image/compressed",
        (1080, 1440),
    ),
    (
        "/gt_box/alphasense_driver_node/cam3/color_corrected/image/compressed",
        (1080, 1440),
    ),
    (
        "/gt_box/alphasense_driver_node/cam4/color_corrected/image/compressed",
        (1080, 1440, 3),
    ),
    (
        "/gt_box/alphasense_driver_node/cam5/color_corrected/image/compressed",
        (1080, 1440, 3),
    ),
    ("/gt_box/hdr_left/image_raw/compressed", (1280, 1920, 3)),
    ("/gt_box/hdr_front/image_raw/compressed", (1280, 1920, 3)),
    ("/gt_box/hdr_right/image_raw/compressed", (1280, 1920, 3)),
    ("/gt_box/zed2i/zed_node/left/image_rect_color/compressed", (1080, 1920, 3)),
    ("/gt_box/zed2i/zed_node/right/image_rect_color/compressed", (1080, 1920, 3)),
]

IMAGE_TOPICS = [topic for topic, _ in IMAGE_TOPIC_RESOLUTIONS]

LIDAR_TOPIC_ATTRIBUTES = [
    ("/gt_box/hesai/points", ["intensity", "ring", "timestamp"]),
    ("/gt_box/hesai/points_last", ["intensity", "ring", "timestamp"]),
    ("/gt_box/livox/lidar", ["intensity", "tag", "line", "timestamp"]),
    ("/dlio/deskewed_point_cloud", ["intensity", "timestamp"]),
]


POSE_TOPICS = [
    ("/dlio/lidar_odometry_as_posestamped", False),
    ("/gt_box/ground_truth/pose_stamped", False),
    ("/gt_box/inertial_explorer/tc/gt_poses_novatel", True),
    ("/gt_box/ground_truth/pose_with_covariance", True),
    ("/gt_box/inertial_explorer/lc/gt_poses_novatel", True),
    ("/gt_box/inertial_explorer/gt_poses_novatel", True),
    ("/gt_box/inertial_explorer/dgps/gt_poses_novatel", True),
    ("/gt_box/inertial_explorer/ppp/gt_poses_novatel", True),
]


IMU_TOPICS = [
    "/gt_box/livox/imu_si_compliant",
    "/gt_box/adis16475_node/imu",
    "/gt_box/livox/imu",
    "/gt_box/zed2i/zed_node/imu_raw/data",
    "/gt_box/zed2i/zed_node/imu/data",
    "/gt_box/cpt7/gps/imu",
    "/gt_box/ap20/imu",
    "/gt_box/cpt7/offline_from_novatel_logs/imu",
    "/gt_box/stim320/imu",
]


ILLUMINANCE_TOPICS = [
    "/gt_box/stim320/continuous_counter",
    "/gt_box/stim320/internal_counter",
]


ODOMETRY_TOPICS = [
    "/gt_box/inertial_explorer/tc/odometry",
    "/gt_box/inertial_explorer/odometry",
    "/gt_box/inertial_explorer/ppp/odometry",
    "/dlio/lidar_map_odometry",
    "/gt_box/inertial_explorer/lc/odometry",
    "/gt_box/inertial_explorer/dgps/odometry",
    "/gt_box/cpt7/odom",
    "/gt_box/zed2i/zed_node/odom",
    "/dlio/lidar_odometry",
]

NAV_SAT_FIX_TOPICS = [
    "/gt_box/cpt7/gps/fix/pos_cov_type",
]
MAGNET_FIELD_TOPICS = [
    "/gt_box/zed2i/zed_node/imu/mag/",
]


POINT_TOPICS = [
    "/gt_box/ap20/prism_position",
]


def get_image_attr_types(shape: Tuple[int, ...]) -> AttributeTypes:
    # return {"image": ArrayType(shape, np.uint8)}
    return {}


# PointCloud2
LIDAR_MAX_POINTS = 64_000


def get_point_cloud_attr_types(properties: Sequence[str]) -> AttributeTypes:
    POINT_CLOUD_PROPERTIES: Dict[str, ArrayType] = {
        "intensity": ArrayType((LIDAR_MAX_POINTS,), np.float32),
        "timestamp": ArrayType((LIDAR_MAX_POINTS,), np.uint64),
        "tag": ArrayType((LIDAR_MAX_POINTS,), np.uint8),
        "line": ArrayType((LIDAR_MAX_POINTS,), np.uint16),
        "ring": ArrayType((LIDAR_MAX_POINTS,), np.uint16),
    }

    column_types: Dict[str, ArrayType] = {
        "point_cloud_points": ArrayType((LIDAR_MAX_POINTS, 3), np.float32),
    }
    for prop in properties:
        column_types[f"point_cloud_{prop}"] = POINT_CLOUD_PROPERTIES[prop]
    return column_types


def get_pose_attr_types(covar: bool) -> AttributeTypes:
    column_types = {
        "pose_pos": ArrayType((3,), np.float64),
        "pose_orien": ArrayType((4,), np.float64),
    }

    if covar:
        column_types["pose_cov"] = ArrayType((6, 6), np.float64)
    return column_types


NAV_SAT_FIX_ATTR_TP = {
    "lat": ArrayType(tuple(), np.float64),
    "long": ArrayType(tuple(), np.float64),
    "alt": ArrayType(tuple(), np.float64),
    "cov": ArrayType((3, 3), np.float64),
    "cov_type": ArrayType(tuple(), np.uint8),
}


MAGNETIC_FIELD_ATTR_TP = {
    "b_field": ArrayType((3,), np.float64),
    "b_field_cov": ArrayType((3, 3), np.float64),
}


IMU_ATTR_TP = {
    "orien_cov": ArrayType((3, 3), np.float64),
    "ang_vel_cov": ArrayType((3, 3), np.float64),
    "lin_acc_cov": ArrayType((3, 3), np.float64),
    "orien": ArrayType((4,), np.float64),
    "ang_vel": ArrayType((3,), np.float64),
    "lin_acc": ArrayType((3,), np.float64),
}


ILLUMINANCE_ATTR_TP = {
    "illum": ArrayType(tuple(), np.float64),
    "illum_var": ArrayType(tuple(), np.float64),
}


ODOMETRY_ATTR_TP = {
    "pose_cov": ArrayType((6, 6), np.float64),
    "twist_cov": ArrayType((6, 6), np.float64),
    "pose_pos": ArrayType((3,), np.float64),
    "pose_orien": ArrayType((4,), np.float64),
    "twist_lin": ArrayType((3,), np.float64),
    "twist_ang": ArrayType((3,), np.float64),
}


POINT_ATTR_TP = {"point": ArrayType((3,), np.float64)}


def build_attribute_registry() -> Dict[str, AttributeTypes]:
    ret = {}
    ret.update(
        {topic: get_image_attr_types(shape) for topic, shape in IMAGE_TOPIC_RESOLUTIONS}
    )
    ret.update(
        {
            topic: get_point_cloud_attr_types(attrs)
            for topic, attrs in LIDAR_TOPIC_ATTRIBUTES
        }
    )
    ret.update({topic: get_pose_attr_types(covar) for topic, covar in POSE_TOPICS})

    # static attribute types
    ret.update({topic: NAV_SAT_FIX_ATTR_TP for topic in NAV_SAT_FIX_TOPICS})
    ret.update({topic: MAGNETIC_FIELD_ATTR_TP for topic in MAGNET_FIELD_TOPICS})
    ret.update({topic: IMU_ATTR_TP for topic in IMU_TOPICS})
    ret.update({topic: ILLUMINANCE_ATTR_TP for topic in ILLUMINANCE_TOPICS})
    ret.update({topic: ODOMETRY_ATTR_TP for topic in ODOMETRY_TOPICS})
    ret.update({topic: POINT_ATTR_TP for topic in POINT_TOPICS})
    return ret


REGISTRY = build_attribute_registry()

UNIVERSAL_ATTRIBUTES: Dict[str, ArrayType] = {
    "timestamp": ArrayType(tuple(), np.uint64),
    "sequence_id": ArrayType(tuple(), np.uint64),
}


def get_attribute_types_for_topic(topic: str) -> Optional[AttributeTypes]:
    if topic not in REGISTRY:
        return None
    return {**UNIVERSAL_ATTRIBUTES, **REGISTRY[topic]}
