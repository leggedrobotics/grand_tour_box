from __future__ import annotations

import logging
from pathlib import Path
from typing import Any
from typing import Dict

import yaml
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage

from dataset_builder.dataset_config import CameraInfoTopic
from dataset_builder.dataset_config import FrameTransformConfig
from dataset_builder.dataset_config import MetadataConfig
from dataset_builder.dataset_config import Topic
from dataset_builder.dataset_config import TopicRegistry
from dataset_builder.message_parsing import (
    extract_camera_info_metadata_from_deserialized_message,
)
from dataset_builder.message_parsing import (
    extract_header_metadata_from_deserialized_message,
)
from dataset_builder.transforms import get_metadata_from_tf_msg
from dataset_builder.utils import messages_in_bag_with_topic

logger = logging.getLogger(__name__)

CAMERA_INTRISICS_FILENAME = "camera_intrinsics.yaml"
FRAME_TRANSFORMS_FILENAME = "frame_transforms.yaml"
FRAME_IDS_FILENAME = "frame_ids.yaml"


def _load_metadata_from_bag_file_and_topic(
    bag_path: Path,
    topic_desc: Topic,
) -> Dict[str, Any]:
    for message in messages_in_bag_with_topic(
        bag_path, topic_desc.topic, progress_bar=False
    ):
        return extract_header_metadata_from_deserialized_message(message)
    raise ValueError(f"no messages found in topic {topic_desc.topic}")


def _load_camera_info_metadata_from_bag_file_and_topic(
    bag_path: Path,
    topic_desc: CameraInfoTopic,
) -> Dict[str, Any]:
    for message in messages_in_bag_with_topic(
        bag_path, topic_desc.topic, progress_bar=False
    ):
        assert isinstance(
            message, CameraInfo
        ), f"topic {topic_desc.topic} does not contain CameraInfo messages"

        return extract_camera_info_metadata_from_deserialized_message(message)
    raise ValueError(f"no messages found in topic {topic_desc.topic}")


def _load_tf_metadata_from_bag_file_and_topic(
    bag_path: Path,
    frame_transform_config: FrameTransformConfig,
) -> Dict[str, Any]:
    for message in messages_in_bag_with_topic(
        bag_path, frame_transform_config.topic, progress_bar=False
    ):
        assert isinstance(
            message, TFMessage
        ), f"topic {frame_transform_config.topic} does not contain TFMessage messages"
        return get_metadata_from_tf_msg(message, frame_transform_config.base_frame)

    raise ValueError(f"no messages found in topic {frame_transform_config.topic}")


def _get_frame_id_from_topic(bags_path: Path, topic_desc: Topic) -> Dict[str, str]:
    bag_path = bags_path / topic_desc.file
    ret = _load_metadata_from_bag_file_and_topic(bag_path, topic_desc)
    return ret


def _get_camera_info_from_topic(
    bags_path: Path, topic_desc: CameraInfoTopic
) -> Dict[str, Any]:
    bag_path = bags_path / topic_desc.file
    ret = _load_camera_info_metadata_from_bag_file_and_topic(bag_path, topic_desc)
    return ret


def _get_frame_ids(bags_path: Path, topic_reg: TopicRegistry) -> Dict[str, Any]:
    ret = {}
    for _, topic_desc in topic_reg.values():
        ret[topic_desc.alias] = _get_frame_id_from_topic(bags_path, topic_desc)
    return ret


def _get_camera_infos(
    bags_path: Path, metadata_config: MetadataConfig
) -> Dict[str, Any]:
    ret = {}
    for cam_info_topic_desc in metadata_config.camera_intrinsics:
        cam_info = _get_camera_info_from_topic(bags_path, cam_info_topic_desc)
        ret[cam_info_topic_desc.alias] = cam_info
    return ret


def _get_frame_transform_metadata(
    bags_path: Path, frame_transform_config: FrameTransformConfig
) -> Dict[str, Any]:
    bag_path = bags_path / frame_transform_config.file
    return _load_tf_metadata_from_bag_file_and_topic(bag_path, frame_transform_config)


def _write_metadata_files(metadata_dct: Dict[str, Any], metadata_dir: Path) -> None:
    for alias, topic_metadata in metadata_dct.items():
        with open(metadata_dir / f"{alias}.yaml", "w") as f:
            yaml.dump(topic_metadata, f)


def build_metadata_part(
    *,
    base_dataset_path: Path,
    bags_path: Path,
    metadata_config: MetadataConfig,
    topic_registry: TopicRegistry,
) -> None:
    frame_id_metadata = _get_frame_ids(bags_path, topic_registry)
    cam_info_metadata = _get_camera_infos(bags_path, metadata_config)
    transform_metadata = _get_frame_transform_metadata(
        bags_path, metadata_config.frame_transforms
    )

    # merge metadata and split by topic
    metadata = {}
    for alias, frame_id in frame_id_metadata.items():
        metadata[alias] = {**frame_id, "topic": alias}
    for alias, cam_info in cam_info_metadata.items():
        assert (
            alias not in metadata
        ), f"alias {alias} for camera_info topic is not unique"
        metadata[alias] = {"camera_info": cam_info}

    # try to get transform for each frame_id
    for alias, alias_metadata in metadata.items():
        frame_id = alias_metadata.get("frame_id")
        if frame_id is None:
            continue

        transform = transform_metadata.get(frame_id)
        if transform is None:
            logger.warning(f"no transform found for frame_id {frame_id!r}")
        else:
            alias_metadata.update({"transform": transform})

    metadata_dir = base_dataset_path / "metadata"
    metadata_dir.mkdir(parents=False, exist_ok=True)

    _write_metadata_files(metadata, metadata_dir)
