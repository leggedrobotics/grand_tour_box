from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Dict, List
from typing import Tuple
from typing import TypedDict
from typing import NamedTuple

from sensor_msgs.msg import CameraInfo
from mcap.reader import make_reader
from dataset_builder.message_parsing import (
    deserialize_message,
    extract_header_metadata_from_serialized_message,
    extract_camera_info_metadata_from_serialized_message,
)
from dataset_builder.dataset_config import (
    MetadataConfig,
    TopicRegistry,
    FrameTransformConfig,
    load_config,
    CameraInfoTopic,
    Topic,
)
from dataset_builder.transforms import get_metadata_from_tf_msg
from tf2_msgs.msg import TFMessage

import yaml

CAMERA_INTRISICS_FILENAME = "camera_intrinsics.yaml"
FRAME_TRANSFORMS_FILENAME = "frame_transforms.yaml"
FRAME_IDS_FILENAME = "frame_ids.yaml"


def load_metadata_from_mcap_file_and_topic(
    mcap_path: Path,
    topic_desc: Topic,
) -> Dict[str, Any]:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, _, ser_message in reader.iter_messages(topics=[topic_desc.topic]):
            assert schema is not None, f"schema is None for message {ser_message}"
            message = deserialize_message(schema, ser_message.data)
            return extract_header_metadata_from_serialized_message(message)
    raise ValueError(f"no messages found in topic {topic_desc.topic}")


def load_camera_info_metadata_from_mcap_file_and_topic(
    mcap_path: Path,
    topic_desc: CameraInfoTopic,
) -> Dict[str, Any]:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, _, ser_message in reader.iter_messages(topics=[topic_desc.topic]):
            assert schema is not None, f"schema is None for message {ser_message}"
            message = deserialize_message(schema, ser_message.data)
            assert isinstance(
                message, CameraInfo
            ), f"topic {topic_desc.topic} does not contain CameraInfo messages"

            return extract_camera_info_metadata_from_serialized_message(message)
    raise ValueError(f"no messages found in topic {topic_desc.topic}")


def load_tf_metadata_from_mcap_file_and_topic(
    mcap_path: Path,
    frame_transform_config: FrameTransformConfig,
) -> List[Dict[str, Any]]:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, _, ser_message in reader.iter_messages(
            topics=[frame_transform_config.topic]
        ):
            assert schema is not None, f"schema is None for message {ser_message}"
            message = deserialize_message(schema, ser_message.data)
            assert isinstance(
                message, TFMessage
            ), f"topic {frame_transform_config.topic} does not contain TFMessage messages"
            return get_metadata_from_tf_msg(message, frame_transform_config.base_frame)

    raise ValueError(f"no messages found in topic {frame_transform_config.topic}")


def get_frame_id_from_topic(mcaps_path: Path, topic_desc: Topic) -> Dict[str, str]:
    mcap_path = mcaps_path / topic_desc.file
    ret = load_metadata_from_mcap_file_and_topic(mcap_path, topic_desc)
    ret["alias"] = topic_desc.alias
    return ret


def get_camera_info_from_topic(
    mcaps_path: Path, topic_desc: CameraInfoTopic
) -> Dict[str, Any]:
    mcap_path = mcaps_path / topic_desc.file
    ret = load_camera_info_metadata_from_mcap_file_and_topic(mcap_path, topic_desc)
    ret["alias"] = topic_desc.alias
    return ret


def get_frame_ids(mcaps_path: Path, topic_reg: TopicRegistry) -> List[Dict[str, str]]:
    ret = []
    for _, topic_desc in topic_reg.values():
        ret.append(get_frame_id_from_topic(mcaps_path, topic_desc))
    return ret


def get_camera_infos(
    mcaps_path: Path, metadata_config: MetadataConfig
) -> List[Dict[str, Any]]:
    ret = []
    for cam_info_topic_desc in metadata_config.camera_intrinsics:
        ret.append(get_camera_info_from_topic(mcaps_path, cam_info_topic_desc))
    return ret


def get_frame_transform_metadata(
    mcaps_path: Path, frame_transform_config: FrameTransformConfig
) -> List[Dict[str, Any]]:
    mcap_path = mcaps_path / frame_transform_config.file
    return load_tf_metadata_from_mcap_file_and_topic(mcap_path, frame_transform_config)


def build_metadata(
    *,
    base_dataset_path: Path,
    mcaps_path: Path,
    metadata_config: MetadataConfig,
    topic_registry: TopicRegistry,
) -> None:
    frame_id_metadata = get_frame_ids(mcaps_path, topic_registry)
    with open(base_dataset_path / FRAME_IDS_FILENAME, "w") as f:
        yaml.dump(frame_id_metadata, f)

    cam_info_metadata = get_camera_infos(mcaps_path, metadata_config)
    with open(base_dataset_path / CAMERA_INTRISICS_FILENAME, "w") as f:
        yaml.dump(cam_info_metadata, f)

    transform_metadata = get_frame_transform_metadata(
        mcaps_path, metadata_config.frame_transforms
    )
    with open(base_dataset_path / FRAME_TRANSFORMS_FILENAME, "w") as f:
        yaml.dump(transform_metadata, f)
