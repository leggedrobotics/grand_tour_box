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
    """
    Load metadata from a bag file and a topic description
    """
    for message in messages_in_bag_with_topic(
        bag_path, topic_desc.topic, progress_bar=False
    ):
        # i.e. {'frame_id': 'zed2i_left_camera_optical_frame'}
        return extract_header_metadata_from_deserialized_message(message, topic_desc)
    
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

        return extract_camera_info_metadata_from_deserialized_message(
            message, topic_desc
        )
    raise ValueError(f"no messages found in topic {topic_desc.topic}")


def _load_tf_metadata_from_bag_file_and_topic(
    bag_path: Path,
    frame_transform_config: FrameTransformConfig,   
) -> Dict[str, Any]:
    """
    Load metadata from a bag file and a topic description

    Args:
        bag_path (Path):                                Path to the bag file.
        frame_transform_config (FrameTransformConfig):  Frame transform configuration.
                                                         i.e. { 'alphasense_base': {'base_frame_id': 'base', 
                                                                                'frame_id': 'alphasense_base', 
                                                                                'translation': {...}, ...}, 
                                                                'box_base': {'base_frame_id': 'base', 
                                                                        'frame_id': 'box_base', 
                                                                        'translation': {...}, ...}}, 
                                                                'alphasense_front_left': {'base_frame_id': 'base', 
                                                                                        'frame_id': 'alphasense_front_left', 
                                                                                        'translation': {...}, 
                                                                                        'rotation': {...}, ...},
                                                                                        ...}
    Raises:
        ValueError: Raised if message is not of type TFMessage (transform message)

    Returns:
        Dict[str, Any]: Conversion beween frames i.e.
                        {'cam2_sensor_frame': {'base_frame_id': 'base', 
                                               'frame_id': 'cam2_sensor_frame', 
                                               'translation': {'x': ..., 'y': ..., 'z': ...}, 
                                               'rotation': {...}},...}

    """

    for message in messages_in_bag_with_topic(
        bag_path, frame_transform_config.topic, progress_bar=False
    ):
        # frame_transform_config.topic      i.e /tf_static
        # frame_transform_config.base_frame i.e. base
        assert isinstance(
            message, TFMessage
        ), f"topic {frame_transform_config.topic} does not contain TFMessage messages"
        metadata = get_metadata_from_tf_msg(message, frame_transform_config.base_frame)
        
        return metadata

    raise ValueError(f"no messages of type tf2_msgs.msg found in topic {frame_transform_config.topic}")


def _get_frame_id_from_topic(bags_path: Path, topic_desc: Topic) -> Dict[str, str]:
    bag_path = bags_path / topic_desc.file

    return _load_metadata_from_bag_file_and_topic(bag_path, topic_desc)
 

def _get_camera_info_from_topic(
    bags_path: Path, topic_desc: CameraInfoTopic
) -> Dict[str, Any]:
    """
    Get camera info metadata from a bag file 
    (deserialized msg) and a topic

    Args:
        bags_path (Path):               Path to all ROS bag files.
        topic_desc (CameraInfoTopic):   Camera info topic description. 
                                        i.e. {'topic': '/zed2i/zed_node/rgb/camera_info',

    Returns:
        Dict[str, Any]:  Metadata from Msg: i.e. {'frame_id': 'zed2i_left_camera_optical_frame', 
                                                  'distortion_model': 'plumb_bob', 
                                                  'width': 1920, 
                                                  'height': 1080, 
                                                  'D': [...], 
                                                  'K': [...], 'R': [...], .
                                                  ...}}
    """

    bag_path = bags_path / topic_desc.file
    ret = _load_camera_info_metadata_from_bag_file_and_topic(bag_path, topic_desc)
    return ret


def _get_frame_ids(bags_path: Path, topic_reg: TopicRegistry) -> Dict[str, Any]:
    """
    Get frame ids from all topics in
    the topic registry

    Args:
        bags_path (Path):               Path to all ROS bag files.
        topic_reg (TopicRegistry):      Topic registry. i.e. {
                                            'zed2i_depth':({'timestamp': ArrayType(shape=(), dtype=<class 'numpy.uint64'>), 
                                            'sequence_id': ArrayType(shape=(), dtype=<class 'numpy.uint64'>)}

    Returns:
        Dict[str, Any]: i.e. {'zed2i_depth': {'frame_id': 'zed2i_left_camera_optical_frame'}, 
                              'cpt7_odometry': {'frame_id': 'cpt7_odom'}, ...}
    """

    ret = {}
    try:
        for _, topic_desc in topic_reg.values():
            ret[topic_desc.alias] = _get_frame_id_from_topic(bags_path, topic_desc)
        return ret
    except ValueError as e:
        # TODO: remove this
        breakpoint()


def _get_camera_infos(
    bags_path: Path, metadata_config: MetadataConfig
) -> Dict[str, Any]:
    """
    Get camera info metadata (parameters etc.) 
    from all camera_info topics

    Args:
        bags_path (Path):                   Path to all ROS bag files.
        metadata_config (MetadataConfig):   Metadata configuration. 
                                            i.e. MetadataConfig(frame_transforms=FrameTransformConfig( topic='/tf_static', 
                                                                                                       file='2024-11-11-12-42-47_tf_static.bag', 
                                                                                                       base_frame='base'), 
                                                                camera_intrinsics=[CameraInfoTopic( alias='zed2i_depth_caminfo', 
                                                                                                    topic='/gt_box/zed2i/zed_node/depth/camera_info', 
                                                                                                    file='2024-11-11-12-42-47_jetson_zed2i_depth.bag'), 
                                                                                    CameraInfoTopic( alias='zed2i_left_caminfo', 
                                                                                                     topic='/gt_box/zed2i/zed_node/left/camera_info', 
                                                                                                     file='2024-11-11-12-42-47_jetson_zed2i_images.bag')

    Returns:
        Dict[str, Any]:     i.e. {'zed2i_depth_caminfo': 
                                    {'frame_id': 'zed2i_left_camera_optical_frame', 
                                     'distortion_model': 'plumb_bob', 
                                    ...}}
    """
    try:
        ret = {}
        for cam_info_topic_desc in metadata_config.camera_intrinsics:
            cam_info = _get_camera_info_from_topic(bags_path, cam_info_topic_desc)
            ret[cam_info_topic_desc.alias] = cam_info
        return ret

    except ValueError as e:
        # TODO: remove this
        breakpoint()


def _get_frame_transform_metadata(
    bags_path: Path, frame_transform_config: FrameTransformConfig
) -> Dict[str, Any]:
    """
    Reads all the frame transform metadata from the bag files
    and the frame transform configuration

    Args:
        bags_path (Path):       Path to all ROS bag files.
        frame_transform_config (FrameTransformConfig): Frame transform configuration.

    Returns:
        Dict[str, Any]: Dictionary with frame transform metadata. i.e.
                            {'stim320_imu': {'base_frame_id': 'base', 
                                            'frame_id': 'stim320_imu', 
                                            'translation': {'x': 0.0, y': 0.0, 'z': 0.0}, 
                                            'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0,}}
    """
    bag_path = bags_path / frame_transform_config.file
    tf_metadata = _load_tf_metadata_from_bag_file_and_topic(bag_path, frame_transform_config)
    return tf_metadata
    

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

    # add frame metadata to dict and split by topic
    metadata = {}
    for alias, frame_id in frame_id_metadata.items():
        metadata[alias] = {**frame_id, "topic": alias}
        # i.e. {'zed2i_depth': {'frame_id': 'zed2i_left_camera_optical_frame', 
        #                       'topic': 'zed2i_depth'}}
      
    # add camera specific metadata to dict and split by topic
    for alias, cam_info in cam_info_metadata.items():
        assert (
            alias not in metadata
        ), f"alias {alias} for camera_info topic is not unique"
        metadata[alias] = {"camera_info": cam_info}
        # add the camera metadata (i.e. camera intrinsics) with alias i.e. zed2i_left_caminfo

    # try to get transform for each frame_id and add it to the metadata
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

    # save metadata to yaml files per alias
    _write_metadata_files(metadata, metadata_dir)