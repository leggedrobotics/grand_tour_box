from __future__ import annotations

from typing import Any
from typing import Tuple
from typing import Sequence
from typing import Dict
from typing import List, NewType
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform
import numpy as np
from scipy.spatial.transform import Rotation

Translation = NewType("Translation", np.ndarray)
HomTransform = NewType("HomTransform", np.ndarray)


def _get_source_dest_frame_ids(tf: TransformStamped) -> tuple[str, str]:
    return tf.header.frame_id, tf.child_frame_id


def _extract_transform_from_transform_msg(
    tf: Transform,
) -> Tuple[Translation, Rotation]:
    position = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    quaternion = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
    return Translation(position), Rotation.from_quat(quaternion)


def _get_source_dest_transforms_from_tf_msg(
    tf: TFMessage,
) -> Dict[Tuple[str, str], Tuple[Translation, Rotation]]:
    return {
        _get_source_dest_frame_ids(tf): _extract_transform_from_transform_msg(
            tf.transform
        )
        for tf in tf.transforms
    }


def _convert_to_homogeneous_transform(
    trans: Translation, rot: Rotation
) -> HomTransform:
    ret = np.eye(4)
    ret[:3, :3] = rot.as_matrix()
    ret[:3, 3] = trans
    return HomTransform(ret)


def _convert_from_homogeneous_transform(
    hom: HomTransform,
) -> Tuple[Translation, Rotation]:
    return Translation(hom[:3, 3]), Rotation.from_matrix(hom[:3, :3])


def _semi_direct_product(
    transforms: Sequence[Tuple[Translation, Rotation]]
) -> Tuple[Translation, Rotation]:
    ret = HomTransform(np.eye(4))
    for t, r in transforms:
        ret = HomTransform(ret @ _convert_to_homogeneous_transform(t, r))
    return _convert_from_homogeneous_transform(ret)


def _absolute_transform_to_metadata(
    frame_id: str, trans: Translation, rot: Rotation, base_frame: str
) -> Dict[str, Any]:
    quat = rot.as_quat()

    translation = {"x": trans[0], "y": trans[1], "z": trans[2]}
    rotation = {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}

    return {
        "base_frame_id": base_frame,
        "frame_id": frame_id,
        "translation": {k: float(v) for k, v in translation.items()},
        "rotation": {k: float(v) for k, v in rotation.items()},
    }


def _get_absolute_transform(
    base_frame: str,
    target_frame: str,
    reverse_edge_dict: dict[str, str],
    transforms: Dict[Tuple[str, str], Tuple[Translation, Rotation]],
) -> Tuple[Translation, Rotation]:

    trans_seq: List[Tuple[Translation, Rotation]] = []
    while target_frame != base_frame:
        if target_frame not in reverse_edge_dict:
            raise ValueError("transforms need to be specified as a (rooted) tree")

        parent_frame = reverse_edge_dict[target_frame]
        trans_seq.append(transforms[(parent_frame, target_frame)])
        target_frame = parent_frame

    trans_seq.reverse()
    return _semi_direct_product(trans_seq)


def _get_metadata_from_transforms(
    base_frame: str,
    transforms: Dict[Tuple[str, str], Tuple[Translation, Rotation]],
) -> Dict[str, Any]:
    reverse_edge_dict = {child: parent for parent, child in transforms.keys()}
    traget_frames = set(child for _, child in transforms.keys())

    assert base_frame not in traget_frames

    # build transforms base_frame -> target_frame
    absolute_transforms = {
        target_frame: _get_absolute_transform(
            base_frame, target_frame, reverse_edge_dict, transforms
        )
        for target_frame in traget_frames
    }

    # convert the transforms to yaml-serializable metadata
    return {
        target_frame: _absolute_transform_to_metadata(
            target_frame, translation, rotation, base_frame
        )
        for target_frame, (translation, rotation) in absolute_transforms.items()
    }


def get_metadata_from_tf_msg(msg: TFMessage, base_frame: str) -> Dict[str, Any]:
    transforms = _get_source_dest_transforms_from_tf_msg(msg)
    return _get_metadata_from_transforms(base_frame, transforms)
