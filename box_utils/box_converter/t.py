from __future__ import annotations

from typing import Any, Tuple, Sequence, Dict, List
from pathlib import Path
from dataset_builder.message_parsing import deserialize_message
from mcap.reader import make_reader
from tqdm import tqdm
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform
import numpy as np
from scipy.spatial.transform import Rotation


MISSION_PATH = Path(__file__).parent / "data" / "files"

FILE_PATH = list(MISSION_PATH.glob("*tf_static.mcap"))[0]


def get_from_to(tf: TransformStamped) -> tuple[str, str]:
    return tf.header.frame_id, tf.child_frame_id


def extract_transform(tf: Transform) -> Tuple[np.ndarray, Rotation]:
    position = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    quaternion = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
    return position, Rotation.from_quat(quaternion)


def homogeneous_transform(trans: np.ndarray, rot: Rotation) -> np.ndarray:
    ret = np.eye(4)
    ret[:3, :3] = rot.as_matrix()
    ret[:3, 3] = trans
    return ret


def to_semidirect_product(hom: np.ndarray) -> Tuple[np.ndarray, Rotation]:
    return hom[:3, 3], Rotation.from_matrix(hom[:3, :3])


def semi_direct_product(
    transforms: Sequence[Tuple[np.ndarray, Rotation]]
) -> Tuple[np.ndarray, Rotation]:
    ret = np.eye(4)
    for t, r in transforms:
        ret = ret @ homogeneous_transform(t, r)
    return to_semidirect_product(ret)


def transform_to_metadata(frame_id: str, trans: np.ndarray, rot: Rotation) -> dict:
    quat = rot.as_quat()
    return {
        "frame_id": frame_id,
        "translation": {"x": trans[0], "y": trans[1], "z": trans[2]},
        "rotation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]},
    }


def get_transform(
    base_frame: str,
    target_frame: str,
    reverse_edge_dict: dict[str, str],
    transforms: Dict[Tuple[str, str], Tuple[np.ndarray, Rotation]],
) -> Tuple[np.ndarray, Rotation]:
    trans_seq: List[Tuple[np.ndarray, Rotation]] = []
    while target_frame != base_frame:
        parent_frame = reverse_edge_dict[target_frame]
        trans_seq.append(transforms[(parent_frame, target_frame)])
        target_frame = parent_frame

    trans_seq.reverse()
    return semi_direct_product(trans_seq)


def test(path: Path, topic: str) -> Any:
    with open(path, "rb") as f:
        reader = make_reader(f)

        # get number of messages in topic for progress bar
        channels = reader.get_summary().channels  # type: ignore
        topic_id = {v.topic: k for k, v in channels.items()}[topic]  # type: ignore
        message_count: int = reader.get_summary().statistics.channel_message_counts[topic_id]  # type: ignore

        for msg_idx, (schema, _, ser_message) in tqdm(
            enumerate(reader.iter_messages(topics=[topic])),
            total=message_count,
            leave=False,
        ):

            assert schema is not None
            return deserialize_message(schema, ser_message.data)


BASE_FRAME = "base"

if __name__ == "__main__":
    msg: TFMessage = test(FILE_PATH, "/tf_static")

    transforms_dict = {
        get_from_to(tf): extract_transform(tf.transform) for tf in msg.transforms
    }

    all_frames = list(set(sum(transforms_dict.keys(), ())))
    reverse_edge_dict = {t: f for f, t in transforms_dict.keys()}

    abs_transforms = {
        frame: get_transform(BASE_FRAME, frame, reverse_edge_dict, transforms_dict)
        for frame in all_frames
    }

    transform_metadata = [
        transform_to_metadata(frame, trans, rot)
        for frame, (trans, rot) in abs_transforms.items()
    ]
