from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Generator

import rosbag  # type: ignore
from roslib.message import get_message_class  # type: ignore
from tqdm import tqdm
import io

# these messages types need to be parsed using roslib
# otherwise some other downstream stuff complains about the exact
# message type. This is super cursed but hey its ros so we dont complain
# since this has a performance overhead we only do it for the messages that really need it
NEEDS_ROSLIB = [
    "CameraInfo",
    "TFMessage",
    "PointCloud2",
]


def messages_in_bag_with_topic(
    bag_path: Path, topic: str, progress_bar: bool = True
) -> Generator[Any, None, None]:
    with rosbag.Bag(bag_path) as bag:
        total_message = bag.get_message_count(topic)
        for _, msg, _ in tqdm(
            bag.read_messages(topic),
            total=total_message,
            disable=not progress_bar,
            leave=False,
        ):
            if any(tp in msg._type for tp in NEEDS_ROSLIB):
                msg_class = get_message_class(msg._type)
                buffer = io.BytesIO()
                msg.serialize(buffer)
                yield msg_class().deserialize(buffer.getvalue())
            else:
                yield msg
