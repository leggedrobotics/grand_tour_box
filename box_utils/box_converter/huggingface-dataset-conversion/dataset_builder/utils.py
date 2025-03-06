from __future__ import annotations

import io
from pathlib import Path
from typing import Any
from typing import Generator

import rosbag  # type: ignore
from roslib.message import get_message_class  # type: ignore
from tqdm import tqdm

# these messages types need to be parsed using roslib
# otherwise some other downstream stuff complains about the exact
# message type. This is super cursed but hey its ros so we dont complain
# since this has a performance overhead we only do it for the messages that really need it
NEEDS_ROSLIB = {
    "CameraInfo": "sensor_msgs/CameraInfo",
    "TFMessage": "tf2_msgs/TFMessage",
    "PointCloud2": "sensor_msgs/PointCloud2",
    "BatteryState": "anymal_msgs/BatteryState",
}


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
            for tp_patter, msg_tp_str in NEEDS_ROSLIB.items():
                if tp_patter in msg._type:
                    msg_class = get_message_class(msg_tp_str)
                    buffer = io.BytesIO()
                    msg.serialize(buffer)
                    yield msg_class().deserialize(buffer.getvalue())
                    break
            else:
                yield msg
