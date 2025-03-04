from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Generator

import rosbag
from tqdm import tqdm


def messages_in_bag_with_topic(
    bag_path: Path, topic: str, progress_bar: bool = True
) -> Generator[Any, None, None]:
    with rosbag.Bag(bag_path) as bag:
        total_message = bag.get_message_count(topic)
        for _, msg, _ in tqdm(
            bag.read_messages(topic), total=total_message, disable=not progress_bar
        ):
            yield msg
