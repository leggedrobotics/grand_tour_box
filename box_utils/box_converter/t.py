from __future__ import annotations

from pathlib import Path
from dataset_builder.message_parsing import deserialize_message
from mcap.reader import make_reader
from dataset_builder.builder import MCAP_PATHS, load_file_topic_dict
from tqdm import tqdm


FILE_TOPICS = load_file_topic_dict(MCAP_PATHS)

my_files = [p for p in FILE_TOPICS if "cpt7_gps" in p.name]


def foo(path: Path, topic: str) -> None:
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
            message = deserialize_message(schema, ser_message.data)

            breakpoint()


topic = FILE_TOPICS[my_files[0]][2]

foo(my_files[0], topic)
