from __future__ import annotations

import shutil
from functools import partial
from pathlib import Path
from typing import Callable
from typing import Dict
from typing import Generator
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Union

import cv2
import numpy as np
import zarr
import zarr.storage
from mcap.reader import make_reader
from sensor_msgs.msg import CompressedImage
from tqdm import tqdm

from dataset_builder.attribute_types import IMAGE_TOPICS
from dataset_builder.attribute_types import ArrayType
from dataset_builder.attribute_types import AttributeTypes
from dataset_builder.attribute_types import get_attribute_types_for_topic
from dataset_builder.files import FILES
from dataset_builder.message_parsing import deserialize_message
from dataset_builder.message_parsing import load_image_from_message
from dataset_builder.message_parsing import parse_deserialized_message

DATA_PATH = Path(__file__).parent.parent / "data"
INPUT_PATH = DATA_PATH / "files"

MCAP_PATHS = [INPUT_PATH / p for p in FILES]

DATASET_PATH = DATA_PATH / "dataset"

BasicType = Union[np.ndarray, int, float, str, bool]


def get_file_topics(path: Path) -> List[str]:
    with open(path, "rb") as f:
        reader = make_reader(f)
        channels = reader.get_summary().channels  # type: ignore
        return [c.topic for c in channels.values()]  # type: ignore


def load_file_topic_dict(paths: Sequence[Path]) -> Dict[Path, List[str]]:
    return {path: get_file_topics(path) for path in paths}


TOPIC_DCT = load_file_topic_dict(MCAP_PATHS)


def arrayify_buffer(
    buffer: List[Dict[str, BasicType]], attr_tps: AttributeTypes
) -> Dict[str, np.ndarray]:
    assert len(buffer) > 0, "buffer must not be empty"

    data = {k: [] for k in attr_tps.keys()}
    for record in buffer:
        for key, value in record.items():
            data[key].append(value)
    array_data = {
        key: np.array(data[key], dtype=attr.dtype) for key, attr in attr_tps.items()
    }

    for key, attr in attr_tps.items():
        assert (
            array_data[key].shape[1:] == attr.shape
        ), f"{key}: {array_data[key].shape[1:]} != {attr.shape}"
    return array_data


SaveImagesCallback = Callable[[np.ndarray, int], None]


def data_chunks_from_mcap_topic(
    path: Path,
    topic: str,
    *,
    attribute_types: AttributeTypes,
    chunk_size: int,
    image_saver: Optional[SaveImagesCallback] = None,
) -> Generator[Dict[str, np.ndarray], None, None]:
    with open(path, "rb") as f:
        reader = make_reader(f)

        # get number of messages in topic for progress bar
        channels = reader.get_summary().channels  # type: ignore
        topic_id = {v.topic: k for k, v in channels.items()}[topic]  # type: ignore
        message_count: int = reader.get_summary().statistics.channel_message_counts[topic_id]  # type: ignore

        buffer = []

        for msg_idx, (schema, _, ser_message) in tqdm(
            enumerate(reader.iter_messages(topics=[topic])),
            total=message_count,
            leave=False,
        ):
            message = deserialize_message(schema, ser_message.data)

            if message is None:
                continue

            # handle data to store in zarr format
            buffer.append(parse_deserialized_message(message))

            if isinstance(message, CompressedImage) and image_saver is not None:
                image = load_image_from_message(message)
                assert image is not None
                image_saver(image, msg_idx)

            if len(buffer) == chunk_size:
                yield arrayify_buffer(buffer, attribute_types)
                buffer = []

        if buffer:
            yield arrayify_buffer(buffer, attribute_types)


def get_flat_topic(topic: str) -> str:
    """\
    flatten topic to avoid creating unnecessary folders,
    i.e., each topic should be one folder
    """
    return topic.strip("/").replace("/", "__")


def create_zarr_group_for_topic(zarr_root: Path, topic: str) -> zarr.Group:
    """\
    create zarr group for topic in the dataset, overwrite the group if it already exists
    don't overwrite the dataset
    """
    # create the zarr dataset root
    store = zarr.storage.DirectoryStore(zarr_root)
    root = zarr.group(store, overwrite=False)

    # overwrite existing sub group
    return root.create_group(get_flat_topic(topic), overwrite=True)


APPROX_CHUNK_SIZE = 256 * (2**20)  # 256 MB


def get_zarr_array_chunk_size(tp: ArrayType) -> Tuple[int, ...]:
    """\
    returns the chunk size to the closes power of two such that
    the uncompressed chunk size is less than 256 MB
    """
    n_elemts = np.prod(tp.shape)
    element_size = np.dtype(tp.dtype).itemsize

    slice_ratio = APPROX_CHUNK_SIZE / (n_elemts * element_size)
    slice_exponent = np.floor(np.log2(slice_ratio))

    n_slices = 2 ** max(slice_exponent, 0)
    return (n_slices,) + tp.shape


def create_zarr_arrays_for_topic(
    attributes: AttributeTypes, zarr_group: zarr.Group
) -> int:
    """\
    initializes zarr arrays for all attributes with appropriate
    shapes, dtypes, and chunk sizes
    """

    slice_sizes = []
    for key, attr_tp in attributes.items():
        chunk_size = get_zarr_array_chunk_size(attr_tp)
        slice_sizes.append(chunk_size[0])

        zarr_group.create_dataset(
            key,
            shape=(0,) + attr_tp.shape,
            dtype=attr_tp.dtype,
            overwrite=True,
            chunks=chunk_size,
        )

    return min(slice_sizes)


def create_jpeg_topic_folder(jpeg_root: Path, topic: str) -> Path:
    topic_folder = jpeg_root / get_flat_topic(topic)

    if topic_folder.exists():
        shutil.rmtree(topic_folder)

    topic_folder.mkdir(parents=True, exist_ok=True)
    return topic_folder


def save_image_to_topic_folder(
    topic_folder: Path, image: np.ndarray, sequence_id: int
) -> None:
    image_path = topic_folder / f"{sequence_id:06d}.jpeg"
    cv2.imwrite(str(image_path), image)


ZARR_PREFIX = "data"
JPEG_PREFIX = "images"


def generate_dataset_from_topic(
    dataset_root: Path, path: Path, topic: str, image_topic: bool
) -> None:
    """\
    generate dataset for specified topic inside specified file, we generally assume
    that topics are unique across files
    """

    attribute_types = get_attribute_types_for_topic(topic)
    if attribute_types is None:
        return

    # zarr file part of the dataset
    zarr_root_path = dataset_root / ZARR_PREFIX

    # images of the dataset
    jpeg_root_path = dataset_root / JPEG_PREFIX

    topic_zarr_group = create_zarr_group_for_topic(zarr_root_path, topic)
    chunk_size = create_zarr_arrays_for_topic(attribute_types, topic_zarr_group)

    # create image saving callback to save images to disk while parsing the remaining data
    if image_topic:
        topic_folder = create_jpeg_topic_folder(jpeg_root_path, topic)
        save_image_cb = partial(save_image_to_topic_folder, topic_folder)
    else:
        save_image_cb = None

    for chunk in data_chunks_from_mcap_topic(
        path,
        topic,
        attribute_types=attribute_types,
        chunk_size=chunk_size,
        image_saver=save_image_cb,
    ):
        for key, data in chunk.items():
            topic_zarr_group[key].append(data, axis=0)  # type: ignore


def generate_dataset() -> None:
    for file in tqdm(MCAP_PATHS):
        for topic in tqdm(TOPIC_DCT[file], leave=False):
            generate_dataset_from_topic(
                DATASET_PATH, file, topic, topic in IMAGE_TOPICS
            )
