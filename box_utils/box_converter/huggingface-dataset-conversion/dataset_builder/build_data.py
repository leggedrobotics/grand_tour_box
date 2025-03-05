from __future__ import annotations

import os
import shutil
import tarfile
from functools import partial
from pathlib import Path
from typing import Callable
from typing import Dict
from typing import Generator
from typing import List
from typing import Optional
from typing import Tuple
from typing import Union
from typing import cast

import numpy as np
import zarr
import zarr.storage
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from tqdm import tqdm
import cv2

from dataset_builder.dataset_config import ArrayType
from dataset_builder.dataset_config import AttributeTypes
from dataset_builder.dataset_config import ImageTopic
from dataset_builder.dataset_config import Topic
from dataset_builder.dataset_config import TopicRegistry
from dataset_builder.message_parsing import parse_deserialized_message
from dataset_builder.utils import messages_in_bag_with_topic

DATA_PREFIX = "data"
IMAGE_PREFIX = "images"


BasicType = Union[np.ndarray, int, float, str, bool]
ImageExtractorCallback = Callable[
    [Union[CompressedImage, Image], int, ImageTopic], None
]


CV_BRIDGE = CvBridge()


def _extract_and_save_image_from_message(
    msg: Union[CompressedImage, Image],
    image_index: int,
    topic_desc: ImageTopic,
    /,
    *,
    image_dir: Path,
    cv_bridge: CvBridge,
) -> None:
    if topic_desc.compressed:
        image = cv_bridge.compressed_imgmsg_to_cv2(msg)
    else:
        image = cv_bridge.imgmsg_to_cv2(msg)
    file_path = image_dir / f"{image_index:06d}.{topic_desc.format}"
    cv2.imwrite(str(file_path), image)


def _np_arrays_from_buffered_messages(
    buffer: List[Dict[str, BasicType]], attr_tps: AttributeTypes
) -> Dict[str, np.ndarray]:
    """\
    converts a sequence of buffered messages into a dict of numpy arrays
    that can be stored in zarr format
    """

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


def _data_chunks_from_bag_topic(
    path: Path,
    *,
    topic_desc: Topic,
    attribute_types: AttributeTypes,
    chunk_size: int,
    image_extractor: Optional[ImageExtractorCallback],
) -> Generator[Dict[str, np.ndarray], None, None]:
    """\
    read messages from bag file and topic yielding chunks of data that can
    be stored in zarr format. For optimal performance the `chunk_size` should be
    similar to the zarr chunk size.
    """

    if image_extractor is None and isinstance(topic_desc, ImageTopic):
        raise ValueError("image_extractor must be provided for image topics")
    if image_extractor is not None and not isinstance(topic_desc, ImageTopic):
        raise ValueError("image_extractor must not be provided for non-image topics")

    buffer = []
    for idx, message in enumerate(
        messages_in_bag_with_topic(path, topic=topic_desc.topic)
    ):
        # handle data to store in zarr format
        buffer.append(parse_deserialized_message(message, topic_desc=topic_desc))

        if image_extractor is not None:
            image_extractor(message, idx, cast(ImageTopic, topic_desc))
        if len(buffer) == chunk_size:
            yield _np_arrays_from_buffered_messages(buffer, attribute_types)
            buffer = []

    if buffer:
        yield _np_arrays_from_buffered_messages(buffer, attribute_types)


def _create_zarr_group_for_topic(zarr_root: Path, topic_alias: str) -> zarr.Group:
    """\
    create zarr group for topic in the dataset, overwrite the group if it already exists
    don't overwrite the dataset
    """
    # create the zarr dataset root
    store = zarr.storage.DirectoryStore(zarr_root)
    root = zarr.group(store, overwrite=False)

    # overwrite existing sub group
    return root.create_group(topic_alias, overwrite=True)


APPROX_CHUNK_SIZE = 256 * (2**20)  # 256 MB


def _compute_zarr_array_chunk_size(tp: ArrayType) -> Tuple[int, ...]:
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


def _create_zarr_arrays_for_topic(
    attributes: AttributeTypes, zarr_group: zarr.Group
) -> int:
    """\
    initializes zarr arrays for all attributes with appropriate
    shapes, dtypes, and chunk sizes
    """

    slice_sizes = []
    for key, attr_tp in attributes.items():
        chunk_size = _compute_zarr_array_chunk_size(attr_tp)
        slice_sizes.append(chunk_size[0])

        zarr_group.create_dataset(
            key,
            shape=(0,) + attr_tp.shape,
            dtype=attr_tp.dtype,
            overwrite=True,
            chunks=chunk_size,
        )

    return min(slice_sizes)


def _create_jpeg_topic_folder(jpeg_root: Path, topic_alias: str) -> Path:
    """\
    create folder for images of a specified topic
    """
    topic_folder = jpeg_root / topic_alias

    if topic_folder.exists():
        shutil.rmtree(topic_folder)

    topic_folder.mkdir(parents=True, exist_ok=True)
    return topic_folder


def _generate_dataset_from_topic_description_and_attribute_types(
    dataset_root: Path,
    path: Path,
    *,
    topic_desc: Topic,
    attribute_types: AttributeTypes,
) -> None:
    """\
    generate dataset for specified topic inside specified file, we generally assume
    that topics are unique across files
    """
    # zarr file part of the dataset
    zarr_root_path = dataset_root / DATA_PREFIX

    # images of the dataset
    jpeg_root_path = dataset_root / IMAGE_PREFIX

    topic_zarr_group = _create_zarr_group_for_topic(zarr_root_path, topic_desc.alias)
    chunk_size = _create_zarr_arrays_for_topic(attribute_types, topic_zarr_group)

    # create image saving callback to save images to disk while parsing the remaining data
    image_extractor = None
    if isinstance(topic_desc, ImageTopic):
        topic_folder = _create_jpeg_topic_folder(jpeg_root_path, topic_desc.alias)
        image_extractor = partial(
            _extract_and_save_image_from_message,
            image_dir=topic_folder,
        )

    for chunk in _data_chunks_from_bag_topic(
        path,
        topic_desc=topic_desc,
        attribute_types=attribute_types,
        chunk_size=chunk_size,
        image_extractor=image_extractor,
    ):
        for key, data in chunk.items():
            topic_zarr_group[key].append(data, axis=0)  # type: ignore


def _tar_ball_dataset(base_dataset_path: Path) -> None:
    """\
    tarball topic folders of the dataset, we make sure to only tar folders and not files
    also we dont add any compression - for performance and also because its not needed
    """
    data_files = base_dataset_path / DATA_PREFIX
    for folder in os.listdir(data_files):
        if not os.path.isdir(data_files / folder):
            continue
        with tarfile.open(data_files / f"{folder}.tar", "w") as tar:
            tar.add(data_files / folder, arcname=os.path.basename(folder))
        shutil.rmtree(data_files / folder)

    image_files = base_dataset_path / IMAGE_PREFIX
    for folder in os.listdir(image_files):
        if not os.path.isdir(image_files / folder):
            continue
        with tarfile.open(image_files / f"{folder}.tar", "w") as tar:
            tar.add(image_files / folder, arcname=os.path.basename(folder))
        shutil.rmtree(image_files / folder)


def build_data_part(
    *, bags_path: Path, dataset_base_path: Path, topic_registry: TopicRegistry
) -> None:
    progress = tqdm(topic_registry.values())
    for attribute_types, topic_desc in progress:
        progress.set_description(f"Processing {topic_desc.alias}")

        bag_file = bags_path / topic_desc.file
        _generate_dataset_from_topic_description_and_attribute_types(
            dataset_base_path,
            bag_file,
            topic_desc=topic_desc,
            attribute_types=attribute_types,
        )
    _tar_ball_dataset(dataset_base_path)
