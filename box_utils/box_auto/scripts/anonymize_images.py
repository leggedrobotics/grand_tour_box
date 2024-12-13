from __future__ import annotations

import os
import shutil
from itertools import chain
from pathlib import Path
from typing import Any, Collection, Mapping, Sequence, Tuple

import cv2
import numpy as np
import rosbag
import supervision as sv
from cv_bridge import CvBridge
from ultralytics import YOLO

# NOTE: use conda env img_anon

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

# yolo settings
MODELS_PATH = Path(__file__).parent / "models"
MODEL_PATH = MODELS_PATH / "yolo-v11.pt"
MODEL = YOLO(MODEL_PATH)
BATCH_SIZE = 4


# blur settings
BLUR_CLASS_IDS = (1, 2)  # 0 for sign
FORWARD_BUFFER = 4
BACKWARD_BUFFER = 4
BLUR_SIZE = 25


def fetch_multiple_files_kleinkram(patterns):
    tmp_dir = "/mission_data/tmp"
    os.makedirs(tmp_dir, exist_ok=True)

    for pattern in patterns:
        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein download --mission {uuid} --dest {tmp_dir} '{pattern}'")

            # Move all files from /mission_data/tmp to /mission_data/
            for file_name in os.listdir(tmp_dir):
                source_file = os.path.join(tmp_dir, file_name)
                destination_file = os.path.join("/mission_data", file_name)
                shutil.move(source_file, destination_file)


Box = Tuple[int, int, int, int]


def _process_detections(
    detections: sv.Detections, class_ids: Collection[int] = BLUR_CLASS_IDS
) -> list[Box]:
    """\
    convert `sv.Detections` to `list[Box]` where `Box` is a tuple of 4 ints
    """
    ret = []
    for xyxy, _, _, cid, _, _ in detections:  # TODO: proper unpacking
        if cid not in class_ids:
            continue
        ret.append(tuple(map(int, xyxy)))
    return ret


def _get_detections_batched(
    images: Sequence[np.ndarray], class_ids: Collection[int] = BLUR_CLASS_IDS
) -> list[list[Box]]:
    """\
    runs a batch of images through the model and returns detection boxes for each image
    """
    print("running batch...")
    sv_detections = [
        sv.Detections.from_ultralytics(d) for d in MODEL.predict(source=list(images))
    ]
    return [_process_detections(d, class_ids=class_ids) for d in sv_detections]


def _blur_image(
    image: np.ndarray,
    detections: Collection[Box],
) -> np.ndarray:
    """\
    applies blurs to images based on detections
    """

    mask = np.zeros_like(image, dtype=np.uint8)
    for x1, y1, x2, y2 in detections:
        mask[y1:y2, x1:x2] = 1
    blurred = cv2.GaussianBlur(image, (BLUR_SIZE, BLUR_SIZE), 0)
    return np.where(mask, blurred, image)


def _apply_temporal_blur(
    buffered_detections: Sequence[Collection[Box]],
    images: Sequence[np.ndarray],
    forward_buffer: int = FORWARD_BUFFER,
    backward_buffer: int = BACKWARD_BUFFER,
) -> list[np.ndarray]:
    """\
    apply temporal blur to images based on detections

    an image is always blurred with the detection from the previous `backward_buffer` frames
    and the detections from the next `forward_buffer` frames

    for this reason `buffered_detections` must have length `len(images) + forward_buffer + backward_buffer`
    where the buffered_detections[i + backward_buffer] are the detections for the image[i]
    """

    assert len(images) + forward_buffer + backward_buffer == len(buffered_detections)
    buffer_size = 1 + forward_buffer + backward_buffer

    blurred_images = []
    for i, image in enumerate(images):
        detections = buffered_detections[i : i + buffer_size]
        image = _blur_image(image, list(chain.from_iterable(detections)))
        blurred_images.append(image)
    return blurred_images


def _msg_to_image(msg: Any, cv_bridge: CvBridge, compressed: bool) -> np.ndarray:
    """\
    deser an image message using cv_bridge
    """
    if compressed:
        return cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    else:
        return cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def _image_to_msg(raw: np.ndarray, cv_bridge: CvBridge, compressed: bool = True) -> Any:
    """\
    ser an image using cv_bridge
    """
    if compressed:
        return cv_bridge.cv2_to_compressed_imgmsg(raw, dst_format="jpg")
    else:
        return cv_bridge.cv2_to_imgmsg(raw, encoding="passthrough")


def _write_images_to_bag(
    bag: rosbag.Bag,
    cv_bridge: CvBridge,
    metadata: Sequence[tuple[int, str, bool]],
    images: Sequence[np.ndarray],
) -> None:
    """\
    writes images with corresponding metadata to an open bagfile
    """
    assert len(metadata) == len(images)

    for (timestamp, topic, compress), image in zip(metadata, images):
        msg = _image_to_msg(image, cv_bridge, compressed=compress)
        msg.header.stamp = timestamp
        bag.write(topic, msg, timestamp)


def anonymize_bag(in_path: Path, out_path: Path, image_topics: Mapping[str, str]):
    """\
    blurs faces and license plates in images from rosbag file
    we implicitly assume that all images in a bagfile can be interpreted as a single video

    - `in_path` path to input bagfile
    - `out_path` path to output bagfile
    - `image_topics` mapping from input image topics to output image topics
    """

    # init buffers
    # detections_buffer is filled with "empty" detections to start off
    batch_buffer: list[np.ndarray] = []
    image_buffer: list[np.ndarray] = []
    metadata_buffer: list[tuple[int, str, bool]] = []  # timestamp, topic
    detections_buffer = [[] for _ in range(BACKWARD_BUFFER)]

    with rosbag.Bag(str(in_path), "r") as in_bag, rosbag.Bag(
        str(out_path), "w", compression="lz4"
    ) as out_bag:
        cv_bridge = CvBridge()
        print("anonymizing images...")

        for idx, (topic, msg, t) in enumerate(in_bag.read_messages()):
            if topic not in image_topics:
                out_bag.write(topic, msg, t)
                continue
            if idx % 50 == 0:
                print(f"processing message {idx:6d} @ {t}")

            is_compressed = "CompressedImage" in type(msg)._type
            image = _msg_to_image(msg, cv_bridge, is_compressed)
            batch_buffer.append(image)
            metadata_buffer.append((t, image_topics[topic], is_compressed))

            # once we reach the batch size, we run the detection model
            if len(batch_buffer) == BATCH_SIZE:
                detections = _get_detections_batched(batch_buffer)
                detections_buffer.extend(detections)
                image_buffer.extend(batch_buffer)
                batch_buffer = []

            # once we have enough detections from our model we can start blurring
            if len(image_buffer) > FORWARD_BUFFER:
                blurred_images = _apply_temporal_blur(
                    detections_buffer,
                    image_buffer[:-FORWARD_BUFFER],
                    forward_buffer=FORWARD_BUFFER,
                    backward_buffer=BACKWARD_BUFFER,
                )
                n_images = len(blurred_images)

                _write_images_to_bag(
                    out_bag, cv_bridge, metadata_buffer[:n_images], blurred_images
                )

                # clear stuff from buffers we no longer need
                image_buffer = image_buffer[n_images:]
                detections_buffer = detections_buffer[n_images:]
                metadata_buffer = metadata_buffer[n_images:]

        # run last batch
        detections = _get_detections_batched(batch_buffer)
        detections_buffer.extend(detections + [[] for _ in range(FORWARD_BUFFER)])
        image_buffer.extend(batch_buffer)
        blurred_images = _apply_temporal_blur(
            detections_buffer,
            image_buffer,
            forward_buffer=FORWARD_BUFFER,
            backward_buffer=BACKWARD_BUFFER,
        )
        _write_images_to_bag(out_bag, cv_bridge, metadata_buffer, blurred_images)

        # reindex the bag
        print("reindexing bag...")

    out_bag.reindex()
    print(f"done anonymizing {in_path} -> {out_path}")


FILE_PATH = Path(__file__).parent / "data" / "eth_campus.bag"
OUT_PATH = Path(__file__).parent / "data" / "anon.bag"

if __name__ == "__main__":
    anonymize_bag(
        FILE_PATH,
        OUT_PATH,
        image_topics={
            "/gt_box/hdr_front/image_raw/compressed": "/gt_box/hdr_front/image_anonymized/compressed"
        },
    )
