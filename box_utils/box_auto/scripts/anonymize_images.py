from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Any
from typing import List
from typing import Mapping
from typing import Sequence
from typing import Tuple
from typing import Optional

import numpy as np
import rosbag
import supervision as sv
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from ultralytics import YOLO

# NOTE: use conda env img_anon

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

# yolo settings
MODELS_PATH = Path(__file__).parent / "models"

FACE_MODEL11_PATH = MODELS_PATH / "yolov11m-faces.pt"
FACE_MODEL11 = YOLO(FACE_MODEL11_PATH)
FACE_MODEL11_CIDS = [0]
FACE_MODEL11_CONF = 0.05

FACE_MODEL8_PATH = MODELS_PATH / "yolov8s-faces.pt"
FACE_MODEL8 = YOLO(FACE_MODEL8_PATH)
FACE_MODEL8_CIDS = [0]
FACE_MODEL8_CONF = 0.10

PLATE_MODEL8_PATH = MODELS_PATH / "yolov8s-plates.pt"
PLATE_MODEL8 = YOLO(PLATE_MODEL8_PATH)
PLATE_MODEL8_CIDS = [0]
PLATE_MODEL8_CONF = 0.35

FACE_PLATE_MODEL11_PATH = MODELS_PATH / "yolov11l-faces-plates.pt"
FACE_PLATE_MODEL11 = YOLO(FACE_PLATE_MODEL11_PATH)
FACE_PLATE_MODEL11_CIDS = [1, 2]  # 0 for sign
FACE_PLATE_MODEL11_CONF = 0.05

MODELS = {
    "faces11": (FACE_MODEL11, FACE_MODEL11_CIDS, FACE_MODEL11_CONF),
    # "plates8": (PLATE_MODEL8, PLATE_MODEL8_CIDS, PLATE_MODEL8_CONF), # this sucks for some reason
    "faces_plates11": (
        FACE_PLATE_MODEL11,
        FACE_PLATE_MODEL11_CIDS,
        FACE_PLATE_MODEL11_CONF,
    ),
    "faces8": (FACE_MODEL8, FACE_MODEL8_CIDS, FACE_MODEL8_CONF),
}


BATCH_SIZE = 4


# blur settings
BLUR_CLASS_IDS = (0,)  # (1, 2)  # 0 for sign
FORWARD_BUFFER = 4
BACKWARD_BUFFER = 4
BLUR_SIZE = 25

# annotators
BOX_ANNOTATOR = sv.BoxAnnotator()
LABEL_ANNOTATOR = sv.LabelAnnotator()
BLUR_ANNOTATOR = sv.BlurAnnotator()


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


def _get_detections_batched(images: Sequence[np.ndarray]) -> list[sv.Detections]:
    """\
    runs a batch of images through the model and returns detection boxes for each image
    """
    print("running batch...")

    model_dets: List[List[sv.Detections]] = []
    for model, cids, conf in MODELS.values():
        dets: List[sv.Detections] = []
        for det in model.predict(source=list(images), conf=conf):
            raw_det = sv.Detections.from_ultralytics(det)
            assert raw_det.class_id is not None
            dets.append(raw_det[np.isin(raw_det.class_id, cids)])  # type: ignore
        model_dets.append(dets)

    ret = []
    for dets in zip(*model_dets):
        stacked_dets = sv.Detections.merge(list(dets))
        ret.append(stacked_dets)

    return ret


def _tracked_detections(
    detections: Sequence[sv.Detections], tracker: sv.ByteTrack
) -> List[Tuple[sv.Detections, sv.Detections]]:
    """\
    tracks detections using a tracker
    """
    ret: List[Tuple[sv.Detections, sv.Detections]] = []
    for dets in detections:
        tracked_dets = tracker.update_with_detections(dets)
        ret.append((tracked_dets, dets))
    return ret


def _blur_image(
    image: np.ndarray,
    detections: tuple[sv.Detections, sv.Detections],
    draw_boxes: bool = False,
) -> np.ndarray:
    """\
    applies blurs to images based on detections
    """
    # filter class ids and blur
    tracked, raw = detections
    image = BLUR_ANNOTATOR.annotate(image, tracked)
    image = BLUR_ANNOTATOR.annotate(image, raw)

    if draw_boxes:
        image = BOX_ANNOTATOR.annotate(image, tracked)
        image = LABEL_ANNOTATOR.annotate(image, tracked)
        image = BOX_ANNOTATOR.annotate(image, raw)
        image = LABEL_ANNOTATOR.annotate(image, raw)
    return image


# used for computing the dimension of the nullspace
EPS = 1e-6


def _compute_number_of_detections(
    detections: Tuple[sv.Detections, sv.Detections]
) -> int:
    raw, tracked = detections  # shape (N, 4) each
    total_dets = np.concatenate([raw.xyxy, tracked.xyxy], axis=0)
    N = total_dets.shape[0]

    if N == 0:
        return 0

    dets_left = total_dets.reshape((-1, N, 4))
    dets_right = total_dets.reshape((N, -1, 4))

    x_max = np.maximum(dets_left[:, :, 0], dets_right[:, :, 0])
    x_min = np.minimum(dets_left[:, :, 2], dets_right[:, :, 2])

    y_max = np.maximum(dets_left[:, :, 1], dets_right[:, :, 1])
    y_min = np.minimum(dets_left[:, :, 3], dets_right[:, :, 3])

    # compute connected components
    intersect = (x_max <= x_min) & (y_max <= y_min)

    # compute the number of connected components of the graph laplacian
    A = intersect.astype(np.int32)
    D = np.diag(np.sum(A, axis=1))
    L = D - A
    w, _ = np.linalg.eig(L)
    return int(np.sum(w < EPS))


def _blur_images(
    images: Sequence[np.ndarray],
    detections: Sequence[Tuple[sv.Detections, sv.Detections]],
    draw_boxes: bool = False,
) -> List[Tuple[np.ndarray, int]]:
    """\
    applies blurs to images based on detections
    """
    ret: List[Tuple[np.ndarray, int]] = []
    for image, dets in zip(images, detections):
        blurred = _blur_image(image, dets, draw_boxes=draw_boxes)
        n_dets = _compute_number_of_detections(dets)
        ret.append((blurred, n_dets))
    return ret


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
    images_with_detections: Sequence[Tuple[np.ndarray, int]],
) -> None:
    """\
    writes images with corresponding metadata to an open bagfile
    """
    assert len(metadata) == len(images_with_detections)

    for (timestamp, topic, compress), (image, n_dets) in zip(
        metadata, images_with_detections
    ):
        # write the image
        msg = _image_to_msg(image, cv_bridge, compressed=compress)
        msg.header.stamp = timestamp
        bag.write(topic, msg, timestamp)

        # write number of detections
        n_dets_msg = Int32()
        n_dets_msg.data = n_dets
        bag.write(f"{topic}/n_detections", n_dets_msg, timestamp)


FRAME_RATE = 10
DRAW_BOXES = True


def anonymize_bag(
    in_path: Path,
    out_path: Path,
    image_topics: Mapping[str, str],
    head: Optional[int] = None,
):
    """\
    blurs faces and license plates in images from rosbag file
    we implicitly assume that all images in a bagfile can be interpreted as a single video

    - `in_path` path to input bagfile
    - `out_path` path to output bagfile
    - `image_topics` mapping from input image topics to output image topics
    """

    # init buffers
    # detections_buffer is filled with "empty" detections to start off
    image_buffer: list[np.ndarray] = []
    metadata_buffer: list[tuple[int, str, bool]] = []  # timestamp, topic

    with rosbag.Bag(str(in_path), "r") as in_bag, rosbag.Bag(
        str(out_path), "w", compression="lz4"
    ) as out_bag:
        cv_bridge = CvBridge()
        print("anonymizing images...")

        tracker = sv.ByteTrack(
            frame_rate=FRAME_RATE,
        )
        for idx, (topic, msg, t) in enumerate(in_bag.read_messages()):
            if topic not in image_topics:
                out_bag.write(topic, msg, t)
                continue
            if idx % 50 == 0:
                print(f"processing message {idx:6d} @ {t}")
            if head is not None and idx >= head:
                break

            is_compressed = "CompressedImage" in type(msg)._type
            image = _msg_to_image(msg, cv_bridge, is_compressed)
            image_buffer.append(image)
            metadata_buffer.append((t, image_topics[topic], is_compressed))

            # once we reach the batch size, we run the detection model
            if len(image_buffer) == BATCH_SIZE:
                detections = _get_detections_batched(image_buffer)
                tracked_detections = _tracked_detections(detections, tracker)
                blurred_images = _blur_images(
                    image_buffer, tracked_detections, draw_boxes=DRAW_BOXES
                )
                _write_images_to_bag(
                    out_bag, cv_bridge, metadata_buffer, blurred_images
                )
                image_buffer = []
                metadata_buffer = []

        detections = _get_detections_batched(image_buffer)
        tracked_detections = _tracked_detections(detections, tracker)
        blurred_images = _blur_images(
            image_buffer, tracked_detections, draw_boxes=DRAW_BOXES
        )

        _write_images_to_bag(out_bag, cv_bridge, metadata_buffer, blurred_images)

    # reindex the bag
    print("reindexing bag...")
    out_bag.reindex()
    print(f"done anonymizing {in_path} -> {out_path}")


FILE_PATH = Path(__file__).parent / "data" / "eth_campus.bag"
OUT_PATH = Path(__file__).parent / "data" / "_bytetrack_anon.bag"

if __name__ == "__main__":
    anonymize_bag(
        FILE_PATH,
        OUT_PATH,
        image_topics={
            "/gt_box/hdr_front/image_raw/compressed": "/gt_box/hdr_front/image_anonymized/compressed"
        },
    )
