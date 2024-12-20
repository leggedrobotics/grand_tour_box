from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Any
from typing import List
from typing import Mapping
from typing import Optional
from typing import Sequence
from typing import Tuple, Dict

import numpy as np
import rosbag
import supervision as sv
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from ultralytics import YOLO

# NOTE: use conda env img_anon

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

# yolo settings
MODELS_PATH = Path(__file__).parent.parent.parent.parent.parent / "models"

FACE_MODEL11_PATH = MODELS_PATH / "yolov11m-faces.pt"
FACE_MODEL11 = YOLO(FACE_MODEL11_PATH)
FACE_MODEL11_CIDS = [0]
FACE_MODEL11_CONF = 0.05

FACE_MODEL8_PATH = MODELS_PATH / "yolov8s-faces.pt"
FACE_MODEL8 = YOLO(FACE_MODEL8_PATH)
FACE_MODEL8_CIDS = [0]
FACE_MODEL8_CONF = 0.10

FACE_PLATE_MODEL11_PATH = MODELS_PATH / "yolov11l-faces-plates.pt"
FACE_PLATE_MODEL11 = YOLO(FACE_PLATE_MODEL11_PATH)
FACE_PLATE_MODEL11_CIDS = [1, 2]  # 0 for sign
FACE_PLATE_MODEL11_CONF = 0.05

MODEL11_PATH = MODELS_PATH / "yolov11m.pt"
MODEL11 = YOLO(MODEL11_PATH)
MODEL11_CIDS = [0]
MODEL11_CONF = 0.10
HUMANS_MAX_SIZE = 4096


# model, class_ids, confidence, max_size
DetectorConfig = Tuple[YOLO, List[int], float, Optional[int]]
MODELS: Dict[str, DetectorConfig] = {
    "faces11": (FACE_MODEL11, FACE_MODEL11_CIDS, FACE_MODEL11_CONF, None),
    "faces_plates11": (
        FACE_PLATE_MODEL11,
        FACE_PLATE_MODEL11_CIDS,
        FACE_PLATE_MODEL11_CONF,
        None,
    ),
    "faces8": (FACE_MODEL8, FACE_MODEL8_CIDS, FACE_MODEL8_CONF, None),
    "full11": (MODEL11, MODEL11_CIDS, MODEL11_CONF, HUMANS_MAX_SIZE),
}

# blur settings
BLUR_CLASS_IDS = (0,)  # (1, 2)  # 0 for sign
FORWARD_BUFFER = 4
BACKWARD_BUFFER = 4
BLUR_SIZE = 25

# annotators
BOX_ANNOTATOR = sv.BoxAnnotator()
LABEL_ANNOTATOR = sv.LabelAnnotator()
BLUR_ANNOTATOR = sv.BlurAnnotator()

# other settings
FRAME_RATE = 10
DRAW_BOXES = True

# TODO: remove this
DATA_PATH = Path(__file__).parent.parent.parent.parent.parent / "data"
FILE_PATH = DATA_PATH / "pilatus.bag"
OUT_PATH = DATA_PATH / "out.bag"


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


def _get_detections(image: np.ndarray) -> sv.Detections:
    """\
    runs an image through all models, filters the detections by class id and max size
    and returns the merged detections
    """
    dets: List[sv.Detections] = []
    for model, cids, conf, max_size in MODELS.values():
        pred = model.predict(image, conf=conf, verbose=False)[0]
        raw_det = sv.Detections.from_ultralytics(pred)

        # filter by class id
        filtered_det: sv.Detections = raw_det[np.isin(raw_det.class_id, cids)]  # type: ignore

        if max_size is not None:  # filter by max size
            filtered_det = filtered_det[filtered_det.area < max_size]  # type: ignore

        dets.append(filtered_det)

    # merge the different detections
    return sv.Detections.merge(dets)


def _tracked_detections(
    detections: sv.Detections, tracker: sv.ByteTrack
) -> Tuple[sv.Detections, sv.Detections]:
    """\
    tracks detections using a tracker, this slightly improves the detection quality
    over using frame by frame detections

    we also return the raw detections
    """
    tracked_dets = tracker.update_with_detections(detections)
    return tracked_dets, detections


def _blur_image(
    image: np.ndarray, detections: tuple[sv.Detections, sv.Detections], draw_boxes: bool
) -> np.ndarray:
    """\
    applies blurs to images based on detections

    If `draw_boxes` is True, we also draw lables and bounding boxes
    around the detections. This is useful for debugging
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
    r"""\
    computes the number of detections by computing the
    connected components of the union of all detections

    for this we first compute an adjacency matrix of all detections
    and wheter or not they overlap

    then we compute the number of connected components of the resuling graph
    by computing the dimension of the kernel of the laplacian matrix
    $$
    L = D - A
    \mathrm{dim}\ker(L)
    $$
    """

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


def _process_image(
    image: np.ndarray, detections: Tuple[sv.Detections, sv.Detections], draw_boxes: bool
) -> Tuple[np.ndarray, int]:
    """\
    blurs an image base on a tuple of detections, and compuets the number of detections
    """
    blurred = _blur_image(image, detections, draw_boxes=draw_boxes)
    n_dets = _compute_number_of_detections(detections)
    return blurred, n_dets


def _process_images(
    images: Sequence[np.ndarray],
    detections: Sequence[Tuple[sv.Detections, sv.Detections]],
    draw_boxes: bool,
) -> List[Tuple[np.ndarray, int]]:
    """\
    does the same as _process_image but for a batch of images
    """
    ret: List[Tuple[np.ndarray, int]] = []
    for image, dets in zip(images, detections):
        ret.append(_process_image(image, dets, draw_boxes=draw_boxes))
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
    with rosbag.Bag(str(in_path), "r") as in_bag, rosbag.Bag(
        str(out_path), "w", compression="lz4"
    ) as out_bag:
        cv_bridge = CvBridge()
        print("anonymizing images...")

        tracker = sv.ByteTrack(
            frame_rate=FRAME_RATE,
            minimum_matching_threshold=0.1,
            track_activation_threshold=0.05,
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
            metadata = (t, image_topics[topic], is_compressed)

            # TODO: make functions nicer
            raw_detection = _get_detections(image)
            detections = _tracked_detections(raw_detection, tracker)
            blurred_image, n_dets = _process_image(
                image, detections, draw_boxes=DRAW_BOXES
            )
            _write_images_to_bag(
                out_bag, cv_bridge, [metadata], [(blurred_image, n_dets)]
            )

    # reindex the bag
    print("reindexing bag...")
    out_bag.reindex()
    print(f"done anonymizing {in_path} -> {out_path}")


if __name__ == "__main__":
    anonymize_bag(
        FILE_PATH,
        OUT_PATH,
        image_topics={
            "/gt_box/hdr_front/image_raw/compressed": "/gt_box/hdr_front/image_anonymized/compressed"
        },
    )
