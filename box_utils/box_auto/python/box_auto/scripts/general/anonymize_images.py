from __future__ import annotations

import os
import shutil
from pathlib import Path
from secrets import token_hex
from typing import (
    Any,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
    cast,
    NamedTuple,
    Union,
)
import time
from datetime import timedelta, datetime

import numpy as np
import rosbag
import supervision as sv
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from ultralytics import YOLO
import pickle

from box_auto.utils import get_bag, upload_bag

DEBUG = bool(os.environ.get("DEBUG_ANON", False))
os.environ.setdefault("YOLO_VERBOSE", "False")

# yolo settings
MODELS_PATH = Path(__file__).parent.parent.parent.parent.parent / "models"

# mounted by container for debugging
DATA_DIR = Path("/tmp_disk") / "bm"

FACE_MODEL11_PATH = MODELS_PATH / "yolov11m-faces.pt"
FACE_MODEL11 = YOLO(FACE_MODEL11_PATH)
FACE_MODEL11_CIDS = [0]
FACE_MODEL11_CONF = 0.10

FACE_MODEL11L_PATH = DATA_DIR.parent / "yolov11l-faces.pt"
FACE_MODEL11L = YOLO(FACE_MODEL11L_PATH)
FACE_MODEL11L_CIDS = [0]
FACE_MODEL11L_CONF = 0.10

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
    # "faces11": (FACE_MODEL11, FACE_MODEL11_CIDS, FACE_MODEL11_CONF, None),
    "faces11l": (FACE_MODEL11L, FACE_MODEL11L_CIDS, FACE_MODEL11L_CONF, None),
    "faces_plates11": (
        FACE_PLATE_MODEL11,
        FACE_PLATE_MODEL11_CIDS,
        FACE_PLATE_MODEL11_CONF,
        None,
    ),
    "faces8": (FACE_MODEL8, FACE_MODEL8_CIDS, FACE_MODEL8_CONF, None),
    "full11": (MODEL11, MODEL11_CIDS, MODEL11_CONF, HUMANS_MAX_SIZE),
}

# annotators
BOX_ANNOTATOR = sv.BoxAnnotator()
LABEL_ANNOTATOR = sv.LabelAnnotator()
BLUR_ANNOTATOR = sv.BlurAnnotator()

# other settings
FRAME_RATE = 10
DRAW_BOXES = DEBUG

# config for running the anonymization action
# key: camera name, is arbitrary and only used for logging
# value: topics..., file_glob
# Some notes:
# - we return the file with a `_anonymized` suffix to the same folder as the original file
# - the original file is not modified
ACTION_CAMERAS_CFG: Dict[str, Tuple[str, ...]] = {
    "cam0-4": (
        "/gt_box/alphasense_driver_node/cam2/color/image/compressed",
        "/gt_box/alphasense_driver_node/cam3/color/image/compressed",
        "/gt_box/alphasense_driver_node/cam1/color/image/compressed",
        "/gt_box/alphasense_driver_node/cam5/color/image/compressed",
        "/gt_box/alphasense_driver_node/cam4/color/image/compressed",
        "*_nuc_alphasense.bag",
    ),
    "cam5": (
        "/gt_box/hdr_left/image_raw/compressed",
        "*_jetson_hdr_left.bag",
    ),
    "cam6": (
        "/gt_box/hdr_front/image_raw/compressed",
        "*_jetson_hdr_front.bag",
    ),
    "cam7": (
        "/gt_box/hdr_right/image_raw/compressed",
        "*_jetson_hdr_right.bag",
    ),
    "cam8-9": (
        "/gt_box/zed2i/zed_node/left/image_rect_color/compressed",
        "/gt_box/zed2i/zed_node/right/image_rect_color/compressed",
        "*_jetson_zed2i_images.bag",
    ),
}


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
    \mathrm{dim}\ker(L) \quad \text{wehre} \quad L := D - A
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


def _flip_image(image: np.ndarray) -> np.ndarray:
    """\
    rotates the imgae
    """
    # opencv really doesnt like negative strides
    # so we need to copy to realize the view
    return np.flipud(np.fliplr(image)).copy()


def _msg_to_image(
    msg: Any, cv_bridge: CvBridge, compressed: bool, flipped: bool
) -> Tuple[np.ndarray, bool]:
    """\
    deser an image message using cv_bridge
    """
    if compressed:
        img = cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    else:
        img = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    if flipped:
        img = _flip_image(img)

    if len(img.shape) == 2:
        img = np.stack([img] * 3, axis=-1)
        return img, True
    return img, False


def _image_to_msg(
    raw: np.ndarray,
    cv_bridge: CvBridge,
    compressed: bool = True,
    grayscale: bool = True,
) -> Any:
    """\
    ser an image using cv_bridge
    """

    if grayscale:
        raw = raw[..., 0]

    if compressed:
        return cv_bridge.cv2_to_compressed_imgmsg(raw, dst_format="jpg")
    else:
        return cv_bridge.cv2_to_imgmsg(raw, encoding="passthrough")


def _write_image_to_bag(
    bag: rosbag.Bag,
    cv_bridge: CvBridge,
    metadata: tuple[int, str, bool, bool, bool],
    image: np.ndarray,
    n_dets: int,
) -> None:
    """\
    writes images with corresponding metadata to an open bagfile
    """
    timestamp, topic, compressed, flipped, grayscale = metadata

    if flipped:
        image = _flip_image(image)

    # write the image
    msg = _image_to_msg(image, cv_bridge, compressed=compressed, grayscale=grayscale)
    msg.header.stamp = timestamp
    bag.write(topic, msg, timestamp)

    # write number of detections
    n_dets_msg = Int32()
    n_dets_msg.data = n_dets
    bag.write(f"{topic}/n_detections", n_dets_msg, timestamp)


# ts: (N), sid: (N), bbox: (N, 8)
DetectionBoxes = Tuple[np.ndarray, np.ndarray, np.ndarray]


class CameraInfo(NamedTuple):
    K: np.ndarray
    D: np.ndarray
    R: np.ndarray
    P: np.ndarray


def _msg_to_camera_info(msg: Any) -> CameraInfo:
    """\
    converts a camera info message to a tuple of numpy arrays
    """
    return CameraInfo(
        np.array(msg.K).reshape((3, 3)),
        np.array(msg.D),
        np.array(msg.R),
        np.array(msg.P).reshape((3, 4)),
    )


def _extrac_camera_info(bag: rosbag.Bag, cinfo_topic: str) -> CameraInfo:
    """\
    extracts camera info from a bagfile
    """
    for _, msg, _ in bag.read_messages(topics=[cinfo_topic]):
        return _msg_to_camera_info(msg)
    raise ValueError(f"no camera info found for topic {cinfo_topic}")


def _detections_to_frame_data(
    dets: Tuple[sv.Detections, sv.Detections],
    ts: int,
    seq_id: int,
    flipped: bool,
    hw: Tuple[int, int],
) -> DetectionBoxes:
    """\
    converts sv.Detections to a frame detection tuple to store in a file
    """
    raw, tracked = dets

    bboxes_xyxy = raw.xyxy.tolist() + tracked.xyxy.tolist()
    bboxes_quad_point: List[List[float]] = []
    for x1, y1, x2, y2 in bboxes_xyxy:
        if flipped:
            h, w = hw
            x1, x2 = w - x2, w - x1
            y1, y2 = h - y2, h - y1

        bboxes_quad_point.append([x1, y1, x2, y1, x2, y2, x1, y2])

    N = len(bboxes_quad_point)
    return (np.ones(N) * ts, np.ones(N) * seq_id, np.array(bboxes_quad_point))


LOG_INTERVAL = 10


class FrameDetections(NamedTuple):
    ts: np.ndarray
    sid: np.ndarray
    points: np.ndarray
    cinfo: CameraInfo
    image_topic: str
    cinfo_topic: str
    bag_path: Path


def run_model_inference(
    bag_path: Path,
    image_topics: Sequence[str],
    cinfo_topics: Sequence[str],
    start: Optional[Union[datetime, int]],
    duration: Optional[Union[timedelta, int]],
    flip_images_for_inference: bool = False,
) -> Dict[str, FrameDetections]:
    assert len(image_topics) == len(cinfo_topics)

    if isinstance(start, datetime):
        start = int(start.timestamp())
    if isinstance(duration, timedelta):
        duration = int(duration.total_seconds())

    with rosbag.Bag(str(bag_path), "r") as bag:
        detection_data: Dict[
            str, Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray]]
        ] = {topic: ([], [], []) for topic in image_topics}
        cinfo_dct = {
            topic: _extrac_camera_info(bag, cinfo_topic)
            for topic, cinfo_topic in zip(image_topics, cinfo_topics)
        }

        # needed for reading images
        cv_bridge = CvBridge()

        trackers = {
            topic: sv.ByteTrack(
                frame_rate=FRAME_RATE,
                minimum_matching_threshold=0.05,
                track_activation_threshold=0.05,
            )
            for topic in image_topics
        }

        image_msg_count = 0
        t0 = time.monotonic()

        for topic, msg, t in bag.read_messages():
            # skip messages that are not images or outside the time range
            if (
                start is not None
                and duration is not None
                and (t.to_sec() <= start or t.to_sec() >= start + duration)
                or topic not in image_topics
            ):
                continue

            is_compressed = "CompressedImage" in type(msg)._type
            image, _ = _msg_to_image(
                msg, cv_bridge, is_compressed, flipped=flip_images_for_inference
            )

            raw_detection = _get_detections(image)
            detections = _tracked_detections(raw_detection, trackers[topic])

            ts, sid, bboxs = _detections_to_frame_data(
                detections,
                t.to_nsec(),
                image_msg_count,
                flipped=flip_images_for_inference,
                hw=tuple(image.shape[:2]),  # type: ignore
            )

            # we have zero detections we skip the frame
            if bboxs.size:
                detection_data[topic][0].append(ts)
                detection_data[topic][1].append(sid)
                detection_data[topic][2].append(bboxs)

            if not image_msg_count % LOG_INTERVAL:
                images_per_second = LOG_INTERVAL / (time.monotonic() - t0)
                print(
                    f"inference... {image_msg_count:6d} at timestamp {t}, {images_per_second:.2f} images/s"
                )
                t0 = time.monotonic()

            image_msg_count += 1

        ret = {}
        for image_topic, cinfo_topic in zip(image_topics, cinfo_topics):
            ts = np.concatenate(detection_data[image_topic][0], axis=0)
            sid = np.concatenate(detection_data[image_topic][1], axis=0)
            points = np.concatenate(detection_data[image_topic][2], axis=0)

            fd = FrameDetections(
                ts=ts,
                sid=sid,
                points=points,
                cinfo=cinfo_dct[image_topic],
                image_topic=image_topic,
                cinfo_topic=cinfo_topic,
                bag_path=bag_path,
            )
            ret[image_topic] = fd

        return ret


def fix_lense_distortions() -> None: ...


import cv2


def export_images(bag_path: Path, out_dir: Path, topic: str) -> None:
    with rosbag.Bag(str(bag_path), "r") as bag:
        cv_bridge = CvBridge()

        for im_id, (_, msg, t) in enumerate(bag.read_messages(topics=[topic])):
            is_compressed = "CompressedImage" in type(msg)._type
            image, _ = _msg_to_image(msg, cv_bridge, is_compressed, flipped=False)

            out_path = out_dir / f"{im_id:06d}.jpg"
            cv2.imwrite(str(out_path), image)


def anonymize_bag(
    in_path: Path,
    out_path: Path,
    image_topic: str,
    cinfo_topic: str,
    flipped_image: bool,
    head: Optional[int] = None,
    start: Optional[int] = None,
    duration: Optional[int] = None,
) -> None:
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
        detection_data: list[FrameDetections] = []
        cv_bridge = CvBridge()

        print("getting camera info...")
        cinfo = _extrac_camera_info(in_bag, cinfo_topic)

        print("anonymizing images...")
        tracker = sv.ByteTrack(
            frame_rate=FRAME_RATE,
            minimum_matching_threshold=0.05,
            track_activation_threshold=0.05,
        )
        image_msg_count = 0
        t0 = time.monotonic()

        for topic, msg, t in in_bag.read_messages():

            if (
                start
                and duration
                and (t.to_sec() <= start or t.to_sec() >= start + duration)
            ):
                continue

            if topic != image_topic:
                out_bag.write(topic, msg, t)
                continue

            is_compressed = "CompressedImage" in type(msg)._type

            image, grayscale = _msg_to_image(
                msg, cv_bridge, is_compressed, flipped_image
            )
            metadata = (
                t,
                f"{image_topic}_anon",
                is_compressed,
                flipped_image,
                grayscale,
            )

            raw_detection = _get_detections(image)
            detections = _tracked_detections(raw_detection, tracker)

            detection_data.append(
                _detections_to_frame_data(
                    detections,
                    t.to_nsec(),
                    image_msg_count,
                    flipped=flipped_image,
                    hw=tuple(image.shape[:2]),  # type: ignore
                )
            )

            blurred_image, n_dets = _process_image(
                image, detections, draw_boxes=DRAW_BOXES
            )
            _write_image_to_bag(out_bag, cv_bridge, metadata, blurred_image, n_dets)

            if not image_msg_count % LOG_INTERVAL:
                images_per_second = LOG_INTERVAL / (time.monotonic() - t0)
                print(
                    f"processing image {image_msg_count:6d} at timestamp {t}, {images_per_second:.2f} images/s"
                )
                t0 = time.monotonic()

            image_msg_count += 1
            if head is not None and image_msg_count >= head:
                break

        dets_ts_stacked: List[np.ndarray] = [d[0] for d in detection_data]
        dets_sid_stacked: List[np.ndarray] = [d[1] for d in detection_data]
        dets_bbox_stacked: List[np.ndarray] = [
            d[2] for d in detection_data if d[0].size > 0
        ]

        detection_data_compact = {
            "ts": np.concatenate(dets_ts_stacked, axis=0),
            "sid": np.concatenate(dets_sid_stacked, axis=0),
            "points": np.concatenate(dets_bbox_stacked, axis=0),
            "topic": image_topic,
            "cinfo": cinfo,
        }

        with open(out_path.with_suffix(".pkl"), "wb") as f:
            pickle.dump(detection_data_compact, f)

    # reindex the bag
    print("reindexing bag...")
    out_bag.reindex()
    print(f"done anonymizing {in_path} -> {out_path}")


RUN_ALL = {
    "alphasense": (
        [
            (
                f"/gt_box/alphasense_driver_node/cam{i}/color_corrected/image/compressed",
                f"/gt_box/alphasense_driver_node/cam{i}/color/camera_info",
            )
            for i in range(1, 6)
        ],
        False,
        DATA_DIR / "2024-10-01-11-29-55_nuc_alphasense_cor.bag",
    ),
    "hdr_front": (
        [
            (
                "/gt_box/hdr_front_rect/image_rect/compressed",
                "/gt_box/hdr_front_rect/camera_info",
            ),
        ],
        False,
        DATA_DIR / "2024-10-01-11-29-55_jetson_hdr_front_rect.bag",
    ),
    "hdr_left": (
        [
            (
                "/gt_box/hdr_left_rect/image_rect/compressed",
                "/gt_box/hdr_left_rect/camera_info",
            ),
        ],
        False,
        DATA_DIR / "2024-10-01-11-29-55_jetson_hdr_left_rect.bag",
    ),
    "hdr_right": (
        [
            (
                "/gt_box/hdr_right_rect/image_rect/compressed",
                "/gt_box/hdr_right_rect/camera_info",
            ),
        ],
        False,
        DATA_DIR / "2024-10-01-11-29-55_jetson_hdr_right_rect.bag",
    ),
    "zed2i": (
        [
            (
                "/gt_box/zed2i/zed_node/left/image_rect_color/compressed",
                "/gt_box/zed2i/zed_node/left/camera_info",
            ),
            (
                "/gt_box/zed2i/zed_node/right/image_rect_color/compressed",
                "/gt_box/zed2i/zed_node/right/camera_info",
            ),
        ],
        True,
        DATA_DIR / "2024-10-01-11-29-55_jetson_zed2i_images.bag",
    ),
}

START_TS = 1727775099
DURATION = 5


def run_debug() -> None:
    out_dir = DATA_DIR / f"out_{time.time_ns()}"
    out_dir.mkdir(exist_ok=True)
    for camera, (topics, flipped, file) in RUN_ALL.items():

        for tid, topic in enumerate(topics):
            dir_path = out_dir / f"{camera}_{tid}"
            dir_path.mkdir()
            export_images(file, dir_path, topic[0])

        continue
        image_topics = [t[0] for t in topics]
        cinfo_topics = [t[1] for t in topics]

        print(f"running {camera}...")
        frame_detections = run_model_inference(
            file,
            image_topics,
            cinfo_topics,
            start=START_TS,
            duration=DURATION,
            flip_images_for_inference=flipped,
        )

        path = out_dir / f"{camera}_frame_detections.pkl"
        with open(path, "wb") as f:
            pickle.dump(frame_detections, f)


def main() -> int:
    run_debug()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
