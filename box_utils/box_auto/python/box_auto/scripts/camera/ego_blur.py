from __future__ import annotations

import time
import warnings
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple, cast

import cv2
import numpy as np
import rosbag
import torch
import torchvision
import tqdm
from cv_bridge import CvBridge
from std_msgs.msg import Int32

from box_auto.utils import get_bag, upload_bag

warnings.filterwarnings("ignore", category=UserWarning)

MODELS_PATH = Path(__file__).parent.parent.parent.parent.parent / "models"

FACE_MODEL_SCORE_THRESHOLD = 0.4
LP_MODEL_SCORE_THRESHOLD = 0.85

BLUR_KERNEL_SIZE_QUOTIENT = 32
PIXELATION_CHUNK_SIZE = 10


class BlurMethod(Enum):
    BLUR = 0
    PIXELATE = 1


class Detectors(NamedTuple):
    face_detector: torch.jit.ScriptModule
    lp_detector: torch.jit.ScriptModule
    face_threshold: float = FACE_MODEL_SCORE_THRESHOLD
    lp_threshold: float = LP_MODEL_SCORE_THRESHOLD


def get_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def load_detector(path: Path, device: torch.device) -> torch.jit.ScriptModule:
    return torch.jit.load(str(path), map_location="cpu").to(device).eval()


def load_detectors(
    models_path: Path,
) -> Detectors:
    device = get_device()

    face_detector = load_detector(models_path / "ego_blur_face.jit", device)
    lp_detector = load_detector(models_path / "ego_blur_lp.jit", device)

    return Detectors(face_detector, lp_detector)


def get_detections(
    detector: torch.jit.ScriptModule,
    image: np.ndarray,
    model_score_threshold: float,
    nms_iou_threshold: Optional[float] = None,
) -> List[List[float]]:
    """\
    takes BRG image and returns a list of detections in the format [x1, y1, x2, y2]
    """
    image_tensor = torch.tensor(image).permute(2, 0, 1).to(get_device())

    with torch.no_grad():
        detections = detector(image_tensor)  # type: ignore

    boxes, _, scores, _ = detections  # returns boxes, labels, scores, dims

    # nms iou is not very useful for blurring
    if nms_iou_threshold is not None:
        nms_keep_idx = torchvision.ops.nms(boxes, scores, nms_iou_threshold)
        boxes = boxes[nms_keep_idx]
        scores = scores[nms_keep_idx]

    boxes = boxes.cpu().numpy()
    scores = scores.cpu().numpy()

    score_keep_idx = np.where(scores > model_score_threshold)[0]
    boxes = boxes[score_keep_idx]
    return boxes.tolist()


def create_mask_from_detections(image: np.ndarray, detections: List[List[float]]) -> np.ndarray:
    """\
    takes a list of [x1, y1, x2, y2] detections and returns a 0 1 mask the same size as the image
    """
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for bbox in detections:
        x1, y1, x2, y2 = map(int, bbox)
        mask[y1:y2, x1:x2] = 1
    return mask


def blur_mask(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """\
    applies a gaussian blur to the image where the mask is 1
    """
    h, w = mask.shape
    kh = int(h / BLUR_KERNEL_SIZE_QUOTIENT)
    if kh % 2 == 0:
        kh += 1
    kw = int(w / BLUR_KERNEL_SIZE_QUOTIENT)
    if kw % 2 == 0:
        kw += 1

    blurred_image = cv2.GaussianBlur(image, (kw, kh), 0)
    image[mask == 1] = blurred_image[mask == 1]

    return image


def pixelate_mask(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """\
    pixelates the image where the mask is 1
    """
    h, w = mask.shape
    ph = int(h / PIXELATION_CHUNK_SIZE)
    pw = int(w / PIXELATION_CHUNK_SIZE)

    pixelated_image = cv2.resize(
        cv2.resize(image, (pw, ph), interpolation=cv2.INTER_LINEAR),
        (w, h),
        interpolation=cv2.INTER_NEAREST,
    )
    image[mask == 1] = pixelated_image[mask == 1]
    return image


def draw_boxes_of_detections(image: np.ndarray, detections: List[List[float]]) -> np.ndarray:
    """\
    draws a box around each detection
    """
    image = image.copy()
    for bbox in detections:
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    return image


# used for computing the dimension of the nullspace
EPS = 1e-6


def count_number_of_detections(detections: List[List[float]]) -> int:
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

    dets_array = np.array(detections)
    N = dets_array.shape[0]

    if N == 0:
        return 0

    dets_left = dets_array.reshape((-1, N, 4))
    dets_right = dets_array.reshape((N, -1, 4))

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


def blur_and_annotated_image_given_detections(
    image: np.ndarray,
    detections: List[List[float]],
    method: BlurMethod = BlurMethod.PIXELATE,
) -> Tuple[np.ndarray, np.ndarray]:
    image = image.copy()
    mask = create_mask_from_detections(image, detections)

    if method == BlurMethod.BLUR:
        blurred_image = blur_mask(image, mask)
    elif method == BlurMethod.PIXELATE:
        blurred_image = pixelate_mask(image, mask)

    annotated_image = draw_boxes_of_detections(blurred_image, detections)

    return blurred_image, annotated_image


def blur_and_annotate_image(
    image_bgr: np.ndarray,
    flipped: bool,
    detectors: Detectors,
) -> Tuple[np.ndarray, np.ndarray, int]:
    """\
    takes and RGB image and returns a blurred and annotated image together with the number of detections
    """

    if flipped:
        image_bgr = cv2.rotate(image_bgr, cv2.ROTATE_180)

    detections = get_detections(detectors.face_detector, image_bgr, detectors.face_threshold)
    detections += get_detections(detectors.lp_detector, image_bgr, detectors.lp_threshold)

    blurred_image, annotated_image = blur_and_annotated_image_given_detections(image_bgr, detections)

    number_of_detections = count_number_of_detections(detections)

    if flipped:
        blurred_image = cv2.rotate(blurred_image, cv2.ROTATE_180)
    return blurred_image, annotated_image, number_of_detections


def load_image_from_ros1_message(
    msg: Any,
    cv_bridge: CvBridge,
) -> Tuple[np.ndarray, bool, bool]:
    """\
    load image from message, output is always an BGR image
    returns the image together with a flag if the image was grayscale and if the image was compressed
    """
    # check if the image is compressed
    compressed = "CompressedImage" in type(msg)._type

    if compressed:
        img = cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    else:
        img = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # check if the image is grayscale
    grayscale = len(img.shape) == 2
    if grayscale:
        img = np.stack([img] * 3, axis=-1)

    return img, grayscale, compressed


def save_image_to_ros1_message(
    raw: np.ndarray,
    cv_bridge: CvBridge,
    compressed: bool = True,
    grayscale: bool = True,
) -> Any:
    """\
    save a BGR image as a message, if grayscale is True the image is converted to grayscale
    """

    if grayscale:
        raw = raw[..., 0]

    if compressed:
        return cv_bridge.cv2_to_compressed_imgmsg(raw, dst_format="jpg")
    else:
        return cv_bridge.cv2_to_imgmsg(raw, encoding="passthrough")


def save_image_to_bag(
    image: np.ndarray,
    bag: rosbag.Bag,
    cv_bridge: CvBridge,
    timestamp: int,
    topic: str,
    compressed: bool,
    grayscale: bool,
    detections_count: int,
) -> None:
    """\
    writes images with corresponding metadata to an open bagfile
    """

    # write the image
    msg = save_image_to_ros1_message(image, cv_bridge, compressed=compressed, grayscale=grayscale)
    msg.header.stamp = timestamp
    bag.write(topic, msg, timestamp)

    # write number of detections
    n_dets_msg = Int32()
    n_dets_msg.data = detections_count
    bag.write(f"{topic}/n_detections", n_dets_msg, timestamp)


def anonymize_image_topics_in_bagfile(
    input_path: Path,
    output_path: Path,
    image_topics: Sequence[str],
    detectors: Detectors,
    flip_images_for_inference: bool = False,
) -> None:
    cv_bridge = CvBridge()
    with rosbag.Bag(str(input_path), "r") as in_bag, rosbag.Bag(str(output_path), "w", compression="lz4") as out_bag:
        for topic, msg, timestamp in tqdm.tqdm(
            in_bag.read_messages(),
            total=in_bag.get_message_count(),
            desc=f"processing {input_path} - {len(image_topics)} image topics",
        ):
            if topic not in image_topics:
                out_bag.write(topic, msg, timestamp)
                continue

            image, grayscale, compressed = load_image_from_ros1_message(msg, cv_bridge)

            # TODO allow storing of annotated images for debugging
            blurred_image, _, detections_count = blur_and_annotate_image(
                image,
                flip_images_for_inference,
                detectors,
            )

            save_image_to_bag(
                blurred_image,
                out_bag,
                cv_bridge,
                timestamp,
                topic,
                compressed,
                grayscale,
                detections_count,
            )


def process_bagfile(
    bag_pattern: str,
    image_topics: Sequence[str],
    flip_images_for_inference: bool,
    detectors: Detectors,
) -> None:
    input_path = Path(cast(str, get_bag(bag_pattern)))

    output_dir = input_path.parent / f"out_{time.time_ns()}"
    output_dir.mkdir(exist_ok=True)

    output_path = output_dir / f"{input_path.stem}_anonymized{input_path.suffix}"
    anonymize_image_topics_in_bagfile(input_path, output_path, image_topics, detectors, flip_images_for_inference)
    upload_bag(output_path)


CAMERA_CONFIG: Dict[str, Tuple[List[str], bool]] = {
    "*_nuc_alphasense_cor.bag": (
        [
            "/gt_box/alphasense_driver_node/cam2/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam3/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam1/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam5/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam4/color_corrected/image/compressed",
        ],
        False,
    ),
    "*_jetson_hdr_left_rect.bag": (
        [
            "/gt_box/hdr_left_rect/image_raw/compressed",
        ],
        False,
    ),
    "*_jetson_hdr_front_rect.bag": (
        [
            "/gt_box/hdr_front_rect/image_raw/compressed",
        ],
        False,
    ),
    "*_jetson_hdr_right_rect.bag": (
        [
            "/gt_box/hdr_right_rect/image_raw/compressed",
        ],
        False,
    ),
    "*_jetson_zed2i_images.bag": (
        [
            "/gt_box/zed2i/zed_node/left/image_rect_color/compressed",
            "/gt_box/zed2i/zed_node/right/image_rect_color/compressed",
        ],
        True,
    ),
}


def main() -> int:
    detectors = load_detectors(MODELS_PATH)
    for pattern, (topics, flip) in CAMERA_CONFIG.items():
        process_bagfile(pattern, topics, flip, detectors)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
