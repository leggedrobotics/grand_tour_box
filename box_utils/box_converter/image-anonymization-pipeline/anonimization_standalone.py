from __future__ import annotations

import re
import warnings
from contextlib import AbstractContextManager
from enum import Enum
from logging import getLogger
from pathlib import Path
import argparse
from typing import Any, List, NamedTuple, Optional, Sequence, Tuple

import cv2
import numpy as np
import rosbag
import torch
import torchvision
import tqdm
from sensor_msgs.msg import CompressedImage as RosCompressedImage
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Int32 as RosInt32


logger = getLogger(__name__)
warnings.filterwarnings("ignore", category=UserWarning)

SCRIPT_DIR = Path(__file__).parent
MODELS_PATH = SCRIPT_DIR / "models"

# --- Configuration ----------------------------------------------------------------
INPUT_BAG = Path("/media/tutuna/LaCie/boxi_etna/rtna_outside_cave_1/2025-06-04-14-10-21_nuc_alphasense_color.bag")
CAM = "alphasense"  # choices: alphasense, hdr_left, hdr_front, hdr_right, zed2i
OUTPUT_BAG = None  # Set to a Path to override; None defaults to <input_stem>_anon.bag next to input
HEAD = None  # Set to an int to process only the first N bag messages (for testing)

FACE_MODEL_SCORE_THRESHOLD = 0.4
LP_MODEL_SCORE_THRESHOLD = 0.85
NMS_IOU_THRESHOLD: Optional[float] = None

BLUR_KERNEL_SIZE_QUOTIENT = 32
PIXELATION_CHUNK_SIZE = 10
EPS = 1e-6


# Hard-coded visualization feature toggle.
# Use NONE to disable, GUI for cv2.imshow, SAVE to write side-by-side preview images to disk.
class VisualizationMode(Enum):
    NONE = "none"
    GUI = "gui"
    SAVE = "save"


VISUALIZATION_MODE = VisualizationMode.NONE
VISUALIZE_SHOW_ANNOTATED = True
VISUALIZE_EVERY_NTH_IMAGE = 100
VISUALIZE_WAIT_MS = 1  # only used in GUI mode
VISUALIZE_MAX_WIDTH = 2200
VISUALIZE_WINDOW_NAME = "anonymization_preview"
VISUALIZE_OUTPUT_DIR = SCRIPT_DIR / "anonymization_preview"
VISUALIZE_OUTPUT_EXT = ".jpg"  # .jpg or .png
VISUALIZE_JPEG_QUALITY = 95
VISUALIZE_PNG_COMPRESSION = 3
VISUALIZE_GUI_FALLBACK_TO_SAVE = True

# Compressed image write params
JPEG_QUALITY = 95
PNG_COMPRESSION = 3
# ----------------------------------------------------------------------------------


class BlurMethod(Enum):
    BLUR = 0
    PIXELATE = 1


class Detectors(NamedTuple):
    device: torch.device
    face_detector: torch.jit.ScriptModule
    lp_detector: torch.jit.ScriptModule
    scripted_instances_patch: Optional[AbstractContextManager]
    face_threshold: float = FACE_MODEL_SCORE_THRESHOLD
    lp_threshold: float = LP_MODEL_SCORE_THRESHOLD


class VisualizationState(NamedTuple):
    mode: VisualizationMode
    output_dir: Optional[Path]


def get_device() -> torch.device:
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available. This script requires a CUDA-capable GPU.")
    return torch.device("cuda")


def _register_detectron2_scripted_instances_patch() -> Optional[AbstractContextManager]:
    """
    Work around the Detectron2 / TorchScript ScriptedInstances registration issue.
    Keep the returned context manager alive for the whole lifetime of the loaded models.
    """
    try:
        from detectron2.export.torchscript_patch import patch_instances
        from detectron2.structures import Boxes
    except Exception as exc:
        raise RuntimeError(
            "This EgoBlur TorchScript model needs Detectron2's scripted Instances patch "
            "at runtime. Install detectron2 in this environment, or use a non-exported model."
        ) from exc

    fields = {
        "proposal_boxes": Boxes,
        "objectness_logits": torch.Tensor,
        "pred_boxes": Boxes,
        "scores": torch.Tensor,
        "pred_classes": torch.Tensor,
    }

    ctx = patch_instances(fields)
    ctx.__enter__()
    return ctx


def load_detector(path: Path, device: torch.device) -> torch.jit.ScriptModule:
    if not path.is_file():
        raise FileNotFoundError(f"Model file not found: {path}")

    model = torch.jit.load(str(path), map_location="cpu")
    model = model.to(device)
    return model


def load_detectors() -> Detectors:
    device = get_device()
    scripted_instances_patch = _register_detectron2_scripted_instances_patch()

    try:
        face_detector = load_detector(MODELS_PATH / "ego_blur_face_gen2.jit", device)
        lp_detector = load_detector(MODELS_PATH / "ego_blur_lp_gen2.jit", device)
    except Exception:
        if scripted_instances_patch is not None:
            scripted_instances_patch.__exit__(None, None, None)
        raise

    return Detectors(
        device=device,
        face_detector=face_detector,
        lp_detector=lp_detector,
        scripted_instances_patch=scripted_instances_patch,
    )


def _as_boxes_tensor(boxes_like: Any) -> torch.Tensor:
    if isinstance(boxes_like, torch.Tensor):
        return boxes_like
    if hasattr(boxes_like, "tensor"):
        tensor = boxes_like.tensor
        if isinstance(tensor, torch.Tensor):
            return tensor
    raise TypeError(f"Unsupported boxes type: {type(boxes_like)}")


def _extract_boxes_scores_from_instances(instances_like: Any) -> Tuple[torch.Tensor, torch.Tensor]:
    if hasattr(instances_like, "pred_boxes") and hasattr(instances_like, "scores"):
        boxes = _as_boxes_tensor(instances_like.pred_boxes)
        scores = instances_like.scores
        if not isinstance(scores, torch.Tensor):
            raise TypeError(f"Unsupported scores type: {type(scores)}")
        return boxes, scores

    if hasattr(instances_like, "get_fields"):
        fields = instances_like.get_fields()
        if isinstance(fields, dict) and "pred_boxes" in fields and "scores" in fields:
            boxes = _as_boxes_tensor(fields["pred_boxes"])
            scores = fields["scores"]
            if not isinstance(scores, torch.Tensor):
                raise TypeError(f"Unsupported scores type: {type(scores)}")
            return boxes, scores

    if hasattr(instances_like, "to_instances"):
        eager_instances = instances_like.to_instances()
        if hasattr(eager_instances, "pred_boxes") and hasattr(eager_instances, "scores"):
            boxes = _as_boxes_tensor(eager_instances.pred_boxes)
            scores = eager_instances.scores
            if not isinstance(scores, torch.Tensor):
                raise TypeError(f"Unsupported scores type: {type(scores)}")
            return boxes, scores

    raise TypeError(f"Unsupported instances type: {type(instances_like)}")


def _extract_boxes_scores(prediction: Any) -> Tuple[torch.Tensor, torch.Tensor]:
    if isinstance(prediction, dict):
        if "instances" in prediction:
            return _extract_boxes_scores_from_instances(prediction["instances"])
        if "pred_boxes" in prediction and "scores" in prediction:
            boxes = _as_boxes_tensor(prediction["pred_boxes"])
            scores = prediction["scores"]
            if not isinstance(scores, torch.Tensor):
                raise TypeError(f"Unsupported scores type: {type(scores)}")
            return boxes, scores

    if isinstance(prediction, (list, tuple)):
        if len(prediction) == 0:
            return (
                torch.empty((0, 4), dtype=torch.float32),
                torch.empty((0,), dtype=torch.float32),
            )

        if len(prediction) == 1:
            return _extract_boxes_scores(prediction[0])

        if len(prediction) >= 3 and isinstance(prediction[0], torch.Tensor) and isinstance(prediction[2], torch.Tensor):
            return prediction[0], prediction[2]

    return _extract_boxes_scores_from_instances(prediction)


def _clip_and_filter_boxes(boxes: np.ndarray, height: int, width: int) -> np.ndarray:
    if boxes.size == 0:
        return boxes.reshape(0, 4)

    boxes = boxes.astype(np.float32, copy=True)
    boxes[:, 0] = np.clip(boxes[:, 0], 0, width)
    boxes[:, 2] = np.clip(boxes[:, 2], 0, width)
    boxes[:, 1] = np.clip(boxes[:, 1], 0, height)
    boxes[:, 3] = np.clip(boxes[:, 3], 0, height)

    keep = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    return boxes[keep]


def get_detections(
    detector: torch.jit.ScriptModule,
    image: np.ndarray,
    model_score_threshold: float,
    device: torch.device,
    nms_iou_threshold: Optional[float] = None,
) -> List[List[float]]:
    """
    Takes a BGR image of shape (H, W, 3) and returns detections as [x1, y1, x2, y2].
    """
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected image with shape (H, W, 3), got {image.shape}")

    if image.dtype == np.uint16:
        image = np.rint(image.astype(np.float64) * (255.0 / 65535.0)).astype(np.uint8)
    elif image.dtype != np.uint8:
        raise ValueError("Detector input must be uint8 or uint16")

    image_tensor = torch.from_numpy(np.ascontiguousarray(image)).to(device=device).permute(2, 0, 1).contiguous().float()

    batched_inputs = [{"image": image_tensor}]

    with torch.no_grad():
        if not hasattr(detector, "inference"):
            raise RuntimeError("Loaded detector has no .inference() method.")
        predictions = detector.inference(batched_inputs, do_postprocess=False)

    boxes, scores = _extract_boxes_scores(predictions)
    boxes = boxes.to(dtype=torch.float32)
    scores = scores.to(dtype=torch.float32)

    if nms_iou_threshold is not None and boxes.numel() > 0:
        keep = torchvision.ops.nms(boxes, scores, nms_iou_threshold)
        boxes = boxes[keep]
        scores = scores[keep]

    if scores.numel() == 0:
        return []

    keep = scores > float(model_score_threshold)
    boxes = boxes[keep]

    boxes_np = boxes.detach().cpu().numpy()
    boxes_np = _clip_and_filter_boxes(boxes_np, image.shape[0], image.shape[1])
    return boxes_np.tolist()


def create_mask_from_detections(image: np.ndarray, detections: List[List[float]]) -> np.ndarray:
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    height, width = mask.shape

    for bbox in detections:
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(width, int(np.floor(x1))))
        y1 = max(0, min(height, int(np.floor(y1))))
        x2 = max(0, min(width, int(np.ceil(x2))))
        y2 = max(0, min(height, int(np.ceil(y2))))
        if x2 > x1 and y2 > y1:
            mask[y1:y2, x1:x2] = 1

    return mask


def blur_mask(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    h, w = mask.shape

    kh = max(1, int(h / BLUR_KERNEL_SIZE_QUOTIENT))
    kw = max(1, int(w / BLUR_KERNEL_SIZE_QUOTIENT))
    if kh % 2 == 0:
        kh += 1
    if kw % 2 == 0:
        kw += 1

    blurred_image = cv2.GaussianBlur(image, (kw, kh), 0)
    image[mask == 1] = blurred_image[mask == 1]
    return image


def pixelate_mask(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    h, w = mask.shape
    ph = max(1, int(h / PIXELATION_CHUNK_SIZE))
    pw = max(1, int(w / PIXELATION_CHUNK_SIZE))

    pixelated_image = cv2.resize(
        cv2.resize(image, (pw, ph), interpolation=cv2.INTER_LINEAR),
        (w, h),
        interpolation=cv2.INTER_NEAREST,
    )
    image[mask == 1] = pixelated_image[mask == 1]
    return image


def draw_boxes_of_detections(image: np.ndarray, detections: List[List[float]]) -> np.ndarray:
    image = image.copy()
    for bbox in detections:
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    return image


def count_number_of_detections(detections: List[List[float]]) -> int:
    """
    Computes the number of detections by computing the connected components
    of the union of all detections.
    """
    if not detections:
        return 0

    dets_array = np.asarray(detections, dtype=np.float32)
    n = dets_array.shape[0]

    dets_left = dets_array.reshape((1, n, 4))
    dets_right = dets_array.reshape((n, 1, 4))

    x_max = np.maximum(dets_left[:, :, 0], dets_right[:, :, 0])
    x_min = np.minimum(dets_left[:, :, 2], dets_right[:, :, 2])

    y_max = np.maximum(dets_left[:, :, 1], dets_right[:, :, 1])
    y_min = np.minimum(dets_left[:, :, 3], dets_right[:, :, 3])

    intersect = (x_max <= x_min) & (y_max <= y_min)

    adjacency = intersect.astype(np.int32)
    degree = np.diag(np.sum(adjacency, axis=1))
    laplacian = degree - adjacency
    eigenvalues = np.linalg.eigvals(laplacian)
    return int(np.sum(np.real(eigenvalues) < EPS))


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
    else:
        raise ValueError(f"Unsupported blur method: {method}")

    annotated_image = draw_boxes_of_detections(blurred_image, detections)
    return blurred_image, annotated_image


def blur_and_annotate_image(
    image_bgr: np.ndarray,
    flipped: bool,
    detectors: Detectors,
) -> Tuple[np.ndarray, np.ndarray, int]:
    if flipped:
        image_bgr = cv2.rotate(image_bgr, cv2.ROTATE_180)

    detections = get_detections(
        detectors.face_detector,
        image_bgr,
        detectors.face_threshold,
        detectors.device,
        nms_iou_threshold=NMS_IOU_THRESHOLD,
    )
    detections += get_detections(
        detectors.lp_detector,
        image_bgr,
        detectors.lp_threshold,
        detectors.device,
        nms_iou_threshold=NMS_IOU_THRESHOLD,
    )

    blurred_image, annotated_image = blur_and_annotated_image_given_detections(image_bgr, detections)
    number_of_detections = count_number_of_detections(detections)

    if flipped:
        blurred_image = cv2.rotate(blurred_image, cv2.ROTATE_180)
        annotated_image = cv2.rotate(annotated_image, cv2.ROTATE_180)

    return blurred_image, annotated_image, number_of_detections


# Shared codecs preserve row padding semantics, byte order, bit depth and alpha.
from image_codec import decode_image_message, _reencode_compressed_image, _encode_raw_image_msg


def _to_uint8_for_visualization(img: np.ndarray) -> np.ndarray:
    if img.dtype == np.uint8:
        out = img.copy()
    elif np.issubdtype(img.dtype, np.integer):
        info = np.iinfo(img.dtype)
        scale = 255.0 / float(max(1, info.max))
        out = np.clip(img.astype(np.float32) * scale, 0.0, 255.0).astype(np.uint8)
    elif np.issubdtype(img.dtype, np.floating):
        finite = img[np.isfinite(img)]
        if finite.size == 0:
            out = np.zeros_like(img, dtype=np.uint8)
        else:
            lo = float(np.min(finite))
            hi = float(np.max(finite))
            if hi <= lo:
                out = np.zeros_like(img, dtype=np.uint8)
            else:
                out = np.clip((img - lo) * (255.0 / (hi - lo)), 0.0, 255.0).astype(np.uint8)
    else:
        raise ValueError(f"Unsupported dtype for visualization: {img.dtype}")

    if out.ndim == 2:
        return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if out.ndim == 3 and out.shape[2] == 4:
        return cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)
    if out.ndim == 3 and out.shape[2] == 3:
        return out
    raise ValueError(f"Unsupported image shape for visualization: {out.shape}")


def _put_label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    banner_h = 36
    cv2.rectangle(out, (0, 0), (out.shape[1], banner_h), (0, 0, 0), thickness=-1)
    cv2.putText(
        out,
        text,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return out


def _resize_to_same_height(images: List[np.ndarray]) -> List[np.ndarray]:
    target_h = min(img.shape[0] for img in images)
    resized: List[np.ndarray] = []

    for img in images:
        if img.shape[0] == target_h:
            resized.append(img)
            continue
        scale = target_h / float(img.shape[0])
        target_w = max(1, int(round(img.shape[1] * scale)))
        resized.append(cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA))

    return resized


def _sanitize_filename_fragment(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", text.strip("/"))


def _build_visualization_panel(
    before_bgr: np.ndarray,
    after_bgr: np.ndarray,
    annotated_bgr: np.ndarray,
    *,
    topic: str,
    timestamp: int,
    detections_count: int,
) -> np.ndarray:
    before_vis = _put_label(_to_uint8_for_visualization(before_bgr), "before")
    after_vis = _put_label(_to_uint8_for_visualization(after_bgr), f"after | n={detections_count}")

    panels = [before_vis, after_vis]
    if VISUALIZE_SHOW_ANNOTATED:
        annotated_vis = _put_label(_to_uint8_for_visualization(annotated_bgr), "annotated")
        panels.append(annotated_vis)

    panels = _resize_to_same_height(panels)
    panel = np.hstack(panels)

    footer_h = 34
    footer = np.zeros((footer_h, panel.shape[1], 3), dtype=np.uint8)
    footer_text = f"{topic} | t={timestamp}"
    cv2.putText(
        footer,
        footer_text,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    panel = np.vstack([panel, footer])

    if VISUALIZE_MAX_WIDTH is not None and panel.shape[1] > VISUALIZE_MAX_WIDTH:
        scale = VISUALIZE_MAX_WIDTH / float(panel.shape[1])
        target_w = max(1, int(round(panel.shape[1] * scale)))
        target_h = max(1, int(round(panel.shape[0] * scale)))
        panel = cv2.resize(panel, (target_w, target_h), interpolation=cv2.INTER_AREA)

    return panel


def _highgui_is_available() -> bool:
    try:
        test_img = np.zeros((8, 8, 3), dtype=np.uint8)
        cv2.namedWindow("__highgui_test__", cv2.WINDOW_NORMAL)
        cv2.imshow("__highgui_test__", test_img)
        cv2.waitKey(1)
        cv2.destroyWindow("__highgui_test__")
        return True
    except cv2.error:
        return False


def initialize_visualization() -> VisualizationState:
    mode = VISUALIZATION_MODE

    if mode == VisualizationMode.NONE:
        return VisualizationState(mode=mode, output_dir=None)

    if mode == VisualizationMode.GUI:
        if _highgui_is_available():
            return VisualizationState(mode=mode, output_dir=None)
        if not VISUALIZE_GUI_FALLBACK_TO_SAVE:
            raise RuntimeError(
                "VISUALIZATION_MODE=GUI but this OpenCV build has no HighGUI backend. "
                "Use VisualizationMode.SAVE or rebuild OpenCV with GUI support."
            )
        output_dir = VISUALIZE_OUTPUT_DIR.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        logger.warning(
            "OpenCV HighGUI is unavailable. Falling back from GUI visualization to SAVE mode: %s",
            output_dir,
        )
        return VisualizationState(mode=VisualizationMode.SAVE, output_dir=output_dir)

    if mode == VisualizationMode.SAVE:
        output_dir = VISUALIZE_OUTPUT_DIR.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        return VisualizationState(mode=mode, output_dir=output_dir)

    raise ValueError(f"Unsupported visualization mode: {mode}")


def finalize_visualization(state: VisualizationState) -> None:
    if state.mode == VisualizationMode.GUI:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def visualize_before_after(
    before_bgr: np.ndarray,
    after_bgr: np.ndarray,
    annotated_bgr: np.ndarray,
    *,
    topic: str,
    timestamp: int,
    detections_count: int,
    frame_index: int,
    state: VisualizationState,
) -> bool:
    if state.mode == VisualizationMode.NONE:
        return True

    panel = _build_visualization_panel(
        before_bgr=before_bgr,
        after_bgr=after_bgr,
        annotated_bgr=annotated_bgr,
        topic=topic,
        timestamp=timestamp,
        detections_count=detections_count,
    )

    if state.mode == VisualizationMode.GUI:
        cv2.imshow(VISUALIZE_WINDOW_NAME, panel)
        key = cv2.waitKey(VISUALIZE_WAIT_MS) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return False
        return True

    if state.mode == VisualizationMode.SAVE:
        assert state.output_dir is not None
        topic_part = _sanitize_filename_fragment(topic)
        ext = VISUALIZE_OUTPUT_EXT.lower()

        output_path = state.output_dir / f"{frame_index:08d}_{topic_part}_t{timestamp}{ext}"
        if ext == ".png":
            ok = cv2.imwrite(
                str(output_path),
                panel,
                [cv2.IMWRITE_PNG_COMPRESSION, VISUALIZE_PNG_COMPRESSION],
            )
        else:
            ok = cv2.imwrite(
                str(output_path),
                panel,
                [cv2.IMWRITE_JPEG_QUALITY, VISUALIZE_JPEG_QUALITY],
            )
        if not ok:
            raise RuntimeError(f"Failed to save visualization image: {output_path}")
        return True

    raise ValueError(f"Unsupported visualization mode: {state.mode}")


def anonymize_image_topics_in_bagfile(
    input_path: Path,
    output_path: Path,
    *,
    image_topics: Sequence[str],
    detectors: Detectors,
    visualization_state: VisualizationState,
    flip_images_for_inference: bool = False,
    head: Optional[int] = None,
) -> None:
    image_topics_set = set(image_topics)

    if output_path.exists():
        raise FileExistsError(f"Output bag already exists: {output_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(input_path), "r") as in_bag:
        topic_type_map = {topic: info.msg_type for topic, info in in_bag.get_type_and_topic_info().topics.items()}
        active_image_topics = image_topics_set & set(topic_type_map)
        if not active_image_topics:
            raise ValueError(f"None of the selected image topics exist in {input_path}: {sorted(image_topics_set)}")
        missing_topics = image_topics_set - active_image_topics
        if missing_topics:
            raise ValueError(f"Selected image topics are missing: {sorted(missing_topics)}")
        if head is not None and head <= 0:
            raise ValueError("head must be positive when provided")
        total = in_bag.get_message_count()
        processed_image_count = 0
        processed_topics = set()

        import os
        import tempfile

        with tempfile.TemporaryDirectory(prefix=".anonymizing-", dir=output_path.parent) as temporary_dir:
            temporary_output = Path(temporary_dir) / "output.bag"
            with rosbag.Bag(str(temporary_output), "w", compression="lz4") as out_bag:
                for idx, (topic, msg, t) in tqdm.tqdm(
                    enumerate(in_bag.read_messages()),
                    total=total,
                    desc=f"processing {input_path.name} - {len(active_image_topics)} image topics",
                ):
                    if head is not None and idx >= head:
                        break

                    if topic not in image_topics_set:
                        out_bag.write(topic, msg, t)
                        continue

                    msgtype = topic_type_map[topic]
                    image, grayscale, is_compressed = decode_image_message(msg, msgtype)
                    image_before = image.copy()

                    blurred_image, annotated_image, detections_count = blur_and_annotate_image(
                        image_bgr=image,
                        flipped=flip_images_for_inference,
                        detectors=detectors,
                    )

                    if (
                        visualization_state.mode != VisualizationMode.NONE
                        and processed_image_count % max(1, VISUALIZE_EVERY_NTH_IMAGE) == 0
                    ):
                        keep_going = visualize_before_after(
                            before_bgr=image_before,
                            after_bgr=blurred_image,
                            annotated_bgr=annotated_image,
                            topic=topic,
                            timestamp=t.to_nsec(),
                            detections_count=detections_count,
                            frame_index=processed_image_count,
                            state=visualization_state,
                        )
                        if not keep_going:
                            raise KeyboardInterrupt("Visualization interrupted by user.")

                    if is_compressed:
                        new_msg = RosCompressedImage()
                        new_msg.header = msg.header
                        new_msg.format = msg.format
                        new_msg.data = _reencode_compressed_image(
                            blurred_image,
                            grayscale,
                            msg.format,
                            original_msg=msg,
                            jpeg_quality=JPEG_QUALITY,
                            png_compression=PNG_COMPRESSION,
                        )
                        out_bag.write(topic, new_msg, t)
                    else:
                        out_bag.write(topic, _encode_raw_image_msg(blurred_image, msg), t)

                    out_bag.write(f"{topic}/n_detections", RosInt32(data=int(detections_count)), t)
                    processed_image_count += 1
                    processed_topics.add(topic)
            if processed_image_count == 0:
                raise ValueError("No images were processed; output bag was not published")
            if processed_topics != active_image_topics:
                raise ValueError("Not every selected topic had a processed image; output was not published")
            os.link(temporary_output, output_path)


CAMERA_TOPICS: dict[str, Tuple[List[str], bool]] = {
    "alphasense": (
        [
            "/gt_box/alphasense_driver_node/cam1/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam2/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam3/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam4/color_corrected/image/compressed",
            "/gt_box/alphasense_driver_node/cam5/color_corrected/image/compressed",
        ],
        False,
    ),
    "hdr_left": (
        ["/gt_box/hdr_left/image_raw/compressed"],
        False,
    ),
    "hdr_front": (
        ["/gt_box/hdr_front/image_raw/compressed"],
        False,
    ),
    "hdr_right": (
        ["/gt_box/hdr_right/image_raw/compressed"],
        False,
    ),
    "zed2i": (
        [
            "/boxi/zed2i/left/image_raw/compressed",
            "/boxi/zed2i/right/image_raw/compressed",
        ],
        False,
    ),
}


def main(argv=None) -> int:
    global MODELS_PATH, VISUALIZATION_MODE, VISUALIZE_OUTPUT_DIR
    parser = argparse.ArgumentParser(description="Anonymize explicitly selected image streams")
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--camera", choices=sorted(CAMERA_TOPICS), default=CAM)
    parser.add_argument("--topics", nargs="+", help="Override the camera preset with exact image topics")
    parser.add_argument("--models", type=Path, default=MODELS_PATH)
    parser.add_argument("--face-threshold", type=float, default=FACE_MODEL_SCORE_THRESHOLD)
    parser.add_argument("--plate-threshold", type=float, default=LP_MODEL_SCORE_THRESHOLD)
    parser.add_argument("--head", type=int, help="Write only a sample containing this many bag messages")
    parser.add_argument("--preview-dir", type=Path, help="Write previews here; default is no preview output")

    args = parser.parse_args(argv)
    if not 0 <= args.face_threshold <= 1 or not 0 <= args.plate_threshold <= 1:
        parser.error("Detector thresholds must lie in [0,1]")
    MODELS_PATH = args.models.resolve()
    VISUALIZATION_MODE = VisualizationMode.SAVE if args.preview_dir else VisualizationMode.NONE
    if args.preview_dir is not None:
        VISUALIZE_OUTPUT_DIR = args.preview_dir.resolve()
    input_path = args.input.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Input bag not found: {input_path}")
    if CAM not in CAMERA_TOPICS:
        raise ValueError(f"CAM must be one of {list(CAMERA_TOPICS.keys())}")

    output_path = args.output or input_path.with_name(
        input_path.stem + ("_anon_sample.bag" if args.head else "_anon.bag")
    )

    image_topics, flip = CAMERA_TOPICS[args.camera]
    if args.topics:
        image_topics = args.topics
    detectors = load_detectors()
    detectors = detectors._replace(face_threshold=args.face_threshold, lp_threshold=args.plate_threshold)
    visualization_state = initialize_visualization()

    try:
        anonymize_image_topics_in_bagfile(
            input_path=input_path,
            output_path=output_path,
            image_topics=image_topics,
            detectors=detectors,
            visualization_state=visualization_state,
            flip_images_for_inference=flip,
            head=args.head,
        )
    finally:
        finalize_visualization(visualization_state)
        if detectors.scripted_instances_patch is not None:
            detectors.scripted_instances_patch.__exit__(None, None, None)

    print(f"Done. Output written to: {output_path}")
    if visualization_state.mode == VisualizationMode.SAVE and visualization_state.output_dir is not None:
        print(f"Visualization previews written to: {visualization_state.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
