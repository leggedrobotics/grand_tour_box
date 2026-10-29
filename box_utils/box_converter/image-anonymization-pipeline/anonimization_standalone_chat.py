from __future__ import annotations

import os
import re
import warnings
from contextlib import AbstractContextManager
from enum import Enum
from logging import getLogger
from pathlib import Path
import argparse
from typing import Any, List, NamedTuple, Optional, Sequence, Tuple, Union

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
MODEL = "gen2"  # choices: gen1, gen2
OUTPUT_BAG = None  # Set to a Path to override; None defaults to <input_stem>_anon.bag next to input
HEAD = None  # Set to an int to process only the first N bag messages (for testing)

FACE_MODEL_SCORE_THRESHOLD = 0.51
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


class ModelVariant(Enum):
    GEN1 = "gen1"
    GEN2 = "gen2"


VISUALIZATION_MODE = VisualizationMode.SAVE
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


class Detection(NamedTuple):
    kind: str
    box: Tuple[float, float, float, float]
    score: float
    threshold: float


class Detectors(NamedTuple):
    device: torch.device
    model_variant: ModelVariant
    face_detector: torch.jit.ScriptModule
    lp_detector: torch.jit.ScriptModule
    scripted_instances_patch: Optional[AbstractContextManager]
    face_threshold: float = FACE_MODEL_SCORE_THRESHOLD
    lp_threshold: float = LP_MODEL_SCORE_THRESHOLD


class VisualizationState(NamedTuple):
    mode: VisualizationMode
    output_dir: Optional[Path]


InferenceInput = Union[torch.Tensor, List[dict[str, torch.Tensor]]]


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


def _validate_model_variant(model_variant: str) -> ModelVariant:
    try:
        return ModelVariant(model_variant)
    except ValueError as exc:
        allowed = [variant.value for variant in ModelVariant]
        raise ValueError(f"MODEL must be one of {allowed}, got: {model_variant}") from exc


def _model_paths(model_variant: Union[str, ModelVariant]) -> Tuple[Path, Path]:
    variant = _validate_model_variant(model_variant.value if isinstance(model_variant, ModelVariant) else model_variant)
    face_path = MODELS_PATH / f"ego_blur_face_{variant.value}.jit"
    lp_path = MODELS_PATH / f"ego_blur_lp_{variant.value}.jit"
    return face_path, lp_path


def load_detector(
    path: Path,
    device: torch.device,
    model_variant: ModelVariant,
) -> torch.jit.ScriptModule:
    if not path.is_file():
        raise FileNotFoundError(f"Model file not found: {path}")

    model = torch.jit.load(str(path), map_location="cpu")
    model = model.to(device)

    # Keep GEN1 behavior unchanged.
    if model_variant == ModelVariant.GEN1:
        model = model.eval()

    return model


def load_detectors(model_variant: str) -> Detectors:
    variant = _validate_model_variant(model_variant)
    device = get_device()
    scripted_instances_patch: Optional[AbstractContextManager] = None

    if variant == ModelVariant.GEN2:
        scripted_instances_patch = _register_detectron2_scripted_instances_patch()

    face_model_path, lp_model_path = _model_paths(variant)

    try:
        face_detector = load_detector(face_model_path, device, variant)
        lp_detector = load_detector(lp_model_path, device, variant)
    except Exception:
        if scripted_instances_patch is not None:
            scripted_instances_patch.__exit__(None, None, None)
        raise

    return Detectors(
        device=device,
        model_variant=variant,
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


def _clip_filter_boxes_and_scores(
    boxes: np.ndarray,
    scores: np.ndarray,
    height: int,
    width: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if boxes.size == 0:
        return boxes.reshape(0, 4), scores.reshape(0)

    boxes = boxes.astype(np.float32, copy=True)
    scores = scores.astype(np.float32, copy=False)

    boxes[:, 0] = np.clip(boxes[:, 0], 0, width)
    boxes[:, 2] = np.clip(boxes[:, 2], 0, width)
    boxes[:, 1] = np.clip(boxes[:, 1], 0, height)
    boxes[:, 3] = np.clip(boxes[:, 3], 0, height)

    keep = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    return boxes[keep], scores[keep]


def _prepare_inference_input(
    image: np.ndarray,
    detectors: Detectors,
) -> Tuple[InferenceInput, Tuple[int, int]]:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected image with shape (H, W, 3), got {image.shape}")

    # Scale only the detector input; the output image retains its numeric depth.
    if image.dtype == np.uint16:
        image = np.rint(image.astype(np.float64) * (255.0 / 65535.0)).astype(np.uint8)
    elif image.dtype != np.uint8:
        raise ValueError("Detector input must be uint8 or uint16")
    image_np = np.ascontiguousarray(image)

    if detectors.model_variant == ModelVariant.GEN1:
        image_tensor = torch.tensor(image_np).permute(2, 0, 1).contiguous().to(device=detectors.device)
        return image_tensor, (image.shape[0], image.shape[1])

    if detectors.model_variant == ModelVariant.GEN2:
        image_tensor = torch.from_numpy(image_np).to(device=detectors.device).permute(2, 0, 1).contiguous().float()
        return [{"image": image_tensor}], (image.shape[0], image.shape[1])

    raise ValueError(f"Unsupported model variant: {detectors.model_variant}")


def get_detections(
    detector: torch.jit.ScriptModule,
    detection_kind: str,
    inference_input: InferenceInput,
    image_shape_hw: Tuple[int, int],
    model_score_threshold: float,
    model_variant: ModelVariant,
    nms_iou_threshold: Optional[float] = None,
) -> List[Detection]:
    """
    Returns typed detections with scores and thresholds.

    GEN1 models use direct forward inference:
        detections = detector(image_tensor)

    GEN2 models use Detectron2-exported inference:
        detections = detector.inference(batched_inputs, do_postprocess=False)
    """
    with torch.no_grad():
        if model_variant == ModelVariant.GEN1:
            if not isinstance(inference_input, torch.Tensor):
                raise TypeError("GEN1 inference expects a torch.Tensor input.")
            predictions = detector(inference_input)  # type: ignore[misc]
        elif model_variant == ModelVariant.GEN2:
            if not isinstance(inference_input, list):
                raise TypeError("GEN2 inference expects batched Detectron2-style inputs.")
            if not hasattr(detector, "inference"):
                raise RuntimeError("Loaded GEN2 detector has no .inference() method.")
            predictions = detector.inference(inference_input, do_postprocess=False)
        else:
            raise ValueError(f"Unsupported model variant: {model_variant}")

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
    scores = scores[keep]

    if scores.numel() == 0:
        return []

    boxes_np = boxes.detach().cpu().numpy()
    scores_np = scores.detach().cpu().numpy()
    boxes_np, scores_np = _clip_filter_boxes_and_scores(
        boxes_np,
        scores_np,
        image_shape_hw[0],
        image_shape_hw[1],
    )

    detections: List[Detection] = []
    for box, score in zip(boxes_np, scores_np):
        detections.append(
            Detection(
                kind=detection_kind,
                box=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                score=float(score),
                threshold=float(model_score_threshold),
            )
        )
    return detections


def create_mask_from_detections(image: np.ndarray, detections: Sequence[Detection]) -> np.ndarray:
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    height, width = mask.shape

    for det in detections:
        x1, y1, x2, y2 = det.box
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


def _detection_color(kind: str) -> Tuple[int, int, int]:
    if kind == "face":
        return (0, 255, 0)  # green
    if kind == "lp":
        return (0, 165, 255)  # orange
    return (255, 0, 0)  # blue fallback


def draw_boxes_of_detections(image: np.ndarray, detections: Sequence[Detection]) -> np.ndarray:
    image = image.copy()

    for det in detections:
        x1, y1, x2, y2 = det.box
        xi1, yi1, xi2, yi2 = map(int, [round(x1), round(y1), round(x2), round(y2)])
        color = _detection_color(det.kind)

        cv2.rectangle(image, (xi1, yi1), (xi2, yi2), color, 2)

        label = f"{det.kind} p={det.score:.2f} th={det.threshold:.2f}"
        (text_w, text_h), baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1,
        )

        text_x = max(0, xi1)
        text_y = yi1 - 6
        if text_y - text_h - baseline < 0:
            text_y = min(image.shape[0] - 1, yi1 + text_h + baseline + 6)

        box_y1 = max(0, text_y - text_h - baseline - 2)
        box_y2 = min(image.shape[0], text_y + 2)
        box_x1 = max(0, text_x - 2)
        box_x2 = min(image.shape[1], text_x + text_w + 2)

        cv2.rectangle(image, (box_x1, box_y1), (box_x2, box_y2), color, thickness=-1)
        cv2.putText(
            image,
            label,
            (text_x, max(text_h, text_y - baseline)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    return image


def count_number_of_detections(detections: Sequence[Detection]) -> int:
    """
    Computes the number of connected components in the overlap graph of detections.
    Boxes that intersect or touch are considered connected.
    """
    if not detections:
        return 0

    dets = np.asarray([det.box for det in detections], dtype=np.float32)
    n = dets.shape[0]

    parent = np.arange(n, dtype=np.int32)
    rank = np.zeros(n, dtype=np.uint8)

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = int(parent[i])
        return i

    def union(i: int, j: int) -> None:
        root_i = find(i)
        root_j = find(j)
        if root_i == root_j:
            return

        if rank[root_i] < rank[root_j]:
            parent[root_i] = root_j
        elif rank[root_i] > rank[root_j]:
            parent[root_j] = root_i
        else:
            parent[root_j] = root_i
            rank[root_i] += 1

    for i in range(n):
        xi1, yi1, xi2, yi2 = dets[i]
        for j in range(i + 1, n):
            xj1, yj1, xj2, yj2 = dets[j]
            if max(xi1, xj1) <= min(xi2, xj2) and max(yi1, yj1) <= min(yi2, yj2):
                union(i, j)

    roots = {find(i) for i in range(n)}
    return len(roots)


def blur_and_annotated_image_given_detections(
    image: np.ndarray,
    detections: Sequence[Detection],
    method: BlurMethod = BlurMethod.PIXELATE,
    return_annotated: bool = True,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    image = image.copy()
    mask = create_mask_from_detections(image, detections)

    if method == BlurMethod.BLUR:
        blurred_image = blur_mask(image, mask)
    elif method == BlurMethod.PIXELATE:
        blurred_image = pixelate_mask(image, mask)
    else:
        raise ValueError(f"Unsupported blur method: {method}")

    annotated_image = draw_boxes_of_detections(blurred_image, detections) if return_annotated else None
    return blurred_image, annotated_image


def blur_and_annotate_image(
    image_bgr: np.ndarray,
    flipped: bool,
    detectors: Detectors,
    return_annotated: bool = True,
) -> Tuple[np.ndarray, Optional[np.ndarray], int]:
    if flipped:
        image_bgr = cv2.rotate(image_bgr, cv2.ROTATE_180)

    inference_input, image_shape_hw = _prepare_inference_input(image_bgr, detectors)

    detections = get_detections(
        detectors.face_detector,
        "face",
        inference_input,
        image_shape_hw,
        detectors.face_threshold,
        detectors.model_variant,
        nms_iou_threshold=NMS_IOU_THRESHOLD,
    )
    detections += get_detections(
        detectors.lp_detector,
        "lp",
        inference_input,
        image_shape_hw,
        detectors.lp_threshold,
        detectors.model_variant,
        nms_iou_threshold=NMS_IOU_THRESHOLD,
    )

    blurred_image, annotated_image = blur_and_annotated_image_given_detections(
        image_bgr,
        detections,
        return_annotated=return_annotated,
    )
    number_of_detections = count_number_of_detections(detections)

    if flipped:
        blurred_image = cv2.rotate(blurred_image, cv2.ROTATE_180)
        if annotated_image is not None:
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


def _prepare_image_for_imwrite(img: np.ndarray) -> np.ndarray:
    if img.size == 0:
        raise ValueError("Cannot save empty image.")

    if img.dtype != np.uint8:
        img = _to_uint8_for_visualization(img)

    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.ndim == 3 and img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif not (img.ndim == 3 and img.shape[2] == 3):
        raise ValueError(f"Unsupported image shape for imwrite: {img.shape}")

    return np.ascontiguousarray(img)


def _write_visualization_image(output_path: Path, panel: np.ndarray) -> None:
    panel = _prepare_image_for_imwrite(panel)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ext = output_path.suffix.lower()
    if ext == ".png":
        ok = cv2.imwrite(
            str(output_path),
            panel,
            [cv2.IMWRITE_PNG_COMPRESSION, VISUALIZE_PNG_COMPRESSION],
        )
        if ok:
            return
    else:
        ok = cv2.imwrite(
            str(output_path),
            panel,
            [cv2.IMWRITE_JPEG_QUALITY, VISUALIZE_JPEG_QUALITY],
        )
        if ok:
            return

        fallback_path = output_path.with_suffix(".png")
        ok_png = cv2.imwrite(
            str(fallback_path),
            panel,
            [cv2.IMWRITE_PNG_COMPRESSION, VISUALIZE_PNG_COMPRESSION],
        )
        if ok_png:
            logger.warning(
                "Failed to save JPEG visualization to %s. Saved PNG instead: %s",
                output_path,
                fallback_path,
            )
            return

    raise RuntimeError(
        "Failed to save visualization image. "
        f"path={output_path}, "
        f"shape={panel.shape}, dtype={panel.dtype}, "
        f"contiguous={panel.flags.c_contiguous}, "
        f"dir_exists={output_path.parent.is_dir()}, "
        f"dir_writable={os.access(output_path.parent, os.W_OK)}"
    )


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
    annotated_bgr: Optional[np.ndarray],
    *,
    topic: str,
    timestamp: int,
    detections_count: int,
) -> np.ndarray:
    before_vis = _put_label(_to_uint8_for_visualization(before_bgr), "before")
    after_vis = _put_label(_to_uint8_for_visualization(after_bgr), f"after | n={detections_count}")

    panels = [before_vis, after_vis]
    if VISUALIZE_SHOW_ANNOTATED:
        if annotated_bgr is None:
            raise ValueError("annotated_bgr is required when VISUALIZE_SHOW_ANNOTATED is enabled")
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
    annotated_bgr: Optional[np.ndarray],
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
        _write_visualization_image(output_path, panel)
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

                    do_visualize_this_frame = (
                        visualization_state.mode != VisualizationMode.NONE
                        and processed_image_count % max(1, VISUALIZE_EVERY_NTH_IMAGE) == 0
                    )
                    return_annotated = do_visualize_this_frame and VISUALIZE_SHOW_ANNOTATED

                    blurred_image, annotated_image, detections_count = blur_and_annotate_image(
                        image_bgr=image,
                        flipped=flip_images_for_inference,
                        detectors=detectors,
                        return_annotated=return_annotated,
                    )

                    if do_visualize_this_frame:
                        keep_going = visualize_before_after(
                            before_bgr=image,
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

    parser.add_argument("--model", choices=("gen1", "gen2"), default=MODEL)
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

    model_variant = _validate_model_variant(args.model)
    output_path = args.output or input_path.with_name(
        input_path.stem + ("_anon_sample.bag" if args.head else "_anon.bag")
    )

    image_topics, flip = CAMERA_TOPICS[args.camera]
    if args.topics:
        image_topics = args.topics
    detectors = load_detectors(model_variant.value)
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
    print(f"Model variant used: {model_variant.value}")
    if visualization_state.mode == VisualizationMode.SAVE and visualization_state.output_dir is not None:
        print(f"Visualization previews written to: {visualization_state.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
