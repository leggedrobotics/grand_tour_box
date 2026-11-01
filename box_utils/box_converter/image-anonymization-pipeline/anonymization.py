from __future__ import annotations

import warnings
import time
import os
import shutil
import tempfile
from argparse import ArgumentParser
from enum import Enum
from logging import getLogger
from pathlib import Path
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple
from uuid import UUID

import cv2
import kleinkram
import numpy as np
import torch
import torchvision
from kleinkram.models import Mission

from bag_anonymization import process_bag


logger = getLogger(__name__)
warnings.filterwarnings("ignore", category=UserWarning)

MODELS_PATH = Path("/models") if Path("/models").is_dir() else Path(__file__).parent / "models"
DATA_PATH = Path("/data")

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

    face_path, lp_path = gen1_model_paths(models_path)
    face_detector = load_detector(face_path, device)
    lp_detector = load_detector(lp_path, device)

    return Detectors(face_detector, lp_detector)


def gen1_model_paths(models_path: Path) -> Tuple[Path, Path]:
    """Keep the deployed Gen1 model choice; accept its original download names."""
    paths = []
    for kind in ("face", "lp"):
        candidates = [models_path / f"ego_blur_{kind}_gen1.jit", models_path / f"ego_blur_{kind}.jit"]
        path = next((candidate for candidate in candidates if candidate.is_file()), None)
        if path is None:
            raise FileNotFoundError(f"Missing Gen1 {kind} model; expected one of {candidates}")
        paths.append(path)
    return tuple(paths)


def get_detections(
    detector: torch.jit.ScriptModule,
    image: np.ndarray,
    model_score_threshold: float,
    nms_iou_threshold: Optional[float] = None,
) -> List[List[float]]:
    """\
    takes BRG image and returns a list of detections in the format [x1, y1, x2, y2]
    """
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected image with shape (H, W, 3), got {image.shape}")
    # Match the standalone scripts: scale only inference input, never output pixels.
    if image.dtype == np.uint16:
        image = np.rint(image.astype(np.float64) * (255.0 / 65535.0)).astype(np.uint8)
    elif image.dtype != np.uint8:
        raise ValueError("Detector input must be uint8 or uint16")
    image_tensor = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1).contiguous().to(get_device())

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
    height, width = mask.shape
    for bbox in detections:
        if len(bbox) != 4 or not np.isfinite(bbox).all():
            raise ValueError(f"Invalid detection box: {bbox}")
        x1, y1, x2, y2 = bbox
        x1, y1 = max(0, min(width, int(np.floor(x1)))), max(0, min(height, int(np.floor(y1))))
        x2, y2 = max(0, min(width, int(np.ceil(x2)))), max(0, min(height, int(np.ceil(y2))))
        if x2 > x1 and y2 > y1:
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
    else:
        raise ValueError(f"Unsupported blur method: {method}")

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
        annotated_image = cv2.rotate(annotated_image, cv2.ROTATE_180)
    return blurred_image, annotated_image, number_of_detections


def anonymize_image_topics_in_bagfile(
    input_path: Path,
    output_path: Path,
    *,
    image_topics: Sequence[str],
    detectors: Detectors,
    flip_images_for_inference: bool = False,
    head: Optional[int] = None,
) -> None:
    process_bag(
        input_path,
        output_path,
        image_topics=image_topics,
        head=head,
        transform=lambda image, index: blur_and_annotate_image(image, flip_images_for_inference, detectors),
        require_all_image_topics=True,
    )


AnonymizerConfig = Dict[str, Tuple[List[str], bool, str]]


FULL_CONFIG: Dict[str, AnonymizerConfig] = {
    "alphasense": {
        "{}_nuc_alphasense_color.bag": (
            [
                "/gt_box/alphasense_driver_node/cam1/color_corrected/image/compressed",
                "/gt_box/alphasense_driver_node/cam2/color_corrected/image/compressed",
                "/gt_box/alphasense_driver_node/cam3/color_corrected/image/compressed",
                "/gt_box/alphasense_driver_node/cam4/color_corrected/image/compressed",
                "/gt_box/alphasense_driver_node/cam5/color_corrected/image/compressed",
            ],
            False,
            "{}_nuc_alphasense_anon.bag",
        )
    },
    "hdr": {
        "{}_jetson_hdr_left_encoding.bag": (
            [
                "/gt_box/hdr_left/image_raw/compressed",
            ],
            False,
            "{}_jetson_hdr_left_anon.bag",
        ),
        "{}_jetson_hdr_front_encoding.bag": (
            [
                "/gt_box/hdr_front/image_raw/compressed",
            ],
            False,
            "{}_jetson_hdr_front_anon.bag",
        ),
        "{}_jetson_hdr_right_encoding.bag": (
            [
                "/gt_box/hdr_right/image_raw/compressed",
            ],
            False,
            "{}_jetson_hdr_right_anon.bag",
        ),
    },
    "zed2i": {
        "{}_jetson_zed2i_images.bag": (
            [
                "/boxi/zed2i/left/image_raw/compressed",
                "/boxi/zed2i/right/image_raw/compressed",
            ],
            False,
            "{}_jetson_zed2i_images_anon.bag",
        ),
    },
}


def get_anonymizer_config(prefix: str, camera: str) -> AnonymizerConfig:
    raw_config = FULL_CONFIG[camera]
    return {
        input_pattern.format(prefix): (topics, flip, output_pattern.format(prefix))
        for input_pattern, (topics, flip, output_pattern) in raw_config.items()
    }


def get_mission(mission_id: str) -> Mission:
    missions = kleinkram.list_missions(mission_ids=[mission_id])
    if len(missions) != 1:
        raise ValueError(f"Expected exactly one mission for {mission_id}, got {len(missions)}")
    return missions[0]


def download_files(*, config: AnonymizerConfig, mission_id: UUID, input_path: Path) -> None:
    result = kleinkram.download(
        mission_ids=[mission_id],
        file_names=list(config.keys()),
        dest=input_path,
    )
    if getattr(result, "failed", 0):
        raise RuntimeError("Mission download failed")
    missing = [name for name in config if not (input_path / name).is_file()]
    if missing:
        raise FileNotFoundError(f"Mission download did not produce expected bags: {missing}")


def _verification_status(mission_id, path: Path) -> str:
    status = kleinkram.verify(mission_id=mission_id, files=[path])[path]
    # The installed SDK returns a string Enum; older supported versions used strings.
    return getattr(status, "value", status)


def _wait_for_verification(mission_id, path, *, attempts, poll_interval, after_upload=False):
    for attempt in range(attempts):
        status = _verification_status(mission_id, path)
        pending = status in ("uploading", "computing hash") or (after_upload and status == "missing")
        if not pending:
            return status
        if attempt + 1 < attempts:
            time.sleep(poll_interval)
    raise RuntimeError(f"Upload verification timed out for {path.name}: {status}")


def upload_simple(mission_id, path, delete=False, *, attempts=60, poll_interval=5):
    """Succeed only after hash verification in the exact requested mission.

    Replacement is explicit and restricted to one exact filename. An in-progress
    upload is waited for, never deleted. Transport/API failures propagate to the job.
    """
    path = Path(path).resolve(strict=True)
    if attempts <= 0 or poll_interval < 0:
        raise ValueError("Verification requires positive attempts and a nonnegative poll interval")
    status = _wait_for_verification(mission_id, path, attempts=attempts, poll_interval=poll_interval)
    if status == "uploaded":
        return True
    if status != "missing":
        if not delete or status not in ("hash mismatch", "size mismatch", "unknown"):
            raise RuntimeError(f"Existing remote file cannot be replaced: {path.name}: {status}")
        matches = [
            file
            for file in kleinkram.list_files(mission_ids=[mission_id], file_names=[path.name])
            if file.name == path.name
        ]
        if len(matches) != 1:
            raise RuntimeError(f"Expected exactly one remote file named {path.name}, got {len(matches)}")
        remote_state = getattr(matches[0].state, "value", matches[0].state)
        if remote_state not in ("OK", "ERROR"):
            raise RuntimeError(f"Cannot replace {path.name} in remote state {remote_state}")
        kleinkram.delete_file(matches[0].id)

    result = kleinkram.upload(mission_id=mission_id, files=[path], create=False)
    if getattr(result, "failed", 0):
        raise RuntimeError(f"Upload failed for {path}")
    status = _wait_for_verification(
        mission_id,
        path,
        attempts=attempts,
        poll_interval=poll_interval,
        after_upload=True,
    )
    if status != "uploaded":
        raise RuntimeError(f"Upload verification failed for {path.name}: {status}")
    return True


def archive_output(source: Path, archive_dir: Path) -> None:
    """Keep the cluster's scratch copy without exposing a partial or overwritten bag."""
    archive_dir.mkdir(parents=True, exist_ok=True)
    destination = archive_dir / source.name
    if source.resolve() == destination.resolve():
        return
    if os.path.lexists(destination):
        raise FileExistsError(f"Archived output already exists: {destination}")
    with tempfile.TemporaryDirectory(prefix=".anonymization-copy-", dir=archive_dir) as temporary:
        staged = Path(temporary) / source.name
        shutil.copyfile(source, staged)
        os.link(staged, destination)


def upload_files(
    *, config: AnonymizerConfig, mission_id: UUID, output_path: Path, replace_existing=False, archive_dir=None
) -> None:
    for _, _, out_name in config.values():
        source = output_path / out_name
        if archive_dir is not None:
            archive_output(source, archive_dir)
        upload_simple(mission_id, source, delete=replace_existing)


def main(argv=None) -> int:
    parser = ArgumentParser(description="Anonymize and optionally upload a cloud mission using EgoBlur Gen1")
    parser.add_argument("--head", type=int, help="Process a sample; sample outputs are never uploaded")
    parser.add_argument("--mission-id", required=True)
    parser.add_argument("--cam", choices=sorted(FULL_CONFIG), required=True)
    parser.add_argument("--models", type=Path, default=MODELS_PATH)
    parser.add_argument("--data-dir", type=Path, default=DATA_PATH)
    parser.add_argument(
        "--archive-dir", type=Path, help="Keep a completed local copy before uploading (cluster scratch)"
    )
    parser.add_argument("--no-upload", action="store_true", help="Keep completed outputs locally")
    parser.add_argument("--replace-existing", action="store_true", help="Allow replacing an exact remote filename")
    args = parser.parse_args(argv)
    if args.head is not None and args.head <= 0:
        parser.error("--head must be positive")
    gen1_model_paths(args.models)
    mission = get_mission(args.mission_id)
    prefix = mission.name[4:] if mission.name.startswith("pub_") else mission.name
    config = get_anonymizer_config(prefix=prefix, camera=args.cam)
    if args.head is not None:
        config = {
            name: (topics, flip, out_name[:-4] + "_sample.bag") for name, (topics, flip, out_name) in config.items()
        }
    input_path, output_path = args.data_dir / "input", args.data_dir / "output"
    for name, (_, _, out_name) in config.items():
        if Path(name).name != name or Path(out_name).name != out_name:
            raise ValueError("Mission-derived filenames must not contain directory components")
        # Keep previous inputs and outputs intact, including after an interrupted run.
        for path in (input_path / name, output_path / out_name):
            if path.exists() or path.is_symlink():
                raise FileExistsError(f"File already exists; choose a fresh --data-dir: {path}")
    detectors = load_detectors(args.models)
    input_path.mkdir(parents=True, exist_ok=True)
    output_path.mkdir(parents=True, exist_ok=True)
    download_files(config=config, mission_id=mission.id, input_path=input_path)
    for filename, (topics, flip, out_name) in config.items():
        anonymize_image_topics_in_bagfile(
            input_path / filename,
            output_path / out_name,
            image_topics=topics,
            detectors=detectors,
            flip_images_for_inference=flip,
            head=args.head,
        )
    if args.head is None and not args.no_upload:
        upload_files(
            config=config,
            mission_id=mission.id,
            output_path=output_path,
            replace_existing=args.replace_existing,
            archive_dir=args.archive_dir,
        )
    print(f"Completed {'sample' if args.head is not None else 'full'} outputs: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
