from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Dict
from typing import Tuple
from typing import TypedDict

import yaml


class CameraIntrisics(TypedDict): ...


class FrameTransform(TypedDict): ...


FILES = []


CAMERA_INTRINSICS = {
    "hdr_front": (..., ...),
}


EXTRINSICS_FRAME_TRANSFORMS = {
    "hdr_front": (..., ...),
}


def extract_camera_intrinsics(file: Path, topic: str) -> CameraIntrisics:
    raise NotImplementedError


def extract_frame_transform(file: Path, topic: str) -> FrameTransform:
    """\
    assumes that the file and topic contain a constant
    frame tranform message that does not change over time
    """

    raise NotImplementedError


def build_extrinsics(
    base_dataset_path: Path, config: Dict[str, Tuple[Path, str]]
) -> None:
    extrinsics_path = base_dataset_path / "extrinsics"
    extrinsics_path.mkdir(exist_ok=True)

    frame_transforms_path = base_dataset_path / "frame_transforms.yaml"

    frame_transforms = {}
    for camera, (file, topic) in config.items():
        frame_transform = extract_frame_transform(file, topic)
        frame_transforms[camera] = frame_transform

    with open(frame_transforms_path, "w") as f:
        yaml.dump(frame_transforms, f)


def build_camera_intrinsics(
    base_dataset_path: Path, config: Dict[str, Tuple[Path, str]]
) -> None:
    cam_intrinsics_path = base_dataset_path / "camera_intrinsics"
    cam_intrinsics_path.mkdir(exist_ok=True)

    for camera, (file, topic) in config.items():
        camera_info = extract_camera_intrinsics(file, topic)

        with open(cam_intrinsics_path / f"{camera}.yaml", "w") as f:
            yaml.dump(camera_info, f)


def build_metadata(base_dataset_path: Path) -> None:
    assert base_dataset_path.is_dir(), f"{base_dataset_path} is not a directory"

    build_camera_intrinsics(base_dataset_path, CAMERA_INTRINSICS)
