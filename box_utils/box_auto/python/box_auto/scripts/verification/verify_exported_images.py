#!/usr/bin/env python3
"""Validate exported image, calibration, pose and timestamp contracts, then optionally view.

Depth units come from camera_info.json; associations come from frames.csv.
An overlay is drawn only when depth and RGB share timestamps and pixel geometry.
Run with DATASET --validate-only for headless validation without Rerun.
"""

from __future__ import annotations

import json
import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from uuid import uuid4

import numpy as np

try:
    import cv2
except Exception as e:
    print(f"ERROR: Failed to import OpenCV (cv2): {e}", file=sys.stderr)
    print("ERROR: Install with: pip install opencv-python", file=sys.stderr)
    raise

# Load the optional viewer only after numerical validation. Some SDK versions
# require a newer Python than ROS Noetic, even when import succeeds initially.
rr = None


# =========================
# VIEWER PREFERENCES
# =========================
APP_ID = "zed2i_all_data_verification"
SPAWN_VIEWER = True

# Always reset viewer layout on startup
FRESH_RECORDING_EVERY_RUN = True
SEND_BLUEPRINT_EVERY_RUN = True

# World + entities
ENTITY_WORLD = "world"
ENTITY_RIGHT_CAM = "world/camera_right"
ENTITY_LEFT_CAM = "world/camera_left"
ENTITY_STEREO = "world/stereo"
ENTITY_RIGHT_TRAJ = "world/trajectory_right"
ENTITY_LEFT_TRAJ = "world/trajectory_left"

# 2D entities
ENTITY_RIGHT_RGB = "world/right/rgb"
ENTITY_LEFT_RGB = "world/left/rgb"
ENTITY_DEPTH_COLOR = "world/right/depth_color"
ENTITY_DEPTH_COLOR_CONF = "world/right/depth_color_conf"
ENTITY_RGB_DEPTH_OVERLAY = "world/right/rgb_depth_overlay"
ENTITY_CONFIDENCE = "world/right/confidence"

# Pose convention in traj_*.txt
POSE_IS_WORLD_FROM_CAMERA = True  # True: traj line is T_world_camera; False: T_camera_world and will be inverted.

# 3D visualization
LOG_CAMERA_AXES = True
AXES_LENGTH = 0.2  # meters
LOG_CAMERA_FRUSTUM = True
FRUSTUM_SCALE = 0.35
DRAW_BASELINE_LINE = True

# Depth visualization / overlay
DEPTH_RANGE_M = (0.2, 15.0)  # primary clamp + validity range
AUTO_DEPTH_RANGE_FALLBACK = True  # if clamp yields almost no pixels, auto-range (percentiles) for viz
AUTO_RANGE_PERCENTILES = (5.0, 95.0)

OVERLAY_ALPHA = 0.85  # base alpha for blending depth over RGB (on valid pixels)
MODULATE_OVERLAY_BY_CONF = True
CONF_GAMMA = 1.0  # alpha *= conf**gamma (after CONF_MIN_ALPHA floor)
CONF_MIN_ALPHA = 0.80  # ensures overlay stays visible even if conf ~ 0
COLORMAP_NAME = "TURBO"  # TURBO preferred; fallback to JET if unavailable

# Matching behavior
# Timestamp manifests join independently sampled streams. An image index alone
# does not establish that a depth map or pose belongs to the same exposure.
# =========================


def eprint(msg: str) -> None:
    print(msg, file=sys.stderr)


def wprint(msg: str) -> None:
    print(msg, file=sys.stderr)


def load_camera_info(json_path: Path, camera="right") -> Tuple[int, int, float, float, float, float]:
    if not json_path.exists():
        raise FileNotFoundError(f"camera_info.json not found: {json_path}")

    data = json.loads(json_path.read_text())
    if camera not in data or not isinstance(data[camera], dict):
        raise RuntimeError(f"Invalid camera_info.json structure (missing requested camera dict): {json_path}")

    cam = data[camera]
    required = ["w", "h", "fx", "fy", "cx", "cy"]
    missing = [k for k in required if k not in cam]
    if missing:
        raise RuntimeError(f"camera_info.json missing keys {missing}: {json_path}")

    w = int(cam["w"])
    h = int(cam["h"])
    fx = float(cam["fx"])
    fy = float(cam["fy"])
    cx = float(cam["cx"])
    cy = float(cam["cy"])

    if w <= 0 or h <= 0 or not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise RuntimeError(f"Invalid image size in camera_info.json: w={w}, h={h}")

    return w, h, fx, fy, cx, cy


def read_traj_txt(traj_path: Path) -> List[np.ndarray]:
    poses = []
    for number, line in enumerate(traj_path.read_text().splitlines(), 1):
        try:
            values = np.array([float(value) for value in line.split()], dtype=float).reshape(4, 4)
        except ValueError as exc:
            raise ValueError(f"Malformed pose at {traj_path}:{number}") from exc
        rotation = values[:3, :3]
        if (
            not np.isfinite(values).all()
            or not np.allclose(values[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)
        ):
            raise ValueError(f"Invalid rigid pose at {traj_path}:{number}")
        # Never skip a malformed row: doing so shifts every later image/pose pair.
        poses.append(values)
    if not poses:
        raise ValueError(f"No poses in {traj_path}")
    return poses


def load_export_manifest(root):
    streams = {}
    with (root / "frames.csv").open(newline="") as source:
        for row in csv.DictReader(source):
            stream = row["stream"]
            if stream not in ("right", "left", "depth", "confidence"):
                raise ValueError(f"Unknown stream in frames.csv: {stream}")
            index, stamp = int(row["index"]), int(row["stamp_ns"])
            file = (root / row["file"]).resolve()
            if root.resolve() not in file.parents or not file.is_file() or index < 0 or stamp <= 0:
                raise ValueError(f"Invalid frame entry: {row}")
            entries = streams.setdefault(stream, {})
            if index in entries or any(entry["stamp_ns"] == stamp for entry in entries.values()):
                raise ValueError(f"Duplicate frame index/timestamp in {stream}")
            entries[index] = dict(
                path=file, stamp_ns=stamp, frame_id=row["frame_id"], width=int(row["width"]), height=int(row["height"])
            )
    for stream, entries in streams.items():
        if sorted(entries) != list(range(len(entries))):
            raise ValueError(f"Non-contiguous frame indices in {stream}")
        stamps = [entries[i]["stamp_ns"] for i in range(len(entries))]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError(f"Non-increasing frame timestamps in {stream}")
    return streams


def validate_export(root, streams, metadata, poses_r, poses_l):
    for side, poses in (("right", poses_r), ("left", poses_l)):
        if len(poses) != len(streams.get(side, {})):
            raise ValueError(f"{side}: pose/image counts differ")
        load_camera_info(root / "camera_info.json", side)
    if [r["stamp_ns"] for r in streams["right"].values()] != [r["stamp_ns"] for r in streams["left"].values()]:
        raise ValueError("Left and right acquisition timestamps differ")
    for stream, entries in streams.items():
        calibration = metadata["depth" if stream == "confidence" else stream]
        for entry in entries.values():
            pixels = read_any_mat(entry["path"])
            if (
                pixels.shape[:2] != (calibration["h"], calibration["w"])
                or (entry["height"], entry["width"]) != pixels.shape[:2]
                or entry["frame_id"] != calibration["frame_id"]
            ):
                raise ValueError(f"{stream}: pixel dimensions/frame disagree with calibration")
            if stream == "depth":
                depth_to_meters(pixels, metadata["depth_scale"])
            elif stream == "confidence":
                confidence_to_01(pixels)


def pixel_aligned_camera(metadata):
    depth = metadata.get("depth")
    if not depth:
        return None
    for side in ("left", "right"):
        camera = metadata[side]
        if (
            camera.get("rectified")
            and depth.get("rectified")
            and camera["frame_id"] == depth["frame_id"]
            and all(np.isclose(camera[key], depth[key]) for key in ("w", "h", "fx", "fy", "cx", "cy"))
        ):
            return side
    return None


def invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Rt = R.T
    ti = -Rt @ t
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = Rt
    Ti[:3, 3] = ti
    return Ti


def rotmat_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    # Project to SO(3)
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt

    m = Rn
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n <= 0:
        raise ValueError("Quaternion normalization failed")
    q /= n
    return q


def rr_set_frame_time(i: int) -> None:
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("frame", i)
    else:
        rr.set_time("frame", sequence=i)


def rr_make_pinhole(w: int, h: int, fx: float, fy: float, cx: float, cy: float):
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
    try:
        return rr.Pinhole(image_from_camera=K, width=w, height=h)
    except TypeError:
        return rr.Pinhole(
            resolution=(w, h),
            focal_length=(fx, fy),
            principal_point=(cx, cy),
        )


def rr_log_transform_parent_from_child(entity: str, t_xyz: np.ndarray, q_xyzw: np.ndarray) -> None:
    t_xyz_f = (float(t_xyz[0]), float(t_xyz[1]), float(t_xyz[2]))
    q_xyzw_f = (float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]), float(q_xyzw[3]))

    try:
        rr.log(
            entity,
            rr.Transform3D(
                translation=t_xyz_f,
                rotation=rr.Quaternion(xyzw=q_xyzw_f),
                relation=rr.TransformRelation.ParentFromChild,
            ),
        )
        return
    except TypeError:
        pass

    rr.log(
        entity,
        rr.Transform3D(
            translation=t_xyz_f,
            rotation=rr.Quaternion(xyzw=q_xyzw_f),
            from_parent=True,
        ),
    )


def log_axes(entity: str, length: float) -> None:
    try:
        rr.log(entity, rr.AxisLength(length))
    except Exception as e:
        wprint(f"WARNING: Failed to log rr.AxisLength on '{entity}': {e}")


def log_frustum(entity_cam: str, scale: float) -> None:
    if not LOG_CAMERA_FRUSTUM:
        return
    try:
        rr.log(entity_cam, rr.Frustum3D(scale=scale))
        return
    except Exception:
        pass
    try:
        rr.log(entity_cam, rr.Frustum(scale=scale))  # type: ignore[attr-defined]
        return
    except Exception:
        pass


def log_baseline_line(t_right: np.ndarray, t_left: np.ndarray) -> None:
    if not DRAW_BASELINE_LINE:
        return
    try:
        pts = np.array(
            [[t_right[0], t_right[1], t_right[2]], [t_left[0], t_left[1], t_left[2]]],
            dtype=np.float32,
        )
        rr.log(ENTITY_STEREO, rr.LineStrips3D([pts]))
    except Exception as e:
        wprint(f"WARNING: Failed to log stereo baseline line: {e}")


def _index_by_regex(dir_path: Path, rx: re.Pattern, kind: str) -> Dict[int, Path]:
    if not dir_path.exists() or not dir_path.is_dir():
        raise FileNotFoundError(f"{kind} directory not found: {dir_path}")

    mapping: Dict[int, Path] = {}
    for p in dir_path.iterdir():
        if not p.is_file():
            continue
        m = rx.match(p.name)
        if not m:
            continue
        idx = int(m.group(1))
        if idx in mapping:
            wprint(f"WARNING: Duplicate {kind} index {idx}: {mapping[idx]} AND {p}. Keeping first.")
            continue
        mapping[idx] = p
    return mapping


def index_dataset(root: Path) -> Tuple[Dict[int, Path], Dict[int, Path], Dict[int, Path], Dict[int, Path]]:
    right_dir = root / "images" / "right"
    left_dir = root / "images" / "left"
    depth_dir = root / "depth_images"
    conf_dir = root / "confidence_images"

    rx_frame = re.compile(r"^frame(\d+)\.(jpg|jpeg|png)$", re.IGNORECASE)
    rx_depth = re.compile(r"^depth(\d+)\.(png|exr)$", re.IGNORECASE)
    rx_conf = re.compile(r"^confidence(\d+)\.(png|exr)$", re.IGNORECASE)

    right = _index_by_regex(right_dir, rx_frame, "right images")
    left = _index_by_regex(left_dir, rx_frame, "left images")
    depth = _index_by_regex(depth_dir, rx_depth, "depth images")

    conf: Dict[int, Path] = {}
    if conf_dir.exists() and conf_dir.is_dir():
        conf = _index_by_regex(conf_dir, rx_conf, "confidence images")

    if not right:
        raise RuntimeError(f"No right images found in: {right_dir}")
    if not left:
        raise RuntimeError(f"No left images found in: {left_dir}")
    if not depth:
        wprint(f"WARNING: No depth images found in: {depth_dir} (depth views will be missing).")

    return right, left, depth, conf


def read_rgb_u8(path: Path) -> np.ndarray:
    img_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img_bgr is None:
        raise RuntimeError(f"Failed to read image: {path}")
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


def read_any_mat(path: Path) -> np.ndarray:
    m = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if m is None:
        raise RuntimeError(f"Failed to read: {path}")
    return m


def depth_to_meters(depth_mat, scale):
    if depth_mat.ndim != 2 or scale not in (1, 1000):
        raise ValueError("Expected a single-channel depth image and explicit metre/millimetre scale")
    if (scale == 1000 and depth_mat.dtype != np.uint16) or (
        scale == 1 and not np.issubdtype(depth_mat.dtype, np.floating)
    ):
        raise ValueError("Depth encoding disagrees with the declared scale")
    return depth_mat.astype(np.float32) / float(scale)


def confidence_to_01(conf_mat):
    # ZED's stored value is an uncertainty score: 0 is best, 100 is worst.
    # Preserve that convention instead of normalizing each frame by its maximum.
    if conf_mat.ndim != 2:
        raise ValueError("Expected single-channel ZED confidence")
    values = conf_mat.astype(np.float32)
    finite = np.isfinite(values)
    if np.any((values[finite] < 0) | (values[finite] > 100)):
        raise ValueError("ZED confidence is outside [0, 100]")
    return np.where(finite, 1.0 - values / 100.0, 0.0)


def _pick_colormap() -> int:
    name = COLORMAP_NAME.upper()
    if name == "TURBO" and hasattr(cv2, "COLORMAP_TURBO"):
        return cv2.COLORMAP_TURBO
    if name == "VIRIDIS" and hasattr(cv2, "COLORMAP_VIRIDIS"):
        return cv2.COLORMAP_VIRIDIS
    return cv2.COLORMAP_TURBO if hasattr(cv2, "COLORMAP_TURBO") else cv2.COLORMAP_JET


def resize_depth_and_valid(depth_m: np.ndarray, target_wh: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Resize depth to (W,H) with correct handling of invalids:
      - valid mask resized with nearest
      - depth values resized with linear after NaNs replaced by 0
      - invalid restored to NaN
    """
    W, H = int(target_wh[0]), int(target_wh[1])
    if depth_m.shape[0] == H and depth_m.shape[1] == W:
        valid = np.isfinite(depth_m)
        return depth_m, valid

    valid0 = np.isfinite(depth_m).astype(np.uint8)
    d0 = depth_m.astype(np.float32).copy()
    d0[~np.isfinite(d0)] = 0.0

    d_rs = cv2.resize(d0, (W, H), interpolation=cv2.INTER_LINEAR)
    v_rs = cv2.resize(valid0, (W, H), interpolation=cv2.INTER_NEAREST).astype(bool)

    d_rs[~v_rs] = np.nan
    return d_rs, v_rs


def ensure_size(mat: np.ndarray, target_wh: Tuple[int, int], is_mask: bool = False) -> np.ndarray:
    W, H = int(target_wh[0]), int(target_wh[1])
    if mat.shape[0] == H and mat.shape[1] == W:
        return mat
    interp = cv2.INTER_NEAREST if is_mask else cv2.INTER_LINEAR
    return cv2.resize(mat, (W, H), interpolation=interp)


def _auto_range_from_percentiles(depth_m: np.ndarray, valid: np.ndarray) -> Optional[Tuple[float, float]]:
    finite = depth_m[np.isfinite(depth_m) & valid]
    if finite.size < 100:
        return None
    lo_p, hi_p = AUTO_RANGE_PERCENTILES
    lo = float(np.percentile(finite, lo_p))
    hi = float(np.percentile(finite, hi_p))
    if not np.isfinite(lo) or not np.isfinite(hi) or not (hi > lo):
        return None
    return lo, hi


def colorize_depth_from_resized(
    depth_m: np.ndarray,
    valid: np.ndarray,
    depth_range_m: Tuple[float, float],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    depth_m is already at target resolution and has NaNs for invalid.
    valid is boolean mask at same resolution.
    Returns:
      depth_color_rgb: uint8 HxWx3
      valid_for_overlay: bool HxW (where overlay should be applied)
    """
    dmin, dmax = float(depth_range_m[0]), float(depth_range_m[1])
    if not (dmax > dmin):
        raise ValueError("DEPTH_RANGE_M must satisfy max > min")

    finite_valid = np.isfinite(depth_m) & valid
    in_range = finite_valid & (depth_m >= dmin) & (depth_m <= dmax)

    # If almost nothing passes the clamp, auto-fallback so visualization isn't blank.
    if AUTO_DEPTH_RANGE_FALLBACK:
        total = int(np.count_nonzero(finite_valid))
        kept = int(np.count_nonzero(in_range))
        if total > 0 and kept < max(100, int(0.01 * total)):
            auto = _auto_range_from_percentiles(depth_m, valid)
            if auto is not None:
                dmin, dmax = auto
                in_range = finite_valid & (depth_m >= dmin) & (depth_m <= dmax)

    norm = np.zeros_like(depth_m, dtype=np.float32)
    if np.any(in_range):
        norm[in_range] = (depth_m[in_range] - dmin) / (dmax - dmin)
    norm = np.clip(norm, 0.0, 1.0)

    u8 = (norm * 255.0).astype(np.uint8)
    cm = _pick_colormap()
    colored_bgr = cv2.applyColorMap(u8, cm)
    colored_rgb = cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB)
    colored_rgb[~in_range] = 0
    return colored_rgb, in_range


def blend_rgb_depth(
    rgb: np.ndarray,
    depth_color: np.ndarray,
    valid_mask: np.ndarray,
    conf01: Optional[np.ndarray],
) -> np.ndarray:
    """
    Overlay depth_color on rgb:
      - Only where valid_mask True
      - Alpha = OVERLAY_ALPHA * f(conf) where f(conf) >= CONF_MIN_ALPHA (if enabled + conf exists)
    """
    H, W = rgb.shape[0], rgb.shape[1]
    dc = ensure_size(depth_color, (W, H), is_mask=False)
    vm = ensure_size(valid_mask.astype(np.uint8), (W, H), is_mask=True).astype(bool)

    alpha = np.zeros((H, W), dtype=np.float32)
    alpha[vm] = float(OVERLAY_ALPHA)

    if MODULATE_OVERLAY_BY_CONF and conf01 is not None:
        c = ensure_size(conf01.astype(np.float32), (W, H), is_mask=True)
        c = np.clip(c, 0.0, 1.0) ** float(CONF_GAMMA)
        c = float(CONF_MIN_ALPHA) + (1.0 - float(CONF_MIN_ALPHA)) * c
        alpha *= c

    a3 = alpha[..., None]
    out = rgb.astype(np.float32) * (1.0 - a3) + dc.astype(np.float32) * a3
    return np.clip(out, 0, 255).astype(np.uint8)


def conf_gray_u8(conf01: np.ndarray) -> np.ndarray:
    c = np.clip(conf01, 0.0, 1.0)
    u8 = (c * 255.0).astype(np.uint8)
    return np.repeat(u8[..., None], 3, axis=2)


def make_default_blueprint():
    """
    Build a deterministic layout so every run starts the same.
    If the blueprint API isn't available in your installed rerun version, returns None.
    """
    try:
        import rerun.blueprint as rrb  # type: ignore
    except Exception:
        return None

    # Use root origin and explicit contents so views always show the intended entities.
    return rrb.Blueprint(
        rrb.Spatial3DView(
            name="3D (World)",
            origin=ENTITY_WORLD,
            contents=[f"{ENTITY_WORLD}/**"],
        ),
        rrb.Spatial2DView(name="Right RGB", origin="/", contents=[ENTITY_RIGHT_RGB]),
        rrb.Spatial2DView(name="Right Overlay", origin="/", contents=[ENTITY_RGB_DEPTH_OVERLAY]),
        rrb.Spatial2DView(name="Depth Color", origin="/", contents=[ENTITY_DEPTH_COLOR]),
        rrb.Spatial2DView(name="Depth Color (Conf)", origin="/", contents=[ENTITY_DEPTH_COLOR_CONF]),
        rrb.Spatial2DView(name="Confidence", origin="/", contents=[ENTITY_CONFIDENCE]),
        rrb.Spatial2DView(name="Left RGB", origin="/", contents=[ENTITY_LEFT_RGB]),
        collapse_panels=True,
    )


def init_rerun_fresh() -> None:
    recording_id = str(uuid4()) if FRESH_RECORDING_EVERY_RUN else None
    blueprint = make_default_blueprint()

    # Prefer rr.init(recording_id=..., default_blueprint=...) when available.
    try:
        if recording_id is not None and blueprint is not None:
            rr.init(APP_ID, spawn=SPAWN_VIEWER, recording_id=recording_id, default_blueprint=blueprint)
        elif recording_id is not None:
            rr.init(APP_ID, spawn=SPAWN_VIEWER, recording_id=recording_id)
        elif blueprint is not None:
            rr.init(APP_ID, spawn=SPAWN_VIEWER, default_blueprint=blueprint)
        else:
            rr.init(APP_ID, spawn=SPAWN_VIEWER)
    except TypeError:
        # Older rerun versions: best-effort init without recording_id/default_blueprint params
        rr.init(APP_ID, spawn=SPAWN_VIEWER)

    if not SPAWN_VIEWER:
        rr.connect()

    # Force-reset layout even if viewer has a persisted active blueprint for this app_id.
    if SEND_BLUEPRINT_EVERY_RUN and blueprint is not None and hasattr(rr, "send_blueprint"):
        try:
            rr.send_blueprint(blueprint, make_active=True, make_default=True)
        except Exception as e:
            wprint(f"WARNING: rr.send_blueprint failed: {e}")


def main(argv=None) -> int:
    global rr
    parser = argparse.ArgumentParser(description="Validate and view a timestamped camera dataset")
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    root = args.data_root
    if not root.exists() or not root.is_dir():
        eprint(f"ERROR: DATA_ROOT is not a directory: {root}")
        return 2

    traj_r_path = root / "traj_right.txt"
    traj_l_path = root / "traj_left.txt"
    caminfo_path = root / "camera_info.json"

    poses_r_raw = read_traj_txt(traj_r_path)
    poses_l_raw = read_traj_txt(traj_l_path)
    w, h, fx, fy, cx, cy = load_camera_info(caminfo_path)
    metadata = json.loads(caminfo_path.read_text())
    streams = load_export_manifest(root)
    validate_export(root, streams, metadata, poses_r_raw, poses_l_raw)
    if args.validate_only:
        print(json.dumps({name: len(entries) for name, entries in streams.items()}, sort_keys=True))
        return 0
    import rerun as rr

    right_imgs = {i: entry["path"] for i, entry in streams["right"].items()}
    left_imgs = {i: entry["path"] for i, entry in streams["left"].items()}

    def aligned_paths(stream):
        by_time = {entry["stamp_ns"]: entry["path"] for entry in streams.get(stream, {}).values()}
        return {i: by_time[entry["stamp_ns"]] for i, entry in streams["right"].items() if entry["stamp_ns"] in by_time}

    depth_imgs, conf_imgs = aligned_paths("depth"), aligned_paths("confidence")
    depth_side = pixel_aligned_camera(metadata)

    poses_r: List[Optional[np.ndarray]] = []
    poses_l: List[Optional[np.ndarray]] = []
    for T in poses_r_raw:
        poses_r.append(T if POSE_IS_WORLD_FROM_CAMERA else invert_T(T))
    for T in poses_l_raw:
        poses_l.append(T if POSE_IS_WORLD_FROM_CAMERA else invert_T(T))

    max_idx = max(
        max(right_imgs.keys()),
        max(left_imgs.keys()),
        max(depth_imgs.keys()) if depth_imgs else 0,
        max(conf_imgs.keys()) if conf_imgs else 0,
        len(poses_r) - 1,
        len(poses_l) - 1,
    )

    # Init rerun with fresh recording + reset layout
    init_rerun_fresh()

    try:
        rr.log(ENTITY_WORLD, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    except Exception as e:
        wprint(f"WARNING: Failed to set ViewCoordinates: {e}")

    pinhole = rr_make_pinhole(w, h, fx, fy, cx, cy)
    rr.log(ENTITY_RIGHT_CAM, pinhole, static=True)
    left_pinhole = rr_make_pinhole(*load_camera_info(caminfo_path, "left"))
    rr.log(ENTITY_LEFT_CAM, left_pinhole, static=True)

    if LOG_CAMERA_AXES:
        log_axes(ENTITY_RIGHT_CAM, AXES_LENGTH)
        log_axes(ENTITY_LEFT_CAM, AXES_LENGTH)

    log_frustum(ENTITY_RIGHT_CAM, FRUSTUM_SCALE)
    log_frustum(ENTITY_LEFT_CAM, FRUSTUM_SCALE)

    traj_r_positions: List[Tuple[float, float, float]] = []
    traj_l_positions: List[Tuple[float, float, float]] = []
    num_overlay_logged = 0

    for i in range(max_idx + 1):
        rr_set_frame_time(i)

        # ---- Poses ----
        t_right: Optional[np.ndarray] = None
        t_left: Optional[np.ndarray] = None

        if i < len(poses_r) and poses_r[i] is not None:
            Tr = poses_r[i]
            assert Tr is not None
            qr = rotmat_to_quat_xyzw(Tr[:3, :3])
            tr = Tr[:3, 3]
            rr_log_transform_parent_from_child(ENTITY_RIGHT_CAM, tr, qr)
            traj_r_positions.append((float(tr[0]), float(tr[1]), float(tr[2])))
            rr.log(ENTITY_RIGHT_TRAJ, rr.LineStrips3D([traj_r_positions]))
            t_right = tr

        if i < len(poses_l) and poses_l[i] is not None:
            Tl = poses_l[i]
            assert Tl is not None
            ql = rotmat_to_quat_xyzw(Tl[:3, :3])
            tl = Tl[:3, 3]
            rr_log_transform_parent_from_child(ENTITY_LEFT_CAM, tl, ql)
            traj_l_positions.append((float(tl[0]), float(tl[1]), float(tl[2])))
            rr.log(ENTITY_LEFT_TRAJ, rr.LineStrips3D([traj_l_positions]))
            t_left = tl

        if t_right is not None and t_left is not None:
            log_baseline_line(t_right, t_left)

        # ---- RGB ----
        right_rgb: Optional[np.ndarray] = None
        left_rgb: Optional[np.ndarray] = None
        if i in right_imgs:
            right_rgb = read_rgb_u8(right_imgs[i])
            rr.log(ENTITY_RIGHT_RGB, rr.Image(right_rgb))
            # Keep raw RGB on its own entity; the camera entity will get the overlay (later) for 3D.
            rr.log(ENTITY_RIGHT_CAM, rr.Image(right_rgb))

        if i in left_imgs:
            left_rgb = read_rgb_u8(left_imgs[i])
            rr.log(ENTITY_LEFT_RGB, rr.Image(left_rgb))
            rr.log(ENTITY_LEFT_CAM, rr.Image(left_rgb))

        # ---- Depth + confidence + overlay ----
        depth_m: Optional[np.ndarray] = None
        conf01: Optional[np.ndarray] = None

        if i in depth_imgs:
            dmat = read_any_mat(depth_imgs[i])
            depth_m = depth_to_meters(dmat, metadata["depth_scale"])

        if i in conf_imgs:
            cmat = read_any_mat(conf_imgs[i])
            conf01 = confidence_to_01(cmat)

        if conf01 is not None:
            rr.log(ENTITY_CONFIDENCE, rr.Image(conf_gray_u8(conf01)))

        if depth_m is not None:
            # Overlay only when both calibration and acquisition time agree.
            # ZED depth normally belongs to LEFT, not RIGHT. Resizing alone
            # cannot correct a stereo baseline or a different camera model.
            depth_color, valid = colorize_depth_from_resized(depth_m, np.isfinite(depth_m), DEPTH_RANGE_M)
            rr.log(ENTITY_DEPTH_COLOR, rr.Image(depth_color))
            rgb = left_rgb if depth_side == "left" else right_rgb if depth_side == "right" else None
            if rgb is not None:
                overlay = blend_rgb_depth(rgb, depth_color, valid, conf01)
                rr.log(ENTITY_RGB_DEPTH_OVERLAY, rr.Image(overlay))
                rr.log(ENTITY_LEFT_CAM if depth_side == "left" else ENTITY_RIGHT_CAM, rr.Image(overlay))
                num_overlay_logged += 1

    print(
        "Rerun logging done.\n"
        f"  overlay frames logged: {num_overlay_logged}\n"
        f"  colormap requested: {COLORMAP_NAME} (OpenCV id={_pick_colormap()})\n"
        f"  fresh recording each run: {FRESH_RECORDING_EVERY_RUN}\n"
        f"  send blueprint each run: {SEND_BLUEPRINT_EVERY_RUN}\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
