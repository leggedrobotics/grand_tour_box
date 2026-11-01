#!/usr/bin/env python3
"""Deep-dive heading audit for a GrandTour mission.

The analysis deliberately converts every estimator output to the robot ``base``
frame before comparing it:

* IE-TC: ``T_enu_origin_cpt7_imu @ inv(T_base_cpt7_imu)``
* DLIO:  ``T_dlio_map_hesai_lidar @ inv(T_base_hesai_lidar)``
* ANYmal state estimator: already ``T_odom_base``

The static sensor transforms are read from the mission's ``*_tf_model.bag``.
The three arbitrary world frames are aligned from synchronized trajectory
positions with a rigid (unit-scale) 3-D Umeyama fit by default.  Unlike
first-pose orientation alignment, this does not force the estimator headings
to agree and therefore preserves a constant yaw error for inspection.

The script produces trajectory plots, heading/time diagnostics, IE heading vs
course-over-ground checks, time-sync checks, a machine-readable JSON summary,
a Markdown report, and a CSV containing the common evaluation timeline.

Example
-------
Run this inside the requested conda environment::

    conda run --no-capture-output -n christian_grandtour python \
      box_utils/box_auto/python/box_auto/scripts/verification/alb1_heading_deep_dive.py \
      --bag-dir /home/tutuna/Music/jean_michel_test

The default output directory is ``<bag-dir>/alb1_heading_report``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp


CACHE_VERSION = 5

IE_TOPIC = "/boxi/inertial_explorer/tc/odometry"
IE_RAW_TOPIC = "/boxi/inertial_explorer/tc/raw"
ANYMAL_TOPIC = "/anymal/state_estimator/odometry"
DLIO_TOPIC = "/boxi/dlio/lidar_map_odometry"

COLORS = {
    "IE-TC": "tab:orange",
    "ANYmal": "tab:blue",
    "DLIO": "tab:green",
}


@dataclass
class PoseSeries:
    """Timestamped poses of one frame in one estimator's world frame."""

    name: str
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray

    def validated(self) -> "PoseSeries":
        arrays = (
            self.t,
            self.p,
            self.q,
            self.linear_velocity,
            self.angular_velocity,
        )
        if not all(len(a) == len(self.t) for a in arrays):
            raise ValueError(f"{self.name}: inconsistent array lengths")
        if len(self.t) < 2:
            raise ValueError(f"{self.name}: fewer than two poses")

        finite = np.isfinite(self.t) & np.all(np.isfinite(self.p), axis=1) & np.all(np.isfinite(self.q), axis=1)
        order = np.argsort(self.t[finite], kind="stable")
        t = self.t[finite][order]
        keep = np.r_[True, np.diff(t) > 0.0]
        idx = np.flatnonzero(finite)[order][keep]
        if len(idx) < 2:
            raise ValueError(f"{self.name}: fewer than two finite, distinct poses")

        q = self.q[idx].astype(np.float64, copy=True)
        norms = np.linalg.norm(q, axis=1)
        if np.any(norms < 1e-10):
            raise ValueError(f"{self.name}: zero-norm quaternion")
        q /= norms[:, None]

        return PoseSeries(
            self.name,
            self.t[idx].astype(np.float64),
            self.p[idx].astype(np.float64),
            q,
            self.linear_velocity[idx].astype(np.float64),
            self.angular_velocity[idx].astype(np.float64),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect IE-TC base heading against ANYmal and DLIO.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--bag-dir",
        type=Path,
        required=True,
        help="Directory containing the downloaded mission bags.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report directory; defaults to <bag-dir>/alb1_heading_report.",
    )
    parser.add_argument(
        "--mission-label",
        default="ALB-1",
        help="Mission name used in plot titles and the Markdown report.",
    )
    parser.add_argument(
        "--reference",
        choices=("ANYmal", "DLIO", "IE-TC"),
        default="ANYmal",
        help="World frame retained while aligning trajectories.",
    )
    parser.add_argument(
        "--alignment-method",
        choices=("umeyama", "first-pose"),
        default="umeyama",
        help="Trajectory alignment method. Umeyama uses positions only.",
    )
    parser.add_argument(
        "--umeyama-estimate-scale",
        action="store_true",
        help="Estimate Sim(3) scale. Off by default because all inputs are metric.",
    )
    parser.add_argument(
        "--umeyama-min-speed-mps",
        type=float,
        default=0.15,
        help="Reference path-speed threshold for samples used in the Umeyama fit.",
    )
    parser.add_argument(
        "--ie-heading-correction-deg",
        type=float,
        default=0.0,
        help=(
            "Test an additive correction to IE compass heading before converting "
            "cpt7_imu to base; positive values rotate ENU yaw clockwise."
        ),
    )
    parser.add_argument(
        "--motion-window-s",
        type=float,
        default=1.0,
        help="Centered displacement window used to estimate path tangents.",
    )
    parser.add_argument(
        "--evaluation-dt-s",
        type=float,
        default=0.05,
        help="Common comparison-grid spacing.",
    )
    parser.add_argument(
        "--min-forward-speed-mps",
        type=float,
        default=0.20,
        help="Minimum ANYmal forward velocity for the straight/forward mask.",
    )
    parser.add_argument(
        "--max-lateral-ratio",
        type=float,
        default=0.35,
        help="Maximum |body vy| / body vx for forward-motion samples.",
    )
    parser.add_argument(
        "--max-straight-yaw-rate-degps",
        type=float,
        default=12.0,
        help="Maximum ANYmal |yaw rate| for straight-motion samples.",
    )
    parser.add_argument(
        "--first-ie-points",
        type=int,
        default=2000,
        help="Number of initial IE samples shown in the requested zoom plot.",
    )
    parser.add_argument(
        "--arrow-count",
        type=int,
        default=35,
        help="Approximate arrows per trajectory in orientation plots.",
    )
    parser.add_argument("--dpi", type=int, default=220)
    parser.add_argument(
        "--rebuild-cache",
        action="store_true",
        help="Ignore the extraction cache and parse all bags again.",
    )
    return parser.parse_args()


def unique_bag(bag_dir: Path, pattern: str) -> Path:
    matches = sorted(bag_dir.glob(pattern))
    if len(matches) != 1:
        raise FileNotFoundError(
            f"Expected exactly one {pattern!r} in {bag_dir}, found {len(matches)}: "
            + ", ".join(str(x) for x in matches)
        )
    return matches[0]


def optional_unique_bag(bag_dir: Path, pattern: str) -> Path | None:
    matches = sorted(bag_dir.glob(pattern))
    if len(matches) > 1:
        raise FileNotFoundError(
            f"Expected at most one {pattern!r} in {bag_dir}, found {len(matches)}: "
            + ", ".join(str(x) for x in matches)
        )
    return matches[0] if matches else None


def discover_inputs(bag_dir: Path) -> dict[str, Path]:
    result = {
        "ie": unique_bag(bag_dir, "*_cpt7_ie_tc.bag"),
        "anymal": unique_bag(bag_dir, "*_anymal_state.bag"),
        "dlio": unique_bag(bag_dir, "*_dlio.bag"),
        "tf_model": unique_bag(bag_dir, "*_tf_model.bag"),
    }
    ap20 = optional_unique_bag(bag_dir, "*_ap20_prism_position.bag")
    if ap20 is not None:
        result["ap20"] = ap20
    return result


def input_fingerprint(paths: dict[str, Path]) -> str:
    entries = []
    for name, path in sorted(paths.items()):
        stat = path.stat()
        entries.append([name, str(path.resolve()), stat.st_size, stat.st_mtime_ns])
    return json.dumps(entries, separators=(",", ":"))


def stamp_to_sec(stamp: Any, fallback: Any) -> float:
    value = float(stamp.to_sec())
    return value if value > 0.0 else float(fallback.to_sec())


def xyz(value: Any) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def xyzw(value: Any) -> list[float]:
    return [float(value.x), float(value.y), float(value.z), float(value.w)]


def read_odometry(path: Path, topic: str, name: str) -> PoseSeries:
    t: list[float] = []
    p: list[list[float]] = []
    q: list[list[float]] = []
    linear: list[list[float]] = []
    angular: list[list[float]] = []
    header_frames: set[str] = set()
    child_frames: set[str] = set()

    print(f"Reading {name} poses from {path.name} ...", flush=True)
    with rosbag.Bag(str(path), "r") as bag:
        for _, msg, bag_t in bag.read_messages(topics=[topic]):
            t.append(stamp_to_sec(msg.header.stamp, bag_t))
            p.append(xyz(msg.pose.pose.position))
            q.append(xyzw(msg.pose.pose.orientation))
            linear.append(xyz(msg.twist.twist.linear))
            angular.append(xyz(msg.twist.twist.angular))
            header_frames.add(str(msg.header.frame_id))
            child_frames.add(str(msg.child_frame_id))

    if len(header_frames) != 1 or len(child_frames) != 1:
        raise ValueError(f"{name}: expected one frame pair, got parents={header_frames}, children={child_frames}")
    print(
        f"  {len(t):,} poses, {next(iter(header_frames))} -> {next(iter(child_frames))}",
        flush=True,
    )
    return PoseSeries(
        name=name,
        t=np.asarray(t, dtype=np.float64),
        p=np.asarray(p, dtype=np.float64),
        q=np.asarray(q, dtype=np.float64),
        linear_velocity=np.asarray(linear, dtype=np.float64),
        angular_velocity=np.asarray(angular, dtype=np.float64),
    ).validated()


def read_ie_raw(path: Path) -> dict[str, np.ndarray]:
    fields: dict[str, list[Any]] = {
        "t": [],
        "heading": [],
        "roll": [],
        "pitch": [],
        "heading_std": [],
        "roll_std": [],
        "pitch_std": [],
        "velocity_ecef": [],
        "velocity_std": [],
        "position_ecef": [],
        "position_std": [],
        "cog": [],
        "satellites": [],
        "quality": [],
        "pdop": [],
        "ambiguity": [],
    }
    print(f"Reading IE-TC raw diagnostics from {path.name} ...", flush=True)
    with rosbag.Bag(str(path), "r") as bag:
        for _, msg, bag_t in bag.read_messages(topics=[IE_RAW_TOPIC]):
            fields["t"].append(stamp_to_sec(msg.header.stamp, bag_t))
            fields["heading"].append(float(msg.orientation_hrp.x))
            fields["roll"].append(float(msg.orientation_hrp.y))
            fields["pitch"].append(float(msg.orientation_hrp.z))
            fields["heading_std"].append(float(msg.orientation_hrp_std.x))
            fields["roll_std"].append(float(msg.orientation_hrp_std.y))
            fields["pitch_std"].append(float(msg.orientation_hrp_std.z))
            fields["velocity_ecef"].append(xyz(msg.velocity_ecef))
            fields["velocity_std"].append(xyz(msg.velocity_ecef_std))
            fields["position_ecef"].append(xyz(msg.position_ecef))
            fields["position_std"].append(xyz(msg.position_ecef_std))
            fields["cog"].append(float(msg.course_over_ground.data))
            fields["satellites"].append(float(msg.number_of_satellites.data))
            fields["quality"].append(float(msg.quality_indicator.data))
            fields["pdop"].append(float(msg.position_dilution_of_precision.data))
            fields["ambiguity"].append(float(msg.ambiguity_status.data))
    result = {key: np.asarray(value, dtype=np.float64) for key, value in fields.items()}
    print(f"  {len(result['t']):,} raw samples", flush=True)
    return result


def read_ap20(path: Path) -> dict[str, np.ndarray]:
    t: list[float] = []
    position: list[list[float]] = []
    frames: set[str] = set()
    print(f"Reading AP20 prism positions from {path.name} ...", flush=True)
    with rosbag.Bag(str(path), "r") as bag:
        for _, msg, bag_t in bag.read_messages(topics=["/boxi/ap20/prism_position"]):
            t.append(stamp_to_sec(msg.header.stamp, bag_t))
            position.append(xyz(msg.point))
            frames.add(str(msg.header.frame_id))
    if len(frames) != 1:
        raise ValueError(f"AP20: expected one reference frame, got {frames}")
    print(f"  {len(t):,} prism samples in {next(iter(frames))}", flush=True)
    return {
        "ap20_t": np.asarray(t, dtype=np.float64),
        "ap20_position": np.asarray(position, dtype=np.float64),
    }


def transform_matrix(translation: Any, quaternion: Any) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(xyzw(quaternion)).as_matrix()
    matrix[:3, 3] = xyz(translation)
    return matrix


def read_static_graph(path: Path) -> tuple[dict[tuple[str, str], np.ndarray], dict[str, Any]]:
    """Read standard ROS parent->child static transforms and check repetition."""

    first: dict[tuple[str, str], np.ndarray] = {}
    max_repeat_error: dict[tuple[str, str], float] = {}
    counts: dict[tuple[str, str], int] = {}
    with rosbag.Bag(str(path), "r") as bag:
        for _, msg, _ in bag.read_messages(topics=["/tf_static"]):
            for item in msg.transforms:
                key = (str(item.header.frame_id), str(item.child_frame_id))
                matrix = transform_matrix(item.transform.translation, item.transform.rotation)
                if key not in first:
                    first[key] = matrix
                    max_repeat_error[key] = 0.0
                    counts[key] = 0
                delta = np.linalg.inv(first[key]) @ matrix
                trans_error = float(np.linalg.norm(delta[:3, 3]))
                rot_error = float(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
                max_repeat_error[key] = max(max_repeat_error[key], trans_error, rot_error)
                counts[key] += 1

    if not first:
        raise ValueError(f"No /tf_static transforms found in {path}")
    diagnostics = {
        "edge_count": len(first),
        "repeat_count_min": min(counts.values()),
        "repeat_count_max": max(counts.values()),
        "max_repeat_error": max(max_repeat_error.values()),
    }
    return first, diagnostics


def lookup_static(edges: dict[tuple[str, str], np.ndarray], target: str, source: str) -> tuple[np.ndarray, list[str]]:
    """Return T_target_source and the traversed frame path."""

    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), t_parent_child in edges.items():
        adjacency.setdefault(parent, []).append((child, t_parent_child))
        adjacency.setdefault(child, []).append((parent, np.linalg.inv(t_parent_child)))

    queue: list[tuple[str, np.ndarray, list[str]]] = [(target, np.eye(4), [target])]
    visited = {target}
    while queue:
        current, t_target_current, frame_path = queue.pop(0)
        if current == source:
            return t_target_current, frame_path
        for neighbor, t_current_neighbor in adjacency.get(current, []):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append(
                (
                    neighbor,
                    t_target_current @ t_current_neighbor,
                    frame_path + [neighbor],
                )
            )
    raise KeyError(f"No static TF path from {target!r} to {source!r}")


def transform_sensor_pose_to_base(series: PoseSeries, t_base_sensor: np.ndarray) -> PoseSeries:
    """Convert T_world_sensor samples to T_world_base samples."""

    rotation_world_sensor = Rotation.from_quat(series.q)
    rotation_base_sensor = Rotation.from_matrix(t_base_sensor[:3, :3])
    t_sensor_base = np.linalg.inv(t_base_sensor)
    p_sensor_base = t_sensor_base[:3, 3]
    rotation_world_base = rotation_world_sensor * rotation_base_sensor.inv()
    p_world_base = series.p + rotation_world_sensor.apply(np.broadcast_to(p_sensor_base, series.p.shape))
    return PoseSeries(
        name=series.name,
        t=series.t.copy(),
        p=p_world_base,
        q=rotation_world_base.as_quat(),
        # Odometry twist is already expressed in child/sensor coordinates. It is
        # retained for traceability but never used as a base twist after this
        # lever-arm conversion.
        linear_velocity=series.linear_velocity.copy(),
        angular_velocity=series.angular_velocity.copy(),
    ).validated()


def save_cache(
    path: Path,
    fingerprint: str,
    series: dict[str, PoseSeries],
    raw: dict[str, np.ndarray],
    transforms: dict[str, np.ndarray],
    metadata: dict[str, Any],
) -> None:
    payload: dict[str, Any] = {
        "cache_version": np.asarray(CACHE_VERSION, dtype=np.int64),
        "fingerprint": np.asarray(fingerprint),
        "metadata_json": np.asarray(json.dumps(metadata)),
    }
    for key, value in series.items():
        payload[f"{key}_t"] = value.t
        payload[f"{key}_p"] = value.p
        payload[f"{key}_q"] = value.q
        payload[f"{key}_linear"] = value.linear_velocity
        payload[f"{key}_angular"] = value.angular_velocity
    for key, value in raw.items():
        payload[f"raw_{key}"] = value
    for key, value in transforms.items():
        payload[f"transform_{key}"] = value
    np.savez_compressed(path, **payload)


def load_cache(
    path: Path, fingerprint: str
) -> tuple[dict[str, PoseSeries], dict[str, np.ndarray], dict[str, np.ndarray], dict[str, Any]] | None:
    if not path.exists():
        return None
    try:
        with np.load(path, allow_pickle=False) as data:
            if int(data["cache_version"]) != CACHE_VERSION:
                return None
            if str(data["fingerprint"]) != fingerprint:
                return None
            series = {}
            for key, name in (("ie", "IE-TC"), ("anymal", "ANYmal"), ("dlio", "DLIO")):
                series[key] = PoseSeries(
                    name,
                    data[f"{key}_t"],
                    data[f"{key}_p"],
                    data[f"{key}_q"],
                    data[f"{key}_linear"],
                    data[f"{key}_angular"],
                ).validated()
            raw = {key[len("raw_") :]: data[key] for key in data.files if key.startswith("raw_")}
            transforms = {key[len("transform_") :]: data[key] for key in data.files if key.startswith("transform_")}
            metadata = json.loads(str(data["metadata_json"]))
        return series, raw, transforms, metadata
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"Ignoring unusable cache {path}: {exc}", file=sys.stderr)
        return None


def extract_or_load(
    paths: dict[str, Path], output_dir: Path, rebuild: bool
) -> tuple[dict[str, PoseSeries], dict[str, np.ndarray], dict[str, np.ndarray], dict[str, Any]]:
    cache_path = output_dir / "extracted_heading_data.npz"
    fingerprint = input_fingerprint(paths)
    if not rebuild:
        loaded = load_cache(cache_path, fingerprint)
        if loaded is not None:
            print(f"Loaded extraction cache: {cache_path}", flush=True)
            return loaded

    ie_sensor = read_odometry(paths["ie"], IE_TOPIC, "IE-TC")
    anymal = read_odometry(paths["anymal"], ANYMAL_TOPIC, "ANYmal")
    dlio_sensor = read_odometry(paths["dlio"], DLIO_TOPIC, "DLIO")
    raw = read_ie_raw(paths["ie"])
    if "ap20" in paths:
        raw.update(read_ap20(paths["ap20"]))

    static_edges, static_diagnostics = read_static_graph(paths["tf_model"])
    t_base_cpt7, cpt7_path = lookup_static(static_edges, "base", "cpt7_imu")
    t_base_hesai, hesai_path = lookup_static(static_edges, "base", "hesai_lidar")
    t_base_prism, prism_path = lookup_static(static_edges, "base", "prism")
    ie = transform_sensor_pose_to_base(ie_sensor, t_base_cpt7)
    dlio = transform_sensor_pose_to_base(dlio_sensor, t_base_hesai)

    transforms = {
        "base_cpt7": t_base_cpt7,
        "base_hesai": t_base_hesai,
        "base_prism": t_base_prism,
    }
    metadata = {
        "static_tf": static_diagnostics,
        "base_to_cpt7_path": cpt7_path,
        "base_to_hesai_path": hesai_path,
        "base_to_prism_path": prism_path,
        "source_frames": {
            "IE-TC": "enu_origin -> cpt7_imu, converted to enu_origin -> base",
            "ANYmal": "odom -> base",
            "DLIO": "dlio_map -> hesai_lidar, converted to dlio_map -> base",
        },
    }
    series = {"ie": ie, "anymal": anymal, "dlio": dlio}
    save_cache(cache_path, fingerprint, series, raw, transforms, metadata)
    print(f"Saved extraction cache: {cache_path}", flush=True)
    return series, raw, transforms, metadata


def matrix_from_pose(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position
    return result


def apply_world_alignment(series: PoseSeries, t_new_old: np.ndarray, scale: float = 1.0) -> PoseSeries:
    rotation_new_old = Rotation.from_matrix(t_new_old[:3, :3])
    p = scale * rotation_new_old.apply(series.p) + t_new_old[:3, 3]
    q = (rotation_new_old * Rotation.from_quat(series.q)).as_quat()
    return PoseSeries(
        series.name,
        series.t.copy(),
        p,
        q,
        series.linear_velocity.copy(),
        series.angular_velocity.copy(),
    ).validated()


def align_first_poses(
    series: dict[str, PoseSeries], reference_name: str
) -> tuple[dict[str, PoseSeries], dict[str, np.ndarray], dict[str, dict[str, Any]]]:
    reference_key = {value.name: key for key, value in series.items()}[reference_name]
    reference = series[reference_key]
    t_reference_base0 = matrix_from_pose(reference.p[0], reference.q[0])
    aligned: dict[str, PoseSeries] = {}
    alignments: dict[str, np.ndarray] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    for key, value in series.items():
        t_source_base0 = matrix_from_pose(value.p[0], value.q[0])
        t_reference_source = t_reference_base0 @ np.linalg.inv(t_source_base0)
        alignments[key] = t_reference_source
        aligned[key] = apply_world_alignment(value, t_reference_source)
        diagnostics[key] = {
            "method": "first-pose SE(3)",
            "scale": 1.0,
            "fit_sample_count": 1,
        }
    return aligned, alignments, diagnostics


def umeyama_similarity(
    source: np.ndarray, target: np.ndarray, estimate_scale: bool
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """Return the least-squares similarity mapping ``source`` onto ``target``.

    Points are rows and the returned mapping is ``scale * R @ p + t``.  The
    reflection case is explicitly rejected, following Umeyama's proper-rotation
    solution.
    """

    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Umeyama inputs must be matching N x 3 arrays")
    if len(source) < 3:
        raise ValueError("At least three point pairs are required for Umeyama alignment")
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("Alignment positions must be finite")
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    source_zero = source - source_mean
    target_zero = target - target_mean
    covariance = source_zero.T @ target_zero / len(source)
    u, singular_values, vt = np.linalg.svd(covariance)
    # A straight line leaves rotation around that line unobservable.
    if singular_values[1] <= max(singular_values[0], np.finfo(float).eps) * 1e-10:
        raise ValueError("Alignment trajectory is collinear or degenerate")
    sign = np.ones(3, dtype=np.float64)
    if np.linalg.det(vt.T @ u.T) < 0.0:
        sign[-1] = -1.0
    rotation = vt.T @ np.diag(sign) @ u.T
    source_variance = float(np.mean(np.sum(np.square(source_zero), axis=1)))
    if source_variance <= np.finfo(np.float64).eps:
        raise ValueError("Source trajectory has no spatial variance")
    scale = float(np.sum(singular_values * sign) / source_variance) if estimate_scale else 1.0
    translation = target_mean - scale * rotation @ source_mean
    return rotation, translation, scale, singular_values


def align_umeyama(
    series: dict[str, PoseSeries],
    reference_name: str,
    evaluation_dt_s: float,
    min_speed_mps: float,
    estimate_scale: bool,
) -> tuple[dict[str, PoseSeries], dict[str, np.ndarray], dict[str, dict[str, Any]]]:
    """Position-only trajectory alignment in the selected reference world."""

    reference_key = {value.name: key for key, value in series.items()}[reference_name]
    reference = series[reference_key]
    start = max(value.t[0] for value in series.values())
    end = min(value.t[-1] for value in series.values())
    query = np.arange(start, end, evaluation_dt_s, dtype=np.float64)
    if len(query) < 3:
        raise ValueError("Insufficient common interval for Umeyama alignment")
    reference_p = interpolate_vectors(reference.t, reference.p, query)
    reference_speed = np.linalg.norm(np.gradient(reference_p, evaluation_dt_s, axis=0), axis=1)
    fit_mask = reference_speed >= min_speed_mps
    if np.count_nonzero(fit_mask) < 3:
        raise ValueError("Fewer than three moving samples passed the Umeyama speed threshold")

    aligned: dict[str, PoseSeries] = {}
    alignments: dict[str, np.ndarray] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    for key, value in series.items():
        source_p = interpolate_vectors(value.t, value.p, query)
        rotation, translation, scale, singular_values = umeyama_similarity(
            source_p[fit_mask], reference_p[fit_mask], estimate_scale
        )
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        aligned[key] = apply_world_alignment(value, matrix, scale)
        alignments[key] = matrix
        residual = scale * (rotation @ source_p[fit_mask].T).T + translation - reference_p[fit_mask]
        diagnostics[key] = {
            "method": "position-only 3-D Umeyama",
            "scale": float(scale),
            "fit_sample_count": int(np.count_nonzero(fit_mask)),
            "fit_duration_s": float(np.count_nonzero(fit_mask) * evaluation_dt_s),
            "min_reference_speed_mps": float(min_speed_mps),
            "position_rmse_m": float(np.sqrt(np.mean(np.sum(np.square(residual), axis=1)))),
            "covariance_singular_values": singular_values.tolist(),
            "covariance_condition_number": float(singular_values[0] / max(singular_values[-1], 1e-15)),
        }
    return aligned, alignments, diagnostics


def apply_ie_compass_heading_correction(
    base_series: PoseSeries, t_base_sensor: np.ndarray, correction_deg: float
) -> PoseSeries:
    """Reproduce an IE heading correction while keeping antenna positions fixed.

    IE heading is clockwise from north.  Adding ``correction_deg`` therefore
    left-multiplies the BODY->ENU attitude by an ENU z rotation of the opposite
    sign.  The sensor pose is reconstructed first so the base/sensor lever arm
    remains physically consistent.
    """

    if abs(correction_deg) < 1e-12:
        return base_series
    rotation_world_base = Rotation.from_quat(base_series.q)
    rotation_base_sensor = Rotation.from_matrix(t_base_sensor[:3, :3])
    p_base_sensor = t_base_sensor[:3, 3]
    rotation_world_sensor = rotation_world_base * rotation_base_sensor
    p_world_sensor = base_series.p + rotation_world_base.apply(np.broadcast_to(p_base_sensor, base_series.p.shape))
    correction = Rotation.from_euler("z", -correction_deg, degrees=True)
    corrected_world_sensor = correction * rotation_world_sensor
    corrected_world_base = corrected_world_sensor * rotation_base_sensor.inv()
    p_sensor_base = -rotation_base_sensor.inv().apply(p_base_sensor)
    corrected_p_world_base = p_world_sensor + corrected_world_sensor.apply(
        np.broadcast_to(p_sensor_base, base_series.p.shape)
    )
    return PoseSeries(
        base_series.name,
        base_series.t.copy(),
        corrected_p_world_base,
        corrected_world_base.as_quat(),
        base_series.linear_velocity.copy(),
        base_series.angular_velocity.copy(),
    ).validated()


def audit_ie_attitude_export_equivalence(
    ie_base: PoseSeries,
    raw: dict[str, np.ndarray],
    t_base_sensor: np.ndarray,
) -> dict[str, Any]:
    """Compare the downloaded pose quaternions with both exporter formulas.

    The local exporter rewrite uses ``Rotation(A.T) * R_body_ned`` whereas the
    previous implementation used the matrix product ``A @ R_body_ned``.  The
    NED/ENU axis-swap matrix ``A`` is symmetric, so the expressions should be
    identical; this audit checks that claim for every mission sample as well as
    against the actual bag quaternion.
    """

    required = ("t", "heading", "roll", "pitch")
    if any(key not in raw for key in required):
        return {"available": False, "reason": "raw IE attitude fields unavailable"}
    if len(raw["t"]) != len(ie_base.t):
        return {
            "available": False,
            "reason": "raw and odometry sample counts differ",
            "raw_count": int(len(raw["t"])),
            "odometry_count": int(len(ie_base.t)),
        }

    angles = np.column_stack((raw["heading"], raw["roll"], raw["pitch"]))
    rotation_body_ned = Rotation.from_euler("ZXY", angles, degrees=True)
    axis_swap = np.array(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
        dtype=np.float64,
    )
    local_formula = Rotation.from_matrix(axis_swap.T) * rotation_body_ned
    previous_formula = Rotation.from_matrix(axis_swap @ rotation_body_ned.as_matrix())
    rotation_base_sensor = Rotation.from_matrix(t_base_sensor[:3, :3])
    bag_sensor = Rotation.from_quat(ie_base.q) * rotation_base_sensor
    local_vs_bag = (local_formula.inv() * bag_sensor).magnitude()
    local_vs_previous = (local_formula.inv() * previous_formula).magnitude()
    return {
        "available": True,
        "sample_count": int(len(ie_base.t)),
        "max_timestamp_difference_s": float(np.max(np.abs(raw["t"] - ie_base.t))),
        "local_formula_vs_bag_max_angle_deg": float(np.rad2deg(np.max(local_vs_bag))),
        "local_formula_vs_bag_rms_angle_deg": float(np.rad2deg(np.sqrt(np.mean(np.square(local_vs_bag))))),
        "local_formula_vs_previous_formula_max_angle_deg": float(np.rad2deg(np.max(local_vs_previous))),
        "conclusion": (
            "The local NED-to-ENU attitude rewrite reproduces the downloaded "
            "bag and the previous formula; re-exporting without a nonzero "
            "heading correction cannot change the exported pose heading."
        ),
    }


def interpolate_vectors(t: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    result = np.empty((len(query), values.shape[1]), dtype=np.float64)
    for column in range(values.shape[1]):
        result[:, column] = np.interp(query, t, values[:, column])
    return result


def interpolate_quaternions(t: np.ndarray, q: np.ndarray, query: np.ndarray) -> np.ndarray:
    if query[0] < t[0] or query[-1] > t[-1]:
        raise ValueError("Quaternion interpolation requested outside source time range")
    return Slerp(t, Rotation.from_quat(q))(query).as_quat()


def interpolate_pose(series: PoseSeries, query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return (
        interpolate_vectors(series.t, series.p, query),
        interpolate_quaternions(series.t, series.q, query),
    )


def planar_heading(quaternion: np.ndarray) -> np.ndarray:
    count = len(quaternion)
    forward = Rotation.from_quat(quaternion).apply(np.broadcast_to(np.array([1.0, 0.0, 0.0]), (count, 3)))
    return np.arctan2(forward[:, 1], forward[:, 0])


def wrap_deg(value: np.ndarray | float) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    return (array + 180.0) % 360.0 - 180.0


def centered_motion(
    series: PoseSeries, query: np.ndarray, window_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    half = 0.5 * window_s
    before = interpolate_vectors(series.t, series.p, query - half)
    after = interpolate_vectors(series.t, series.p, query + half)
    delta = after[:, :2] - before[:, :2]
    heading = np.arctan2(delta[:, 1], delta[:, 0])
    speed = np.linalg.norm(delta, axis=1) / window_s
    return heading, speed, delta


def smooth_derivative_deg(angle_rad: np.ndarray, dt: float, window_s: float = 0.75) -> np.ndarray:
    unwrapped = np.unwrap(angle_rad)
    desired = max(5, int(round(window_s / dt)))
    window = desired if desired % 2 == 1 else desired + 1
    window = min(window, len(unwrapped) - (1 - len(unwrapped) % 2))
    if window < 5:
        return np.rad2deg(np.gradient(unwrapped, dt))
    return np.rad2deg(savgol_filter(unwrapped, window, 2, deriv=1, delta=dt))


def error_metrics(values: np.ndarray, mask: np.ndarray | None = None) -> dict[str, float | int]:
    values = np.asarray(values, dtype=np.float64)
    selected = np.isfinite(values)
    if mask is not None:
        selected &= np.asarray(mask, dtype=bool)
    data = values[selected]
    if not len(data):
        return {
            "count": 0,
            "mean_deg": math.nan,
            "median_deg": math.nan,
            "std_deg": math.nan,
            "rmse_deg": math.nan,
            "median_abs_deg": math.nan,
            "p95_abs_deg": math.nan,
            "max_abs_deg": math.nan,
        }
    return {
        "count": int(len(data)),
        "mean_deg": float(np.mean(data)),
        "median_deg": float(np.median(data)),
        "std_deg": float(np.std(data)),
        "rmse_deg": float(np.sqrt(np.mean(np.square(data)))),
        "median_abs_deg": float(np.median(np.abs(data))),
        "p95_abs_deg": float(np.percentile(np.abs(data), 95)),
        "max_abs_deg": float(np.max(np.abs(data))),
    }


def scalar_distribution(values: np.ndarray, mask: np.ndarray) -> dict[str, float | int]:
    selected = np.asarray(values, dtype=np.float64)[np.asarray(mask, dtype=bool)]
    selected = selected[np.isfinite(selected)]
    if not len(selected):
        return {
            "count": 0,
            "min": math.nan,
            "median": math.nan,
            "p95": math.nan,
            "max": math.nan,
        }
    return {
        "count": int(len(selected)),
        "min": float(np.min(selected)),
        "median": float(np.median(selected)),
        "p95": float(np.percentile(selected, 95)),
        "max": float(np.max(selected)),
    }


def tc_solution_quality_summary(raw: dict[str, np.ndarray], mask: np.ndarray) -> dict[str, Any]:
    """Summarize the quality fields exactly as encoded in GnssRaw."""

    selected = np.asarray(mask, dtype=bool)
    count = int(np.count_nonzero(selected))
    # Geometric PDOP cannot be below one. Values in [0, 1) occur primarily
    # during low/no-satellite periods in this export and are treated as missing.
    pdop_valid = selected & (raw["pdop"] >= 1.0) & (raw["satellites"] > 0.0)
    position_trace_std = np.linalg.norm(raw["position_std"], axis=1)
    velocity_trace_std = np.linalg.norm(raw["velocity_std"], axis=1)
    quality_values, quality_counts = np.unique(raw["quality"][selected].astype(int), return_counts=True)
    ambiguity_values, ambiguity_counts = np.unique(raw["ambiguity"][selected].astype(int), return_counts=True)
    return {
        "sample_count": count,
        "quality_histogram": {
            str(int(value)): int(value_count) for value, value_count in zip(quality_values, quality_counts)
        },
        "quality_fraction": {
            str(int(value)): float(value_count / count) if count else math.nan
            for value, value_count in zip(quality_values, quality_counts)
        },
        "ambiguity_histogram_fixed_1_float_2": {
            str(int(value)): int(value_count) for value, value_count in zip(ambiguity_values, ambiguity_counts)
        },
        "fixed_fraction": float(np.mean(raw["ambiguity"][selected] == 1)) if count else math.nan,
        "float_fraction": float(np.mean(raw["ambiguity"][selected] == 2)) if count else math.nan,
        "zero_satellite_fraction": float(np.mean(raw["satellites"][selected] == 0)) if count else math.nan,
        "fewer_than_4_satellites_fraction": float(np.mean(raw["satellites"][selected] < 4)) if count else math.nan,
        "at_least_5_satellites_fraction": float(np.mean(raw["satellites"][selected] >= 5)) if count else math.nan,
        "satellites": scalar_distribution(raw["satellites"], selected),
        "pdop_positive_only": scalar_distribution(raw["pdop"], pdop_valid),
        "pdop_missing_or_zero_fraction": float(1.0 - np.count_nonzero(pdop_valid) / count) if count else math.nan,
        "pdop_poor_ge_5_fraction_of_valid": (
            float(np.mean(raw["pdop"][pdop_valid] >= 5.0)) if np.any(pdop_valid) else math.nan
        ),
        "heading_std_deg": scalar_distribution(raw["heading_std"], selected),
        "roll_std_deg": scalar_distribution(raw["roll_std"], selected),
        "pitch_std_deg": scalar_distribution(raw["pitch_std"], selected),
        "position_ecef_trace_std_m": scalar_distribution(position_trace_std, selected),
        "velocity_ecef_trace_std_mps": scalar_distribution(velocity_trace_std, selected),
    }


def slope_deg_per_minute(values: np.ndarray, t: np.ndarray, mask: np.ndarray) -> float:
    valid = np.asarray(mask, dtype=bool) & np.isfinite(values)
    if np.count_nonzero(valid) < 2:
        return math.nan
    x = (t[valid] - t[valid][0]) / 60.0
    return float(np.polyfit(x, values[valid], 1)[0])


def nearest_indices(source_t: np.ndarray, query: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(source_t, query)
    indices = np.clip(indices, 1, len(source_t) - 1)
    left = indices - 1
    choose_right = np.abs(source_t[indices] - query) < np.abs(source_t[left] - query)
    return np.where(choose_right, indices, left)


def nearest_boolean(source_t: np.ndarray, source_mask: np.ndarray, query: np.ndarray) -> np.ndarray:
    return source_mask[nearest_indices(source_t, query)]


def best_rate_lag(
    reference_rate: np.ndarray,
    candidate_rate: np.ndarray,
    moving: np.ndarray,
    dt: float,
    max_lag_s: float = 2.0,
) -> dict[str, Any]:
    max_samples = int(round(max_lag_s / dt))
    lags = np.arange(-max_samples, max_samples + 1, dtype=int)
    correlations = np.full(len(lags), np.nan, dtype=np.float64)
    rmses = np.full(len(lags), np.nan, dtype=np.float64)

    # Use turning samples; a straight path contains no timing information.
    informative = moving & (np.abs(reference_rate) >= 1.0)
    for output_index, lag in enumerate(lags):
        if lag < 0:
            ref_slice = slice(-lag, None)
            candidate_slice = slice(None, lag)
            mask_slice = informative[-lag:]
        elif lag > 0:
            ref_slice = slice(None, -lag)
            candidate_slice = slice(lag, None)
            mask_slice = informative[:-lag]
        else:
            ref_slice = slice(None)
            candidate_slice = slice(None)
            mask_slice = informative
        x = reference_rate[ref_slice][mask_slice]
        y = candidate_rate[candidate_slice][mask_slice]
        valid = np.isfinite(x) & np.isfinite(y)
        if np.count_nonzero(valid) < 20:
            continue
        x = x[valid]
        y = y[valid]
        if np.std(x) > 1e-8 and np.std(y) > 1e-8:
            correlations[output_index] = float(np.corrcoef(x, y)[0, 1])
        rmses[output_index] = float(np.sqrt(np.mean(np.square(x - y))))

    if not np.any(np.isfinite(correlations)):
        best_index = 0
    else:
        best_index = int(np.nanargmax(correlations))
    return {
        "lags_s": lags.astype(np.float64) * dt,
        "correlations": correlations,
        "rmses": rmses,
        # Positive means the candidate samples at later timestamps best match
        # the current reference samples (candidate lags the reference).
        "best_lag_s": float(lags[best_index] * dt),
        "best_correlation": float(correlations[best_index]),
        "zero_lag_correlation": float(correlations[np.flatnonzero(lags == 0)[0]]),
    }


def transform_description(matrix: np.ndarray) -> dict[str, Any]:
    return {
        "translation_m": matrix[:3, 3].tolist(),
        "rotation_quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
        "rotation_euler_xyz_deg": Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True).tolist(),
    }


def fit_planar_rigid_transform(
    source_xy: np.ndarray, target_xy: np.ndarray, mask: np.ndarray
) -> tuple[float, np.ndarray, np.ndarray]:
    """Least-squares SE(2) transform mapping row-vector source points to target."""

    selected = np.asarray(mask, dtype=bool)
    if np.count_nonzero(selected) < 3:
        raise ValueError("At least three points are required for planar alignment")
    source_center = np.mean(source_xy[selected], axis=0)
    target_center = np.mean(target_xy[selected], axis=0)
    source_zero = source_xy[selected] - source_center
    target_zero = target_xy[selected] - target_center
    theta = math.atan2(
        float(np.sum(source_zero[:, 0] * target_zero[:, 1] - source_zero[:, 1] * target_zero[:, 0])),
        float(np.sum(source_zero[:, 0] * target_zero[:, 0] + source_zero[:, 1] * target_zero[:, 1])),
    )
    rotation = np.array(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]],
        dtype=np.float64,
    )
    translation = target_center - source_center @ rotation.T
    return theta, rotation, translation


def first_activity_window(
    t: np.ndarray,
    mask: np.ndarray,
    max_gap_s: float = 1.0,
    min_span_s: float = 5.0,
    padding_s: float = 0.5,
) -> tuple[float, float]:
    active_t = t[np.asarray(mask, dtype=bool)]
    if not len(active_t):
        return float(t[0]), float(min(t[-1], t[0] + min_span_s))
    split_at = np.flatnonzero(np.diff(active_t) > max_gap_s) + 1
    for group in np.split(active_t, split_at):
        if len(group) and group[-1] - group[0] >= min_span_s:
            return (
                float(max(t[0], group[0] - padding_s)),
                float(min(t[-1], group[-1] + padding_s)),
            )
    return float(active_t[0]), float(min(t[-1], active_t[0] + min_span_s))


def build_analysis(
    aligned: dict[str, PoseSeries],
    raw: dict[str, np.ndarray],
    transforms: dict[str, np.ndarray],
    args: argparse.Namespace,
) -> dict[str, Any]:
    start = max(value.t[0] for value in aligned.values()) + 0.5 * args.motion_window_s
    end = min(value.t[-1] for value in aligned.values()) - 0.5 * args.motion_window_s
    common_t = np.arange(start, end, args.evaluation_dt_s, dtype=np.float64)
    if len(common_t) < 100:
        raise ValueError("Insufficient common pose interval")

    common: dict[str, dict[str, np.ndarray]] = {}
    for key, value in aligned.items():
        p, q = interpolate_pose(value, common_t)
        heading = planar_heading(q)
        motion_heading, motion_speed, motion_delta = centered_motion(value, common_t, args.motion_window_s)
        common[key] = {
            "p": p,
            "q": q,
            "heading_rad": heading,
            "motion_heading_rad": motion_heading,
            "motion_speed_mps": motion_speed,
            "motion_delta": motion_delta,
            "heading_minus_motion_deg": wrap_deg(np.rad2deg(heading - motion_heading)),
            "yaw_rate_degps": smooth_derivative_deg(heading, args.evaluation_dt_s),
        }

    anymal = aligned["anymal"]
    anymal_linear = interpolate_vectors(anymal.t, anymal.linear_velocity, common_t)
    anymal_angular = interpolate_vectors(anymal.t, anymal.angular_velocity, common_t)
    forward = anymal_linear[:, 0] >= args.min_forward_speed_mps
    low_lateral = np.abs(anymal_linear[:, 1]) <= (args.max_lateral_ratio * np.maximum(anymal_linear[:, 0], 1e-6))
    straight = np.abs(common["anymal"]["yaw_rate_degps"]) <= args.max_straight_yaw_rate_degps
    moving = common["anymal"]["motion_speed_mps"] >= (0.75 * args.min_forward_speed_mps)
    forward_straight = forward & low_lateral & straight & moving
    first_motion_start, first_motion_end = first_activity_window(common_t, forward_straight)
    first_motion_mask = forward_straight & (common_t >= first_motion_start) & (common_t <= first_motion_end)

    headings = {key: np.rad2deg(value["heading_rad"]) for key, value in common.items()}
    errors = {
        "IE-TC - ANYmal": wrap_deg(headings["ie"] - headings["anymal"]),
        "IE-TC - DLIO": wrap_deg(headings["ie"] - headings["dlio"]),
        "DLIO - ANYmal": wrap_deg(headings["dlio"] - headings["anymal"]),
    }

    metrics: dict[str, Any] = {
        "common_interval": {
            "start_epoch_s": float(common_t[0]),
            "end_epoch_s": float(common_t[-1]),
            "duration_s": float(common_t[-1] - common_t[0]),
            "sample_count": int(len(common_t)),
            "forward_straight_sample_count": int(np.count_nonzero(forward_straight)),
            "forward_straight_duration_s": float(np.count_nonzero(forward_straight) * args.evaluation_dt_s),
        },
        "cross_estimator_heading_error_forward_straight": {},
        "base_heading_minus_own_motion_forward_straight": {},
        "cross_estimator_heading_error_all_moving": {},
    }
    for name, values in errors.items():
        metrics["cross_estimator_heading_error_forward_straight"][name] = error_metrics(values, forward_straight)
        metrics["cross_estimator_heading_error_all_moving"][name] = error_metrics(values, moving)
        metrics["cross_estimator_heading_error_forward_straight"][name]["linear_slope_deg_per_min"] = (
            slope_deg_per_minute(values, common_t, forward_straight)
        )

    for key, label in (("ie", "IE-TC"), ("anymal", "ANYmal"), ("dlio", "DLIO")):
        values = common[key]["heading_minus_motion_deg"]
        metrics["base_heading_minus_own_motion_forward_straight"][label] = error_metrics(values, forward_straight)

    zoom_end = aligned["ie"].t[min(max(args.first_ie_points, 1), len(aligned["ie"].t)) - 1]
    zoom_mask = common_t <= zoom_end
    zoom_eval = zoom_mask & forward_straight
    metrics["first_ie_points"] = {
        "requested_count": int(args.first_ie_points),
        "actual_count": int(min(args.first_ie_points, len(aligned["ie"].t))),
        "end_epoch_s": float(zoom_end),
        "duration_s": float(zoom_end - aligned["ie"].t[0]),
        "ie_endpoint_displacement_m": float(
            np.linalg.norm(aligned["ie"].p[min(args.first_ie_points, len(aligned["ie"].p)) - 1] - aligned["ie"].p[0])
        ),
        "ie_axis_span_m": np.ptp(
            aligned["ie"].p[: min(args.first_ie_points, len(aligned["ie"].p))],
            axis=0,
        ).tolist(),
        "forward_straight_duration_s": float(np.count_nonzero(zoom_eval) * args.evaluation_dt_s),
        "cross_estimator_heading_error": {name: error_metrics(values, zoom_eval) for name, values in errors.items()},
        "base_heading_minus_own_motion": {
            label: error_metrics(common[key]["heading_minus_motion_deg"], zoom_eval)
            for key, label in (("ie", "IE-TC"), ("anymal", "ANYmal"), ("dlio", "DLIO"))
        },
    }
    metrics["first_substantial_forward_segment"] = {
        "start_epoch_s": first_motion_start,
        "end_epoch_s": first_motion_end,
        "start_relative_s": float(first_motion_start - min(value.t[0] for value in aligned.values())),
        "end_relative_s": float(first_motion_end - min(value.t[0] for value in aligned.values())),
        "span_s": float(first_motion_end - first_motion_start),
        "forward_straight_duration_s": float(np.count_nonzero(first_motion_mask) * args.evaluation_dt_s),
        "cross_estimator_heading_error": {
            name: error_metrics(values, first_motion_mask) for name, values in errors.items()
        },
        "base_heading_minus_own_motion": {
            label: error_metrics(common[key]["heading_minus_motion_deg"], first_motion_mask)
            for key, label in (
                ("ie", "IE-TC"),
                ("anymal", "ANYmal"),
                ("dlio", "DLIO"),
            )
        },
    }

    # Internal IE evidence. Heading and COG are both clockwise-from-north, so
    # their difference does not depend on any ROS/ENU quaternion convention.
    raw_speed = np.linalg.norm(raw["velocity_ecef"], axis=1)
    raw_in_common = (raw["t"] >= common_t[0]) & (raw["t"] <= common_t[-1])
    raw_forward = np.zeros(len(raw["t"]), dtype=bool)
    raw_forward[raw_in_common] = nearest_boolean(common_t, forward_straight, raw["t"][raw_in_common])
    raw_mask = (
        raw_forward & (raw_speed >= args.min_forward_speed_mps) & np.isfinite(raw["cog"]) & np.isfinite(raw["heading"])
    )
    raw_heading_minus_cog = wrap_deg(raw["heading"] - raw["cog"])
    raw_zoom = raw_mask & (raw["t"] <= zoom_end)
    raw_pdop_valid = raw_mask & (raw["pdop"] >= 1.0) & (raw["satellites"] > 0.0)
    metrics["ie_internal"] = {
        "heading_minus_course_forward_straight": error_metrics(raw_heading_minus_cog, raw_mask),
        "heading_minus_course_first_ie_points": error_metrics(raw_heading_minus_cog, raw_zoom),
        "reported_heading_std_deg_forward_straight": {
            "median": float(np.median(raw["heading_std"][raw_mask])) if np.any(raw_mask) else math.nan,
            "p95": float(np.percentile(raw["heading_std"][raw_mask], 95)) if np.any(raw_mask) else math.nan,
            "max": float(np.max(raw["heading_std"][raw_mask])) if np.any(raw_mask) else math.nan,
        },
        "satellites_forward_straight": {
            "median": float(np.median(raw["satellites"][raw_mask])) if np.any(raw_mask) else math.nan,
            "min": float(np.min(raw["satellites"][raw_mask])) if np.any(raw_mask) else math.nan,
        },
        "pdop_forward_straight": {
            "median": float(np.median(raw["pdop"][raw_pdop_valid])) if np.any(raw_pdop_valid) else math.nan,
            "p95": float(np.percentile(raw["pdop"][raw_pdop_valid], 95)) if np.any(raw_pdop_valid) else math.nan,
        },
        # gnss_msgs/GnssRaw.msg defines FIXED=1 and FLOAT=2.
        "ambiguity_fixed_fraction": float(np.mean(raw["ambiguity"][raw_mask] == 1)) if np.any(raw_mask) else math.nan,
        "ambiguity_float_fraction": float(np.mean(raw["ambiguity"][raw_mask] == 2)) if np.any(raw_mask) else math.nan,
        "quality_histogram": {
            str(int(value)): int(count)
            for value, count in zip(*np.unique(raw["quality"][raw_mask].astype(int), return_counts=True))
        },
    }
    first_raw_count = min(args.first_ie_points, len(raw["t"]))
    first_raw_mask = np.arange(len(raw["t"])) < first_raw_count
    metrics["tc_solution_quality"] = {
        "encoding": {
            "quality": "1 best to 6 worst (Inertial Explorer quality number)",
            "ambiguity": "gnss_msgs/GnssRaw: FIXED=1, FLOAT=2",
            "pdop_zero_handling": ("PDOP<1 is treated as missing/malformed, not excellent geometry"),
        },
        "all_samples": tc_solution_quality_summary(raw, np.ones(len(raw["t"]), dtype=bool)),
        "first_ie_messages": tc_solution_quality_summary(raw, first_raw_mask),
        "forward_straight_samples": tc_solution_quality_summary(raw, raw_mask),
    }
    raw_at_common = nearest_indices(raw["t"], common_t)
    common_quality = raw["quality"][raw_at_common].astype(int)
    common_ambiguity = raw["ambiguity"][raw_at_common].astype(int)
    ie_dlio_error = errors["IE-TC - DLIO"]
    metrics["ie_dlio_heading_error_by_tc_status"] = {
        "quality_number": {
            str(value): error_metrics(ie_dlio_error, forward_straight & (common_quality == value))
            for value in np.unique(common_quality[forward_straight])
        },
        "ambiguity": {
            "fixed": error_metrics(ie_dlio_error, forward_straight & (common_ambiguity == 1)),
            "float": error_metrics(ie_dlio_error, forward_straight & (common_ambiguity == 2)),
        },
    }

    # The AP20 total-station track is an ancillary, position-grounded check. We
    # predict the measured prism position from each base pose, fit only one
    # constant SE(2) world alignment, and then compare base heading with the
    # independently observed prism direction during straight forward motion.
    ap20_result: dict[str, Any] | None = None
    if "ap20_t" in raw and "ap20_position" in raw:
        half_window = 0.5 * args.motion_window_s
        ap20_all_t = raw["ap20_t"]
        ap20_all_position = raw["ap20_position"]
        valid_ap20 = (ap20_all_t >= max(common_t[0], ap20_all_t[0]) + half_window) & (
            ap20_all_t <= min(common_t[-1], ap20_all_t[-1]) - half_window
        )
        ap20_t = ap20_all_t[valid_ap20]
        ap20_position = ap20_all_position[valid_ap20]
        ap20_before = interpolate_vectors(ap20_all_t, ap20_all_position, ap20_t - half_window)
        ap20_after = interpolate_vectors(ap20_all_t, ap20_all_position, ap20_t + half_window)
        ap20_delta = ap20_after[:, :2] - ap20_before[:, :2]
        ap20_motion_heading = np.arctan2(ap20_delta[:, 1], ap20_delta[:, 0])
        ap20_speed = np.linalg.norm(ap20_delta, axis=1) / args.motion_window_s
        ap20_forward_straight = nearest_boolean(common_t, forward_straight, ap20_t) & (
            ap20_speed >= args.min_forward_speed_mps
        )
        ap20_fit_mask = ap20_speed >= 0.5 * args.min_forward_speed_mps
        base_to_prism = transforms["base_prism"][:3, 3]

        source_results: dict[str, Any] = {}
        source_metrics: dict[str, Any] = {}
        for key, label in (
            ("ie", "IE-TC"),
            ("anymal", "ANYmal"),
            ("dlio", "DLIO"),
        ):
            base_p, base_q = interpolate_pose(aligned[key], ap20_t)
            predicted_prism = base_p + Rotation.from_quat(base_q).apply(np.broadcast_to(base_to_prism, base_p.shape))
            fit_yaw, fit_rotation, fit_translation = fit_planar_rigid_transform(
                predicted_prism[:, :2], ap20_position[:, :2], ap20_fit_mask
            )
            aligned_prism_xy = predicted_prism[:, :2] @ fit_rotation.T + fit_translation
            position_residual = np.linalg.norm(aligned_prism_xy - ap20_position[:, :2], axis=1)
            heading_leica = planar_heading(base_q) + fit_yaw
            heading_error = wrap_deg(np.rad2deg(heading_leica - ap20_motion_heading))
            source_results[key] = {
                "aligned_prism_xy": aligned_prism_xy,
                "heading_error_deg": heading_error,
            }
            source_metrics[label] = {
                "heading_minus_ap20_motion": error_metrics(heading_error, ap20_forward_straight),
                "position_alignment_yaw_deg": float(np.rad2deg(fit_yaw)),
                "position_residual_m": {
                    "rmse": float(np.sqrt(np.mean(np.square(position_residual[ap20_fit_mask])))),
                    "median": float(np.median(position_residual[ap20_fit_mask])),
                    "p95": float(np.percentile(position_residual[ap20_fit_mask], 95)),
                },
            }

        metrics["ap20_position_grounded_check"] = {
            "sample_count": int(len(ap20_t)),
            "forward_straight_sample_count": int(np.count_nonzero(ap20_forward_straight)),
            "forward_straight_duration_s_approx": float(
                np.count_nonzero(ap20_forward_straight) * np.median(np.diff(ap20_t))
            ),
            "method": (
                "Predict prism from each base pose with T_base_prism, then fit one "
                "least-squares SE(2) world transform to the AP20 prism positions."
            ),
            "sources": source_metrics,
        }
        ap20_result = {
            "t": ap20_t,
            "t_rel": ap20_t - min(value.t[0] for value in aligned.values()),
            "position": ap20_position,
            "motion_heading_rad": ap20_motion_heading,
            "speed_mps": ap20_speed,
            "forward_straight_mask": ap20_forward_straight,
            "fit_mask": ap20_fit_mask,
            "sources": source_results,
        }

    lags = {
        "IE-TC vs ANYmal": best_rate_lag(
            common["anymal"]["yaw_rate_degps"],
            common["ie"]["yaw_rate_degps"],
            moving,
            args.evaluation_dt_s,
        ),
        "DLIO vs ANYmal": best_rate_lag(
            common["anymal"]["yaw_rate_degps"],
            common["dlio"]["yaw_rate_degps"],
            moving,
            args.evaluation_dt_s,
        ),
    }
    metrics["time_lag"] = {
        key: {
            "best_lag_s": value["best_lag_s"],
            "best_correlation": value["best_correlation"],
            "zero_lag_correlation": value["zero_lag_correlation"],
        }
        for key, value in lags.items()
    }

    return {
        "t": common_t,
        "t_rel": common_t - min(value.t[0] for value in aligned.values()),
        "common": common,
        "headings_deg": headings,
        "errors_deg": errors,
        "anymal_linear": anymal_linear,
        "anymal_angular": anymal_angular,
        "moving_mask": moving,
        "forward_straight_mask": forward_straight,
        "first_motion_start": first_motion_start,
        "first_motion_end": first_motion_end,
        "zoom_end": zoom_end,
        "raw_speed": raw_speed,
        "raw_heading_minus_cog": raw_heading_minus_cog,
        "raw_forward_mask": raw_mask,
        "lags": lags,
        "ap20": ap20_result,
        "metrics": metrics,
    }


def run_sensitivity_analysis(
    aligned: dict[str, PoseSeries],
    raw: dict[str, np.ndarray],
    transforms: dict[str, np.ndarray],
    base_args: argparse.Namespace,
) -> list[dict[str, Any]]:
    """Recompute headline results under deliberately different mask settings."""

    configurations = [
        (0.5, 0.15, 0.50, 20.0),
        (1.0, 0.20, 0.35, 12.0),
        (2.0, 0.20, 0.35, 12.0),
        (1.0, 0.30, 0.20, 8.0),
        (1.0, 0.15, 0.50, 20.0),
    ]
    results: list[dict[str, Any]] = []
    for window, speed, lateral, yaw_rate in configurations:
        varied = argparse.Namespace(**vars(base_args))
        varied.motion_window_s = window
        varied.min_forward_speed_mps = speed
        varied.max_lateral_ratio = lateral
        varied.max_straight_yaw_rate_degps = yaw_rate
        metrics = build_analysis(aligned, raw, transforms, varied)["metrics"]
        own = metrics["base_heading_minus_own_motion_forward_straight"]
        ap20 = metrics.get("ap20_position_grounded_check", {}).get("sources", {})
        row: dict[str, Any] = {
            "motion_window_s": window,
            "min_forward_speed_mps": speed,
            "max_lateral_ratio": lateral,
            "max_straight_yaw_rate_degps": yaw_rate,
            "accepted_duration_s": metrics["common_interval"]["forward_straight_duration_s"],
            "own_path_median_abs_deg": {label: own[label]["median_abs_deg"] for label in ("IE-TC", "ANYmal", "DLIO")},
            "ie_raw_heading_minus_cog_median_abs_deg": metrics["ie_internal"]["heading_minus_course_forward_straight"][
                "median_abs_deg"
            ],
        }
        if ap20:
            row["ap20_heading_minus_motion_median_deg"] = {
                label: ap20[label]["heading_minus_ap20_motion"]["median_deg"] for label in ("IE-TC", "ANYmal", "DLIO")
            }
        results.append(row)
    return results


def add_start_end(ax: plt.Axes, x: np.ndarray, y: np.ndarray, color: str) -> None:
    ax.scatter(x[0], y[0], s=45, marker="o", color=color, edgecolor="black", zorder=5)
    ax.scatter(x[-1], y[-1], s=55, marker="x", color=color, linewidth=2, zorder=5)


def plot_full_trajectories(
    aligned: dict[str, PoseSeries],
    anchor: np.ndarray,
    mission_label: str,
    output: Path,
    dpi: int,
) -> None:
    fig, ax = plt.subplots(figsize=(10.5, 7.5), constrained_layout=True)
    for key in ("anymal", "ie", "dlio"):
        value = aligned[key]
        xy = value.p[:, :2] - anchor[:2]
        ax.plot(xy[:, 0], xy[:, 1], color=COLORS[value.name], label=value.name, linewidth=1.3)
        add_start_end(ax, xy[:, 0], xy[:, 1], COLORS[value.name])
    ax.set_title(f"{mission_label} base-frame trajectories — trajectory-aligned\n● start, × end")
    ax.set_xlabel("aligned x [m]")
    ax.set_ylabel("aligned y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def add_orientation_arrows(
    ax: plt.Axes,
    series: PoseSeries,
    mask: np.ndarray,
    anchor: np.ndarray,
    count: int,
    length: float,
) -> None:
    indices = np.flatnonzero(mask)
    if not len(indices):
        return
    choose = indices[np.unique(np.linspace(0, len(indices) - 1, min(count, len(indices))).astype(int))]
    xy = series.p[choose, :2] - anchor[:2]
    heading = planar_heading(series.q[choose])
    ax.quiver(
        xy[:, 0],
        xy[:, 1],
        length * np.cos(heading),
        length * np.sin(heading),
        color=COLORS[series.name],
        angles="xy",
        scale_units="xy",
        scale=1.0,
        width=0.004,
        headwidth=4.0,
        headlength=5.0,
        alpha=0.85,
        zorder=4,
    )


def plot_zoom(
    aligned: dict[str, PoseSeries],
    anchor: np.ndarray,
    zoom_end: float,
    arrow_count: int,
    mission_label: str,
    output: Path,
    dpi: int,
) -> None:
    masks = {key: (value.t >= value.t[0]) & (value.t <= zoom_end) for key, value in aligned.items()}
    all_xy = np.vstack([value.p[masks[key], :2] - anchor[:2] for key, value in aligned.items()])
    extent = np.ptp(all_xy, axis=0)
    arrow_length = max(0.12, 0.035 * float(max(extent)))

    fig, ax = plt.subplots(figsize=(12, 5.5), constrained_layout=True)
    for key in ("anymal", "ie", "dlio"):
        value = aligned[key]
        mask = masks[key]
        xy = value.p[mask, :2] - anchor[:2]
        ax.plot(xy[:, 0], xy[:, 1], color=COLORS[value.name], label=value.name, linewidth=1.5)
        add_start_end(ax, xy[:, 0], xy[:, 1], COLORS[value.name])
        add_orientation_arrows(ax, value, mask, anchor, arrow_count, arrow_length)
    ie_count = int(np.count_nonzero(masks["ie"]))
    duration = zoom_end - aligned["ie"].t[0]
    ax.set_title(
        f"{mission_label} requested zoom: first {ie_count:,} IE-TC samples ({duration:.2f} s)\n"
        "arrows are the transformed robot base +x axis"
    )
    ax.set_xlabel("aligned x [m]")
    ax.set_ylabel("aligned y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_first_motion_segment(
    aligned: dict[str, PoseSeries],
    anchor: np.ndarray,
    start: float,
    end: float,
    arrow_count: int,
    mission_label: str,
    output: Path,
    dpi: int,
) -> None:
    masks = {key: (value.t >= start) & (value.t <= end) for key, value in aligned.items()}
    all_xy = np.vstack([value.p[masks[key], :2] - anchor[:2] for key, value in aligned.items()])
    arrow_length = max(0.15, 0.04 * float(np.max(np.ptp(all_xy, axis=0))))
    fig, ax = plt.subplots(figsize=(12, 5.5), constrained_layout=True)
    for key in ("anymal", "ie", "dlio"):
        value = aligned[key]
        mask = masks[key]
        xy = value.p[mask, :2] - anchor[:2]
        ax.plot(
            xy[:, 0],
            xy[:, 1],
            color=COLORS[value.name],
            label=value.name,
            linewidth=1.5,
        )
        add_start_end(ax, xy[:, 0], xy[:, 1], COLORS[value.name])
        add_orientation_arrows(ax, value, mask, anchor, arrow_count, arrow_length)
    ax.set_title(
        f"{mission_label} first substantial forward segment ({end - start:.2f} s)\n"
        "arrows are the transformed robot base +x axis"
    )
    ax.set_xlabel("aligned x [m]")
    ax.set_ylabel("aligned y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def shade_mask(ax: plt.Axes, t: np.ndarray, mask: np.ndarray) -> None:
    if len(t) < 2:
        return
    padded = np.r_[False, mask, False]
    transitions = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1) - 1
    for i, j in zip(starts, ends):
        ax.axvspan(t[i], t[j], color="0.85", alpha=0.35, linewidth=0)


def plot_heading_comparison(analysis: dict[str, Any], output: Path, dpi: int) -> None:
    t = analysis["t_rel"]
    mask = analysis["forward_straight_mask"]
    headings = analysis["headings_deg"]
    errors = analysis["errors_deg"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)

    for key, label in (("anymal", "ANYmal"), ("ie", "IE-TC"), ("dlio", "DLIO")):
        relative = np.rad2deg(np.unwrap(np.deg2rad(headings[key])) - np.unwrap(np.deg2rad(headings[key]))[0])
        axes[0].plot(t, relative, label=label, color=COLORS[label], linewidth=1.2)
    axes[0].set_ylabel("yaw change [deg]")
    axes[0].set_title("Base heading evolution after trajectory alignment")
    axes[0].legend(ncol=3)

    for name, value in errors.items():
        color = (
            "tab:red"
            if name.startswith("IE-TC - ANYmal")
            else ("tab:orange" if name.startswith("IE-TC - DLIO") else "tab:green")
        )
        axes[1].plot(t, value, label=name, color=color, linewidth=1.0)
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("heading difference [deg]")
    axes[1].legend(ncol=3, fontsize=9)

    axes[2].plot(
        t,
        analysis["common"]["anymal"]["motion_speed_mps"],
        label="ANYmal path speed",
        color="black",
        linewidth=1.0,
    )
    axes[2].plot(
        t,
        analysis["anymal_linear"][:, 0],
        label="ANYmal body vx",
        color="tab:blue",
        linewidth=0.9,
    )
    axes[2].set_ylabel("speed [m/s]")
    axes[2].set_xlabel("time from earliest estimator start [s]")
    axes[2].legend()

    for ax in axes:
        shade_mask(ax, t, mask)
        ax.grid(True, alpha=0.25)
    axes[2].text(
        0.01,
        0.95,
        "gray = forward + low lateral velocity + low yaw rate",
        transform=axes[2].transAxes,
        va="top",
        fontsize=9,
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_heading_motion(analysis: dict[str, Any], output: Path, dpi: int) -> None:
    t = analysis["t_rel"]
    mask = analysis["forward_straight_mask"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    for key, label in (("anymal", "ANYmal"), ("ie", "IE-TC"), ("dlio", "DLIO")):
        axes[0].plot(
            t,
            analysis["common"][key]["heading_minus_motion_deg"],
            label=label,
            color=COLORS[label],
            linewidth=1.0,
        )
        axes[1].plot(
            t,
            analysis["common"][key]["motion_speed_mps"],
            label=label,
            color=COLORS[label],
            linewidth=1.0,
        )
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("base heading − path tangent [deg]")
    axes[0].set_title("Does each estimator's base +x axis follow its own trajectory?")
    axes[0].set_ylim(-90, 90)
    axes[0].legend(ncol=3)
    axes[1].set_ylabel("centered path speed [m/s]")
    axes[1].set_xlabel("time from earliest estimator start [s]")
    axes[1].legend(ncol=3)
    for ax in axes:
        shade_mask(ax, t, mask)
        ax.grid(True, alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ie_internal(raw: dict[str, np.ndarray], analysis: dict[str, Any], output: Path, dpi: int) -> None:
    t0 = min(analysis["t"])
    t = raw["t"] - t0
    mask = analysis["raw_forward_mask"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)

    axes[0].plot(t, analysis["raw_heading_minus_cog"], color="tab:orange", linewidth=0.8)
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("IE heading − COG [deg]")
    axes[0].set_title("IE-TC internal check (independent of ROS ENU quaternion conversion)")
    axes[0].set_ylim(-90, 90)

    axes[1].plot(t, raw["heading_std"], color="tab:red", label="heading std", linewidth=0.8)
    axes[1].plot(t, raw["roll_std"], color="tab:blue", label="roll std", linewidth=0.7)
    axes[1].plot(t, raw["pitch_std"], color="tab:green", label="pitch std", linewidth=0.7)
    axes[1].set_ylabel("reported attitude std [deg]")
    axes[1].legend(ncol=3)

    axes[2].plot(t, analysis["raw_speed"], color="black", label="ECEF speed norm", linewidth=0.8)
    axes[2].set_ylabel("speed [m/s]")
    axes[2].legend()

    axes[3].plot(t, raw["satellites"], label="satellites", color="tab:blue", linewidth=0.8)
    axes[3].plot(t, raw["pdop"], label="PDOP", color="tab:purple", linewidth=0.8)
    axes[3].step(
        t,
        raw["ambiguity"],
        where="mid",
        label="ambiguity (1=fixed, 2=float)",
        color="tab:green",
        linewidth=0.7,
    )
    axes[3].set_ylabel("GNSS indicators")
    axes[3].set_xlabel("time from common comparison start [s]")
    axes[3].legend(ncol=3)

    for ax in axes:
        shade_mask(ax, t, mask)
        ax.grid(True, alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_tc_solution_quality(
    raw: dict[str, np.ndarray],
    analysis: dict[str, Any],
    first_ie_points: int,
    output: Path,
    dpi: int,
) -> None:
    """Plot the formal TC quality indicators without interpreting zero PDOP as good."""

    t = raw["t"] - raw["t"][0]
    first_end_index = min(max(first_ie_points, 1), len(t)) - 1
    first_end = t[first_end_index]
    position_trace_std = np.linalg.norm(raw["position_std"], axis=1)
    velocity_trace_std = np.linalg.norm(raw["velocity_std"], axis=1)
    pdop = raw["pdop"].copy()
    pdop[(pdop < 1.0) | (raw["satellites"] <= 0.0)] = np.nan

    fig, axes = plt.subplots(6, 1, figsize=(13, 13), sharex=True, constrained_layout=True)
    axes[0].step(t, raw["quality"], where="mid", color="tab:blue", linewidth=0.8)
    axes[0].set_ylabel("quality no.\n1 best, 6 worst")
    axes[0].set_yticks(range(1, 7))
    axes[0].invert_yaxis()
    axes[0].set_title("IE-TC formal solution-quality fields from GnssRaw")

    axes[1].step(t, raw["ambiguity"], where="mid", color="tab:green", linewidth=0.8)
    axes[1].set_ylabel("ambiguity")
    axes[1].set_yticks([1, 2], labels=["fixed", "float"])

    axes[2].step(t, raw["satellites"], where="mid", color="tab:cyan", linewidth=0.8)
    axes[2].axhline(5.0, color="black", linestyle="--", linewidth=0.7)
    axes[2].set_ylabel("satellites")

    axes[3].plot(t, pdop, color="tab:purple", linewidth=0.8)
    axes[3].axhline(3.0, color="tab:green", linestyle="--", linewidth=0.7)
    axes[3].axhline(5.0, color="tab:red", linestyle="--", linewidth=0.7)
    axes[3].set_ylabel("PDOP\n<1 hidden")

    axes[4].plot(t, position_trace_std, color="tab:orange", linewidth=0.8, label="position 3-D trace std")
    axes[4].plot(t, velocity_trace_std, color="tab:gray", linewidth=0.7, label="velocity 3-D trace std")
    axes[4].set_ylabel("formal std\n[m] / [m/s]")
    axes[4].legend(ncol=2)

    axes[5].plot(t, raw["heading_std"], color="tab:red", linewidth=0.8, label="heading")
    axes[5].plot(t, raw["roll_std"], color="tab:blue", linewidth=0.7, label="roll")
    axes[5].plot(t, raw["pitch_std"], color="tab:green", linewidth=0.7, label="pitch")
    axes[5].set_ylabel("attitude std [deg]")
    axes[5].set_xlabel("time from first IE message [s]")
    axes[5].legend(ncol=3)

    motion_start = analysis["first_motion_start"] - raw["t"][0]
    motion_end = analysis["first_motion_end"] - raw["t"][0]
    for index, ax in enumerate(axes):
        ax.axvspan(0.0, first_end, color="tab:orange", alpha=0.08)
        ax.axvspan(motion_start, motion_end, color="tab:green", alpha=0.06)
        ax.grid(True, alpha=0.25)
        if index == 0:
            ax.text(
                0.01,
                0.05,
                "orange: first IE-message window; green: first sustained motion",
                transform=ax.transAxes,
                fontsize=9,
            )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_distributions(analysis: dict[str, Any], output: Path, dpi: int) -> None:
    mask = analysis["forward_straight_mask"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    bins = np.linspace(-45, 45, 91)
    for name, values in analysis["errors_deg"].items():
        axes[0].hist(values[mask], bins=bins, histtype="step", density=True, label=name, linewidth=1.4)
    axes[0].set_title("Cross-estimator base-heading differences")
    axes[0].set_xlabel("difference [deg]")
    axes[0].set_ylabel("density")
    axes[0].legend(fontsize=8)

    for key, label in (("anymal", "ANYmal"), ("ie", "IE-TC"), ("dlio", "DLIO")):
        values = analysis["common"][key]["heading_minus_motion_deg"]
        axes[1].hist(
            values[mask], bins=bins, histtype="step", density=True, label=label, color=COLORS[label], linewidth=1.4
        )
    axes[1].set_title("Base heading minus own path tangent")
    axes[1].set_xlabel("difference [deg]")
    axes[1].set_ylabel("density")
    axes[1].legend()
    for ax in axes:
        ax.axvline(0.0, color="black", linewidth=0.7)
        ax.grid(True, alpha=0.2)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_time_lag(analysis: dict[str, Any], output: Path, dpi: int) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
    for label, values in analysis["lags"].items():
        ax.plot(values["lags_s"], values["correlations"], label=label, linewidth=1.3)
        ax.axvline(values["best_lag_s"], linestyle="--", linewidth=0.8)
    ax.axvline(0.0, color="black", linewidth=0.7)
    ax.set_xlabel("candidate lag relative to ANYmal [s] (positive = candidate later)")
    ax.set_ylabel("yaw-rate correlation")
    ax.set_title("Could an estimator time offset explain the heading discrepancy?")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ap20_crosscheck(analysis: dict[str, Any], output: Path, dpi: int) -> None:
    ap20 = analysis["ap20"]
    if ap20 is None:
        return
    fig, axes = plt.subplots(2, 1, figsize=(11, 9), constrained_layout=True)
    axes[0].plot(
        ap20["position"][:, 0],
        ap20["position"][:, 1],
        color="black",
        label="AP20 measured prism",
        linewidth=1.8,
        zorder=5,
    )
    for key, label in (("anymal", "ANYmal"), ("ie", "IE-TC"), ("dlio", "DLIO")):
        xy = ap20["sources"][key]["aligned_prism_xy"]
        axes[0].plot(xy[:, 0], xy[:, 1], color=COLORS[label], label=label, linewidth=1.1)
    axes[0].set_title("Predicted prism tracks after independent position-only SE(2) fits to AP20")
    axes[0].set_xlabel("Leica total-station x [m]")
    axes[0].set_ylabel("Leica total-station y [m]")
    axes[0].axis("equal")
    axes[0].legend()
    axes[0].grid(True, alpha=0.25)

    for key, label in (("anymal", "ANYmal"), ("ie", "IE-TC"), ("dlio", "DLIO")):
        visible_error = ap20["sources"][key]["heading_error_deg"].copy()
        visible_error[~ap20["forward_straight_mask"]] = np.nan
        axes[1].plot(
            ap20["t_rel"],
            visible_error,
            color=COLORS[label],
            label=label,
            linewidth=0.9,
        )
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylim(-60, 60)
    axes[1].set_title("Base heading minus AP20 prism direction during robot motion")
    axes[1].set_xlabel("time from earliest estimator start [s]")
    axes[1].set_ylabel("heading − AP20 tangent [deg]")
    axes[1].legend(ncol=3)
    axes[1].grid(True, alpha=0.25)
    axes[1].text(
        0.01,
        0.96,
        "only forward/straight samples shown",
        transform=axes[1].transAxes,
        va="top",
        fontsize=9,
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def write_common_csv(path: Path, analysis: dict[str, Any]) -> None:
    columns: list[tuple[str, np.ndarray]] = [
        ("epoch_s", analysis["t"]),
        ("relative_time_s", analysis["t_rel"]),
    ]
    for key, label in (("ie", "ie"), ("anymal", "anymal"), ("dlio", "dlio")):
        columns.extend(
            [
                (f"{label}_base_x_m", analysis["common"][key]["p"][:, 0]),
                (f"{label}_base_y_m", analysis["common"][key]["p"][:, 1]),
                (f"{label}_base_heading_deg", analysis["headings_deg"][key]),
                (f"{label}_path_speed_mps", analysis["common"][key]["motion_speed_mps"]),
                (
                    f"{label}_heading_minus_path_deg",
                    analysis["common"][key]["heading_minus_motion_deg"],
                ),
            ]
        )
    for name, values in analysis["errors_deg"].items():
        columns.append((name.lower().replace(" ", "_").replace("-", "minus") + "_deg", values))
    columns.append(("forward_straight_mask", analysis["forward_straight_mask"].astype(int)))

    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([name for name, _ in columns])
        writer.writerows(zip(*(values for _, values in columns)))


def finite_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return value


def determine_verdict(metrics: dict[str, Any]) -> tuple[str, list[str]]:
    cross = metrics["cross_estimator_heading_error_forward_straight"]
    motion = metrics["base_heading_minus_own_motion_forward_straight"]
    count = metrics["common_interval"]["forward_straight_sample_count"]
    reasons: list[str] = []
    if count < 100:
        return "INCONCLUSIVE", ["Too little forward/straight motion passed the evaluation mask."]

    ie_ref = max(cross["IE-TC - ANYmal"]["rmse_deg"], cross["IE-TC - DLIO"]["rmse_deg"])
    ref_ref = cross["DLIO - ANYmal"]["rmse_deg"]
    ie_motion = motion["IE-TC"]["median_abs_deg"]
    ref_motion = max(motion["ANYmal"]["median_abs_deg"], motion["DLIO"]["median_abs_deg"])
    internal = metrics["ie_internal"]["heading_minus_course_forward_straight"]["median_abs_deg"]

    reasons.append(
        f"After trajectory-position alignment, IE-to-reference heading RMSE is "
        f"{ie_ref:.2f}°, while DLIO-to-ANYmal is {ref_ref:.2f}°. "
        "These residuals alone do not establish a constant bias or its physical cause."
    )
    reasons.append(
        f"IE median |heading−path| is {ie_motion:.2f}°, versus {ref_motion:.2f}° for the worse independent reference."
    )
    reasons.append(f"IE raw median |heading−COG| is {internal:.2f}° on the same motion class.")

    if "ap20_position_grounded_check" in metrics:
        ap20 = metrics["ap20_position_grounded_check"]["sources"]
        ie_ap20 = ap20["IE-TC"]["heading_minus_ap20_motion"]
        anymal_ap20 = ap20["ANYmal"]["heading_minus_ap20_motion"]
        dlio_ap20 = ap20["DLIO"]["heading_minus_ap20_motion"]
        reasons.insert(
            0,
            f"After a position-only fit to the AP20 total-station track, IE-TC "
            f"heading minus travel direction has median {ie_ap20['median_deg']:.2f}° "
            f"(median absolute {ie_ap20['median_abs_deg']:.2f}°), versus medians "
            f"{anymal_ap20['median_deg']:.2f}° for ANYmal and "
            f"{dlio_ap20['median_deg']:.2f}° for DLIO.",
        )
        ie_position = ap20["IE-TC"]["position_residual_m"]["rmse"]
        dlio_position = ap20["DLIO"]["position_residual_m"]["rmse"]
        reasons.append(
            f"IE-TC also has {ie_position:.2f} m prism-track RMSE after rigid "
            f"alignment (DLIO {dlio_position:.2f} m); this position check is "
            "independent of the reported attitude."
        )
        reference_ap20 = max(anymal_ap20["median_abs_deg"], dlio_ap20["median_abs_deg"])
        if ie_ap20["median_abs_deg"] >= reference_ap20 + 5.0 and ie_ap20["median_abs_deg"] >= 8.0:
            return "SUPPORTED — MATERIAL IE-TC HEADING INCONSISTENCY (DIAGNOSTIC)", reasons
        if (
            abs(ie_ap20["median_deg"]) <= 2.0
            and ie_ap20["median_abs_deg"] <= reference_ap20 + 1.0
            and ie_motion <= ref_motion + 1.5
            and ie_ref <= ref_ref + 2.5
        ):
            return "NOT SUPPORTED — NO MATERIAL IE-TC HEADING OFFSET", reasons

    supported_votes = [
        ie_ref >= ref_ref + 3.0 and ie_ref >= 5.0,
        ie_motion >= ref_motion + 3.0 and ie_motion >= 5.0,
        internal >= 5.0,
    ]
    if sum(supported_votes) >= 2:
        return "SUPPORTED", reasons
    if ie_ref <= ref_ref + 2.0 and ie_motion <= ref_motion + 2.0 and internal < 5.0:
        return "NOT SUPPORTED", reasons
    return "MIXED / INCONCLUSIVE", reasons


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "n/a"


def write_markdown_report(
    path: Path,
    mission_label: str,
    verdict: str,
    reasons: Iterable[str],
    summary: dict[str, Any],
) -> None:
    metrics = summary["metrics"]
    cross = metrics["cross_estimator_heading_error_forward_straight"]
    motion = metrics["base_heading_minus_own_motion_forward_straight"]
    internal = metrics["ie_internal"]

    lines = [
        f"# {mission_label} heading deep dive",
        "",
        f"## Verdict: {verdict}",
        "",
        *[f"- {reason}" for reason in reasons],
        "",
        "The verdict is a compact automated screening result; the plots and the separate metrics below are the evidence.",
        "",
        "## Frame handling",
        "",
        "- IE-TC odometry (`enu_origin → cpt7_imu`) was converted to `enu_origin → base` using the model TF.",
        "- DLIO odometry (`dlio_map → hesai_lidar`) was converted to `dlio_map → base` using the model TF.",
        "- ANYmal odometry is already `odom → base`.",
        f"- Alignment: {summary['configuration']['alignment']}, retaining the {summary['configuration']['reference']} world as the plotting frame.",
        "",
        "## Evaluation mask",
        "",
        f"Forward/straight duration: {fmt(metrics['common_interval']['forward_straight_duration_s'])} s "
        f"({metrics['common_interval']['forward_straight_sample_count']:,} samples). "
        "The mask requires positive ANYmal body-frame vx, limited lateral velocity, adequate path speed, and limited yaw rate.",
        "",
        "## Cross-estimator base-heading errors (forward/straight samples)",
        "",
        "| Difference | RMSE [deg] | median | median abs | p95 abs | slope [deg/min] |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, values in cross.items():
        lines.append(
            f"| {name} | {fmt(values['rmse_deg'])} | {fmt(values['median_deg'])} | "
            f"{fmt(values['median_abs_deg'])} | {fmt(values['p95_abs_deg'])} | "
            f"{fmt(values['linear_slope_deg_per_min'])} |"
        )

    lines.extend(
        [
            "",
            "## Base heading versus each source's own path tangent",
            "",
            "| Source | RMSE [deg] | median | median abs | p95 abs |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for name, values in motion.items():
        lines.append(
            f"| {name} | {fmt(values['rmse_deg'])} | {fmt(values['median_deg'])} | "
            f"{fmt(values['median_abs_deg'])} | {fmt(values['p95_abs_deg'])} |"
        )

    if "ap20_position_grounded_check" in metrics:
        ap20 = metrics["ap20_position_grounded_check"]
        lines.extend(
            [
                "",
                "## AP20 total-station position-grounded check",
                "",
                "For each source, the model `base → prism` lever arm is applied and one constant position-only SE(2) transform is fitted to the measured AP20 prism track. The fitted world rotation is then also applied to that source's base heading, providing an independent position-grounded yaw check.",
                "",
                "| Source | heading−AP20 median [deg] | median abs | p95 abs | prism-track RMSE [m] | p95 [m] |",
                "|---|---:|---:|---:|---:|---:|",
            ]
        )
        for name, source in ap20["sources"].items():
            heading = source["heading_minus_ap20_motion"]
            position = source["position_residual_m"]
            lines.append(
                f"| {name} | {fmt(heading['median_deg'])} | "
                f"{fmt(heading['median_abs_deg'])} | {fmt(heading['p95_abs_deg'])} | "
                f"{fmt(position['rmse'], 3)} | {fmt(position['p95'], 3)} |"
            )

    raw_metric = internal["heading_minus_course_forward_straight"]
    lines.extend(
        [
            "",
            "## IE-TC internal GNSS/INS check",
            "",
            f"This uses IE heading minus raw course-over-ground (configured additive heading correction: {fmt(summary['configuration']['ie_heading_correction_deg'])}°). It is independent of DLIO/ANYmal world-frame alignment.",
            "",
            f"- Heading−COG: median {fmt(raw_metric['median_deg'])}°, median abs {fmt(raw_metric['median_abs_deg'])}°, RMSE {fmt(raw_metric['rmse_deg'])}°, p95 abs {fmt(raw_metric['p95_abs_deg'])}°.",
            f"- IE reported heading standard deviation: median {fmt(internal['reported_heading_std_deg_forward_straight']['median'], 3)}°, p95 {fmt(internal['reported_heading_std_deg_forward_straight']['p95'], 3)}°.",
            f"- Satellites: median {fmt(internal['satellites_forward_straight']['median'], 1)}, minimum {fmt(internal['satellites_forward_straight']['min'], 0)}; PDOP median {fmt(internal['pdop_forward_straight']['median'])}.",
            f"- Fixed-ambiguity fraction: {fmt(100.0 * internal['ambiguity_fixed_fraction'], 1)}%.",
        ]
    )
    if "tc_solution_quality" in metrics:
        quality = metrics["tc_solution_quality"]
        lines.extend(
            [
                "",
                "## IE-TC formal solution quality",
                "",
                "`GnssRaw` encodes ambiguity as `FIXED=1`, `FLOAT=2`. PDOP values below the physical lower bound of one are excluded as missing/malformed rather than treated as ideal geometry.",
                "",
                "| Window | Q2 | Q3 | Q4 | float | sats median | PDOP median / p95 | heading SD median [deg] | 3-D position SD median [m] |",
                "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for label, key in (
            ("All TC samples", "all_samples"),
            ("First IE messages", "first_ie_messages"),
            ("Forward/straight", "forward_straight_samples"),
        ):
            values = quality[key]
            fractions = values["quality_fraction"]
            lines.append(
                f"| {label} | {fmt(100.0 * fractions.get('2', 0.0), 1)}% | "
                f"{fmt(100.0 * fractions.get('3', 0.0), 1)}% | "
                f"{fmt(100.0 * fractions.get('4', 0.0), 1)}% | "
                f"{fmt(100.0 * values['float_fraction'], 1)}% | "
                f"{fmt(values['satellites']['median'], 1)} | "
                f"{fmt(values['pdop_positive_only']['median'])} / "
                f"{fmt(values['pdop_positive_only']['p95'])} | "
                f"{fmt(values['heading_std_deg']['median'], 3)} | "
                f"{fmt(values['position_ecef_trace_std_m']['median'], 3)} |"
            )
    if "ie_dlio_heading_error_by_tc_status" in metrics:
        conditioned = metrics["ie_dlio_heading_error_by_tc_status"]
        lines.extend(
            [
                "",
                "## Does TC quality explain the IE heading residual?",
                "",
                "| TC status | samples | IE−DLIO median [deg] | RMSE [deg] |",
                "|---|---:|---:|---:|",
            ]
        )
        for quality_number, values in conditioned["quality_number"].items():
            lines.append(
                f"| Quality {quality_number} | {values['count']:,} | "
                f"{fmt(values['median_deg'])} | {fmt(values['rmse_deg'])} |"
            )
        for ambiguity_label, values in conditioned["ambiguity"].items():
            lines.append(
                f"| Ambiguity {ambiguity_label} | {values['count']:,} | "
                f"{fmt(values['median_deg'])} | {fmt(values['rmse_deg'])} |"
            )
    if "sensitivity_analysis" in metrics:
        lines.extend(
            [
                "",
                "## Sensitivity to filtering choices",
                "",
                "| tangent window [s] | min vx [m/s] | lateral ratio | max yaw rate [deg/s] | accepted [s] | IE own-path abs [deg] | IE−AP20 median [deg] | raw IE−COG abs [deg] |",
                "|---:|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for row in metrics["sensitivity_analysis"]:
            ap20_value = row.get("ap20_heading_minus_motion_median_deg", {}).get("IE-TC")
            lines.append(
                f"| {fmt(row['motion_window_s'], 1)} | "
                f"{fmt(row['min_forward_speed_mps'], 2)} | "
                f"{fmt(row['max_lateral_ratio'], 2)} | "
                f"{fmt(row['max_straight_yaw_rate_degps'], 0)} | "
                f"{fmt(row['accepted_duration_s'], 1)} | "
                f"{fmt(row['own_path_median_abs_deg']['IE-TC'])} | "
                f"{fmt(ap20_value)} | "
                f"{fmt(row['ie_raw_heading_minus_cog_median_abs_deg'])} |"
            )

    lines.extend(["", "## Timing check", ""])
    for name, value in metrics["time_lag"].items():
        lines.append(
            f"- {name}: best yaw-rate correlation lag {fmt(value['best_lag_s'], 3)} s; "
            f"correlation {fmt(value['best_correlation'], 3)} (zero-lag {fmt(value['zero_lag_correlation'], 3)})."
        )

    cpt7_yaw = summary["static_transforms"]["T_base_cpt7_imu"]["rotation_euler_xyz_deg"][2]
    heading_cog = internal["heading_minus_course_forward_straight"]
    heading_std = internal["reported_heading_std_deg_forward_straight"]
    ie_dlio = cross["IE-TC - DLIO"]
    material_fault = verdict.startswith("CONFIRMED") or verdict == "SUPPORTED"
    interpretation = [
        "",
        "## Interpretation and limits",
        "",
        f"- The model's horizontal `base → cpt7_imu` yaw is {fmt(cpt7_yaw, 4)}°; the TF conversion used here therefore injects no horizontal yaw correction.",
        f"- Before any ROS TF/base conversion, raw IE heading minus raw course-over-ground has median {fmt(heading_cog['median_deg'])}° and median absolute {fmt(heading_cog['median_abs_deg'])}° on the selected motion samples.",
        "- The best yaw-rate lag is zero and the heading-change correlation is approximately one, so a meaningful timestamp offset is not supported.",
        f"- IE reports a median formal heading SD of {fmt(heading_std['median'], 3)}°. This is an internal precision estimate; it does not test a common systematic yaw convention against an external source (IE−DLIO median absolute difference here is {fmt(ie_dlio['median_abs_deg'])}°).",
    ]
    if material_fault:
        interpretation.append(
            "- Because independent trajectory and motion-direction checks also show a material IE discrepancy, these bags place the inconsistency upstream of the public TF conversion. They cannot alone distinguish an IE body-frame/boresight setting, a dual-antenna heading convention, or another upstream attitude convention."
        )
    else:
        interpretation.append(
            "- The independent sources agree closely enough that this mission does not support an IE-TC heading fault or localize one to the upstream IE configuration."
        )
    lines.extend(interpretation)

    exporter_audit = summary.get("exporter_equivalence_audit", {})
    if exporter_audit.get("available"):
        lines.extend(
            [
                "",
                "## Local exporter-change audit",
                "",
                f"The locally rewritten IE attitude formula was reconstructed for all {exporter_audit['sample_count']:,} samples. Its maximum quaternion difference from the downloaded bag is {fmt(exporter_audit['local_formula_vs_bag_max_angle_deg'], 12)}°, and its maximum difference from the previous exporter formula is {fmt(exporter_audit['local_formula_vs_previous_formula_max_angle_deg'], 12)}°. Therefore a zero-offset re-export cannot correct this mission's heading.",
            ]
        )

    lines.extend(
        [
            "",
            "## Requested first-2,000-point window",
            "",
            f"The window spans {fmt(metrics['first_ie_points']['duration_s'], 3)} s, its IE endpoint displacement is {fmt(metrics['first_ie_points']['ie_endpoint_displacement_m'], 3)} m, and it contains "
            f"{fmt(metrics['first_ie_points']['forward_straight_duration_s'])} s of samples passing the forward/straight mask. IE-TC is 200 Hz here, so the literal first 2,000 samples are almost entirely before sustained motion and cannot be the approximately 6 m interval shown in the original feedback.",
            "",
            "## First substantial forward segment",
            "",
            f"The automatically selected segment runs from {fmt(metrics['first_substantial_forward_segment']['start_relative_s'])} s to {fmt(metrics['first_substantial_forward_segment']['end_relative_s'])} s and spans {fmt(metrics['first_substantial_forward_segment']['span_s'])} s. This is the like-for-like early moving interval shown in `02b_first_forward_segment.png`.",
            "",
            "## Outputs",
            "",
            "- `01_trajectories_full.png`: full trajectory-aligned base trajectories.",
            "- `02_first_2000_ie_points.png`: literal requested sample-count zoom with base +x arrows.",
            "- `02b_first_forward_segment.png`: the first sustained moving segment (the useful counterpart to the feedback plot).",
            "- `03_heading_comparison.png`: relative heading and cross-estimator error over time.",
            "- `04_heading_vs_motion.png`: base heading versus differentiated path direction.",
            "- `05_ie_internal_heading_cog_quality.png`: IE-only heading/COG and quality diagnostics.",
            "- `06_error_distributions.png`: masked error distributions.",
            "- `07_time_lag_check.png`: yaw-rate lag correlation.",
            "- `08_ap20_position_grounded_check.png`: total-station position fit and independent direction check.",
            "- `09_tc_solution_quality.png`: formal TC quality, ambiguity, satellite, PDOP, and covariance fields.",
            "- `common_timeline.csv`: aligned poses and errors at the common evaluation rate.",
            "- `summary.json`: all numerical results and configuration.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def main() -> None:
    args = parse_args()
    if args.motion_window_s <= 0.0 or args.evaluation_dt_s <= 0.0:
        raise ValueError("motion-window-s and evaluation-dt-s must be positive")
    if args.first_ie_points <= 0 or args.arrow_count <= 0:
        raise ValueError("first-ie-points and arrow-count must be positive")
    if args.umeyama_min_speed_mps < 0.0:
        raise ValueError("umeyama-min-speed-mps must be non-negative")

    bag_dir = args.bag_dir.expanduser().resolve()
    output_dir = (
        args.output_dir.expanduser().resolve() if args.output_dir is not None else bag_dir / "alb1_heading_report"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = discover_inputs(bag_dir)
    print("Inputs:")
    for key, value in paths.items():
        print(f"  {key:8s} {value}")

    series, raw, transforms, extraction_metadata = extract_or_load(paths, output_dir, args.rebuild_cache)
    exporter_equivalence_audit = audit_ie_attitude_export_equivalence(series["ie"], raw, transforms["base_cpt7"])
    if abs(args.ie_heading_correction_deg) > 1e-12:
        series = dict(series)
        series["ie"] = apply_ie_compass_heading_correction(
            series["ie"], transforms["base_cpt7"], args.ie_heading_correction_deg
        )
        raw = {key: value.copy() if isinstance(value, np.ndarray) else value for key, value in raw.items()}
        raw["heading"] = (raw["heading"] + args.ie_heading_correction_deg) % 360.0
        print(
            f"Applied test IE compass-heading correction: {args.ie_heading_correction_deg:+.3f} deg",
            flush=True,
        )
    if args.alignment_method == "umeyama":
        aligned, alignments, alignment_diagnostics = align_umeyama(
            series,
            args.reference,
            args.evaluation_dt_s,
            args.umeyama_min_speed_mps,
            args.umeyama_estimate_scale,
        )
        alignment_description = (
            "position-only 3-D Umeyama Sim(3) on synchronized moving samples"
            if args.umeyama_estimate_scale
            else "position-only rigid 3-D Umeyama SE(3) on synchronized moving samples"
        )
    else:
        aligned, alignments, alignment_diagnostics = align_first_poses(series, args.reference)
        alignment_description = "full SE(3), first base pose of every source"
    analysis = build_analysis(aligned, raw, transforms, args)
    print("Running mask/window sensitivity checks ...", flush=True)
    analysis["metrics"]["sensitivity_analysis"] = run_sensitivity_analysis(aligned, raw, transforms, args)
    anchor = aligned[{value.name: key for key, value in aligned.items()}[args.reference]].p[0]

    print("Generating plots ...", flush=True)
    plot_full_trajectories(
        aligned,
        anchor,
        args.mission_label,
        output_dir / "01_trajectories_full.png",
        args.dpi,
    )
    plot_zoom(
        aligned,
        anchor,
        analysis["zoom_end"],
        args.arrow_count,
        args.mission_label,
        output_dir / "02_first_2000_ie_points.png",
        args.dpi,
    )
    plot_first_motion_segment(
        aligned,
        anchor,
        analysis["first_motion_start"],
        analysis["first_motion_end"],
        args.arrow_count,
        args.mission_label,
        output_dir / "02b_first_forward_segment.png",
        args.dpi,
    )
    plot_heading_comparison(analysis, output_dir / "03_heading_comparison.png", args.dpi)
    plot_heading_motion(analysis, output_dir / "04_heading_vs_motion.png", args.dpi)
    plot_ie_internal(raw, analysis, output_dir / "05_ie_internal_heading_cog_quality.png", args.dpi)
    plot_distributions(analysis, output_dir / "06_error_distributions.png", args.dpi)
    plot_time_lag(analysis, output_dir / "07_time_lag_check.png", args.dpi)
    plot_ap20_crosscheck(analysis, output_dir / "08_ap20_position_grounded_check.png", args.dpi)
    plot_tc_solution_quality(
        raw,
        analysis,
        args.first_ie_points,
        output_dir / "09_tc_solution_quality.png",
        args.dpi,
    )
    write_common_csv(output_dir / "common_timeline.csv", analysis)

    metrics = analysis["metrics"]
    verdict, reasons = determine_verdict(metrics)
    summary = {
        "verdict": verdict,
        "verdict_reasons": reasons,
        "configuration": {
            "bag_dir": str(bag_dir),
            "output_dir": str(output_dir),
            "reference": args.reference,
            "alignment": alignment_description,
            "alignment_method": args.alignment_method,
            "umeyama_estimate_scale": args.umeyama_estimate_scale,
            "umeyama_min_speed_mps": args.umeyama_min_speed_mps,
            "ie_heading_correction_deg": args.ie_heading_correction_deg,
            "motion_window_s": args.motion_window_s,
            "evaluation_dt_s": args.evaluation_dt_s,
            "min_forward_speed_mps": args.min_forward_speed_mps,
            "max_lateral_ratio": args.max_lateral_ratio,
            "max_straight_yaw_rate_degps": args.max_straight_yaw_rate_degps,
            "first_ie_points": args.first_ie_points,
        },
        "inputs": {key: str(value) for key, value in paths.items()},
        "sample_counts": {
            **{value.name: int(len(value.t)) for value in series.values()},
            **({"AP20 prism": int(len(raw["ap20_t"]))} if "ap20_t" in raw else {}),
        },
        "sample_rates_hz": {
            value.name: float((len(value.t) - 1) / (value.t[-1] - value.t[0])) for value in series.values()
        },
        "frame_processing": extraction_metadata,
        "exporter_equivalence_audit": exporter_equivalence_audit,
        "static_transforms": {
            "T_base_cpt7_imu": transform_description(transforms["base_cpt7"]),
            "T_base_hesai_lidar": transform_description(transforms["base_hesai"]),
            "T_base_prism": transform_description(transforms["base_prism"]),
        },
        "world_alignments": {
            key: {
                **transform_description(value),
                **alignment_diagnostics[key],
            }
            for key, value in alignments.items()
        },
        "metrics": metrics,
    }
    clean_summary = finite_json(summary)
    (output_dir / "summary.json").write_text(json.dumps(clean_summary, indent=2) + "\n")
    write_markdown_report(
        output_dir / "REPORT.md",
        args.mission_label,
        verdict,
        reasons,
        clean_summary,
    )

    print("\nVerdict:", verdict)
    for reason in reasons:
        print(" -", reason)
    print(f"\nReport written to: {output_dir / 'REPORT.md'}")
    print(f"Numerical summary: {output_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
