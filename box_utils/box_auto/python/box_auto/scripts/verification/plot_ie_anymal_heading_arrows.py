#!/usr/bin/env python3
"""Generate complaint-style IE-TC/ANYmal base-heading arrow plots.

The report cache already contains actual odometry messages converted to the
robot ``base`` frame.  The stored position-only Umeyama world transforms are
applied here.  For the complaint-style display only, each trajectory receives
an additional translation so that its position at the beginning of the shown
window is at (0, 0).  No additional yaw/orientation alignment is performed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


COLORS = {"ANYmal": "tab:blue", "IE-TC": "tab:orange"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mission",
        action="append",
        required=True,
        metavar="LABEL=REPORT_DIR",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--first-ie-points", type=int, default=2000)
    parser.add_argument("--dpi", type=int, default=200)
    return parser.parse_args()


def alignment_matrix(description: dict[str, Any]) -> tuple[Rotation, np.ndarray]:
    rotation = Rotation.from_quat(description["rotation_quaternion_xyzw"])
    translation = np.asarray(description["translation_m"], dtype=np.float64)
    return rotation, translation


def apply_alignment(
    position: np.ndarray,
    quaternion: np.ndarray,
    description: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    rotation, translation = alignment_matrix(description)
    scale = float(description.get("scale", 1.0))
    aligned_position = scale * rotation.apply(position) + translation
    aligned_quaternion = (rotation * Rotation.from_quat(quaternion)).as_quat()
    return aligned_position, aligned_quaternion


def interpolate_position(t: np.ndarray, p: np.ndarray, query: float) -> np.ndarray:
    return np.asarray([np.interp(query, t, p[:, axis]) for axis in range(3)])


def load_mission(label: str, report_dir: Path, first_ie_points: int) -> dict[str, Any]:
    summary = json.loads((report_dir / "summary.json").read_text())
    if summary["configuration"]["alignment_method"] != "umeyama":
        raise ValueError(f"{label}: report was not generated with Umeyama alignment")
    if abs(float(summary["configuration"]["ie_heading_correction_deg"])) > 1e-12:
        raise ValueError(f"{label}: report used a nonzero IE heading correction")

    with np.load(report_dir / "extracted_heading_data.npz", allow_pickle=False) as cache:
        raw = {
            "IE-TC": {
                "t": np.asarray(cache["ie_t"], dtype=np.float64),
                "p": np.asarray(cache["ie_p"], dtype=np.float64),
                "q": np.asarray(cache["ie_q"], dtype=np.float64),
            },
            "ANYmal": {
                "t": np.asarray(cache["anymal_t"], dtype=np.float64),
                "p": np.asarray(cache["anymal_p"], dtype=np.float64),
                "q": np.asarray(cache["anymal_q"], dtype=np.float64),
            },
        }

    aligned: dict[str, dict[str, np.ndarray]] = {}
    alignment_keys = {"IE-TC": "ie", "ANYmal": "anymal"}
    for source, values in raw.items():
        p, q = apply_alignment(
            values["p"],
            values["q"],
            summary["world_alignments"][alignment_keys[source]],
        )
        aligned[source] = {"t": values["t"], "p": p, "q": q}

    ie_t = aligned["IE-TC"]["t"]
    actual_count = min(max(first_ie_points, 1), len(ie_t))
    first_window = (float(ie_t[0]), float(ie_t[actual_count - 1]))
    motion = summary["metrics"]["first_substantial_forward_segment"]
    motion_window = (float(motion["start_epoch_s"]), float(motion["end_epoch_s"]))
    return {
        "label": label,
        "aligned": aligned,
        "first_window": first_window,
        "first_count": actual_count,
        "first_forward_s": float(summary["metrics"]["first_ie_points"]["forward_straight_duration_s"]),
        "motion_window": motion_window,
        "motion_forward_s": float(motion["forward_straight_duration_s"]),
    }


def window_values(source: dict[str, np.ndarray], start: float, end: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mask = (source["t"] >= start) & (source["t"] <= end)
    if np.count_nonzero(mask) < 2:
        raise ValueError("Fewer than two samples in requested plot window")
    # Translation-only start overlay for the complaint-style visualization.
    origin = interpolate_position(source["t"], source["p"], start)
    return source["t"][mask], source["p"][mask] - origin, source["q"][mask]


def plot_source(
    ax: plt.Axes,
    label: str,
    t: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    arrow_length: float,
) -> None:
    color = COLORS[label]
    ax.plot(p[:, 0], p[:, 1], color=color, linewidth=1.5, label=label)
    arrow_count = min(28, len(t))
    indices = np.unique(np.linspace(0, len(t) - 1, arrow_count).astype(int))
    forward = Rotation.from_quat(q[indices]).apply(np.broadcast_to(np.array([1.0, 0.0, 0.0]), (len(indices), 3)))[:, :2]
    norm = np.linalg.norm(forward, axis=1)
    valid = norm > 1e-9
    forward[valid] /= norm[valid, None]
    ax.quiver(
        p[indices, 0],
        p[indices, 1],
        forward[:, 0],
        forward[:, 1],
        color=color,
        angles="xy",
        scale_units="xy",
        scale=1.0 / arrow_length,
        width=0.006,
        headwidth=4.0,
        headlength=5.0,
        alpha=0.8,
    )
    ax.scatter(p[0, 0], p[0, 1], s=65, color=color, edgecolor="black", zorder=5)
    ax.scatter(p[-1, 0], p[-1, 1], s=70, color=color, marker="x", linewidth=2.2, zorder=5)


def plot_window(
    missions: list[dict[str, Any]],
    window_key: str,
    output: Path,
    dpi: int,
) -> None:
    fig, axes = plt.subplots(1, len(missions), figsize=(8.2 * len(missions), 6.2), constrained_layout=True)
    axes = np.atleast_1d(axes)
    for ax, mission in zip(axes, missions):
        start, end = mission[window_key]
        prepared: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        all_xy: list[np.ndarray] = []
        for source in ("ANYmal", "IE-TC"):
            prepared[source] = window_values(mission["aligned"][source], start, end)
            all_xy.append(prepared[source][1][:, :2])
        stacked = np.vstack(all_xy)
        span = max(float(np.ptp(stacked[:, 0])), float(np.ptp(stacked[:, 1])), 0.25)
        arrow_length = float(np.clip(0.07 * span, 0.08, 1.2))
        for source in ("ANYmal", "IE-TC"):
            plot_source(ax, source, *prepared[source], arrow_length)

        duration = end - start
        if window_key == "first_window":
            subtitle = (
                f"first {mission['first_count']:,} IE messages, {duration:.2f} s\n"
                f"accepted forward/straight: {mission['first_forward_s']:.2f} s"
            )
        else:
            subtitle = (
                f"first sustained-motion window, {duration:.2f} s\n"
                f"accepted forward/straight: {mission['motion_forward_s']:.2f} s"
            )
        ax.set_title(f"{mission['label']}\n{subtitle}")
        ax.set_xlabel("display-aligned x [m]")
        ax.set_ylabel("display-aligned y [m]")
        ax.axis("equal")
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle(
        "Base-frame heading arrows after position-only Umeyama world alignment\n"
        "translation-only start overlay for display; ● start, × end",
        fontsize=14,
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    missions: list[dict[str, Any]] = []
    for specification in args.mission:
        if "=" not in specification:
            raise ValueError(f"Expected LABEL=REPORT_DIR, got {specification!r}")
        label, directory = specification.split("=", 1)
        missions.append(
            load_mission(
                label.strip(),
                Path(directory).expanduser().resolve(),
                args.first_ie_points,
            )
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    first_output = args.output_dir / "01_heap_eig_first_2000_heading_arrows.png"
    motion_output = args.output_dir / "02_heap_eig_first_motion_heading_arrows.png"
    plot_window(missions, "first_window", first_output, args.dpi)
    plot_window(missions, "motion_window", motion_output, args.dpi)
    print(first_output)
    print(motion_output)


if __name__ == "__main__":
    main()
