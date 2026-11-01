#!/usr/bin/env python3
"""Compare raw Inertial Explorer heading and course-over-ground across missions.

The inputs are report directories created by ``alb1_heading_deep_dive.py``.
Those reports cache the fields read directly from the IE-TC ``GnssRaw`` topic
and contain the common forward/straight evaluation mask.  This script does not
use ROS poses, TF, Umeyama rotations, DLIO attitude, or ANYmal attitude when it
computes the heading residual itself::

    residual_deg = wrap_180(orientation_hrp.x - course_over_ground.data)

Both source fields use the IE compass convention (clockwise from north).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mission",
        action="append",
        required=True,
        metavar="LABEL=REPORT_DIR",
        help="Mission label and deep-dive report directory; repeat for each mission.",
    )
    parser.add_argument(
        "--bag-mission",
        action="append",
        default=[],
        metavar="LABEL=BAG_DIR",
        help=(
            "Mission without a full DLIO report. Read IE-TC, ANYmal, and TF-model "
            "bags directly, and compute raw/quality plus the IE-ANYmal check."
        ),
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def wrap_deg(values: np.ndarray) -> np.ndarray:
    return (np.asarray(values, dtype=np.float64) + 180.0) % 360.0 - 180.0


def nearest_boolean(source_t: np.ndarray, source_mask: np.ndarray, query: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(source_t, query)
    indices = np.clip(indices, 1, len(source_t) - 1)
    left = indices - 1
    choose_right = np.abs(source_t[indices] - query) < np.abs(source_t[left] - query)
    return source_mask[np.where(choose_right, indices, left)]


def distribution(values: np.ndarray) -> dict[str, float | int]:
    absolute = np.abs(values)
    return {
        "count": int(len(values)),
        "mean_deg": float(np.mean(values)),
        "median_deg": float(np.median(values)),
        "std_deg": float(np.std(values)),
        "rmse_deg": float(np.sqrt(np.mean(np.square(values)))),
        "p05_deg": float(np.percentile(values, 5)),
        "p25_deg": float(np.percentile(values, 25)),
        "p75_deg": float(np.percentile(values, 75)),
        "p95_deg": float(np.percentile(values, 95)),
        "median_abs_deg": float(np.median(absolute)),
        "p95_abs_deg": float(np.percentile(absolute, 95)),
        "max_abs_deg": float(np.max(absolute)),
    }


def fraction(mask: np.ndarray) -> float:
    return float(np.mean(mask)) if len(mask) else math.nan


def temporal_statistics(
    raw_t: np.ndarray,
    residual: np.ndarray,
    selected: np.ndarray,
    bin_width_s: float = 10.0,
) -> dict[str, Any]:
    """Summarize persistence without treating 200 Hz samples as independent."""

    selected_t = raw_t[selected]
    selected_residual = residual[selected]
    start = float(selected_t[0])
    stop = float(selected_t[-1])
    bin_index = np.floor((selected_t - start) / bin_width_s).astype(int)
    bins: list[dict[str, float | int]] = []
    for index in np.unique(bin_index):
        values = selected_residual[bin_index == index]
        if len(values) < 20:
            continue
        bins.append(
            {
                "center_s": float((index + 0.5) * bin_width_s),
                "count": int(len(values)),
                "median_deg": float(np.median(values)),
                "median_abs_deg": float(np.median(np.abs(values))),
                "p25_deg": float(np.percentile(values, 25)),
                "p75_deg": float(np.percentile(values, 75)),
            }
        )
    bin_medians = np.asarray([value["median_deg"] for value in bins])
    bin_centers = np.asarray([value["center_s"] for value in bins])
    slope = float(np.polyfit(bin_centers / 60.0, bin_medians, 1)[0]) if len(bins) >= 2 else math.nan

    # A block bootstrap avoids the fictitious precision obtained by treating
    # every 200 Hz raw message as an independent observation.
    rng = np.random.default_rng(20260803)
    if len(bin_medians):
        resampled = rng.choice(bin_medians, size=(20000, len(bin_medians)), replace=True)
        bootstrap_medians = np.median(resampled, axis=1)
        block_ci = [
            float(np.percentile(bootstrap_medians, 2.5)),
            float(np.percentile(bootstrap_medians, 97.5)),
        ]
        block_p10_p90 = [
            float(np.percentile(bin_medians, 10)),
            float(np.percentile(bin_medians, 90)),
        ]
    else:
        block_ci = [math.nan, math.nan]
        block_p10_p90 = [math.nan, math.nan]

    span = max(stop - start, np.finfo(np.float64).eps)
    third_index = np.clip(((selected_t - start) / span * 3).astype(int), 0, 2)
    third_medians = [float(np.median(selected_residual[third_index == index])) for index in range(3)]
    absolute = np.abs(selected_residual)
    return {
        "bin_width_s": bin_width_s,
        "bin_count": len(bins),
        "bins": bins,
        "median_of_bin_medians_deg": float(np.median(bin_medians)),
        "block_median_bootstrap_95_ci_deg": block_ci,
        "bin_median_p10_p90_deg": block_p10_p90,
        "linear_slope_of_bin_medians_deg_per_min": slope,
        "chronological_third_medians_deg": {
            "early": third_medians[0],
            "middle": third_medians[1],
            "late": third_medians[2],
        },
        "negative_fraction": fraction(selected_residual < 0.0),
        "abs_gt_3_deg_fraction": fraction(absolute > 3.0),
        "abs_gt_5_deg_fraction": fraction(absolute > 5.0),
        "abs_gt_10_deg_fraction": fraction(absolute > 10.0),
        "abs_gt_15_deg_fraction": fraction(absolute > 15.0),
    }


def load_mission(label: str, report_dir: Path) -> dict[str, Any]:
    summary_path = report_dir / "summary.json"
    cache_path = report_dir / "extracted_heading_data.npz"
    timeline_path = report_dir / "common_timeline.csv"
    for path in (summary_path, cache_path, timeline_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    summary = json.loads(summary_path.read_text())
    if abs(float(summary["configuration"]["ie_heading_correction_deg"])) > 1e-12:
        raise ValueError(f"{label}: report used a nonzero IE heading correction")

    timeline = np.genfromtxt(timeline_path, delimiter=",", names=True)
    common_t = np.asarray(timeline["epoch_s"], dtype=np.float64)
    common_mask = np.asarray(timeline["forward_straight_mask"], dtype=bool)
    with np.load(cache_path, allow_pickle=False) as cache:
        raw_t = np.asarray(cache["raw_t"], dtype=np.float64)
        raw_heading = np.asarray(cache["raw_heading"], dtype=np.float64)
        raw_cog = np.asarray(cache["raw_cog"], dtype=np.float64)
        raw_velocity = np.asarray(cache["raw_velocity_ecef"], dtype=np.float64)
        raw_quality = np.asarray(cache["raw_quality"], dtype=np.float64)
        raw_ambiguity = np.asarray(cache["raw_ambiguity"], dtype=np.float64)
        raw_satellites = np.asarray(cache["raw_satellites"], dtype=np.float64)
        raw_pdop = np.asarray(cache["raw_pdop"], dtype=np.float64)
        raw_heading_std = np.asarray(cache["raw_heading_std"], dtype=np.float64)

    in_common = (raw_t >= common_t[0]) & (raw_t <= common_t[-1])
    forward_straight = np.zeros(len(raw_t), dtype=bool)
    forward_straight[in_common] = nearest_boolean(common_t, common_mask, raw_t[in_common])
    speed = np.linalg.norm(raw_velocity, axis=1)
    selected = (
        forward_straight
        & (speed >= float(summary["configuration"]["min_forward_speed_mps"]))
        & np.isfinite(raw_heading)
        & np.isfinite(raw_cog)
    )
    residual = wrap_deg(raw_heading - raw_cog)
    selected_residual = residual[selected]
    if not len(selected_residual):
        raise ValueError(f"{label}: no raw samples passed the common mask")

    metrics = distribution(selected_residual)
    expected = summary["metrics"]["ie_internal"]["heading_minus_course_forward_straight"]
    for key in ("count", "median_deg", "rmse_deg", "median_abs_deg", "p95_abs_deg"):
        if not np.isclose(float(metrics[key]), float(expected[key]), atol=1e-9, rtol=1e-9):
            raise RuntimeError(
                f"{label}: independent recomputation disagrees with summary for {key}: "
                f"{metrics[key]} vs {expected[key]}"
            )

    selected_pdop = selected & (raw_pdop >= 1.0)
    quality = {
        "fixed_fraction": fraction(raw_ambiguity[selected] == 1),
        "quality_1_fraction": fraction(raw_quality[selected] == 1),
        "satellites_median": float(np.median(raw_satellites[selected])),
        "pdop_median": float(np.median(raw_pdop[selected_pdop])),
        "heading_std_median_deg": float(np.median(raw_heading_std[selected])),
    }
    temporal = temporal_statistics(raw_t, residual, selected)
    cross = summary["metrics"]["cross_estimator_heading_error_forward_straight"]
    own_path = summary["metrics"]["base_heading_minus_own_motion_forward_straight"]
    ap20 = summary["metrics"].get("ap20_position_grounded_check")
    all_quality = summary["metrics"]["tc_solution_quality"]["all_samples"]
    external = {
        "cross_estimator": cross,
        "own_path": own_path,
        "ap20": ap20,
        "all_tc_quality": {
            "fixed_fraction": all_quality["fixed_fraction"],
            "quality_1_fraction": all_quality["quality_fraction"].get("1", 0.0),
            "satellites_median": all_quality["satellites"]["median"],
            "pdop_median": all_quality["pdop_positive_only"]["median"],
            "heading_std_median_deg": all_quality["heading_std_deg"]["median"],
        },
        "availability": {
            "DLIO": True,
            "AP20": ap20 is not None,
            "IE-ANYmal": True,
        },
    }
    return {
        "label": label,
        "report_dir": str(report_dir),
        "t": raw_t,
        "residual": residual,
        "selected": selected,
        "metrics": metrics,
        "quality": quality,
        "temporal": temporal,
        "external": external,
        "source_note": "Full IE-TC/ANYmal/DLIO deep-dive report",
        "forward_straight_duration_s": float(summary["metrics"]["common_interval"]["forward_straight_duration_s"]),
    }


def unique_file(directory: Path, pattern: str) -> Path:
    matches = sorted(directory.glob(pattern))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {pattern!r} in {directory}, found {len(matches)}")
    return matches[0]


def load_bag_mission(label: str, bag_dir: Path) -> dict[str, Any]:
    """Load a mission lacking DLIO/AP20 while preserving the shared mask."""

    import alb1_heading_deep_dive as deep

    ie_bag = unique_file(bag_dir, "*_cpt7_ie_tc.bag")
    anymal_bag = unique_file(bag_dir, "*_anymal_state.bag")
    tf_bag = unique_file(bag_dir, "*_tf_model.bag")
    raw = deep.read_ie_raw(ie_bag)
    ie_sensor = deep.read_odometry(ie_bag, deep.IE_TOPIC, "IE-TC")
    anymal = deep.read_odometry(anymal_bag, deep.ANYMAL_TOPIC, "ANYmal")
    edges, _ = deep.read_static_graph(tf_bag)
    t_base_cpt7, _ = deep.lookup_static(edges, "base", "cpt7_imu")
    ie = deep.transform_sensor_pose_to_base(ie_sensor, t_base_cpt7)
    aligned, _, alignment_diagnostics = deep.align_umeyama(
        {"ie": ie, "anymal": anymal},
        "ANYmal",
        evaluation_dt_s=0.05,
        min_speed_mps=0.15,
        estimate_scale=False,
    )

    motion_window_s = 1.0
    evaluation_dt_s = 0.05
    min_forward_speed_mps = 0.2
    start = max(value.t[0] for value in aligned.values()) + 0.5 * motion_window_s
    end = min(value.t[-1] for value in aligned.values()) - 0.5 * motion_window_s
    common_t = np.arange(start, end, evaluation_dt_s, dtype=np.float64)
    common: dict[str, dict[str, np.ndarray]] = {}
    for key, series in aligned.items():
        p, q = deep.interpolate_pose(series, common_t)
        heading = deep.planar_heading(q)
        motion_heading, motion_speed, _ = deep.centered_motion(series, common_t, motion_window_s)
        common[key] = {
            "heading": heading,
            "motion_heading": motion_heading,
            "motion_speed": motion_speed,
            "heading_minus_motion": wrap_deg(np.rad2deg(heading - motion_heading)),
        }

    anymal_linear = deep.interpolate_vectors(aligned["anymal"].t, aligned["anymal"].linear_velocity, common_t)
    yaw_rate = deep.smooth_derivative_deg(common["anymal"]["heading"], evaluation_dt_s)
    forward = anymal_linear[:, 0] >= min_forward_speed_mps
    low_lateral = np.abs(anymal_linear[:, 1]) <= (0.35 * np.maximum(anymal_linear[:, 0], 1e-6))
    straight = np.abs(yaw_rate) <= 12.0
    moving = common["anymal"]["motion_speed"] >= 0.75 * min_forward_speed_mps
    common_mask = forward & low_lateral & straight & moving

    raw_t = raw["t"]
    raw_heading = raw["heading"]
    raw_cog = raw["cog"]
    in_common = (raw_t >= common_t[0]) & (raw_t <= common_t[-1])
    raw_forward_straight = np.zeros(len(raw_t), dtype=bool)
    raw_forward_straight[in_common] = nearest_boolean(common_t, common_mask, raw_t[in_common])
    raw_speed = np.linalg.norm(raw["velocity_ecef"], axis=1)
    selected = (
        raw_forward_straight & (raw_speed >= min_forward_speed_mps) & np.isfinite(raw_heading) & np.isfinite(raw_cog)
    )
    residual = wrap_deg(raw_heading - raw_cog)
    selected_residual = residual[selected]
    if not len(selected_residual):
        raise ValueError(f"{label}: no raw samples passed the common mask")
    metrics = distribution(selected_residual)
    temporal = temporal_statistics(raw_t, residual, selected)

    selected_pdop = selected & (raw["pdop"] >= 1.0) & (raw["satellites"] > 0.0)
    quality = {
        "fixed_fraction": fraction(raw["ambiguity"][selected] == 1),
        "quality_1_fraction": fraction(raw["quality"][selected] == 1),
        "satellites_median": float(np.median(raw["satellites"][selected])),
        "pdop_median": float(np.median(raw["pdop"][selected_pdop])),
        "heading_std_median_deg": float(np.median(raw["heading_std"][selected])),
    }
    all_quality = deep.tc_solution_quality_summary(raw, np.ones(len(raw_t), dtype=bool))
    ie_any_error = wrap_deg(np.rad2deg(common["ie"]["heading"] - common["anymal"]["heading"]))
    external = {
        "cross_estimator": {
            "IE-TC - DLIO": None,
            "IE-TC - ANYmal": deep.error_metrics(ie_any_error, common_mask),
            "DLIO - ANYmal": None,
        },
        "own_path": {
            "IE-TC": deep.error_metrics(common["ie"]["heading_minus_motion"], common_mask),
            "ANYmal": deep.error_metrics(common["anymal"]["heading_minus_motion"], common_mask),
            "DLIO": None,
        },
        "ap20": None,
        "all_tc_quality": {
            "fixed_fraction": all_quality["fixed_fraction"],
            "quality_1_fraction": all_quality["quality_fraction"].get("1", 0.0),
            "satellites_median": all_quality["satellites"]["median"],
            "pdop_median": all_quality["pdop_positive_only"]["median"],
            "heading_std_median_deg": all_quality["heading_std_deg"]["median"],
        },
        "availability": {
            "DLIO": False,
            "AP20": False,
            "IE-ANYmal": True,
        },
        "alignment": alignment_diagnostics,
    }
    return {
        "label": label,
        "report_dir": str(bag_dir),
        "t": raw_t,
        "residual": residual,
        "selected": selected,
        "metrics": metrics,
        "quality": quality,
        "temporal": temporal,
        "external": external,
        "forward_straight_duration_s": float(np.count_nonzero(common_mask) * evaluation_dt_s),
        "source_note": "Direct bags; DLIO and AP20 unavailable",
    }


def plot_time_series(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    fig, axes = plt.subplots(len(missions), 1, figsize=(15, 2.7 * len(missions)), sharey=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    for ax, mission in zip(axes, missions):
        selected = mission["selected"]
        time_s = mission["t"] - mission["t"][0]
        visible = np.where(selected, mission["residual"], np.nan)
        ax.plot(time_s, visible, linewidth=0.65, color="tab:orange")
        median = mission["metrics"]["median_deg"]
        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.axhline(median, color="tab:red", linestyle="--", linewidth=1.0)
        clipped = np.count_nonzero(selected & (np.abs(mission["residual"]) > 25.0))
        clipped_fraction = 100.0 * clipped / mission["metrics"]["count"]
        ax.set_ylim(-25.0, 25.0)
        ax.set_ylabel(f"{mission['label']}\nheading−COG [deg]")
        ax.grid(alpha=0.25)
        ax.text(
            0.995,
            0.94,
            f"median {median:+.2f}°, median |e| {mission['metrics']['median_abs_deg']:.2f}°\n"
            f"p95 |e| {mission['metrics']['p95_abs_deg']:.2f}°, >25° clipped {clipped_fraction:.2f}%",
            transform=ax.transAxes,
            ha="right",
            va="top",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
        )
    axes[-1].set_xlabel("time from first raw IE message [s]")
    fig.suptitle(
        "Raw IE-TC check: orientation_hrp.x − course_over_ground\n"
        "only samples in the common forward/straight motion mask",
        fontsize=14,
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_distributions(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    labels = [mission["label"] for mission in missions]
    values = [mission["residual"][mission["selected"]] for mission in missions]
    positions = np.arange(1, len(missions) + 1)
    fig, (ax_box, ax_bar) = plt.subplots(1, 2, figsize=(14, 5.5), constrained_layout=True)

    box = ax_box.boxplot(
        values,
        positions=positions,
        vert=False,
        whis=(5, 95),
        showfliers=False,
        patch_artist=True,
    )
    for patch in box["boxes"]:
        patch.set_facecolor("tab:orange")
        patch.set_alpha(0.55)
    ax_box.axvline(0.0, color="black", linewidth=0.9)
    ax_box.set_xlim(-25.0, 25.0)
    ax_box.set_yticks(positions, labels)
    ax_box.set_xlabel("raw heading − COG [deg]")
    ax_box.set_title("Signed distribution (whiskers: p5–p95)")
    ax_box.grid(axis="x", alpha=0.25)

    signed = np.array([mission["metrics"]["median_deg"] for mission in missions])
    absolute = np.array([mission["metrics"]["median_abs_deg"] for mission in missions])
    x = np.arange(len(missions))
    width = 0.36
    ax_bar.bar(x - width / 2, signed, width, label="signed median")
    ax_bar.bar(x + width / 2, absolute, width, label="median absolute")
    ax_bar.axhline(0.0, color="black", linewidth=0.9)
    ax_bar.set_xticks(x, labels)
    ax_bar.set_ylabel("degrees")
    ax_bar.set_title("Mission-level residual summary")
    ax_bar.legend()
    ax_bar.grid(axis="y", alpha=0.25)
    fig.suptitle("Raw IE heading/course consistency across missions", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ecdf(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    fig, (ax_signed, ax_absolute) = plt.subplots(1, 2, figsize=(14, 5.5), constrained_layout=True)
    for mission in missions:
        values = np.sort(mission["residual"][mission["selected"]])
        probability = np.arange(1, len(values) + 1) / len(values)
        ax_signed.plot(values, probability, label=mission["label"], linewidth=1.5)
        absolute = np.sort(np.abs(values))
        ax_absolute.plot(absolute, probability, label=mission["label"], linewidth=1.5)
    ax_signed.axvline(0.0, color="black", linewidth=0.8)
    ax_signed.set_xlim(-30.0, 30.0)
    ax_signed.set_xlabel("raw heading − COG [deg]")
    ax_signed.set_ylabel("empirical cumulative probability")
    ax_signed.set_title("Signed residual ECDF")
    ax_absolute.set_xlim(0.0, 30.0)
    ax_absolute.set_xlabel("|raw heading − COG| [deg]")
    ax_absolute.set_ylabel("empirical cumulative probability")
    ax_absolute.set_title("Absolute residual ECDF")
    for ax in (ax_signed, ax_absolute):
        ax.set_ylim(0.0, 1.0)
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("Full residual distributions (forward/straight motion)", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_temporal_bins(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    fig, axes = plt.subplots(len(missions), 1, figsize=(15, 2.7 * len(missions)), sharey=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    for ax, mission in zip(axes, missions):
        bins = mission["temporal"]["bins"]
        x = np.asarray([value["center_s"] for value in bins])
        median = np.asarray([value["median_deg"] for value in bins])
        p25 = np.asarray([value["p25_deg"] for value in bins])
        p75 = np.asarray([value["p75_deg"] for value in bins])
        ax.fill_between(x, p25, p75, color="tab:orange", alpha=0.25, label="within-bin IQR")
        ax.plot(x, median, color="tab:orange", marker="o", markersize=3, label="10 s median")
        ax.axhline(0.0, color="black", linewidth=0.8)
        thirds = mission["temporal"]["chronological_third_medians_deg"]
        slope = mission["temporal"]["linear_slope_of_bin_medians_deg_per_min"]
        ci = mission["temporal"]["block_median_bootstrap_95_ci_deg"]
        ax.set_ylabel(f"{mission['label']}\nheading−COG [deg]")
        ax.set_ylim(-30.0, 30.0)
        ax.grid(alpha=0.25)
        ax.text(
            0.995,
            0.94,
            f"third medians {thirds['early']:+.1f}°, {thirds['middle']:+.1f}°, {thirds['late']:+.1f}°\n"
            f"bin slope {slope:+.2f}°/min; block-median CI [{ci[0]:+.2f}, {ci[1]:+.2f}]°",
            transform=ax.transAxes,
            ha="right",
            va="top",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
        )
    axes[-1].set_xlabel("time from first selected forward/straight raw sample [s]")
    axes[0].legend(loc="lower left")
    fig.suptitle(
        "Time persistence / convergence diagnostic using 10-second blocks",
        fontsize=14,
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_external_evidence(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    labels = [mission["label"] for mission in missions]
    x = np.arange(len(missions))
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), constrained_layout=True)

    cross_names = ["IE-TC - DLIO", "IE-TC - ANYmal", "DLIO - ANYmal"]
    cross_labels = ["IE−DLIO", "IE−ANYmal", "DLIO−ANYmal"]
    width = 0.24
    for index, (key, name) in enumerate(zip(cross_names, cross_labels)):
        values = []
        for mission in missions:
            metric = mission["external"]["cross_estimator"].get(key)
            values.append(metric["median_deg"] if metric else np.nan)
        axes[0].bar(x + (index - 1) * width, values, width, label=name)
    axes[0].set_xticks(x, labels)
    axes[0].set_ylabel("signed median [deg], ENU convention")
    axes[0].set_title("Position-only Umeyama heading comparison")
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].legend()

    source_names = ["IE-TC", "ANYmal", "DLIO"]
    for index, source in enumerate(source_names):
        values = []
        for mission in missions:
            metric = mission["external"]["own_path"].get(source)
            values.append(metric["median_deg"] if metric else np.nan)
        axes[1].bar(x + (index - 1) * width, values, width, label=source)
    axes[1].set_xticks(x, labels)
    axes[1].set_ylabel("signed heading−own-path median [deg]")
    axes[1].set_title("Each estimator versus its own motion")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].legend()

    ap20_missions = [mission for mission in missions if mission["external"]["ap20"]]
    ap20_labels = [mission["label"] for mission in ap20_missions]
    ap20_x = np.arange(len(ap20_missions))
    for index, source in enumerate(source_names):
        values = [
            mission["external"]["ap20"]["sources"][source]["heading_minus_ap20_motion"]["median_deg"]
            for mission in ap20_missions
        ]
        axes[2].bar(ap20_x + (index - 1) * width, values, width, label=source)
    axes[2].set_xticks(ap20_x, ap20_labels)
    axes[2].set_ylabel("signed heading−AP20 direction median [deg]")
    axes[2].set_title("Independent AP20 direction check")
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].legend()

    for ax in axes:
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("Independent evidence locating the large residual on IE heading", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_quality_context(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    labels = [mission["label"] for mission in missions]
    x = np.arange(len(missions))
    quality = [mission["external"]["all_tc_quality"] for mission in missions]
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2), constrained_layout=True)

    width = 0.36
    fixed = 100.0 * np.asarray([value["fixed_fraction"] for value in quality])
    q1 = 100.0 * np.asarray([value["quality_1_fraction"] for value in quality])
    axes[0].bar(x - width / 2, fixed, width, label="fixed ambiguity")
    axes[0].bar(x + width / 2, q1, width, label="quality 1")
    axes[0].set_xticks(x, labels)
    axes[0].set_ylabel("fraction [%]")
    axes[0].set_ylim(0.0, 105.0)
    axes[0].set_title("IE-TC solution status")
    axes[0].legend()

    pdop = np.asarray([value["pdop_median"] for value in quality])
    satellites = np.asarray([value["satellites_median"] for value in quality])
    axes[1].bar(x - width / 2, pdop, width, label="PDOP median")
    axes[1].bar(x + width / 2, satellites, width, label="satellites median")
    axes[1].set_xticks(x, labels)
    axes[1].set_ylabel("reported value")
    axes[1].set_title("GNSS geometry / availability")
    axes[1].legend()

    formal = np.asarray([value["heading_std_median_deg"] for value in quality])
    raw_error = np.asarray([mission["metrics"]["median_abs_deg"] for mission in missions])
    external_reference = []
    for mission in missions:
        cross = mission["external"]["cross_estimator"]
        metric = cross.get("IE-TC - DLIO") or cross.get("IE-TC - ANYmal")
        external_reference.append(metric["median_abs_deg"])
    external_reference = np.asarray(external_reference)
    width3 = 0.25
    axes[2].bar(x - width3, formal, width3, label="formal heading SD")
    axes[2].bar(x, raw_error, width3, label="raw median |heading−COG|")
    axes[2].bar(
        x + width3,
        external_reference,
        width3,
        label="median |IE−reference| (DLIO, else ANYmal)",
    )
    axes[2].set_xticks(x, labels)
    axes[2].set_yscale("log")
    axes[2].set_ylabel("degrees (log scale)")
    axes[2].set_title("Internal precision versus observed discrepancy")
    axes[2].legend(fontsize=8)

    for ax in axes:
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("TC quality context: useful evidence, not proof of causation", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def write_csv(missions: list[dict[str, Any]], output: Path) -> None:
    metric_keys = list(missions[0]["metrics"])
    quality_keys = list(missions[0]["quality"])
    temporal_keys = [
        "negative_fraction",
        "abs_gt_3_deg_fraction",
        "abs_gt_5_deg_fraction",
        "abs_gt_10_deg_fraction",
        "abs_gt_15_deg_fraction",
        "linear_slope_of_bin_medians_deg_per_min",
    ]
    extra_keys = [
        "block_ci_low_deg",
        "block_ci_high_deg",
        "early_third_median_deg",
        "middle_third_median_deg",
        "late_third_median_deg",
    ]
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "mission",
                "forward_straight_duration_s",
                *metric_keys,
                *quality_keys,
                *temporal_keys,
                *extra_keys,
            ],
        )
        writer.writeheader()
        for mission in missions:
            temporal = mission["temporal"]
            thirds = temporal["chronological_third_medians_deg"]
            ci = temporal["block_median_bootstrap_95_ci_deg"]
            writer.writerow(
                {
                    "mission": mission["label"],
                    "forward_straight_duration_s": mission["forward_straight_duration_s"],
                    **mission["metrics"],
                    **mission["quality"],
                    **{key: temporal[key] for key in temporal_keys},
                    "block_ci_low_deg": ci[0],
                    "block_ci_high_deg": ci[1],
                    "early_third_median_deg": thirds["early"],
                    "middle_third_median_deg": thirds["middle"],
                    "late_third_median_deg": thirds["late"],
                }
            )


def write_report(missions: list[dict[str, Any]], output: Path) -> None:
    def signed_median(metric: dict[str, Any] | None) -> str:
        return f"{metric['median_deg']:+.2f}°" if metric else "n/a"

    material_issue: list[str] = []
    no_material_issue: list[str] = []
    for mission in missions:
        cross = mission["external"]["cross_estimator"]
        reference_metric = cross.get("IE-TC - DLIO") or cross.get("IE-TC - ANYmal")
        if (
            mission["metrics"]["median_abs_deg"] >= 5.0
            and reference_metric
            and reference_metric["median_abs_deg"] >= 5.0
        ):
            material_issue.append(mission["label"])
        else:
            no_material_issue.append(mission["label"])
    issue_text = ", ".join(material_issue)
    nominal_text = ", ".join(no_material_issue)
    lines = [
        f"# {len(missions)}-mission IE heading evidence report",
        "",
        "## Executive conclusion",
        "",
        f"- Material, persistent IE heading inconsistency: **{issue_text}**. The raw IE compass heading disagrees with IE's own GNSS course, and available independent references place the large systematic component on IE heading rather than on one reference trajectory.",
        f"- No comparable material systematic error: **{nominal_text}**. Their raw central residuals and available external comparisons remain much smaller.",
        f"- Saying that heading and COG disagree is not, by itself, identical to proving that the entire IE filter did not converge. It proves an internal heading/velocity-direction inconsistency. In {issue_text}, the available external evidence indicates that IE heading did not converge to the correct physical heading, or converged precisely to a systematically wrong convention/calibration.",
        "",
        "## Raw-message check and conventions",
        "",
        "This check uses fields read directly from each IE-TC `GnssRaw` message:",
        "",
        "`wrap_180(orientation_hrp.x - course_over_ground.data)`",
        "",
        "Both fields are compass angles (clockwise from north). No ROS pose quaternion, TF, Umeyama transform, DLIO attitude, or ANYmal attitude participates in the residual calculation. The ANYmal-derived forward/straight mask is used only to select periods when heading should be comparable to course-over-ground.",
        "",
        "A negative raw residual means the numeric compass heading is counter-clockwise from COG. ROS ENU yaw increases in the opposite angular convention, so compare the magnitudes—not raw signs—against the ENU cross-estimator tables.",
        "",
        "## Signed median and full distribution",
        "",
        "The signed median measures systematic centering. Median absolute error measures typical magnitude without cancellation. IQR describes the middle 50%; p95 and RMSE are tail-sensitive and are inflated by brief COG failures.",
        "",
        "| Mission | samples | motion [s] | mean | signed median | IQR [deg] | median abs | RMSE | p95 abs | max abs | negative | |e|>5° | |e|>10° |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for mission in missions:
        metric = mission["metrics"]
        temporal = mission["temporal"]
        lines.append(
            f"| {mission['label']} | {metric['count']:,} | "
            f"{mission['forward_straight_duration_s']:.2f} | {metric['mean_deg']:+.2f}° | "
            f"{metric['median_deg']:+.2f}° | {metric['p25_deg']:+.2f} to {metric['p75_deg']:+.2f} | "
            f"{metric['median_abs_deg']:.2f}° | {metric['rmse_deg']:.2f}° | "
            f"{metric['p95_abs_deg']:.2f}° | {metric['max_abs_deg']:.2f}° | "
            f"{100.0 * temporal['negative_fraction']:.1f}% | "
            f"{100.0 * temporal['abs_gt_5_deg_fraction']:.1f}% | "
            f"{100.0 * temporal['abs_gt_10_deg_fraction']:.1f}% |"
        )
    lines.extend(
        [
            "",
            "### Distribution interpretation",
            "",
            "- ALB-1 and LMB-1 have shifted central distributions, not merely a few outliers. Their middle 50% stays predominantly negative.",
            "- KAB2 has a smaller but still material positive shift and a strongly time-varying residual: 55.5% of accepted samples exceed 5° absolute error.",
            f"- The missions without a material central offset ({nominal_text}) can still have nontrivial RMSE/p95 values from intermittent COG spikes; those tails alone do not establish a constant heading rotation.",
            "- COG becomes noisy at low speed and can differ from body heading during sideslip or turning; those are the reasons for applying the shared forward/straight and speed mask.",
            "",
            "## Time persistence and the convergence question",
            "",
            "Raw samples arrive at approximately 200 Hz and are strongly correlated. The table therefore also uses 10-second block medians. The interval is a bootstrap interval for the median of those blocks, not a falsely tiny independent-sample confidence interval.",
            "",
            "| Mission | 10 s blocks | early / middle / late median [deg] | block-median 95% interval [deg] | block p10–p90 [deg] | slope [deg/min] |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for mission in missions:
        temporal = mission["temporal"]
        thirds = temporal["chronological_third_medians_deg"]
        ci = temporal["block_median_bootstrap_95_ci_deg"]
        spread = temporal["bin_median_p10_p90_deg"]
        lines.append(
            f"| {mission['label']} | {temporal['bin_count']} | "
            f"{thirds['early']:+.2f} / {thirds['middle']:+.2f} / {thirds['late']:+.2f} | "
            f"{ci[0]:+.2f} to {ci[1]:+.2f} | {spread[0]:+.2f} to {spread[1]:+.2f} | "
            f"{temporal['linear_slope_of_bin_medians_deg_per_min']:+.2f} |"
        )

    lines.extend(
        [
            "",
            "`Did not converge` needs qualification:",
            "",
            "- If convergence means *IE produced a stable numerical INS solution with small formal covariance*, the messages alone do not prove non-convergence. A filter can be stable and overconfident.",
            f"- If convergence means *heading approached the correct physical base heading*, {issue_text} did not achieve that outcome over these missions. Their available external heading residuals remain material; KAB2 has lower fault-localization confidence because DLIO/AP20 are absent.",
            "- Plausible mechanisms include inadequate heading observability under poor GNSS conditions, wrong initial/mounting/boresight configuration, or convergence to a systematically rotated convention. These public bags do not uniquely distinguish them.",
            "",
            "## IE-TC quality and formal convergence indicators",
            "",
            "These are all-sample TC fields. `FIXED=1`; Quality 1 is best. Formal heading SD is the estimator's internal precision, not external accuracy.",
            "",
            "| Mission | fixed ambiguity | Quality 1 | sats median | PDOP median | formal heading SD [deg] | raw median abs [deg] | IE−reference median abs [deg] | reference |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---|",
        ]
    )
    for mission in missions:
        quality = mission["external"]["all_tc_quality"]
        cross = mission["external"]["cross_estimator"]
        ie_reference = cross.get("IE-TC - DLIO") or cross.get("IE-TC - ANYmal")
        reference_label = "DLIO" if cross.get("IE-TC - DLIO") else "ANYmal"
        lines.append(
            f"| {mission['label']} | {100.0 * quality['fixed_fraction']:.1f}% | "
            f"{100.0 * quality['quality_1_fraction']:.1f}% | "
            f"{quality['satellites_median']:.1f} | {quality['pdop_median']:.2f} | "
            f"{quality['heading_std_median_deg']:.3f} | "
            f"{mission['metrics']['median_abs_deg']:.2f} | "
            f"{ie_reference['median_abs_deg']:.2f} | {reference_label} |"
        )

    lines.extend(
        [
            "",
            f"ALB-1 and KAB2 are entirely ambiguity-float with no Quality 1 samples; LMB-1 is overwhelmingly float and Quality 2/3. The nominal missions with fixed/Quality-1 solutions provide a useful contrast. Across only {len(missions)} missions this remains association, not proof of causality. The small formal heading SD in the affected missions compared with the external discrepancy shows that formal covariance did not capture the systematic error.",
            "",
            "## Independent DLIO and ANYmal evidence",
            "",
            "Each trajectory is converted to `base`, synchronized, and aligned to the ANYmal world using a unit-scale, position-only Umeyama fit. Pose orientations are not used to estimate the fit. Signed medians below use ROS ENU convention.",
            "",
            "| Mission | IE−DLIO median | IE−ANYmal median | DLIO−ANYmal median | IE own-path median | DLIO own-path median | ANYmal own-path median |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for mission in missions:
        cross = mission["external"]["cross_estimator"]
        own = mission["external"]["own_path"]
        lines.append(
            f"| {mission['label']} | {signed_median(cross.get('IE-TC - DLIO'))} | "
            f"{signed_median(cross.get('IE-TC - ANYmal'))} | "
            f"{signed_median(cross.get('DLIO - ANYmal'))} | "
            f"{signed_median(own.get('IE-TC'))} | {signed_median(own.get('DLIO'))} | "
            f"{signed_median(own.get('ANYmal'))} |"
        )

    lines.extend(
        [
            "",
            "The critical control is DLIO−ANYmal: it is approximately +0.8° in ALB-1 and +0.3° in LMB-1, while IE is approximately +10–12° away from both. Two independent reference estimators therefore agree with each other much better than either agrees with IE. Missions without a DLIO bag retain the IE−ANYmal and own-path checks but cannot provide that two-reference control.",
            "",
            "## Independent AP20 direction evidence",
            "",
            "For each estimator, the model base-to-prism lever arm is applied and a separate position-only SE(2) fit maps its predicted prism track to measured AP20 positions. The fitted world yaw is then applied to base heading, which is compared with AP20's measured direction of travel. EIG-1 and KAB2 have no AP20 bag.",
            "",
            "| Mission | IE−AP20 median | DLIO−AP20 median | ANYmal−AP20 median | IE prism RMSE [m] | DLIO prism RMSE [m] | ANYmal prism RMSE [m] |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for mission in missions:
        ap20 = mission["external"]["ap20"]
        if not ap20:
            lines.append(f"| {mission['label']} | n/a | n/a | n/a | n/a | n/a | n/a |")
            continue
        source = ap20["sources"]
        lines.append(
            f"| {mission['label']} | "
            f"{source['IE-TC']['heading_minus_ap20_motion']['median_deg']:+.2f}° | "
            f"{source['DLIO']['heading_minus_ap20_motion']['median_deg']:+.2f}° | "
            f"{source['ANYmal']['heading_minus_ap20_motion']['median_deg']:+.2f}° | "
            f"{source['IE-TC']['position_residual_m']['rmse']:.3f} | "
            f"{source['DLIO']['position_residual_m']['rmse']:.3f} | "
            f"{source['ANYmal']['position_residual_m']['rmse']:.3f} |"
        )

    lines.extend(
        [
            "",
            "AP20 independently reproduces the separation: ALB-1 IE is about +13.0° while DLIO/ANYmal are about +1.3°/−0.5°; LMB-1 IE is about +14.7° while DLIO/ANYmal are about +2.5°/+1.8°. HEAP-1 places all three between about +1° and +2°. Thus the large ALB/LMB component follows IE heading, not the AP20 motion direction or a single localization reference.",
            "",
            "## Mission-level diagnosis",
            "",
        ]
    )
    for mission in missions:
        cross = mission["external"]["cross_estimator"]
        quality = mission["external"]["all_tc_quality"]
        ap20 = mission["external"]["ap20"]
        diagnosis = (
            "material IE heading inconsistency"
            if mission["label"] in material_issue
            else "no comparable material systematic heading inconsistency"
        )
        details = [
            f"raw median {mission['metrics']['median_deg']:+.2f}°",
            f"IE−ANYmal {signed_median(cross.get('IE-TC - ANYmal'))}",
        ]
        if cross.get("IE-TC - DLIO"):
            details.append(f"IE−DLIO {signed_median(cross['IE-TC - DLIO'])}")
        else:
            details.append("DLIO unavailable")
        if ap20:
            details.append(f"IE−AP20 {ap20['sources']['IE-TC']['heading_minus_ap20_motion']['median_deg']:+.2f}°")
        else:
            details.append("AP20 unavailable")
        details.append(f"fixed {100.0 * quality['fixed_fraction']:.1f}%")
        details.append(f"Quality 1 {100.0 * quality['quality_1_fraction']:.1f}%")
        lines.append(f"- **{mission['label']}: {diagnosis}.** " + "; ".join(details) + ".")

    lines.extend(
        [
            "",
            "## Limits",
            "",
            "- Heading need not equal COG during real sideslip, reversing, turning, or near-zero motion; the mask reduces but cannot mathematically eliminate those effects.",
            "- Umeyama and AP20 fits establish constant world-frame rotations from positions. They intentionally remove arbitrary map yaw while preserving heading-versus-trajectory inconsistency.",
            "- Missions without DLIO/AP20 have weaker fault localization and are explicitly marked as such.",
            "- The evidence identifies a mission-dependent upstream IE heading problem, not the unique root configuration. Resolving boresight, dual-antenna convention, initialization, and IE processing settings requires the original IE project/configuration and processing logs.",
            "",
            "## Outputs",
            "",
            "- `01_raw_heading_minus_cog_time.png`: selected 200 Hz raw residual over time.",
            "- `02_raw_heading_minus_cog_distributions.png`: box distributions and mission medians.",
            "- `03_raw_heading_minus_cog_ecdf.png`: signed and absolute empirical CDFs.",
            "- `04_raw_heading_minus_cog_10s_bins.png`: time persistence/convergence diagnostic.",
            "- `05_external_heading_evidence.png`: Umeyama, own-path, and AP20 signed medians.",
            "- `06_tc_quality_context.png`: solution status, GNSS geometry, and formal-vs-observed errors.",
            "- `raw_heading_minus_cog_metrics.csv`: detailed aggregate statistics.",
            "- `raw_heading_minus_cog_summary.json`: machine-readable raw, temporal, quality, and external results.",
        ]
    )
    output.write_text("\n".join(lines) + "\n")


def main() -> None:
    args = parse_args()
    missions: list[dict[str, Any]] = []
    for specification in args.mission:
        if "=" not in specification:
            raise ValueError(f"Expected LABEL=REPORT_DIR, got {specification!r}")
        label, directory = specification.split("=", 1)
        missions.append(load_mission(label.strip(), Path(directory).expanduser().resolve()))
    for specification in args.bag_mission:
        if "=" not in specification:
            raise ValueError(f"Expected LABEL=BAG_DIR, got {specification!r}")
        label, directory = specification.split("=", 1)
        missions.append(load_bag_mission(label.strip(), Path(directory).expanduser().resolve()))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    plot_time_series(missions, args.output_dir / "01_raw_heading_minus_cog_time.png", args.dpi)
    plot_distributions(
        missions,
        args.output_dir / "02_raw_heading_minus_cog_distributions.png",
        args.dpi,
    )
    plot_ecdf(
        missions,
        args.output_dir / "03_raw_heading_minus_cog_ecdf.png",
        args.dpi,
    )
    plot_temporal_bins(
        missions,
        args.output_dir / "04_raw_heading_minus_cog_10s_bins.png",
        args.dpi,
    )
    plot_external_evidence(
        missions,
        args.output_dir / "05_external_heading_evidence.png",
        args.dpi,
    )
    plot_quality_context(
        missions,
        args.output_dir / "06_tc_quality_context.png",
        args.dpi,
    )
    write_csv(missions, args.output_dir / "raw_heading_minus_cog_metrics.csv")
    serializable = [
        {
            "mission": mission["label"],
            "report_dir": mission["report_dir"],
            "forward_straight_duration_s": mission["forward_straight_duration_s"],
            "metrics": mission["metrics"],
            "quality": mission["quality"],
            "temporal": mission["temporal"],
            "external": mission["external"],
            "source_note": mission["source_note"],
        }
        for mission in missions
    ]
    (args.output_dir / "raw_heading_minus_cog_summary.json").write_text(json.dumps(serializable, indent=2) + "\n")
    write_report(missions, args.output_dir / "REPORT.md")
    for mission in missions:
        metric = mission["metrics"]
        print(
            f"{mission['label']}: median={metric['median_deg']:+.3f} deg, "
            f"median_abs={metric['median_abs_deg']:.3f} deg, "
            f"RMSE={metric['rmse_deg']:.3f} deg, p95_abs={metric['p95_abs_deg']:.3f} deg"
        )
    print(f"Report written to {args.output_dir / 'REPORT.md'}")


if __name__ == "__main__":
    main()
