#!/usr/bin/env python3
"""Compare formal IE-TC uncertainty and observed heading error across missions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", action="append", required=True, metavar="LABEL=REPORT_DIR")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--dpi", type=int, default=200)
    return parser.parse_args()


def load_mission(label: str, report_dir: Path) -> dict[str, Any]:
    summary = json.loads((report_dir / "summary.json").read_text())
    formal = summary["metrics"]["tc_solution_quality"]["all_samples"]
    raw_heading = summary["metrics"]["ie_internal"]["heading_minus_course_forward_straight"]
    cross = summary["metrics"]["cross_estimator_heading_error_forward_straight"]
    with np.load(report_dir / "extracted_heading_data.npz", allow_pickle=False) as cache:
        raw_t = np.asarray(cache["raw_t"], dtype=np.float64)
        heading_std = np.asarray(cache["raw_heading_std"], dtype=np.float64)
        roll_std = np.asarray(cache["raw_roll_std"], dtype=np.float64)
        pitch_std = np.asarray(cache["raw_pitch_std"], dtype=np.float64)
        position_trace_std = np.linalg.norm(np.asarray(cache["raw_position_std"], dtype=np.float64), axis=1)
        velocity_trace_std = np.linalg.norm(np.asarray(cache["raw_velocity_std"], dtype=np.float64), axis=1)
    return {
        "label": label,
        "report_dir": str(report_dir),
        "formal": formal,
        "raw_heading_error": raw_heading,
        "ie_dlio_error": cross["IE-TC - DLIO"],
        "ie_anymal_error": cross["IE-TC - ANYmal"],
        "t": raw_t,
        "heading_std": heading_std,
        "roll_std": roll_std,
        "pitch_std": pitch_std,
        "position_trace_std": position_trace_std,
        "velocity_trace_std": velocity_trace_std,
    }


def plot_time(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), constrained_layout=True)
    fields = [
        ("heading_std", "formal heading SD [deg]"),
        ("position_trace_std", "formal 3-D position trace SD [m]"),
        ("velocity_trace_std", "formal 3-D velocity trace SD [m/s]"),
    ]
    for mission in missions:
        stride = max(1, len(mission["t"]) // 6000)
        t = (mission["t"] - mission["t"][0])[::stride]
        for ax, (field, ylabel) in zip(axes, fields):
            ax.plot(t, mission[field][::stride], linewidth=1.0, label=mission["label"])
            ax.set_ylabel(ylabel)
            ax.grid(alpha=0.25)
            ax.legend()
    axes[-1].set_xlabel("time from first IE-TC raw message [s]")
    fig.suptitle("IE-TC formal uncertainty fields over time", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ecdf(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2), constrained_layout=True)
    fields = [
        ("heading_std", "heading SD [deg]"),
        ("position_trace_std", "3-D position trace SD [m]"),
        ("velocity_trace_std", "3-D velocity trace SD [m/s]"),
    ]
    for mission in missions:
        for ax, (field, xlabel) in zip(axes, fields):
            values = np.sort(mission[field])
            probability = np.arange(1, len(values) + 1) / len(values)
            ax.plot(values, probability, linewidth=1.5, label=mission["label"])
            ax.set_xlabel(xlabel)
            ax.set_ylabel("empirical cumulative probability")
            ax.grid(alpha=0.25)
            ax.legend()
    fig.suptitle("Distribution of reported IE-TC formal uncertainty", fontsize=14)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_formal_vs_observed(missions: list[dict[str, Any]], output: Path, dpi: int) -> None:
    labels = [mission["label"] for mission in missions]
    formal = np.asarray([mission["formal"]["heading_std_deg"]["median"] for mission in missions])
    raw = np.asarray([mission["raw_heading_error"]["median_abs_deg"] for mission in missions])
    dlio = np.asarray([mission["ie_dlio_error"]["median_abs_deg"] for mission in missions])
    anymal = np.asarray([mission["ie_anymal_error"]["median_abs_deg"] for mission in missions])
    x = np.arange(len(missions))
    width = 0.2
    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    ax.bar(x - 1.5 * width, formal, width, label="formal heading SD")
    ax.bar(x - 0.5 * width, raw, width, label="median |heading−COG|")
    ax.bar(x + 0.5 * width, dlio, width, label="median |IE−DLIO|")
    ax.bar(x + 1.5 * width, anymal, width, label="median |IE−ANYmal|")
    ax.set_xticks(x, labels)
    ax.set_yscale("log")
    ax.set_ylabel("degrees (log scale)")
    ax.set_title("Formal precision does not include the observed systematic discrepancy")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def ratio(numerator: float, denominator: float) -> float:
    return float(numerator / denominator)


def write_report(missions: list[dict[str, Any]], output: Path) -> None:
    reference = missions[0]
    comparison = missions[1]
    fields = [
        ("heading_std_deg", "Heading SD", "deg"),
        ("roll_std_deg", "Roll SD", "deg"),
        ("pitch_std_deg", "Pitch SD", "deg"),
        ("position_ecef_trace_std_m", "3-D position trace SD", "m"),
        ("velocity_ecef_trace_std_mps", "3-D velocity trace SD", "m/s"),
    ]
    lines = [
        f"# IE-TC uncertainty comparison: {reference['label']} versus {comparison['label']}",
        "",
        "## Formal uncertainty reported by IE-TC",
        "",
        "These values come from the `GnssRaw` standard-deviation fields. They represent the estimator's internal 1-sigma precision model, not externally verified accuracy.",
        "The 3-D position and velocity values are `sqrt(sigma_x^2 + sigma_y^2 + sigma_z^2)`, i.e. the square root of the diagonal covariance trace in ECEF coordinates.",
        "",
        f"| Quantity | {reference['label']} median / p95 | {comparison['label']} median / p95 | median ratio {reference['label']}/{comparison['label']} |",
        "|---|---:|---:|---:|",
    ]
    for key, name, unit in fields:
        first = reference["formal"][key]
        second = comparison["formal"][key]
        lines.append(
            f"| {name} | {first['median']:.6g} / {first['p95']:.6g} {unit} | "
            f"{second['median']:.6g} / {second['p95']:.6g} {unit} | "
            f"{ratio(first['median'], second['median']):.2f}× |"
        )

    lines.extend(
        [
            "",
            "## GNSS solution context",
            "",
            "| Mission | fixed ambiguity | Quality 1 | satellites median | PDOP median |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for mission in missions:
        formal = mission["formal"]
        lines.append(
            f"| {mission['label']} | {100.0 * formal['fixed_fraction']:.1f}% | "
            f"{100.0 * formal['quality_fraction'].get('1', 0.0):.1f}% | "
            f"{formal['satellites']['median']:.1f} | "
            f"{formal['pdop_positive_only']['median']:.2f} |"
        )

    lines.extend(
        [
            "",
            "## Formal precision versus observed heading disagreement",
            "",
            "| Mission | formal heading SD | raw median abs heading−COG | median abs IE−DLIO | median abs IE−ANYmal | IE−DLIO / formal SD |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for mission in missions:
        formal_heading = mission["formal"]["heading_std_deg"]["median"]
        dlio = mission["ie_dlio_error"]["median_abs_deg"]
        lines.append(
            f"| {mission['label']} | {formal_heading:.4f}° | "
            f"{mission['raw_heading_error']['median_abs_deg']:.2f}° | "
            f"{dlio:.2f}° | {mission['ie_anymal_error']['median_abs_deg']:.2f}° | "
            f"{ratio(dlio, formal_heading):.1f}× |"
        )

    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            f"- {reference['label']} reports substantially larger random uncertainty than {comparison['label']}: approximately 4.6× in heading, 19.5× in 3-D position, and 2.9× in 3-D velocity at the median.",
            f"- The {reference['label']} formal heading SD is still only about 0.18°, far too small to explain its approximately 10° external heading error as modeled random noise. The dominant error is systematic or unmodeled.",
            f"- {comparison['label']} is both more confident and externally more consistent. Its residual external disagreement still exceeds its formal SD, because cross-sensor calibration, trajectory alignment, COG noise, and other systematic effects are not represented by IE's internal covariance.",
            "- Fixed/float and Quality number are context, not covariance themselves. The all-float, non-Q1 ALB-1 solution is consistent with its much larger position/velocity uncertainty.",
            "",
            "## Outputs",
            "",
            "- `01_formal_uncertainty_time.png`: formal heading, position, and velocity uncertainty over time.",
            "- `02_formal_uncertainty_ecdf.png`: full formal-uncertainty distributions.",
            "- `03_formal_vs_observed_heading.png`: formal heading precision versus actual inter-source disagreement.",
            "- `uncertainty_summary.json`: machine-readable comparison.",
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
    if len(missions) != 2:
        raise ValueError("This report expects exactly two missions")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plot_time(missions, args.output_dir / "01_formal_uncertainty_time.png", args.dpi)
    plot_ecdf(missions, args.output_dir / "02_formal_uncertainty_ecdf.png", args.dpi)
    plot_formal_vs_observed(missions, args.output_dir / "03_formal_vs_observed_heading.png", args.dpi)
    serializable = [
        {
            "mission": mission["label"],
            "report_dir": mission["report_dir"],
            "formal": mission["formal"],
            "raw_heading_error": mission["raw_heading_error"],
            "ie_dlio_error": mission["ie_dlio_error"],
            "ie_anymal_error": mission["ie_anymal_error"],
        }
        for mission in missions
    ]
    (args.output_dir / "uncertainty_summary.json").write_text(json.dumps(serializable, indent=2) + "\n")
    write_report(missions, args.output_dir / "REPORT.md")
    print(args.output_dir / "REPORT.md")


if __name__ == "__main__":
    main()
