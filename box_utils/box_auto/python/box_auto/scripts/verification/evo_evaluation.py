#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    MISSION_DATA,
    BOX_AUTO_SCRIPTS_DIR,
    get_bag,
    run_ros_command,
    kill_roscore,
    find_and_extract_non_matching,
)
from pathlib import Path
import yaml
import shutil
from evo.tools import file_interface
from fpdf import FPDF
import pandas as pd
import argparse


def run_required_ros_command(command, **kwargs):
    status = run_ros_command(command, **kwargs)
    if status != 0:
        raise RuntimeError(f"Evaluation stage failed (exit status {status}): {command}")


def save_results_as_pdf(results, output_pdf_path, title="APE Results Summary", units=None):
    # Convert results to a DataFrame
    df = pd.DataFrame(results)

    # OPTIONAL: append units to selected column headers (e.g., Mean -> Mean(cm))
    if units:
        rename_map = {}
        for col, unit in units.items():
            if col in df.columns and unit:
                rename_map[col] = f"{col}({unit})"
        if rename_map:
            df = df.rename(columns=rename_map)

    # Add left-most numbering column (1..N)
    df.insert(0, "No.", range(1, len(df) + 1))

    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()
    pdf.set_font("Arial", size=10)

    pdf.set_font("Arial", style="B", size=14)
    pdf.cell(0, 10, txt=title, ln=True, align="C")
    pdf.ln(5)

    max_width = pdf.w - 20
    col_widths = [max_width / len(df.columns)] * len(df.columns)

    # Width tweaks for "No." and "Result Name"
    col_widths[0] = max_width * 0.08   # No.
    col_widths[1] = max_width * 0.40   # Result Name
    remaining_width = max_width - (col_widths[0] + col_widths[1])
    for i in range(2, len(col_widths)):
        col_widths[i] = remaining_width / (len(col_widths) - 2)

    row_height = 8

    def _wrap_text_to_lines(pdf_obj: FPDF, text: str, cell_w: float):
        if text is None:
            text = ""
        text = str(text)
        effective_w = max(0.0, cell_w - 2.0 * getattr(pdf_obj, "c_margin", 0.0))
        if effective_w <= 0:
            return [text] if text else [""]

        lines = []
        for paragraph in text.split("\n"):
            if paragraph == "":
                lines.append("")
                continue
            words = paragraph.split(" ")
            current = ""

            def fits(s: str) -> bool:
                return pdf_obj.get_string_width(s) <= effective_w

            for w in words:
                candidate = w if current == "" else current + " " + w
                if fits(candidate):
                    current = candidate
                    continue

                if current != "":
                    lines.append(current)
                    current = ""

                if not fits(w):
                    chunk = ""
                    for ch in w:
                        cand = chunk + ch
                        if fits(cand):
                            chunk = cand
                        else:
                            if chunk:
                                lines.append(chunk)
                            chunk = ch
                    current = chunk
                else:
                    current = w

            if current != "":
                lines.append(current)

        return lines if lines else [""]

    # Header
    pdf.set_font("Arial", style="B", size=10)
    headers = df.columns.tolist()
    for i, col_name in enumerate(headers):
        pdf.cell(col_widths[i], row_height, col_name, border=1, align="C")
    pdf.ln(row_height)

    # Rows
    pdf.set_font("Arial", size=10)
    for _, row in df.iterrows():
        # Wrap the "Result Name" column (index 1)
        wrapped_lines = _wrap_text_to_lines(pdf, str(row.iloc[1]), col_widths[1])
        needed_height = row_height * max(1, len(wrapped_lines))

        if pdf.get_y() + needed_height > pdf.page_break_trigger:
            pdf.add_page()
            pdf.set_font("Arial", style="B", size=10)
            for i, col_name in enumerate(headers):
                pdf.cell(col_widths[i], row_height, col_name, border=1, align="C")
            pdf.ln(row_height)
            pdf.set_font("Arial", size=10)

        x_start = pdf.get_x()
        y_start = pdf.get_y()

        # "No." cell (single line)
        pdf.cell(col_widths[0], needed_height, str(row.iloc[0]), border=1, align="C")

        # "Result Name" cell (wrapped)
        pdf.set_xy(x_start + col_widths[0], y_start)
        pdf.multi_cell(col_widths[1], row_height, "\n".join(wrapped_lines), border=1, align="L")

        # Remaining columns (single line each)
        pdf.set_xy(x_start + col_widths[0] + col_widths[1], y_start)
        for i in range(2, len(headers)):
            pdf.cell(col_widths[i], needed_height, str(row.iloc[i]), border=1, align="C")

        pdf.set_xy(x_start, y_start + needed_height)

    pdf.output(output_pdf_path)
    print(f"PDF saved to: {output_pdf_path}")


def version_to_tuple(version):
    numeric_parts = []
    for part in version.split("."):
        numeric_part = "".join(c for c in part if c.isdigit())
        if numeric_part:
            numeric_parts.append(int(numeric_part))
    return tuple(numeric_parts)


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def parse_unit_to_scale(unit_str, default_unit="m"):
    """
    Returns (unit_label, scale_factor).
    - 'm'  -> ('m', 1.0)
    - 'cm' -> ('cm', 100.0)
    """
    if unit_str is None:
        unit_str = default_unit
    u = str(unit_str).strip().lower()
    if u in ("cm", "centimeter", "centimeters"):
        return "cm", 100.0
    return "m", 1.0


if __name__ == "__main__":
    # For Time offset evaluation on DLIO and CPT7 -> evo_evaluations_time_offset
    # For Time offset evaluation on DLIO and STIM320 -> evo_evaluations_time_offset_STIM
    # For General Evaluation -> evo_evaluations
    # For Ground Truth comparison -> evo_evaluations_gt_truth_comparison
    # For IMU comparisons on DLIO -> evo_evaluations_imu_comparison
    # For IMU comparisons on GMSF -> evo_evaluations_GMSF_IMU_comparison
    # For translation offset -> evo_evaluations_translation_offset
    # For rotational offset -> evo_evaluations_rotation_offset

    parser = argparse.ArgumentParser(description="Script configuration")
    parser.add_argument(
        "--config_name",
        type=str,
        default="evo_evaluations",  # evo_evaluations_IJRR #evo_evaluations_resple #evo_evaluations_gt_truth_comparison
        help="The name of the configuration file. Default is 'evo_evaluations'.",
    )
    parser.add_argument(
        "--ate_rte_unit",
        type=str,
        default=None,
        choices=["m", "cm"],
        help="Unit for ATE and RTE tables only. If omitted, can be taken from YAML key 'ate_rte_unit' (default: m).",
    )
    args = parser.parse_args()
    print(f"Using configuration: {args.config_name}")

    IS_POINT_DISTANCES_SUPPORTED = False
    from evo import __version__

    evo_version_tuple = version_to_tuple(__version__)
    required_version_tuple = (1, 30, 1)

    if evo_version_tuple < required_version_tuple:
        print(
            f"\033[91mEvo version {__version__} is less than the required version {'.'.join(map(str, required_version_tuple))}. Point_distances not supported.\033[0m"
        )
        IS_POINT_DISTANCES_SUPPORTED = False
    else:
        print(f"\033[92mEvo version {__version__} meets the required version.\033[0m")
        IS_POINT_DISTANCES_SUPPORTED = True

    path = Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg" / (args.config_name + ".yaml")

    # Evaluation Configuration
    evaluation_config_path = str(path)
    evaluation_config = load_config(evaluation_config_path)

    # Ground truth and visualization can be selected per evaluation config.
    # Existing configs retain the historical TPS/headless defaults.
    DISABLE_VIZ = bool(evaluation_config.get("disable_viz", True))
    GT_SOURCE = str(evaluation_config.get("gt_source", "TPS")).upper()
    IS_TPS_GT = GT_SOURCE == "TPS"

    gt_patterns = {
        "IE": "*_cpt7_ie_tc.bag",
        "GMSF": "*_gt_pose.bag",
        "TPS": "*_jetson_ap20_robot.bag",
    }
    GT_PATTERN = evaluation_config.get("gt_pattern", gt_patterns.get(GT_SOURCE))
    if not GT_PATTERN:
        raise ValueError(
            f"Invalid ground truth source: {GT_SOURCE}. Valid options are: {', '.join(gt_patterns.keys())}"
        )

    # Unit handling for ATE/RTE only:
    # - CLI --ate_rte_unit overrides YAML
    # - YAML optional: ate_rte_unit: cm
    ate_rte_unit, ate_rte_scale = parse_unit_to_scale(
        args.ate_rte_unit,
        default_unit=evaluation_config.get("ate_rte_unit", "m"),
    )

    # Retrieve the to-be-prepared bags from config.
    PATTERNS = ["*" + v["bag"] for v in evaluation_config["bag_topic_pairs"]]

    for pattern in PATTERNS:
        get_bag(pattern=pattern, auto_download=False, rglob=False)

    # Mission time for folder naming
    time_as_string = find_and_extract_non_matching(MISSION_DATA, GT_PATTERN)

    # Create output folder
    p = Path(MISSION_DATA) / f"{time_as_string}_{args.config_name}"
    if p.exists():
        shutil.rmtree(p)
    p.mkdir(parents=True)
    p = str(p)

    kill_roscore()

    # Run evo preparation
    run_required_ros_command(
        f"roslaunch box_auto evo_preparation.launch bag_file_directory:={MISSION_DATA} output_folder_path:={p} config_path:={evaluation_config_path} prefix:={time_as_string}",
        background=False,
    )
    kill_roscore()
    sleep(1)

    EVALUATION_PATTERN = []
    for param_set in evaluation_config["bag_topic_pairs"]:
        if param_set.get("test_name") == "":
            continue

        estimated_file = param_set.get("test_name") + ".tum"
        if estimated_file.startswith("_"):
            estimated_file = time_as_string + estimated_file

        EVALUATION_PATTERN.append(estimated_file)

    for pattern in EVALUATION_PATTERN:
        get_bag(pattern=pattern, auto_download=False, rglob=False, directory=p)

    # Create result folders
    p_ape = None
    p_rpe = None
    p_point_relation = None

    if not IS_TPS_GT:
        p_ape = Path(p) / f"{time_as_string}_ape_results"
        p_ape.mkdir()
        p_ape = str(p_ape)

        run_required_ros_command(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_APE.py --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_ape} --prefix={time_as_string} --disable_viz={DISABLE_VIZ}"
        )
        sleep(1)

        p_rpe = Path(p) / f"{time_as_string}_rpe_results"
        p_rpe.mkdir()
        p_rpe = str(p_rpe)

        run_required_ros_command(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_RPE.py  --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_rpe} --prefix={time_as_string} --disable_viz={DISABLE_VIZ}"
        )
        sleep(1)

    if IS_POINT_DISTANCES_SUPPORTED:
        p_point_relation = Path(p) / f"{time_as_string}_point_relation_results"
        p_point_relation.mkdir(exist_ok=True)
        p_point_relation = str(p_point_relation)
        run_required_ros_command(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_point_relation.py --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_point_relation} --prefix={time_as_string} --disable_viz={DISABLE_VIZ}"
        )
    else:
        print("\033[93mSkipping point relation evaluation: point distances not supported.\033[0m")

    results_summary_ate = []
    results_summary_rte = []
    results_summary_ape = []
    results_summary_rpe = []
    point_relation_references = {
        param_set.get("evaluations", {}).get("point_relation", {}).get("reference_file")
        for param_set in evaluation_config.get("bag_topic_pairs", [])
        if param_set.get("evaluations", {}).get("point_relation", {}).get("reference_file")
    }

    DIRECTORIES = [p_point_relation, p_ape, p_rpe]
    for dir in DIRECTORIES:
        if dir is None:
            continue

        res_paths = get_bag(
            pattern="*.zip",
            auto_download=False,
            rglob=True,
            return_list=True,
            directory=dir,
            return_upon_no_files=True,
        )
        if res_paths is None:
            continue
        if isinstance(res_paths, str):
            res_paths = [res_paths]

        for res_path in res_paths:
            res_name = res_path.split("/")[-1][:-4]  # strip .zip
            res = file_interface.load_res_file(res_path, load_trajectories=True)

            res_name_l = res_name.lower()
            metric_kind = res_name_l.split("_", 1)[0]
            is_ate_or_rte = metric_kind in ("ate", "rte")

            # A ground-truth trajectory evaluated against itself is only a
            # pipeline sanity check and must not appear in method rankings.
            if any(
                res_name_l.startswith(f"{metric_kind}_{reference_name.lower()}_")
                for reference_name in point_relation_references
            ):
                continue
            scale = ate_rte_scale if is_ate_or_rte else 1.0

            res_entry = {
                "Result Name": res_name,
                "Status": res.info.get("status", "success"),
                "Mean": round(res.stats["mean"] * scale, 4),
                "STD": round(res.stats["std"] * scale, 4),
                "RMSE": round(res.stats["rmse"] * scale, 4),
            }

            if metric_kind == "ate":
                results_summary_ate.append(res_entry)
            if metric_kind == "rte":
                results_summary_rte.append(res_entry)
            if metric_kind == "ape":
                results_summary_ape.append(res_entry)
            if metric_kind == "rpe":
                results_summary_rpe.append(res_entry)

    # Sort results from smallest to largest (by Mean error)
    result_order = lambda row: (row["Status"] != "success", row["Mean"])
    results_summary_ate = sorted(results_summary_ate, key=result_order)
    results_summary_rte = sorted(results_summary_rte, key=result_order)
    results_summary_ape = sorted(results_summary_ape, key=result_order)
    results_summary_rpe = sorted(results_summary_rpe, key=result_order)

    # Save results as PDF
    output_pdf_ate = "evo_results_summary_ate.pdf"
    output_pdf_rte = "evo_results_summary_rte.pdf"
    output_pdf_ape = "evo_results_summary_ape.pdf"
    output_pdf_rpe = "evo_results_summary_rpe.pdf"

    if len(results_summary_ate) > 0:
        pd.DataFrame(results_summary_ate).to_csv(
            Path(MISSION_DATA) / "evo_results_summary_ate.csv", index=False
        )
        save_results_as_pdf(
            results_summary_ate,
            str(Path(MISSION_DATA) / output_pdf_ate),
            title=f"ATE Results Summary ({ate_rte_unit})",
            units={"Mean": ate_rte_unit, "STD": ate_rte_unit, "RMSE": ate_rte_unit},
        )
    if len(results_summary_rte) > 0:
        pd.DataFrame(results_summary_rte).to_csv(
            Path(MISSION_DATA) / "evo_results_summary_rte.csv", index=False
        )
        save_results_as_pdf(
            results_summary_rte,
            str(Path(MISSION_DATA) / output_pdf_rte),
            title=f"RTE Results Summary ({ate_rte_unit})",
            units={"Mean": ate_rte_unit, "STD": ate_rte_unit, "RMSE": ate_rte_unit},
        )
    if len(results_summary_ape) > 0:
        save_results_as_pdf(
            results_summary_ape,
            str(Path(MISSION_DATA) / output_pdf_ape),
            title="APE Results Summary (m)",
            units={"Mean": "m", "STD": "m", "RMSE": "m"},
        )
    if len(results_summary_rpe) > 0:
        save_results_as_pdf(
            results_summary_rpe,
            str(Path(MISSION_DATA) / output_pdf_rpe),
            title="RPE Results Summary (m)",
            units={"Mean": "m", "STD": "m", "RMSE": "m"},
        )

    # Deploy .zip file reader e.g.
