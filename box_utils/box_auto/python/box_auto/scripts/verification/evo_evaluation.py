#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    MISSION_DATA,
    BOX_AUTO_SCRIPTS_DIR,
    get_bag,
    run_ros_command,
    find_and_extract_non_matching,
)
import os
from pathlib import Path
import yaml
import shutil
from evo.tools import file_interface
from fpdf import FPDF
import pandas as pd


def save_results_as_pdf(results, output_pdf_path):
    """
    Create a table of results and save it as a PDF.

    Args:
        results (list of dict): Each dictionary contains result name, mean, and std.
        output_pdf_path (str): Path to save the PDF.
    """
    # Convert results to a DataFrame
    df = pd.DataFrame(results)

    # Create a PDF object
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()
    pdf.set_font("Arial", size=10)

    # Add Title
    pdf.set_font("Arial", style="B", size=14)
    pdf.cell(0, 10, txt="APE Results Summary", ln=True, align="C")
    pdf.ln(5)  # Add a space after the title

    # Dynamically calculate column widths based on the number of columns
    max_width = pdf.w - 20  # Total width available for the table
    col_widths = [max_width / len(df.columns)] * len(df.columns)

    # Adjust column width for "Result Name" (first column)
    col_widths[0] = max_width * 0.4  # 40% of the total width for "Result Name"
    remaining_width = max_width - col_widths[0]
    for i in range(1, len(col_widths)):
        col_widths[i] = remaining_width / (len(col_widths) - 1)

    row_height = 8  # Base row height

    # Add headers
    pdf.set_font("Arial", style="B", size=10)
    headers = df.columns.tolist()
    for i, col_name in enumerate(headers):
        pdf.cell(col_widths[i], row_height, col_name, border=1, align="C")
    pdf.ln(row_height)

    # Add rows
    pdf.set_font("Arial", size=10)
    for _, row in df.iterrows():
        max_cell_height = row_height  # Track the maximum height for the current row
        y_start = pdf.get_y()  # Remember the starting y-coordinate of the row
        for i, item in enumerate(row):
            if i == 0:  # For the "Result Name" column
                x_start = pdf.get_x()
                # Write wrapped text using MultiCell
                pdf.multi_cell(col_widths[i], row_height, str(item), border=1, align="L")
                max_cell_height = max(max_cell_height, pdf.get_y() - y_start)  # Update max height
                pdf.set_xy(x_start + col_widths[i], y_start)  # Reset cursor for the next cell
            else:  # For other columns
                pdf.cell(col_widths[i], row_height, str(item), border=1, align="C")

        pdf.ln(max_cell_height)  # Move to the next row using the tallest cell's height

    # Save PDF
    pdf.output(output_pdf_path)
    print(f"PDF saved to: {output_pdf_path}")


def version_to_tuple(version):
    # Split version by '.' and keep only numeric parts
    numeric_parts = []
    for part in version.split("."):
        # Extract leading numeric portion of each part
        numeric_part = "".join(c for c in part if c.isdigit())
        if numeric_part:
            numeric_parts.append(int(numeric_part))
    return tuple(numeric_parts)


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


if __name__ == "__main__":

    CONFIG_NAME = "evo_dlio_ablation"

    IS_POINT_DISTANCES_SUPPORTED = False
    from evo import __version__

    evo_version_tuple = version_to_tuple(__version__)
    required_version_tuple = (1, 30, 1)

    # Check if the version meets the required version
    if evo_version_tuple < required_version_tuple:
        print(
            f"\033[91mEvo version {__version__} is less than the required version {'.'.join(map(str, required_version_tuple))}. Point_distances not supported.\033[0m"
        )
        IS_POINT_DISTANCES_SUPPORTED = False
    else:
        print(f"\033[92mEvo version {__version__} meets the required version.\033[0m")
        IS_POINT_DISTANCES_SUPPORTED = True

    # Ground truth
    ENABLE_VIZ = False
    USE_IE = False
    USE_GMSF = True

    if USE_IE:
        GT_PATTERN = "*_cpt7_ie_tc.bag"
    elif USE_GMSF:
        GT_PATTERN = "*_gt_pose.bag"

    # Evaluation Configuration.
    evaluation_config_path = os.path.join(
        f"{WS}" + "/src/grand_tour_box/box_utils/box_auto/cfg/" + CONFIG_NAME + ".yaml"
    )
    evaluation_config = load_config(evaluation_config_path)

    # Retrieve the to-be-prepared bags from config.
    PATTERNS = ["*" + v["bag"] for v in evaluation_config["bag_topic_pairs"]]

    # Iterate through each output pattern to ensure it is located where its expected.
    for pattern in PATTERNS:
        get_bag(pattern=pattern, auto_download=False, rglob=False)

    # Get the mission time for folder naming
    # CPT7 is the ground truth and expected.
    time_as_string = find_and_extract_non_matching(MISSION_DATA, GT_PATTERN)

    # Create the output folder for the evo evaluations.
    p = Path(MISSION_DATA) / f"{time_as_string}_evo_evaluations"
    if p.exists():
        shutil.rmtree(p)
    p.mkdir(parents=True)
    p = str(p)

    # Run the evo preparation.
    run_ros_command(
        f"roslaunch box_auto evo_preparation.launch bag_file_directory:={MISSION_DATA} output_folder_path:={p} config_path:={evaluation_config_path} prefix:={time_as_string}",
        background=False,
    )
    print("\033[92mEVO preparation finished.\033[0m")
    sleep(1)

    EVALUATION_PATTERN = []
    for param_set in evaluation_config["bag_topic_pairs"]:
        if param_set.get("test_name") == "":
            continue

        estimated_file = param_set.get("test_name") + ".tum"
        if estimated_file.startswith("_"):
            estimated_file = time_as_string + estimated_file

        EVALUATION_PATTERN.append(estimated_file)

    # Iterate through each pattern to ensure it is located where its expected.
    for pattern in EVALUATION_PATTERN:
        get_bag(pattern=pattern, auto_download=False, rglob=False, directory=p)

    # Create the output folder for the evo evaluations.
    p_ape = Path(p) / f"{time_as_string}_ape_results"
    p_ape.mkdir()
    p_ape = str(p_ape)

    run_ros_command(
        f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_APE.py --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_ape} --prefix={time_as_string} --disable_viz"
    )
    sleep(1)

    p_rpe = Path(p) / f"{time_as_string}_rpe_results"
    p_rpe.mkdir()
    p_rpe = str(p_rpe)

    run_ros_command(
        f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_RPE.py  --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_rpe} --prefix={time_as_string} --disable_viz"
    )
    sleep(1)

    if IS_POINT_DISTANCES_SUPPORTED:
        p_point_relation = Path(p) / f"{time_as_string}_point_relation_results"
        p_point_relation.mkdir(exist_ok=True)
        p_point_relation = str(p_point_relation)
        run_ros_command(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_point_relation.py --config={evaluation_config_path} --input_folder_path={p} --output_dir_name={p_point_relation} --prefix={time_as_string} --disable_viz"
        )

    results_summary = []
    DIRECTORIES = [p_point_relation, p_ape, p_rpe]
    for dir in DIRECTORIES:
        res_paths = get_bag(
            pattern="*.zip", auto_download=False, rglob=True, return_list=True, directory=dir, return_upon_no_files=True
        )
        if res_paths is None:
            continue
        # Collect stats
        for res_path in res_paths:

            # Get Rid of .zip extension
            res_name = res_path.split("/")[-1][:-4]

            # Read results
            res = file_interface.load_res_file(res_path, load_trajectories=True)

            # Collect stats
            results_summary.append(
                {
                    "Result Name": res_name,
                    "Mean": round(res.stats["mean"], 4),
                    "STD": round(res.stats["std"], 4),
                    "RMSE": round(res.stats["rmse"], 4),
                }
            )

    # Sort results by "Result Name"
    results_summary = sorted(results_summary, key=lambda x: x["Result Name"])

    # Save results as PDF
    output_pdf = "/evo_results_summary.pdf"
    save_results_as_pdf(results_summary, MISSION_DATA + output_pdf)

    # Deploy .zip file reader e.g.
