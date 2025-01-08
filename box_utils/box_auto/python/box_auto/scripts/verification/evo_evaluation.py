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
    USE_IE = True
    USE_GMSF = False

    if USE_IE:
        GT_PATTERN = "*_cpt7_ie_tc.bag"
    elif USE_GMSF:
        GT_PATTERN = "not_implemented"

    # Prepare for evo.
    evo_prep_config_path = os.path.join(f"{WS}" + "/src/grand_tour_box/box_utils/box_auto/cfg/evo_preparation.yaml")
    evo_prep_config = load_config(evo_prep_config_path)

    # Retrieve the to-be-prepared bags from config.
    PATTERNS = ["*" + v["bag"] for v in evo_prep_config["bag_topic_pairs"]]

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
        f"roslaunch box_auto evo_preparation.launch bag_file_directory:={MISSION_DATA} output_folder_path:={p} config_path:={evo_prep_config_path} prefix:={time_as_string}",
        background=False,
    )
    print("\033[92mEVO preparation finished.\033[0m")
    sleep(1)

    ape_config_path = os.path.join(WS, "src/grand_tour_box/box_utils/box_auto/cfg/evo_evaluation_ape.yaml")
    ape_config = load_config(ape_config_path)

    EVALUATION_PATTERN = []

    for _, param_set in list(ape_config.items()):
        if isinstance(param_set, (bool, str)):
            continue
        estimated_file = param_set.get("estimated_file")
        reference_file = param_set.get("reference_file")

        if estimated_file.startswith("_"):
            estimated_file = time_as_string + estimated_file
        if reference_file.startswith("_"):
            reference_file = time_as_string + reference_file

        EVALUATION_PATTERN.append(estimated_file)
        EVALUATION_PATTERN.append(reference_file)

    # Iterate through each pattern to ensure it is located where its expected.
    for pattern in EVALUATION_PATTERN:
        get_bag(pattern=pattern, auto_download=False, rglob=False, directory=p)

    # Create the output folder for the evo evaluations.
    p_ape = Path(p) / f"{time_as_string}_ape_results"
    p_ape.mkdir()
    p_ape = str(p_ape)

    run_ros_command(
        f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_APE.py --config={ape_config_path} --input_folder_path={p} --output_dir_name={p_ape} --prefix={time_as_string} --disable_viz"
    )
    sleep(1)

    rpe_config_path = os.path.join(WS, "src/grand_tour_box/box_utils/box_auto/cfg/evo_evaluation_rpe.yaml")

    p_rpe = Path(p) / f"{time_as_string}_rpe_results"
    p_rpe.mkdir()
    p_rpe = str(p_rpe)

    run_ros_command(
        f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_SE3_RPE.py  --config={rpe_config_path} --input_folder_path={p} --output_dir_name={p_rpe} --prefix={time_as_string} --disable_viz"
    )
    sleep(1)

    if IS_POINT_DISTANCES_SUPPORTED:
        point_relation_config_path = os.path.join(
            WS, "src/grand_tour_box/box_utils/box_auto/cfg/evo_evaluation_point_relation.yaml"
        )
        p_point_relation = Path(p) / f"{time_as_string}_point_relation_results"
        p_point_relation.mkdir(exist_ok=True)
        p_point_relation = str(p_point_relation)
        run_ros_command(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/verification/grandtour_point_relation.py --config={point_relation_config_path} --input_folder_path={p} --output_dir_name={p_point_relation} --prefix={time_as_string} --disable_viz"
        )

    # Deploy .zip file reader e.g.
