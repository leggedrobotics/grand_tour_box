"""
Standalone script that loads calibration output data from Frank Fu (fu@oxfordrobotics.institute) and saves
it in a format usable by our .xacro files.

This script relies on the presence of default_calibration.yaml, and will overwrite individual calibrations that it
can compute, such that the output always contains all necessary calibrations.

Example Usage:
python box_utils/box_calibration/calib_importer.py test_calib.yaml box_model/box_model/urdf
"""

import os
import glob
import sys
import numpy as np
import argparse
from typing import Dict
from calibration_tools.parsers.calib_parser import Calibration, CalibParser
from calibration_tools.parsers.yaml_calib_parser import YamlCalibParser
from calibration_tools.calib_file_manager import CalibFileManager

DEFAULT_CALIBRATION_FILE = "calibration_default_data/default_calibration.yaml"
INPUT_CALIBRATION_DIR = "calibration_input_data"


class TopicMap:
    """Calib Input data is named by Ros Topic Name."""

    ALPHASENSE_FRONT_LEFT_TO_STIM_320 = "alphasense_front_left_to_imu_stim320"
    ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_LEFT = "/gt_box/alphasense_driver_node/cam0"
    ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_RIGHT = "/gt_box/alphasense_driver_node/cam1"
    ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_MIDDLE = "/gt_box/alphasense_driver_node/cam2/color/image"
    ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_LEFT = "/gt_box/alphasense_driver_node/cam3/color/image"
    ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_RIGHT = "/gt_box/alphasense_driver_node/cam4/color/image"
    ALPHASENSE_FRONT_LEFT_TO_ZED_LEFT = "/gt_box/zed2i_driver_node/zed_node/left_raw/image_raw_color"
    ALPHASENSE_FRONT_LEFT_TO_ZED_RIGHT = "/gt_box/zed2i_driver_node/zed_node/right_raw/image_raw_color"


def inverse_transform(T):
    """Compute the inverse of an SE(3) transformation matrix."""
    R = T[0:3, 0:3]  # Rotation matrix
    t = T[0:3, 3]  # Translation vector
    T_inv = np.eye(4)  # Initialize a 4x4 identity matrix
    R_inv = R.T  # Transpose of rotation matrix is its inverse
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = -R_inv @ t
    return T_inv


def process_alphasense(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    # Alphasense Cameras - Cam0 / front-left -> camX is equivalent to alphasense_base -> camX
    manager.update_calibration(
        "alphasense_base_to_alphasense_front_left",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_LEFT].to_output_format(),
    )
    manager.update_calibration(
        "alphasense_base_to_alphasense_front_right",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_RIGHT].to_output_format(),
    )
    manager.update_calibration(
        "alphasense_base_to_alphasense_front_middle",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_FRONT_MIDDLE].to_output_format(),
    )
    manager.update_calibration(
        "alphasense_base_to_alphasense_left",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_LEFT].to_output_format(),
    )
    manager.update_calibration(
        "alphasense_base_to_alphasense_right",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ALPHASENSE_RIGHT].to_output_format(),
    )


def process_zed2i(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    # Zed2i Cameras - compute the location of the Zed2i base_link, and then use Zed2i's
    # published tf_static for the nested calibrations.
    alphasense_to_zed_left = raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_ZED_LEFT].se3
    zed_left_to_zed_base = np.identity(4)  # TODO(load from Zed2i .xacro)
    box_base_to_alphasense = inverse_transform(raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_STIM_320].se3)
    box_base_to_zed_base = box_base_to_alphasense @ alphasense_to_zed_left @ zed_left_to_zed_base

    manager.update_calibration(
        "box_base_to_zed_base",
        **CalibParser.row_major_se3_to_xyz_rpy(box_base_to_zed_base),
    )


def process_hdr(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    pass


def process_stim320(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    # Set the alphasense base, coincident with alphasense Cam0 / front-left.
    manager.update_calibration(
        "box_base_to_alphasense_base",
        *raw_calibrations[TopicMap.ALPHASENSE_FRONT_LEFT_TO_STIM_320].to_output_format(),
    )

    # Set the box_base frame to be coincident with the Stim320 IMU
    manager.update_calibration(
        "box_base_to_imu_stim320",
        **CalibParser.row_major_se3_to_xyz_rpy(np.identity(4)),
    )


def process_calibrations(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    print("Processing calibrations...")
    # Note: stim320 must be done first to establish the box_base frame.
    try:
        process_stim320(raw_calibrations, manager)
    except KeyError as e:
        sys.exit(
            f">>> FAILED! : \n"
            f"{TopicMap.ALPHASENSE_FRONT_LEFT_TO_STIM_320} not found in calibration data. "
            f"box_base cannot be established - error: .",
            e,
        )

    process_alphasense(raw_calibrations, manager)

    process_zed2i(raw_calibrations, manager)

    process_hdr(raw_calibrations, manager)

    # TODO(kappi): Lasers, IMUs

    print(f"Writing calibrations to {manager.calibration_output_file}")
    manager.save_calibration_file()


def main(args):
    # Path of the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to the calibration_data directory
    calib_dir = os.path.join(script_dir, INPUT_CALIBRATION_DIR)

    # List of all YAML files in the calibration_data directory
    yaml_input_paths = glob.glob(os.path.join(calib_dir, "**/*.yaml"), recursive=True)
    parser = YamlCalibParser()
    all_calibrations = {}

    print(f"Found raw input calib files: {yaml_input_paths}")
    for file_path in yaml_input_paths:
        calibrations = parser.parse_calib_file(file_path)
        all_calibrations.update(calibrations)

    # Tool to manage safely saving calibrations for our .xacro files.
    print("Loading .xacro files to register valid calibrations...")
    manager = CalibFileManager(
        os.path.join(script_dir, DEFAULT_CALIBRATION_FILE),
        args.calib_output_file,
        args.box_model_dir,
    )
    process_calibrations(all_calibrations, manager)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update .xacro calibration files from calibration data.")
    parser.add_argument(
        "calib_output_file",
        type=str,
        help="The relative output file path for calibration data.",
    )
    parser.add_argument(
        "box_model_dir",
        type=str,
        help="Relative directory path for the box model URDF files.",
    )
    args = parser.parse_args()
    main(args)
