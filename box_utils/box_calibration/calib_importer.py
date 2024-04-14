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


def process_calibrations(raw_calibrations: Dict[str, Calibration], manager: CalibFileManager):
    print("Processing calibrations...")
    # Set the alphasense base, coincident with alphasense Cam0 / front-left.
    try:
        manager.update_calibration(
            "box_base_to_alphasense_base",
            *raw_calibrations["stim320_to_alphasense_front_left"].to_output_format(),
        )
    except KeyError as e:
        sys.exit(f"stim320_to_alphasense_front_left not found in calibration data. box_base cannot be established.")
    box_base_to_alphasense_front_right_tf = raw_calibrations["stim320_to_alphasense_front_left"].se3

    parser = CalibParser()

    # Set the box_base frame to be coincident with the Stim320 IMU
    manager.update_calibration("box_base_to_imu_stim320", **parser.row_major_se3_to_xyz_rpy(np.identity(4)))

    # Alphasense Cameras - Cam0 / front-left -> camX is equivalent to alphasense_base -> camX
    cams = ["front_left", "front_right", "front_middle", "left", "right"]
    for cam in cams:
        manager.update_calibration(
            f"alphasense_base_to_alphasense_{cam}",
            *raw_calibrations[f"alphasense_front_left_to_alphasense_{cam}"].to_output_format(),
        )

    # Zed2i Cameras - transform from Alphasense Cam0 / front-left to box_base frame.
    tf = box_base_to_alphasense_front_right_tf @ raw_calibrations[f"alphasense_front_left_to_zed_left"].se3
    manager.update_calibration(f"box_base_to_zed_base", **parser.row_major_se3_to_xyz_rpy(tf))
    manager.update_calibration(
        f"zed_base_to_zed_right",
        **parser.row_major_se3_to_xyz_rpy(np.identity(4)),
    )
    # TODO(kappi): Get zed_right_to_zed_left from Zed2i driver.

    # TODO(kappi): HDR, Lasers, IMU

    print(f"Writing calibrations to {manager.calibration_output_file}")
    manager.save_calibration_file()


def main(args):
    # Path of the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to the calibration_data directory
    calib_dir = os.path.join(script_dir, "calibration_input_data")

    # List of all YAML files in the calibration_data directory
    yaml_input_paths = glob.glob(os.path.join(calib_dir, "**/*.yaml"), recursive=True)
    parser = YamlCalibParser()
    all_calibrations = {}

    print(f"Found raw input calib files: {yaml_input_paths}")
    for file_path in yaml_input_paths:
        calibrations = parser.parse_calib_file(file_path)
        all_calibrations.update(calibrations)

    # Tool to manage safely saving calibrations to our .xacro files.
    print(f"Loading .xacro files to register valid calibrations...")
    manager = CalibFileManager(
        os.path.join(script_dir, "default_calibration.yaml"), args.calib_output_file, args.box_model_dir
    )

    process_calibrations(all_calibrations, manager)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update .xacro calibration files from calibration data.")
    parser.add_argument("calib_output_file", type=str, help="The relative output file path for calibration data.")
    parser.add_argument("box_model_dir", type=str, help="Relative directory path for the box model URDF files.")
    args = parser.parse_args()
    main(args)
