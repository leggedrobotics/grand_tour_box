#!/usr/bin/env python3
import argparse
import os
import subprocess
import yaml
from rosbag import Bag

import numpy as np
import matplotlib.pyplot as plt

# Initialize parser
parser = argparse.ArgumentParser(description="Process input folder paths")

# Adding arguments with shorthands
parser.add_argument('-cc', '--cameracamerafolderpath', type=str, required=True, help='Path to the camera camera folder')
parser.add_argument('-cl', '--cameralidarfolderpath', type=str, required=True, help='Path to the camera lidar folder')
parser.add_argument("--dry_run", action="store_true", help="If set, do not run commands, only print them.")


def safe_subprocess_run(command):
    if args.dry_run:
        print("Dry run mode: would run command:", ' '.join(command))
    else:
        subprocess.run(command)


# Parse the arguments
args = parser.parse_args()

# Access the folder paths
camera_camera_folder_path = args.cameracamerafolderpath
camera_lidar_folder_path = args.cameralidarfolderpath

# Access the folder paths
camera_camera_folder_path = args.cameracamerafolderpath
camera_lidar_folder_path = args.cameralidarfolderpath

# Collect all .bag files in the camera camera folder
bag_files = [os.path.join(camera_camera_folder_path, f) for f in os.listdir(camera_camera_folder_path) if
             f.endswith('.bag')]

camcam_calibration_path = "cameracamera_calibration.yaml"
# Construct the command with all .bag files and the output path
command = ["rosrun", "grand_tour_ceres_apps", "camera_camera_offline_calibration", "--bags"] + bag_files + [
    "--output_path", camcam_calibration_path]

# Run the command
safe_subprocess_run(command)

# Run the second command using the output of the previous step
livox_input_path = "kalibr_camcam_livox_initial_guess.yaml"
convert_command = ["rosrun", "box_calibration", "convert_ceres_camera_output_to_kalibr_style.py",
                   camcam_calibration_path, livox_input_path, "--livox"]
safe_subprocess_run(convert_command)

# Run the second command using the output of the previous step
hesai_camcam_calib_and_initial_guess_file = "kalibr_camcam_hesai_initial_guess.yaml"
convert_command = ["rosrun", "box_calibration", "convert_ceres_camera_output_to_kalibr_style.py",
                   camcam_calibration_path, hesai_camcam_calib_and_initial_guess_file, "--hesai"]
safe_subprocess_run(convert_command)

# YAML configuration data
data = {
    'target_type': 'checkerboard',
    'targetCols': 7,
    'targetRows': 8,
    'rowSpacingMeters': 0.08,
    'colSpacingMeters': 0.08
}

# Output file path
default_grand_tour_lidar_board_path = 'grand_tour_default_lidar_board.yaml'

# Dump the YAML configuration to the file
with open(default_grand_tour_lidar_board_path, 'w') as file:
    yaml.dump(data, file)

hesai_calib_output_folder = "./hesai_calib_output"
# New configuration data for grand_tour_default_hesai_calib_config.yaml
hesai_calib_config_data = {
    'logging_dir': hesai_calib_output_folder,
    'stationarity': {
        'max_rotation_deg': 0.01,
        'max_translation_m': 0.001,
        'longest_outage_secs': 0.50
    },
    'lidar_topic': '/gt_box/hesai/points',
    'pointcloud_plane_segmenter': {
        'board_inflation': 0.5,
        'ksearch': 50,
        'normal_distance_weight': 0.005,
        'max_iterations_ransac': 100,
        'plane_distance_threshold_m': 0.25
    }
}

# Output file path for the new YAML configuration
hesai_calib_settings_path = 'grand_tour_default_hesai_calib_config.yaml'

# Dump the new YAML configuration to the file
with open(hesai_calib_settings_path, 'w') as file:
    yaml.dump(hesai_calib_config_data, file)

livox_calib_output_folder = "livox_calib_output"
# New configuration data for grand_tour_default_livox_calib_config.yaml
livox_calib_config_data = {
    'logging_dir': livox_calib_output_folder,
    'stationarity': {
        'max_rotation_deg': 0.01,
        'max_translation_m': 0.001,
        'longest_outage_secs': 0.50
    },
    'lidar_topic': '/gt_box/livox/lidar',
    'pointcloud_plane_segmenter': {
        'board_inflation': 0.5,
        'ksearch': 50,
        'normal_distance_weight': 0.005,
        'max_iterations_ransac': 100,
        'plane_distance_threshold_m': 0.25
    }
}

# Output file path for the new YAML configuration
livox_calib_settings_path = 'grand_tour_default_livox_calib_config.yaml'

# Dump the new YAML configuration to the file
with open(livox_calib_settings_path, 'w') as file:
    yaml.dump(livox_calib_config_data, file)


# Helper function to reindex a bag file
def reindex_bag(bag_path):
    reindex_command = ["rosbag", "reindex", bag_path]
    subprocess.run(reindex_command, check=True)


# Get all .bag files in the camera lidar folder
lidar_bag_files = [os.path.join(camera_lidar_folder_path, f) for f in os.listdir(camera_lidar_folder_path) if
                   f.endswith('.bag')]


# Function to check if a bag file is valid
def is_bag_valid(bag_path):
    try:
        with Bag(bag_path, 'r'):
            return True
    except:
        return False


# Merge all valid bag files into a single bag
merged_bag_path = os.path.join(camera_lidar_folder_path, 'cam_lidar_calib_merged.bag')

# Check each bag file for validity and reindex if necessary
if not args.dry_run:
    for bag_file in lidar_bag_files:
        if not is_bag_valid(bag_file):
            reindex_bag(bag_file)

    with Bag(merged_bag_path, 'w') as merged_bag:
        for bag_file in lidar_bag_files:
            with Bag(bag_file, 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    merged_bag.write(topic, msg, t)

# Find a .bag file in the camera lidar folder path that matches the pattern "hesai"
hesai_bag_file = merged_bag_path

if hesai_bag_file:
    # Define the parameters for the 'rosrun' command
    intrinsic_calibrations_path = hesai_camcam_calib_and_initial_guess_file
    initial_guess_config_path = hesai_camcam_calib_and_initial_guess_file
    target_config_path = default_grand_tour_lidar_board_path
    application_parameters_path = hesai_calib_settings_path

    # Construct the command
    hesai_command = [
        "rosrun", "diffcal_gui_ros", "offline_calibrator.py",
        "--bag_path", hesai_bag_file,
        "--intrinsic_calibrations_path", intrinsic_calibrations_path,
        "--initial_guess_config_path", initial_guess_config_path,
        "--target_config_path", target_config_path,
        "--application_parameters_path", application_parameters_path
    ]

    # Run the command
    safe_subprocess_run(hesai_command)

# Find a .bag file in the camera lidar folder path that matches the pattern "livox"
livox_bag_file = merged_bag_path

if livox_bag_file:
    # Define the parameters for the 'rosrun' command
    intrinsic_calibrations_path = livox_input_path
    initial_guess_config_path = livox_input_path
    target_config_path = default_grand_tour_lidar_board_path
    application_parameters_path = livox_calib_settings_path

    # Construct the command
    livox_command = [
        "rosrun", "diffcal_gui_ros", "offline_calibrator.py",
        "--bag_path", livox_bag_file,
        "--intrinsic_calibrations_path", intrinsic_calibrations_path,
        "--initial_guess_config_path", initial_guess_config_path,
        "--target_config_path", target_config_path,
        "--application_parameters_path", application_parameters_path
    ]

    # Run the command
    safe_subprocess_run(livox_command)

reports = {
    "hesai" : [hesai_calib_output_folder, "hesai_calib_report.pdf"],
    "livox" : [livox_calib_output_folder, "livox_calib_report.pdf"],
}

from matplotlib.backends.backend_pdf import PdfPages

with PdfPages('calibration_reports.pdf') as pdf:
    for k, v in reports.items():
        calib_output_folder, output_report = v

        # Define the path to the reprojection_errors.txt file
        reprojection_errors_path = os.path.join(calib_output_folder, '04_alignment', 'reprojection_errors.txt')

        # Load the reprojection errors as a numpy array
        reprojection_errors = np.loadtxt(reprojection_errors_path)

        # Create a scatter plot of the reprojection errors
        plt.scatter(reprojection_errors[:, 0], reprojection_errors[:, 1])
        plt.xlabel('X Error (m)')
        plt.ylabel('Y Error (m)')
        plt.title(f'{k} Calibration Reprojection Errors')
        plt.grid(True)

        # Save the plot to the PDF file
        pdf.savefig()

        # Close the plot to free memory
        plt.close()
