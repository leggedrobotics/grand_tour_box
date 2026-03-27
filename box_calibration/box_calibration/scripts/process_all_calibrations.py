#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rosbag
import rospkg
import yaml
from box_calibration.calibration_utils import extract_image_topics, filter_yaml_by_rostopics, list_to_folder_name, \
    move_files_to_folder
from rosbag import Bag
from yaml import MappingNode, SequenceNode

if __name__ == "__main__":

    rospack = rospkg.RosPack()

    # Initialize parser
    parser = argparse.ArgumentParser(description="Process input folder paths")

    # Adding arguments with shorthands
    parser.add_argument('-cc', '--cameracamerafolderpath', type=str, required=True,
                        help='Path to the camera camera folder')
    parser.add_argument('-cl', '--cameralidarfolderpath', type=str, required=False,
                        help='Path to the camera lidar folder')
    parser.add_argument('-cp', '--cameraprismfolderpath', type=str, required=False,
                        help='Path to the camera prism folder')
    parser.add_argument('-pt', '--prismtopic', type=str,
                        required=False, default="/gt_box/ap20/prism_position", help='The prism topic name')
    parser.add_argument('-ci', '--cameraimufolderpath', type=str, required=False, default='',
                        help='Path to the camera imu folder')
    parser.add_argument('--solve_ap20_time_offset', action='store_true',
                        required=False, default=False, help='Solve AP20 time offset')
    parser.add_argument('--skip_camera', action='store_true',
                        required=False, default=False, help='Calibrate camera')
    parser.add_argument('--skip_lidar', action='store_true',
                        required=False, default=False, help='Calibrate camera')
    parser.add_argument("--dry_run", action="store_true", help="If set, do not run commands, only print them.")
    parser.add_argument('-ho', '--hesai_output_dir', type=str,
                        required=False, default="./hesai_calib_output", help='Path to the Hesai output')
    parser.add_argument('-lo', '--livox_output_dir', type=str,
                        required=False, default="./livox_calib_output", help='Path to the Livox output')


    def mutate_sequence_flowstyle_to_inline(node):
        if isinstance(node, MappingNode):
            node.flow_style = False
            for _, child in node.value:
                mutate_sequence_flowstyle_to_inline(child)
        elif isinstance(node, SequenceNode):
            node.flow_style = True
        else:
            node.flow_style = False


    def write_raw_img_pipeline_format(camera_parameters, out_filename, comment_header):
        data = dict()

        data["image_width"] = camera_parameters['resolution'][0]
        data["image_height"] = camera_parameters['resolution'][1]

        data["rostopic"] = camera_parameters["rostopic"]

        data["camera_matrix"] = dict()
        data["camera_matrix"]["rows"] = 3
        data["camera_matrix"]["cols"] = 3
        fx = camera_parameters['intrinsics'][0]
        fy = camera_parameters['intrinsics'][1]
        cx = camera_parameters['intrinsics'][2]
        cy = camera_parameters['intrinsics'][3]
        data["camera_matrix"]["data"] = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        data["distortion_model"] = camera_parameters['distortion_model']

        data["distortion_coefficients"] = dict()
        data["distortion_coefficients"]["rows"] = 1
        data["distortion_coefficients"]["cols"] = 4
        data["distortion_coefficients"]["data"] = camera_parameters['distortion_coeffs']

        data["rectification_matrix"] = dict()
        data["rectification_matrix"]["rows"] = 3
        data["rectification_matrix"]["cols"] = 3
        data["rectification_matrix"]["data"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        data["projection_matrix"] = dict()
        data["projection_matrix"]["rows"] = 3
        data["projection_matrix"]["cols"] = 4
        data["projection_matrix"]["data"] = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        with open(out_filename, 'w') as outfile:
            outfile.write('#Calibration program comments:' + comment_header + '\n')
            yaml_composition = yaml.compose(yaml.safe_dump(data))
            mutate_sequence_flowstyle_to_inline(yaml_composition)
            yaml.serialize(yaml_composition, outfile)  # , default_flow_style=False)


    def add_comment_to_yaml(file_path, comment):
        # Load the YAML data
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        # Prepare the comment to prepend
        comment_lines = [f"# {line}" for line in comment.splitlines()]
        full_comment = '\n'.join(comment_lines) + '\n'

        # Write the comment and YAML data back to the file
        with open(file_path, 'w') as file:
            file.write(full_comment)
            yaml.dump(data, file, default_flow_style=False)


    def add_tuple_to_yaml(file_path, kv):
        # Load the YAML data
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        for k, v in kv.items():
            data[k] = v
        # Write the comment and YAML data back to the file
        with open(file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)


    def filter_for_bags_with_rostopics(bag_paths, rostopics):
        filtered_camera_bag_paths = []
        for bag_path in bag_paths:
            with rosbag.Bag(bag_path, 'r') as bag:
                bag_topics = bag.get_type_and_topic_info()[1].keys()
                if any(topic in bag_topics for topic in rostopics):
                    filtered_camera_bag_paths.append(bag_path)
        return filtered_camera_bag_paths


    def extract_bag_start_time(bag_file_path):
        """
        Extracts the start time of a ROS bag file.

        Args:
            bag_file_path (str): Path to the ROS bag file.

        Returns:
            str: The start time of the bag file in human-readable format (YYYY-MM-DD-HH-MM-SS).
        """
        with rosbag.Bag(bag_file_path, 'r') as bag:
            # Get the start time as a ROS time
            start_time_ros = bag.get_start_time()
            # Convert to a datetime object
            start_time = datetime.fromtimestamp(start_time_ros)
        return start_time.strftime("%Y-%m-%d-%H-%M-%S")


    def read_yaml_with_comment_header(yaml_file_path):
        """
        Reads a YAML file and extracts the comment header and the YAML content.

        Args:
            yaml_file_path (str): Path to the YAML file.

        Returns:
            tuple: A tuple containing:
                - comment_header_text (str): The extracted comment header.
                - yaml_content (dict): The parsed YAML content.
        """
        # Read the file and separate the comment header and YAML content
        with open(yaml_file_path, "r") as file:
            lines = file.readlines()

        # Extract the comment header
        comment_header = []
        for line in lines:
            if line.startswith("#"):
                comment_header.append(line[1:].strip())
            else:
                break

        # Join the comment header lines
        comment_header_text = "\n".join(comment_header)

        return comment_header_text


    def pretty_write_yaml_array_with_comment(data, output_path, comment=""):
        with open(output_path, 'w') as outfile:
            outfile.write('#' + comment + '\n')
            yaml_composition = yaml.compose(yaml.safe_dump(data))
            mutate_sequence_flowstyle_to_inline(yaml_composition)
            yaml.serialize(yaml_composition, outfile)

    def safe_subprocess_run_non_blocking(command, force=False):
        if args.dry_run and not force:
            print("[DRY RUN] Would run command:", " ".join(command))
            return None

        try:
            process = subprocess.Popen(command)
            return process  # returns immediately
        except Exception as e:
            print(f"[ERROR] Failed to start command: {e}")
            sys.exit(1)

    def safe_subprocess_run(command, force=False):
        """
        Run a subprocess command safely with optional dry-run support.

        Args:
            command (list[str]): Command to execute.
            force (bool): If True, run even in dry-run mode.

        Returns:
            subprocess.CompletedProcess | None
        """
        if args.dry_run and not force:
            print("[DRY RUN] Would run command:", " ".join(command))
            return None

        result = subprocess.run(command)
        if result.returncode != 0:
            print(f"[ERROR] Command failed with exit code {result.returncode}")
            sys.exit(result.returncode)

        return result


    # Parse the arguments
    args = parser.parse_args()

    # Access the folder paths
    camera_camera_folder_path = args.cameracamerafolderpath
    camera_lidar_folder_path = args.cameralidarfolderpath
    camera_prism_folder_path = args.cameraprismfolderpath
    prism_topic = args.prismtopic
    camera_imu_folder_path = args.cameraimufolderpath
    solve_ap20_time_offset = args.solve_ap20_time_offset

    # Collect all .bag files in the camera camera folder
    bag_files = [os.path.join(camera_camera_folder_path, f) for f in os.listdir(camera_camera_folder_path) if
                 f.endswith('.bag')]

    camcam_calibration_path = os.path.join(camera_camera_folder_path, "cameracamera_calibration.yaml")
    # Construct the command with all .bag files and the output path
    command = ["rosrun", "grand_tour_ceres_apps", "camera_camera_offline_calibration", "--bags"] + bag_files + [
        "--output_path", camcam_calibration_path]

    # Run the command
    if not args.skip_camera:
        result = safe_subprocess_run(command)
        if result.returncode != 0:
            print(f"[ERROR] Command failed with code {result.returncode}")
            print(f"stdout:\n{result.stdout}")
            print(f"stderr:\n{result.stderr}")
            sys.exit(result.returncode)

        print("[INFO] Camera-camera calibration succeeded.")
    else:
        print(f"Skipping camera calibration")
    camcam_calibration_time_header = read_yaml_with_comment_header(camcam_calibration_path)

    assert (os.path.exists(camcam_calibration_path))
    box_calibration_package = rospack.get_path("box_calibration")

    with open(camcam_calibration_path, 'r') as file:
        camcam_calibration_data = yaml.load(file, Loader=yaml.FullLoader)

    for k, camera_params in camcam_calibration_data.items():
        rostopic = camera_params["rostopic"]
        output_path = None

        if "alphasense" in rostopic:
            output_path = os.path.join(box_calibration_package, "calibration", "alphasense",
                                       rostopic.split("/")[-1] + ".yaml")
        elif "hdr" in rostopic:
            if "front" in rostopic:
                output_path = os.path.join(box_calibration_package, "calibration", "hdr",
                                           "hdr_front.yaml")
            elif "left" in rostopic:
                output_path = os.path.join(box_calibration_package, "calibration", "hdr",
                                           "hdr_left.yaml")
            elif "right" in rostopic:
                output_path = os.path.join(box_calibration_package, "calibration", "hdr",
                                           "hdr_right.yaml")
            else:
                raise ValueError(f"Unexpected rostopic name: {rostopic}")
        elif "zed" in rostopic:
            if "left" in rostopic:
                output_path = os.path.join(box_calibration_package, "calibration", "zed2i",
                                           "zed_left.yaml")
            elif "right" in rostopic:
                output_path = os.path.join(box_calibration_package, "calibration", "zed2i",
                                           "zed_right.yaml")
        else:
            raise ValueError(f"Unexpected rostopic name: {rostopic}")

        if output_path is not None:
            write_raw_img_pipeline_format(camera_params, output_path, comment_header=camcam_calibration_time_header)

    # Run the second command using the output of the previous step
    camera_bundle_and_livox_initial_guess_file = os.path.join(camera_lidar_folder_path,
                                                             "kalibr_camcam_livox_initial_guess.yaml")
    convert_command = ["rosrun", "box_calibration", "convert_ceres_camera_output_to_kalibr_style.py",
                       camcam_calibration_path, camera_bundle_and_livox_initial_guess_file, "--livox"]
    safe_subprocess_run(convert_command, force=True)

    # Run the second command using the output of the previous step
    camera_bundle_and_hesai_initial_guess_file = os.path.join(camera_lidar_folder_path,
                                                             "kalibr_camcam_hesai_initial_guess.yaml")
    convert_command = ["rosrun", "box_calibration", "convert_ceres_camera_output_to_kalibr_style.py",
                       camcam_calibration_path, camera_bundle_and_hesai_initial_guess_file, "--hesai"]
    safe_subprocess_run(convert_command, force=True)

    default_grand_tour_lidar_board_path = os.path.join(camera_lidar_folder_path,
                                                       'grand_tour_default_lidar_board.yaml')

    # Dump the YAML configuration to the file
    with open(default_grand_tour_lidar_board_path, 'w') as file:
        # YAML configuration data
        default_grand_tour_camlidar_target_config = {'target_type': 'checkerboard',
                                                     'targetCols': 7, 'targetRows': 8,
                                                     'rowSpacingMeters': 0.08,
                                                     'colSpacingMeters': 0.08}
        yaml.dump(default_grand_tour_camlidar_target_config, file)

    hesai_calib_output_folder = os.path.join(camera_lidar_folder_path, args.hesai_output_dir)
    # New configuration data for grand_tour_default_hesai_calib_config.yaml
    hesai_calib_config_data = {'logging_dir': hesai_calib_output_folder,
                               'stationarity': {'max_rotation_deg': 0.1, 'max_translation_m': 0.01,
                                                'longest_outage_secs': 2},
                               'lidar_topic': '/gt_box/hesai/points',
                               'pointcloud_plane_segmenter': {'board_inflation': 0.1, 'ksearch': 50,
                                                              'normal_distance_weight': 0.005,
                                                              'max_iterations_ransac': 100,
                                                              'plane_distance_threshold_m': 0.1},
                               'optimizer': {
                                   'type': 'intensity'}
                               }

    # Output file path for the new YAML configuration
    hesai_calib_settings_path = os.path.join(camera_lidar_folder_path,
                                             'grand_tour_default_hesai_calib_config.yaml')

    # Dump the new YAML configuration to the file
    with open(hesai_calib_settings_path, 'w') as file:
        yaml.dump(hesai_calib_config_data, file)

    livox_calib_output_folder = os.path.join(camera_lidar_folder_path, args.livox_output_dir)
    # New configuration data for grand_tour_default_livox_calib_config.yaml
    livox_calib_config_data = {'logging_dir': livox_calib_output_folder,
                               'stationarity': {'max_rotation_deg': 0.01, 'max_translation_m': 0.001,
                                                'longest_outage_secs': 0.50},
                               'lidar_topic': '/gt_box/livox/lidar',
                               'pointcloud_plane_segmenter': {'board_inflation': 0.1, 'ksearch': 50,
                                                              'normal_distance_weight': 0.005,
                                                              'max_iterations_ransac': 100,
                                                              'plane_distance_threshold_m': 0.1},
                               'optimizer': {
                                   'type': 'intensity'}
                               }

    # Output file path for the new YAML configuration
    livox_calib_settings_path = os.path.join(camera_lidar_folder_path,
                                             'grand_tour_default_livox_calib_config.yaml')

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


    def reindex_and_merge_bags(input_bags, merged_bag_path):
        for bag_file in input_bags:
            if not is_bag_valid(bag_file):
                reindex_bag(bag_file)
        with Bag(merged_bag_path, 'w') as merged_bag:
            for bag_file in input_bags:
                if bag_file == merged_bag_path:
                    continue
                with Bag(bag_file, 'r') as bag:
                    for topic, msg, t in bag.read_messages():
                        merged_bag.write(topic, msg, t)


    # Merge all valid bag files into a single bag
    cam_lidar_merged_bag_path = os.path.join(camera_lidar_folder_path, 'cam_lidar_calib_merged.bag')

    # Check each bag file for validity and reindex if necessary
    if not args.dry_run and not os.path.exists(cam_lidar_merged_bag_path):
        reindex_and_merge_bags(lidar_bag_files, cam_lidar_merged_bag_path)

    # Find a .bag file in the camera lidar folder path that matches the pattern "hesai"
    hesai_bag_file = cam_lidar_merged_bag_path

    if hesai_bag_file:
        # Define the parameters for the 'rosrun' command
        intrinsic_calibrations_path = camera_bundle_and_hesai_initial_guess_file
        initial_guess_config_path = camera_bundle_and_hesai_initial_guess_file
        target_config_path = default_grand_tour_lidar_board_path
        application_parameters_path = hesai_calib_settings_path

        # Construct the command
        hesai_command = ["rosrun", "diffcal_gui_ros", "offline_calibrator.py", "--bag_path", hesai_bag_file,
                         "--intrinsic_calibrations_path", intrinsic_calibrations_path, "--extrinsic_guess_config",
                         initial_guess_config_path, "--target_config_path", target_config_path,
                         "--application_parameters_path",
                         application_parameters_path]

        # Run the command
        if not args.skip_lidar:
            print(hesai_command)
            safe_subprocess_run(hesai_command)
            hesai_viz_command = ["rosrun", "diffcal_gui_ros", "offline_alignment_viewer.py",
                             "--intrinsic_calibrations_path", intrinsic_calibrations_path,
                             "--target_config_path", target_config_path,
                             "--application_parameters_path",
                             application_parameters_path,
                                 "--port", "8030"]
            safe_subprocess_run_non_blocking(hesai_viz_command)
        else:
            print(f"Skipping lidar calibration")

    # Find a .bag file in the camera lidar folder path that matches the pattern "livox"
    livox_bag_file = cam_lidar_merged_bag_path

    if livox_bag_file:
        # Define the parameters for the 'rosrun' command
        intrinsic_calibrations_path = camera_bundle_and_livox_initial_guess_file
        initial_guess_config_path = camera_bundle_and_livox_initial_guess_file
        target_config_path = default_grand_tour_lidar_board_path
        application_parameters_path = livox_calib_settings_path

        # Construct the command
        livox_command = ["rosrun", "diffcal_gui_ros", "offline_calibrator.py", "--bag_path", livox_bag_file,
                         "--intrinsic_calibrations_path", intrinsic_calibrations_path, "--extrinsic_guess_config",
                         initial_guess_config_path, "--target_config_path", target_config_path,
                         "--application_parameters_path",
                         application_parameters_path]

        # Run the command
        if not args.skip_lidar:
            safe_subprocess_run(livox_command)
            livox_viz_command = ["rosrun", "diffcal_gui_ros", "offline_alignment_viewer.py",
                         "--intrinsic_calibrations_path", intrinsic_calibrations_path,
                         "--target_config_path", target_config_path,
                         "--application_parameters_path", application_parameters_path,
                         "--port", "8060"]
            safe_subprocess_run_non_blocking(livox_viz_command)

    camlidar_calibration_time_header = extract_bag_start_time(hesai_bag_file)
    camlidar_calibration_comment = (f"#Camera Calibration Data Time: {camcam_calibration_time_header}\n"
                                    f"#LiDAR Calibration Data Time: {camlidar_calibration_time_header}")

    report_lidar = True
    if report_lidar:
        reports = {"hesai": [hesai_calib_output_folder, "hesai_calib_report.pdf"],
                   "livox": [livox_calib_output_folder, "livox_calib_report.pdf"], }

        from matplotlib.backends.backend_pdf import PdfPages

        T_cam_lidar_for_lidarname = dict()
        with PdfPages(os.path.join(camera_lidar_folder_path, 'calibration_reports.pdf')) as pdf:
            for k, v in reports.items():
                calib_output_folder, output_report = v

                # Define the path to the reprojection_errors.txt file
                reprojection_errors_path = os.path.join(calib_output_folder, '04_alignment',
                                                        'reprojection_errors.txt')

                # Load the reprojection errors as a numpy array
                reprojection_errors = np.loadtxt(reprojection_errors_path)

                # Create a scatter plot of the reprojection errors
                plt.scatter(reprojection_errors[:, 0], reprojection_errors[:, 1])
                plt.xlabel('X Error (m)')
                plt.ylabel('Y Error (m)')
                plt.title(f'{k} Calibration Reprojection Errors')
                plt.grid(True)

                # Save the plot to the PDF file
                pdf.savefig(dpi=300)
                plt.close()

                # Define the path to the diffcal-calib.yaml file
                diffcal_calib_path = os.path.join(calib_output_folder, '04_alignment', 'diffcal-calib.yaml')

                # Copy diffcal_calib_path file into the current folder with the new name
                new_calib_file_name = f"{k}_calibration.yaml"
                shutil.copy(diffcal_calib_path, new_calib_file_name)

                # Read the YAML file
                with open(diffcal_calib_path, 'r') as file:
                    diffcal_calib_data = yaml.load(file, Loader=yaml.FullLoader)

                # Extract the T_cam_lidar for cam0
                T_cam_lidar = np.array(diffcal_calib_data['cam0']['T_cam_lidar'])
                T_cam_lidar_for_lidarname[k] = T_cam_lidar
                rotation_matrix = T_cam_lidar[:3, :3]
                translation_vector = T_cam_lidar[:3, 3]

                # Convert rotation matrix to roll, pitch, yaw in degrees
                from scipy.spatial.transform import Rotation as R

                r = R.from_matrix(rotation_matrix)
                roll, pitch, yaw = r.as_euler('xyz', degrees=True)

                # Convert translation to millimeters
                translation_vector_mm = translation_vector * 1000

                # Add a single page for T_cam_lidar contents
                plt.figure()
                plt.title(f'T_cam_lidar for cam0 to {k}')
                plt.axis('off')
                text = (f"Rotation (Roll, Pitch, Yaw) in degrees:\n"
                        f"{roll:.4f} {pitch:.4f} {yaw:.4f}\n\n"
                        f"Translation (x, y, z) in mm:\n"
                        f"{translation_vector_mm[0]:.2f} {translation_vector_mm[1]:.2f} {translation_vector_mm[2]:.2f}")
                plt.text(0.0, 0.8, text, ha='left', va='center', wrap=True)
                pdf.savefig(dpi=300)
                plt.close()

                # Define the path to the homography-merged-cloud.txt file
                homography_merged_cloud_path = os.path.join(calib_output_folder, '04_alignment',
                                                            'homography-merged-cloud.txt')
                # Load the homography-merged-cloud data as a numpy array
                homography_merged_cloud = np.loadtxt(homography_merged_cloud_path)
                homography_merged_cloud = homography_merged_cloud[
                    np.linspace(0, len(homography_merged_cloud) - 1, 20000, dtype=int)]
                # Create a scatter plot of the xy coordinates, showing intensity as color
                plt.scatter(homography_merged_cloud[:, 0], homography_merged_cloud[:, 1], c=homography_merged_cloud[:, 3],
                            cmap='viridis', s=0.1)
                plt.colorbar(label='Intensity')
                plt.xlabel('X Coordinate (m)')
                plt.ylabel('Y Coordinate (m)')
                plt.title(f'{k} Reconstructed LiDAR board model')
                plt.grid(True)
                # Save the plot to the PDF file
                pdf.savefig(dpi=300)
                plt.close()

            T_cam_hesai = T_cam_lidar_for_lidarname["hesai"]
            T_cam_livox = T_cam_lidar_for_lidarname["livox"]
            T_hesai_livox = np.linalg.inv(T_cam_hesai) @ T_cam_livox

            r = R.from_matrix(T_hesai_livox[:3, :3])
            roll, pitch, yaw = r.as_euler('xyz', degrees=True)

            # Convert translation to millimeters
            translation_vector_mm = T_hesai_livox[:3, -1] * 1000
            # Add a single page for T_cam_lidar contents
            plt.figure()
            plt.title(f'T_hesai_livox for hesai to livox')
            plt.axis('off')
            text = (f"Rotation (Roll, Pitch, Yaw) in degrees:\n"
                    f"{roll:.4f} {pitch:.4f} {yaw:.4f}\n\n"
                    f"Translation (x, y, z) in mm:\n"
                    f"{translation_vector_mm[0]:.2f} {translation_vector_mm[1]:.2f} {translation_vector_mm[2]:.2f}")
            plt.text(0.0, 0.8, text, ha='left', va='center', wrap=True)
            pdf.savefig(dpi=300)
            plt.close()

        camcamlidar_calibration_data = camcam_calibration_data.copy()

        camcamlidar_calibration_path = os.path.join(camera_camera_folder_path,
                                                    "cameracameralidar_calibration.yaml")
        # Add the new fields for each key in reports
        for key in camcam_calibration_data.keys():
            camcamlidar_calibration_data[key]['T_bundle_hesai'] = T_cam_hesai.tolist()
            camcamlidar_calibration_data[key]['T_bundle_livox'] = T_cam_livox.tolist()

        pretty_write_yaml_array_with_comment(camcamlidar_calibration_data, camcamlidar_calibration_path,
                                             comment=camlidar_calibration_comment)

    camera_prism_folder_bag_paths = [os.path.join(camera_prism_folder_path, x) for x in
                                     os.listdir(camera_prism_folder_path)
                                     if ".bag" in x]

    with open(camcam_calibration_path, 'r') as file:
        camcam_calibration_data = yaml.load(file, Loader=yaml.FullLoader)

    camcamlidar_calibration_data = camcam_calibration_data.copy()
    camera_topics = [v["rostopic"] for v in camcamlidar_calibration_data.values() for y in ["cam1", "cam2", "cam3"]
                     if y in v["rostopic"]]
    camera_prism_camera_bag_paths = filter_for_bags_with_rostopics(camera_prism_folder_bag_paths, camera_topics)
    camera_prism_prism_bag_path = filter_for_bags_with_rostopics(camera_prism_folder_bag_paths, [prism_topic])[0]

    camcamlidarprism_calibration_path = os.path.join(camera_prism_folder_path,
                                                     "camcamlidarprism_calib.yaml")

    camcam_calibration_path = os.path.join(camera_camera_folder_path,
                                                "cameracamera_calibration.yaml")
    prism_calibration_time_header = extract_bag_start_time(camera_prism_prism_bag_path)
    if solve_ap20_time_offset:
        prism_command = ["rosrun", "grand_tour_ceres_apps", "camera_prism_offline_calibration", "-c",
                         camcam_calibration_path,
                         "--camera_bags", *camera_prism_camera_bag_paths, "-p", camera_prism_prism_bag_path,
                         "-t", prism_topic, "--solve_time_offset",
                         "-o", camcamlidarprism_calibration_path]
    else:

        prism_command = ["rosrun", "grand_tour_ceres_apps", "camera_prism_offline_calibration", "-c",
                         camcam_calibration_path,
                         "--camera_bags", *camera_prism_camera_bag_paths, "-p", camera_prism_prism_bag_path,
                         "-t", prism_topic,
                         "-o", camcamlidarprism_calibration_path]
    print(prism_command)
    # Run the command
    safe_subprocess_run(prism_command, force=False)

    # Append a comment to the top of the prism_calib_output_path YAML file
    with open(camcamlidarprism_calibration_path, 'r') as file:
        yaml_content = file.read()

    # Write the comment followed by the original content back to the file
    with open(camcamlidarprism_calibration_path, 'w') as file:
        file.write(camlidar_calibration_comment + "\n" + yaml_content)


    default_grand_tour_camera_board_path = 'grand_tour_default_camera_board.yaml'
    # Dump the YAML configuration to the file
    with open(default_grand_tour_camera_board_path, 'w') as file:
        # YAML configuration data
        default_grand_tour_camcam_target_config = {'target_type': 'aprilgrid',
                                                   'tagCols': 6, 'tagRows': 6,
                                                   'tagSize': 0.083,
                                                   'tagSpacing': 0.3}
        yaml.dump(default_grand_tour_camcam_target_config, file)

    allan_variances_root = os.path.join(box_calibration_package, "calibration/allan_variances")
    imu_calibration_batches = [
        ["stim320"],
        ["cpt7"],
        ["zed2i"],
        ["adis"],
        ["alphasense"],
        ["livox"],
        ["ap20"],
    ]

    imu_topic_to_frame_mappings = {
        "/gt_box/cpt7/imu/data_raw": "cpt7_imu_raw",
        "/gt_box/cpt7/offline_from_novatel_logs/imu": "cpt7_imu",
        "/gt_box/adis16475_node/imu": "adis16475_imu",
        "/gt_box/alphasense_driver_node/imu": "imu_sensor_frame",
        "/gt_box/livox/imu_si_compliant": "livox_imu",
        "/gt_box/zed2i_driver_node/imu/data_raw": "zed2i_imu_link",
        "/gt_box/stim320/imu": "stim320_imu",
        "/gt_box/ap20/imu" : "ap20_imu",
    }
    camera_imu_calibration_data = {}
    output_imu_calibration_path = os.path.join(camera_imu_folder_path,
                                               "./imu_calibration.yaml")

    camimu_calibration_time_header = None
    if camera_imu_folder_path != "":
        cam_imu_merged_name = "cam_imu_merged_cropped.bag"
        cam_imu_bags = [x for x in os.listdir(camera_imu_folder_path) if x != cam_imu_merged_name and ".bag" in x]
        cam_imu_merged_bag_path = os.path.join(camera_imu_folder_path, cam_imu_merged_name)
        if not os.path.exists(cam_imu_merged_bag_path):
            reindex_and_merge_bags([os.path.join(camera_imu_folder_path, x) for x in cam_imu_bags],
                                   cam_imu_merged_bag_path)
        camimu_image_topics = extract_image_topics(cam_imu_merged_bag_path)
        print("Camera IMU Image Topics:", camimu_image_topics)

        # Pre-process: detect april-grid corners in the camera bag once,
        # producing a detections bag reused for all IMU calibrations.
        cam_imu_detections_bag_path = os.path.join(
            camera_imu_folder_path, "cam_imu_detections.bag")
        if not os.path.exists(cam_imu_detections_bag_path):
            preprocess_command = [
                "rosrun", "grand_tour_ceres_apps", "ros_camera_imu_preprocess",
                "--bags", cam_imu_merged_bag_path,
                "-o", cam_imu_detections_bag_path,
            ]
            safe_subprocess_run(preprocess_command, force=True)

        for imu_names in imu_calibration_batches:
            for imu_name in imu_names:
                imu_intrinsics_path = os.path.join(allan_variances_root, f"{imu_name}/imu.yaml")
                with open(imu_intrinsics_path, 'r') as file:
                    imu_intrinsics_data = yaml.safe_load(file)
                imu_topic = imu_intrinsics_data["rostopic"]

                gt_camimu_output_path = os.path.join(
                    camera_imu_folder_path, f"{imu_name}_imu_calibration.yaml")

                camera_imu_command = [
                    "rosrun", "grand_tour_ceres_apps", "camera_imu_offline_calibration",
                    "-c", camcamlidarprism_calibration_path,
                    "--camera_bags", cam_imu_detections_bag_path,
                    "-b", cam_imu_merged_bag_path,
                    "-t", imu_topic,
                    "-o", gt_camimu_output_path,
                ]
                safe_subprocess_run(camera_imu_command, force=True)

                with open(gt_camimu_output_path, 'r') as file:
                    gt_camimu_data = yaml.safe_load(file)

                for topic, data in gt_camimu_data.items():
                    frame_id = imu_topic_to_frame_mappings[topic]
                    camera_imu_calibration_data[frame_id] = {
                        "T_camerabundle_imu": data["T_camerabundle_imu"]
                    }

        camimu_calibration_time_header = extract_bag_start_time(cam_imu_merged_bag_path)

        if not args.dry_run:
            with open(output_imu_calibration_path, 'w') as outfile:
                outfile.write(f'#{camimu_calibration_time_header}\n')
                yaml_composition = yaml.compose(yaml.safe_dump(camera_imu_calibration_data))
                mutate_sequence_flowstyle_to_inline(yaml_composition)
                yaml.serialize(yaml_composition, outfile)

    # Construct the full path to the file
    tf_calibration_output_path = os.path.join(box_calibration_package, "calibration/tf/calibration_latest.yaml")

    conversion_command = ["rosrun", "box_calibration", "convert_graph.py", "-i", camcamlidarprism_calibration_path,
                          "-imu", output_imu_calibration_path,
                          "-o", tf_calibration_output_path]

    if camimu_calibration_time_header is not None:
        imu_meta_info = camimu_calibration_time_header
    else:
        imu_meta_info = ""

    calibration_metadata = {
        "camera": camcam_calibration_time_header,
        "lidar": camlidar_calibration_time_header,
        "prism": prism_calibration_time_header,
        "imu": imu_meta_info
    }

    # Run the command
    safe_subprocess_run(conversion_command, force=True)
    add_tuple_to_yaml(tf_calibration_output_path, {"calibration_metadata": calibration_metadata})

    if calib_output_folder is None:
        calib_output_folder = hesai_calib_output_folder

