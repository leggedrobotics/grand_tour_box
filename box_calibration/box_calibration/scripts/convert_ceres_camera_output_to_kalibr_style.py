#!/usr/bin/env python3
import argparse
import os
import warnings

import numpy as np
import yaml
from rospy import logerr


def parse_args():
    parser = argparse.ArgumentParser(description="Process and transform camera YAML file.")
    parser.add_argument("input_path", type=str, help="Path to the input YAML file.")
    parser.add_argument("output_path", type=str, help="Path to the output YAML file.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--hesai", action="store_true", help="Enable compatibility with Hesai.")
    group.add_argument("--livox", action="store_true", help="Enable compatibility with Livox.")

    return parser.parse_args()


def read_yaml(input_path):
    with open(input_path, 'r') as file:
        data = yaml.safe_load(file)
    return data


def write_yaml(data, output_path):
    with open(output_path, 'w') as file:
        yaml.dump(data, file)
    print(f"Data written to {output_path}")


def compute_relative_transform(T_bundle_cn, T_bundle_cnm1):
    T_bundle_cn = np.array(T_bundle_cn)
    T_bundle_cnm1 = np.array(T_bundle_cnm1)
    T_cn_cnm1 = np.linalg.inv(T_bundle_cn).dot(T_bundle_cnm1)
    return T_cn_cnm1.tolist()


def compute_t_cam_lidar(T_bundle_camera, T_bundle_lidar):
    T_bundle_camera = np.array(T_bundle_camera)
    T_bundle_lidar = np.array(T_bundle_lidar)
    T_cam_lidar = np.linalg.inv(T_bundle_camera).dot(T_bundle_lidar)
    return T_cam_lidar.tolist()


def process_camera_data(camera_data, T_bundle_lidar):
    T_bundle_cnm1 = None
    for i, (camera, params) in enumerate(camera_data.items()):
        print(f"Processing camera: {camera}")

        # Get the current transformation matrix
        T_bundle_cn = params['T_bundle_camera']

        # Compute the relative transformation for cameras after the first
        if i > 0 and T_bundle_cnm1 is not None:
            T_cn_cnm1 = compute_relative_transform(T_bundle_cn, T_bundle_cnm1)
            params['T_cn_cnm1'] = T_cn_cnm1
            print(f"  Relative Transform (T_cn_cnm1) for {camera}: {T_cn_cnm1}")

        # Calculate T_cam_lidar for each camera based on T_bundle_camera
        T_cam_lidar = compute_t_cam_lidar(T_bundle_cn, T_bundle_lidar)
        params['T_cam_lidar'] = T_cam_lidar
        params['camera_model'] = "pinhole"
        print(f"  Computed T_cam_lidar for {camera}: {T_cam_lidar}")

        # Update previous transform for the next iteration
        T_bundle_cnm1 = T_bundle_cn


def main():
    args = parse_args()

    input_path = args.input_path
    output_path = args.output_path

    # Check if the input file exists
    if not os.path.isfile(input_path):
        print(f"Error: Input file '{input_path}' does not exist.")
        return

    # Read the YAML file
    try:
        camera_data = read_yaml(input_path)
        print("Camera data successfully read.")
    except yaml.YAMLError as e:
        print(f"Error reading YAML file: {e}")
        return

    # Check and create output directory if needed
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
        print(f"Created output directory '{output_dir}'.")

    # Warn if the output file already exists
    if os.path.isfile(output_path):
        warnings.warn(f"Warning: Output file '{output_path}' already exists.", UserWarning)

    # Define the hardcoded T_bundle_lidar matrix
    if args.hesai:
        T_bundle_lidar = [
            [-0.9999281619849881, -0.006010770995409205, 0.010370221857026046, -0.005763996987117227],
            [-0.011273051948827523, 0.177608418731531, -0.9840366699953026, -0.0624975811909965],
            [0.004072980368505267, -0.9840828828039464, -0.1776634194289637, -0.07276315632961762],
            [0.0, 0.0, 0.0, 1.0]
        ]  # Replace this with the actual T_bundle_lidar values as required
    elif args.livox:
        T_bundle_lidar = [
            [-0.008163331242335037, 0.9998394784993669, -0.015949208668200376, -0.009337306167501986],
            [-0.161744626324704, 0.014419459201293172, 0.9867272951790804, -0.17065549483669043],
            [0.9867988831966267, 0.010634680552612963, 0.16160095201277164, -0.09210429395722895],
            [0.0, 0.0, 0.0, 1.0]
        ]
    else:
        logerr("Unexpected LiDAR")
        return False
    # Process and calculate relative transformations
    process_camera_data(camera_data, T_bundle_lidar)

    # Write the modified data back to the output YAML file
    write_yaml(camera_data, output_path)


if __name__ == "__main__":
    main()
