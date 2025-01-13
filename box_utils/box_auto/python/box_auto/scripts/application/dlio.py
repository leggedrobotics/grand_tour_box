#!/usr/bin/env python3

import os
from time import sleep
import shutil
import argparse
from box_auto.utils import (
    get_bag,
    upload_bag,
    BOX_AUTO_SCRIPTS_DIR,
    MISSION_DATA,
    kill_roscore,
    start_roscore,
    run_ros_command,
    check_duplicate_timestamps,
)


def launch_nodes(patterns, output_bag_name, lidar_topic, imu_topic, tag):
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    inputs = []
    for pattern in patterns:
        f = get_bag(pattern)
        inputs.append(f)
        timestamp = str(f).split("/")[-1]
        timestamp = timestamp.split("_")[0]

    assert len(inputs) == len(patterns), "Failure did not find all the bags required"

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_dlio_prep.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Removing existing merged rosbag at {merged_rosbag_path}")
        os.system(f"rm {merged_rosbag_path}")

    print(f"Merging bags into {merged_rosbag_path}")
    os.system(f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}")

    kill_roscore()

    check_duplicate_timestamps(merged_rosbag_path, lidar_topic)

    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch direct_lidar_inertial_odometry dlio_replay.launch input_rosbag_path:={merged_rosbag_path}  output_rosbag_folder_path:={MISSION_DATA} output_rosbag_name:={output_bag_name} imu_topic:={imu_topic} pointcloud_topic:={lidar_topic}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 0.5 --clock {merged_rosbag_path}")
    print("Waiting for 10s before uploading!")
    sleep(10)
    print("Moving and uploading bag!")
    kill_roscore()

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_{tag}dlio.bag")
    shutil.move(f"{MISSION_DATA}/{output_bag_name}.bag", output_bag_path)

    check_duplicate_timestamps(output_bag_path, "/dlio/deskewed_point_cloud")

    upload_bag(output_bag_path)


def parse_arguments():
    parser = argparse.ArgumentParser(description="Configure LiDAR and IMU options.")
    parser.add_argument(
        "--lidar", choices=["hesai", "livox"], default="hesai", help="Select the LiDAR configuration (default: hesai)."
    )
    parser.add_argument(
        "--imu", choices=["stim320", "cpt7"], default="cpt7", help="Select the IMU configuration (default: cpt7)."
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()

    # Resolve IMU information
    if args.imu == "stim320":
        imu_pattern = "*_jetson_stim.bag"
        imu_topic = "/gt_box/stim320/imu"
    elif args.imu == "cpt7":
        imu_pattern = "*_cpt7_raw_imu.bag"
        imu_topic = "/gt_box/cpt7/offline_from_novatel_logs/imu"
    else:
        print(f"IMU not found: {args.imu}")
        exit(1)

    # Resolve LiDAR information prefer filtered pointclouds
    if args.lidar == "hesai":
        try:
            get_bag("*_nuc_hesai_filtered123.bag")
            patterns = [imu_pattern, "*_tf_static.bag", "*_nuc_hesai_filtered.bag"]
            lidar_topic = "/gt_box/hesai/points_filtered"
        except Exception:
            patterns = [imu_pattern, "*_tf_static.bag", "*_nuc_hesai_post_processed.bag"]
            lidar_topic = "/gt_box/hesai/points"
        tag = ""
    elif args.lidar == "livox":
        try:
            get_bag("*_nuc_livox_filtered123.bag")
            patterns = [imu_pattern, "*_tf_static.bag", "*_nuc_livox_filtered.bag"]
            lidar_topic = "/gt_box/livox/lidar_filtered"
        except:
            patterns = [imu_pattern, "*_tf_static.bag", "*_nuc_livox.bag"]
            lidar_topic = "/gt_box/livox/lidar"
        tag = "livox_"
    else:
        print(f"LiDAR not supported: {args.lidar}")
        exit(2)

    output_bag_name = f"dlio_replayed_{args.lidar}_{args.imu}"
    launch_nodes(patterns, output_bag_name, lidar_topic, imu_topic, tag)
    exit(0)
