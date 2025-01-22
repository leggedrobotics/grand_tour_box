#!/usr/bin/env python3

import os
from time import sleep
import shutil

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

PATTERNS = None
OUTPUT_BAG_NAME = "dlio_replayed"

# TODO make these env var. ?
# Set IMU configuration
USE_STIM320 = False
USE_CPT7_IMU = True

# Set LiDAR configuration
USE_LIVOX = False
USE_HESAI = True

# Resolve IMU information
imu_pattern = None
if USE_STIM320:
    imu_pattern = "*_jetson_stim.bag"
    imu_topic = "/gt_box/stim320/imu"
elif USE_CPT7_IMU:
    imu_pattern = "*_cpt7_raw_imu.bag"
    imu_topic = "/gt_box/cpt7/offline_from_novatel_logs/imu"

if imu_pattern is None:
    print(f"IMU pattern not set correctly: {imu_pattern}")
    exit(-1)

# Resolve LiDAR information prefer filtered pointclouds
if USE_HESAI:

    try:
        get_bag("*_nuc_hesai_filtered.bag")
        PATTERNS = [imu_pattern, "*_tf_static.bag", "*_nuc_hesai_filtered.bag"]
        lidar_topic = "/gt_box/hesai/points_filtered"
    except Exception:
        PATTERNS = [imu_pattern, "*_tf_static.bag", "*_nuc_hesai_post_processed.bag"]
        lidar_topic = "/gt_box/hesai/points"
elif USE_LIVOX:
    try:
        get_bag("*_nuc_livox_filtered.bag")
        PATTERNS = [imu_pattern, "*_tf_static.bag", "*_nuc_livox_filtered.bag"]
        lidar_topic = "/gt_box/livox/lidar_filtered"
    except:
        PATTERNS = [imu_pattern, "*_tf_static.bag", "*_nuc_livox.bag"]
        lidar_topic = "/gt_box/livox/lidar"

else:
    raise ValueError("Use Livox or Hesai")


def launch_nodes():
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    inputs = []
    for pattern in PATTERNS:
        f = get_bag(pattern)
        inputs.append(f)
        timestamp = str(f).split("/")[-1]
        timestamp = timestamp.split("_")[0]

    assert len(inputs) == len(PATTERNS), "Failure did not find all the bags required"

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_dlio_prep.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Using existing merged rosbag at {merged_rosbag_path}")
    else:
        print(f"Merging bags into {merged_rosbag_path}")
        os.system(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
        )

    kill_roscore()

    check_duplicate_timestamps(merged_rosbag_path, lidar_topic)

    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch direct_lidar_inertial_odometry dlio_replay.launch input_rosbag_path:={merged_rosbag_path}  output_rosbag_folder_path:={MISSION_DATA} output_rosbag_name:={OUTPUT_BAG_NAME} imu_topic:={imu_topic} pointcloud_topic:={lidar_topic}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 0.5 --clock {merged_rosbag_path}")
    print("Waiting for 10s before uploading!")
    sleep(10)
    print("Moving and uploading bag!")
    kill_roscore()

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_dlio.bag")
    shutil.move(f"{MISSION_DATA}/{OUTPUT_BAG_NAME}.bag", output_bag_path)

    check_duplicate_timestamps(output_bag_path, "/dlio/deskewed_point_cloud")

    upload_bag(output_bag_path)


if __name__ == "__main__":
    launch_nodes()
    exit(0)
