#!/usr/bin/env python3

import os
from time import sleep
import shutil

from box_auto.utils import get_bag, upload_bag, WS, MISSION_DATA, kill_roscore, start_roscore, run_ros_command

PATTERNS = ["*_jetson_stim.bag", "*_tf_static.bag", "*_nuc_hesai_post_processed.bag"]
OUTPUT_BAG_NAME = "dlio_replayed"


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
            f"python3 {WS}/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
        )

    kill_roscore()
    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch direct_lidar_inertial_odometry dlio_replay.launch input_rosbag_path:={merged_rosbag_path}  output_rosbag_folder_path:={MISSION_DATA} output_rosbag_name:={OUTPUT_BAG_NAME}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 0.5 --clock {merged_rosbag_path}")
    print("Waiting for 10s before uploading!")
    sleep(10)
    print("Moving and uploading bag!")

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_dlio.bag")
    shutil.move(f"{MISSION_DATA}/{OUTPUT_BAG_NAME}.bag", output_bag_path)
    upload_bag(output_bag_path)


if __name__ == "__main__":
    launch_nodes()
