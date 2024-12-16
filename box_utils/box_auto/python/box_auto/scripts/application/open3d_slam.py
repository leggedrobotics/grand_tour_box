#!/usr/bin/env python3

import os
from time import sleep
import shutil
from box_auto.utils import (
    MISSION_DATA,
    BOX_AUTO_SCRIPTS_DIR,
    get_bag,
    upload_bag,
    run_ros_command,
    kill_roscore,
    start_roscore,
)

# It is up-to-debate how we want to use this node. We regardless need the deskewed point cloud and some sort of pose estimation.
# If the ground truth poses are available, we can use them instead of dlio_poses.
PATTERNS = ["*_dlio.bag", "*_tf_static.bag"]


def launch_nodes():
    inputs = []
    for pattern in PATTERNS:
        f = get_bag(pattern, return_suc=True)
        inputs.append(f)
        timestamp = str(f).split("/")[-1]
        timestamp = timestamp.split("_")[0]

    inputs = ",".join(inputs)
    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_open3d_prep.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Using existing merged rosbag at {merged_rosbag_path}")
    else:
        print(f"Merging bags into {merged_rosbag_path}")
        os.system(
            f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
        )

    kill_roscore()
    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch open3d_slam_ros grandtour_replay.launch rosbag_filepath:={merged_rosbag_path}  map_saving_folder:={MISSION_DATA}",
        background=True,
    )
    sleep(1)

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_open3d_slam.bag")

    shutil.move(f"{MISSION_DATA}/open3d_slam_replayed.bag", output_bag_path)

    upload_bag(output_bag_path)


if __name__ == "__main__":
    launch_nodes()
