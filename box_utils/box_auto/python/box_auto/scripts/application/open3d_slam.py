#!/usr/bin/env python3

import os
from time import sleep
import shutil
from box_auto.utils import (
    MISSION_DATA,
    BOX_AUTO_SCRIPTS_DIR,
    ARTIFACT_FOLDER,
    get_bag,
    upload_bag,
    run_ros_command,
    kill_roscore,
    start_roscore,
)
from pathlib import Path

USE_LEGGED_ESTIMATOR = True
USE_DLIO = False

# It is up-to-debate how we want to use this node. We regardless need the deskewed point cloud and some sort of pose estimation.
# If the ground truth poses are available, we can use them instead of dlio_poses.

if USE_LEGGED_ESTIMATOR and not USE_DLIO:
    PATTERNS = ["*[0-9]_dlio.bag", "*_lpc_state_estimator.bag", "*_tf_static.bag"]
    prior_frame = "base"
    pose_topic = "/state_estimator/pose_in_odom"
    
else:
    PATTERNS = ["*[0-9]_dlio.bag", "*_tf_static.bag"]
    prior_frame = "hesai_lidar"
    pose_topic = "/dlio/lidar_map_odometry"


def launch_nodes():
    inputs = []
    for pattern in PATTERNS:
        f = get_bag(pattern)
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

    p = Path(ARTIFACT_FOLDER) / "open3d_slam"
    p.mkdir(exist_ok=True, parents=True)
    p = str(p)

    run_ros_command(
        f"roslaunch open3d_slam_ros grandtour_replay.launch rosbag_filepath:={merged_rosbag_path}  map_saving_folder:={p} async_pose_topic:={pose_topic} assumed_external_odometry_tracked_frame:={prior_frame}",
        background=False,
    )
    print("Open3D SLAM replaying has been completed.")
    sleep(1)
    kill_roscore()

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_open3d_slam.bag")
    shutil.move(str(Path(MISSION_DATA) / "open3d_slam_replayed.bag"), output_bag_path)

    print("Replayed bag is renamed. Uploading.")
    upload_bag(output_bag_path)


if __name__ == "__main__":
    launch_nodes()
    exit(0)
