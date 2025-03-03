#!/usr/bin/env python3

import os
from time import sleep
from box_auto.utils import (
    get_bag,
    upload_bag,
    BOX_AUTO_SCRIPTS_DIR,
    MISSION_DATA,
    kill_roscore,
    start_roscore,
    run_ros_command,
)


def launch_nodes(patterns, output_bag_path):

    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    inputs = []
    for pattern in patterns:
        f = get_bag(pattern)
        inputs.append(f)

    assert len(inputs) == len(patterns), "Failure did not find all the bags required"

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_velodyne_replay.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Removing existing merged rosbag at {merged_rosbag_path}")
        os.system(f"rm {merged_rosbag_path}")

    print(f"Merging bags into {merged_rosbag_path}")
    os.system(f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}")
    kill_roscore()

    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch any_velodyne_pointcloud grandtour_replay.launch input_rosbag_path:={merged_rosbag_path} output_rosbag_path:={output_bag_path} input_rosbag_path:={merged_rosbag_path}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 5 --clock {merged_rosbag_path}")
    print("Waiting for 10s before uploading!")
    sleep(10)
    print("Moving and uploading bag!")
    kill_roscore()

    # Remove the merged rosbag file as it's no longer needed
    if os.path.exists(merged_rosbag_path):
        print(f"Removing merged rosbag at {merged_rosbag_path}")
        os.remove(merged_rosbag_path)

    upload_bag(output_bag_path)


if __name__ == "__main__":

    # Velodyne Packet Processor
    # This script processes Velodyne LiDAR data from ROS bag files. It performs the following operations:
    # 1. Merges multiple input bag files containing Velodyne packets, TF, and TF static data
    # 2. Replays the merged bag through the Velodyne driver
    # 3. The processing converts raw Velodyne packets to point clouds
    # 4. Applies motion undistortion to the point clouds using TF data
    # 5. Outputs both the original (distorted) and undistorted point clouds

    patterns = ["*_lpc_tf.bag", "*_tf_static.bag", "*_npc_velodyne.bag"]

    # To get global path of the output bag
    output_bag_path = get_bag("*_npc_velodyne.bag")
    output_bag_path = output_bag_path.replace(".bag", "_processed.bag")

    launch_nodes(patterns, output_bag_path)
    exit(0)
