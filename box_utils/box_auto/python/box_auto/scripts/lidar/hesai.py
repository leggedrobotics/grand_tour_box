#!/usr/bin/env python3

from time import sleep
from box_auto.utils import WS, run_ros_command, start_roscore, kill_roscore, get_bag, upload_bag
import os
from pathlib import Path
import time


def check_file_size_stable(file_path, check_interval=1, stability_duration=15):
    """
    Check if file size remains stable for the specified duration.
    Returns True if size is stable, False otherwise.
    """
    previous_size = -1
    stable_since = 0

    while True:
        if not os.path.exists(file_path):
            sleep(check_interval)
            continue

        current_size = Path(file_path).stat().st_size

        if current_size == previous_size:
            if stable_since == 0:
                stable_since = time.time()
            elif time.time() - stable_since >= stability_duration:
                return True
        else:
            stable_since = 0
            previous_size = current_size

        sleep(check_interval)


if __name__ == "__main__":
    input_rosbag_path = get_bag("*_nuc_hesai.bag")

    kill_roscore()
    start_roscore()
    sleep(1)
    run_ros_command(
        f"rosrun hesai_ros_driver hesai_ros_driver_node _config_path:={WS}/src/grand_tour_box/box_drivers/hesai_lidar_ros_driver/config/packet_replay.yaml _use_sim_time:=True _input_rosbag_path:={input_rosbag_path}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 5 --clock {input_rosbag_path} --topics /gt_box/hesai/packets")

    output_bag_path = input_rosbag_path.replace("_nuc_hesai.bag", "_nuc_hesai_post_processed.bag")

    # Wait for file size to stabilize
    print("Waiting for output bag size to stabilize...")
    check_file_size_stable(output_bag_path)
    print("Output bag size has stabilized. Proceeding with upload.")

    upload_bag(output_bag_path)
