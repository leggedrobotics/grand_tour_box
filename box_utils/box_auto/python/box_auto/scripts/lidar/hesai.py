#!/usr/bin/env python3

from time import sleep
from box_auto.utils import WS, run_ros_command, start_roscore, kill_roscore, get_bag, upload_bag

if __name__ == "__main__":
    input_rosbag_path = get_bag("*_nuc_hesai.bag")

    kill_roscore()
    start_roscore()
    sleep(1)
    run_ros_command(
        f"rosrun hesai_ros_driver hesai_ros_driver_node _config_path:={WS}/src/grand_tour_box/box_drivers/hesai_lidar_ros_driver/config/packet_replay.yaml _use_sim_time:=True _input_rosbag_path:={input_rosbag_path}",
        background=True,
    )
    sleep(10)
    run_ros_command(f"rosbag play -r 5 --clock {input_rosbag_path} --topics /gt_box/hesai/packets")

    output_bag_path = input_rosbag_path.replace("_nuc_hesai.bag", "_nuc_hesai_post_processed.bag")
    upload_bag(output_bag_path)
