#!/usr/bin/env python3
from box_auto.utils import get_bag, upload_bag, run_ros_command

if __name__ == "__main_":
    input_bag = get_bag("*_nuc_cpt7.bag")
    output_bag = input_bag.replace("_nuc_cpt7.bag", "_nuc_cpt7_post.bag")

    run_ros_command(f"roslaunch novatel_oem7_driver cpt7_replay.launch input_bag_path:={input_bag}")
    upload_bag(output_bag)
    exit(0)
