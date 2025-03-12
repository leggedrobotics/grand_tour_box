#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    get_bag,
    kill_roscore,
    run_ros_command,
)

if __name__ == "__main__":

    patterns = [
        "*_jetson_zed2i_prop.bag",
        "*_jetson_zed2i_images.bag",
        "*_jetson_zed2i_depth.bag",
        "*_boxi_tf_full.bag",
        # "*_tf_pure_perception.bag",
        #
        "*_nuc_livox_ready.bag",
        "*_npc_velodyne_ready.bag",
        "*_nuc_hesai_ready.bag",
        #
        "*_jetson_hdr_front.bag",
        "*_jetson_hdr_left.bag",
        "*_jetson_hdr_right.bag",
    ]

    inputs = []
    # Iterate through each output pattern to ensure it is located where its expected.
    for pattern in patterns:
        f = get_bag(pattern=pattern, auto_download=False, rglob=False)
        inputs.append(f)

    rosbag_paths = " ".join(inputs)

    str_command = f'roslaunch box_auto box_overlay_verification.launch data_containing_rosbag_paths:="{rosbag_paths}"'

    # Ensure roscore is not running
    kill_roscore()
    sleep(1)

    # Run the command
    run_ros_command(
        str_command,
        background=False,
    )
