#!/usr/bin/env python3
from box_auto.utils import get_file, run_ros_command

# Find relevant files
zed_svos = get_file("*_jetson_zed2i.svo2")

# Process CPT bags
for zed_svo in zed_svos:
    print(f"Processing CPT bag: {zed_svo}")
    # TODO CHECK IF THIS WORKS
    return_code = run_ros_command(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")

    if return_code == 0:
        print(f"Successfully processed {zed_svo}")
    else:
        print(f"Error processing {zed_svo} (Return code: {return_code})")
