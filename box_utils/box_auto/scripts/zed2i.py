#!/usr/bin/env python3
import os
import glob

# Find relevant files
zed_svos = sorted(glob.glob(os.path.join("/mission_data", "*_jetson_zed2i.svo2")))

# Process CPT bags
for zed_svo in zed_svos:
    print(f"Processing CPT bag: {zed_svo}")
    os.system(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")