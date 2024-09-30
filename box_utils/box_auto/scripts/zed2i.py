#!/usr/bin/env python3
import subprocess
import glob
import os
import sys
from boxi import shell_run
PRE = "source /opt/ros/noetic/setup.bash; source /home/opencv_gtsam_ws/devel/setup.bash; source /home/catkin_ws/devel/setup.bash; "
# Find relevant files
zed_svos = sorted(glob.glob(os.path.join("/mission_data", "*_jetson_zed2i.svo2")))
# Process CPT bags
for zed_svo in zed_svos:
    print(f"Processing CPT bag: {zed_svo}")
    command = PRE + f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}"
    return_code = shell_run(command)
    
    if return_code == 0:
        print(f"Successfully processed {zed_svo}")
    else:
        print(f"Error processing {zed_svo} (Return code: {return_code})")