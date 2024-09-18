#!/usr/bin/env python3
import os
import glob
from boxi import shell_run
PRE = "source /opt/ros/noetic/setup.bash; source /home/opencv_ws/devel/setup.bash; source /home/catkin_ws/devel/setup.bash; "

# Find relevant files
cpt_bags = sorted(glob.glob(os.path.join("/mission_data", "*_nuc_cpt7.bag")))

# Process CPT bags
for cpt_bag in cpt_bags:
    print(f"Processing CPT bag: {cpt_bag}")
    shell_run(PRE + f"roslaunch novatel_oem7_driver cpt7_replay.launch input_bag_path:={cpt_bag}")