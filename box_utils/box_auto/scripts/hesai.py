#!/usr/bin/env python3
import os
import glob

# Find relevant files
hesai_bags = sorted(glob.glob(os.path.join("/mission_data", "*_nuc_hesai.bag")))

# Process CPT bags
for hesai_bag in hesai_bags:
    print(f"Processing CPT bag: {hesai_bag}")
    os.system(f"roslaunch hesai_ros_driver replay_packets.launch input_rosbag_path:={hesai_bag}")