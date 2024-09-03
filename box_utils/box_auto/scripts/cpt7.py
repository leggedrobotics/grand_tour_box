#!/usr/bin/env python3
import os
import glob

# Find relevant files
cpt_bags = sorted(glob.glob(os.path.join("/mission_data", "*_nuc_cpt7.bag")))

# Process CPT bags
for cpt_bag in cpt_bags:
    print(f"Processing CPT bag: {cpt_bag}")
    os.system(f"roslaunch novatel_oem7_driver cpt7_replay.launch input_bag_path:={cpt_bag}")