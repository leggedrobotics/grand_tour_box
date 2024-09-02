#!/usr/bin/env python3

import os
import subprocess
import glob
import time

def find_files(mission_folder, pattern):
    return sorted(glob.glob(os.path.join(mission_folder, pattern)))

def run_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print(f"Error running command: {command}")
        print(stderr.decode())
    else:
        print(stdout.decode())

def main(mission_folder):
    # Create necessary directories
    os.makedirs(os.path.join(mission_folder, "combined_files"), exist_ok=True)
    os.makedirs(os.path.join(mission_folder, "log"), exist_ok=True)

    run_command("python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/scripts/repair_bags.py")
    run_command("python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/scripts/merge_mission.py")

    # Find relevant files
    cpt_bags = find_files(mission_folder, "*_nuc_cpt7.bag")
    tf_bags = find_files(mission_folder, "*_lpc_tf.bag")
    hesai_bags = find_files(mission_folder, "*_nuc_hesai.bag")
    zed_svos = find_files(mission_folder, "*_jetson_zed2i.svo2")

    print(f"Processing mission folder: {mission_folder}")

    # Process CPT bags
    for cpt_bag in cpt_bags:
        print(f"Processing CPT bag: {cpt_bag}")
        run_command(f"roslaunch novatel_oem7_driver cpt7_replay.launch input_bag_path:={cpt_bag}")
        time.sleep(2)

    # Process TF bags
    for tf_bag in tf_bags:
        print(f"Processing TF bag: {tf_bag}")
        run_command(f"roslaunch box_post_processor box_post_processor.launch input_filepath:={tf_bag}")
        time.sleep(2)

    # Process Hesai bags
    for hesai_bag in hesai_bags:
        print(f"Processing Hesai bag: {hesai_bag}")
        run_command(f"roslaunch hesai_ros_driver replay_packets.launch input_rosbag_path:={hesai_bag}")
        time.sleep(2)

    # Process ZED SVOs
    for zed_svo in zed_svos:
        print(f"Processing ZED SVO: {zed_svo}")
        run_command(f"roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}")
        time.sleep(2)

    print(f"\033[92mSuccessfully processed files in {mission_folder}\033[0m")

if __name__ == "__main__":
    mission_folder = "/mission_data"
    main(mission_folder)