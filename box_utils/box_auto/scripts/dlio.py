#!/usr/bin/env python3

import rospy
import os
from time import sleep
import subprocess
import psutil
from pathlib import Path
import shutil

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

WS = "/home/catkin_ws"
# WS = "/home/jonfrey/workspaces/box_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"
PATTERNS = ["*_jetson_stim.bag", "*_tf_static.bag", "*_nuc_hesai_post_processed.bag"]


def get_bag(directory, pattern):
    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False

    return files[0], True


def check_ros_node_exists(node_name):
    try:
        # Run the `rosnode list` command to get a list of all nodes
        result = subprocess.run(["rosnode", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Check if the command was successful
        if result.returncode != 0:
            raise RuntimeError(f"Error running rosnode list: {result.stderr}")

        # Get the output of the command and split it into lines
        nodes = result.stdout.splitlines()

        # Check if the node_name is in the list of nodes
        return node_name in nodes

    except Exception as e:
        print(f"An error occurred: {e}")
        return False


def kill_rosmaster():
    # Find all processes with name 'rosmaster'
    for proc in psutil.process_iter(["pid", "name"]):
        if "rosmaster" in proc.info["name"]:
            try:
                # Terminate the process
                print(f"Terminating rosmaster with PID {proc.info['pid']}")
                proc.terminate()
                proc.wait(timeout=60)  # Wait up to 60 seconds for the process to terminate
                print("rosmaster process terminated successfully.")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired) as e:
                print(f"Failed to terminate rosmaster process: {e}")


def launch_nodes():
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"

    inputs = []

    for pattern in PATTERNS:
        f, suc = get_bag(MISSION_DATA, pattern)
        if suc:
            inputs.append(f)
            timestamp = str(f).split("/")[-1]
            timestamp = timestamp.split("_")[0]

        else:
            raise ValueError("Pattern not found: ", pattern, " in Directory")

    assert len(inputs) == len(PATTERNS), "Failure did not find all the bags required"

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged.bag")
    os.system(
        f"python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/scripts/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
    )

    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(1)
    os.system(
        "bash -c '"
        + PRE
        + f"roslaunch direct_lidar_inertial_odometry dlio_replay.launch input_rosbag_path:={merged_rosbag_path}  output_rosbag_path:={MISSION_DATA} &' "
    )
    sleep(10)
    os.system("bash -c '" + PRE + f"rosbag play -r 5 -d 5 --wait-for-subscribers --clock {merged_rosbag_path}' ")
    sleep(60)
    kill_rosmaster()

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_dlio.bag")
    shutil.move(f"{WS}/src/grand_tour_box/box_applications/dlio/data/dlio_replayed.bag", output_bag_path)

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein mission upload --mission-uuid {uuid} --path {output_bag_path}")
    else:
        print(f"Finished processing. Hesai bag saved as: {output_bag_path}")


def fetch_multiple_files_kleinkram(patterns):
    tmp_dir = os.path.join(MISSION_DATA, "/tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    for pattern in patterns:
        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein mission download --mission-uuid {uuid} --local-path {tmp_dir} --pattern {pattern}")

            # Move all files from /mission_data/tmp to /mission_data/
            for file_name in os.listdir(tmp_dir):
                source_file = os.path.join(tmp_dir, file_name)
                destination_file = os.path.join("/mission_data", file_name)
                shutil.move(source_file, destination_file)


if __name__ == "__main__":
    fetch_multiple_files_kleinkram(PATTERNS)
    try:
        launch_nodes()
    except rospy.ROSInterruptException:
        pass
