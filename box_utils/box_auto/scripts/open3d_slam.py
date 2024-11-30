#!/usr/bin/env python3

import rospy
import os
from time import sleep
import subprocess
import psutil
from pathlib import Path
import shutil

MISSION_DATA = os.environ.get("MISSION_DATA", "/home/ttuna/Videos/dlio_verification")

WS = "/home/ttuna/grandtour_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"

# It is up-to-debate how we want to use this node. We regardless need the deskewed point cloud and some sort of pose estimation.
# If the ground truth poses are available, we can use them instead of dlio_poses.
PATTERNS = ["*_dlio.bag", "*_tf_static.bag"]


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

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_open3d_prep.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Using existing merged rosbag at {merged_rosbag_path}")
    else:
        print(f"Merging bags into {merged_rosbag_path}")
        os.system(
            f"python3 {WS}/src/grand_tour_box/box_utils/box_auto/scripts/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
        )

    kill_rosmaster()
    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(1)
    os.system(
        "bash -c '"
        + PRE
        + f"roslaunch open3d_slam_ros grandtour_replay.launch rosbag_filepath:={merged_rosbag_path}  map_saving_folder:={MISSION_DATA} &' "
    )
    sleep(1)
    # The console feels stuck but it continues.
    
    print("Moving and uploading bag!")

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_open3d_slam.bag")
    shutil.move(f"{MISSION_DATA}/open3d_slam_replayed.bag", output_bag_path)

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein upload --mission {uuid} {output_bag_path}")
    else:
        print(f"Finished processing. Open3d SLAM replayed bag saved to: {output_bag_path}")


def fetch_multiple_files_kleinkram(patterns):
    tmp_dir = os.path.join(MISSION_DATA, "tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    for pattern in patterns:
        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein download --mission {uuid} --dest {tmp_dir} '{pattern}'")

            # Move all files from /mission_data/tmp to /mission_data/
            for file_name in os.listdir(tmp_dir):
                source_file = os.path.join(tmp_dir, file_name)
                destination_file = os.path.join("/mission_data", file_name)
                shutil.move(source_file, destination_file)


if __name__ == "__main__":
    # fetch_multiple_files_kleinkram(PATTERNS)
    try:
        launch_nodes()
    except rospy.ROSInterruptException:
        pass
