#!/usr/bin/env python3

import rospy
import os
from time import sleep
import subprocess
import glob
import subprocess
import psutil


MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"


def check_ros_node_exists(node_name):
    try:
        # Run the `rosnode list` command to get a list of all nodes
        result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
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
    for proc in psutil.process_iter(['pid', 'name']):
        if 'rosmaster' in proc.info['name']:
            try:
                # Terminate the process
                print(f"Terminating rosmaster with PID {proc.info['pid']}")
                proc.terminate()
                proc.wait(timeout=5)  # Wait up to 5 seconds for the process to terminate
                print("rosmaster process terminated successfully.")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired) as e:
                print(f"Failed to terminate rosmaster process: {e}")

def launch_nodes(input_rosbag_path):
    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    
    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(3)
    os.system("bash -c '" + PRE + f"rosrun hesai_ros_driver hesai_ros_driver_node _config_path:={WS}/src/grand_tour_box/box_drivers/hesai_lidar_ros_driver/config/packet_replay.yaml _use_sim_time:=True _input_rosbag_path:={input_rosbag_path} &' ")
    sleep(3)
    os.system("bash -c '" + PRE + f"rosbag play -r 5 -d 5 --wait-for-subscribers --clock {input_rosbag_path} --topics /gt_box/hesai/packets' ")
    sleep(30)
    kill_rosmaster()


if __name__ == '__main__':
    input_rosbag_paths = sorted(glob.glob(os.path.join(MISSION_DATA, "*_nuc_hesai.bag")))

    print(f"Found {len(input_rosbag_paths)} Hesai bags in {MISSION_DATA}")

    if len(input_rosbag_paths) == 0:
        print("Bags available: " , [str(s) for s in glob.glob(os.path.join(MISSION_DATA, "*.bag"))])

    for input_rosbag_path in input_rosbag_paths:
        print(f"Processing Hesai bag: {input_rosbag_path}")
        try:
            launch_nodes(input_rosbag_path)
        except rospy.ROSInterruptException:
            pass