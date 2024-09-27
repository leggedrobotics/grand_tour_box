#!/usr/bin/env python3

import rospy
import os
from time import sleep
import psutil
import glob

WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source /home/opencv_gtsam_ws/setup.bash; source {WS}/devel/setup.bash;"

if __name__ == '__main__':
    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    
    gps = glob.glob(os.path.join("/mission_data", "*_state_6D_pose.bag"))[0]
    tf = glob.glob(os.path.join("/mission_data", "*_nuc_tf.bag"))[0]
    cpt7 = glob.glob(os.path.join("/mission_data", "*_nuc_livox.bag"))[0]
    livox = glob.glob(os.path.join("/mission_data", "*_nuc_livox.bag"))[0]
    
    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(3)
    # os.system("bash -c '" + PRE + f"roslaunch  atn_position3_fuser --estimate_file {gps} --pt_file {livox} tf_file {tf}' ")
    os.system("bash -c '" + PRE + f"roslaunch  atn_position3_fuser position3_fuser_replay.launch --rviz False")
    sleep(30)

    