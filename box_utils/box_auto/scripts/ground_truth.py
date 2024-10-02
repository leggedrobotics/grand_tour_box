#!/usr/bin/env python3

import rospy
import os
from time import sleep
import psutil
import glob

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source /home/opencv_gtsam_ws/setup.bash; source {WS}/devel/setup.bash;"

if __name__ == '__main__':
    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    
    gps = glob.glob(os.path.join(MISSION_DATA, "*_cpt7_gps_optimized_trajectory.bag"))[0]
    tf_static = glob.glob(os.path.join(MISSION_DATA, "*_tf_static.bag"))[0]
    imu = glob.glob(os.path.join(MISSION_DATA, "*_cpt7_raw_imu.bag"))[0]
    total_station = glob.glob(os.path.join(MISSION_DATA, "*_jetson_ap20_synced.bag"))[0]
    
    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(3)
    os.system("bash -c '" + PRE + f"roslaunch  bringup_graph_msf position3_fuser_replay_offline_bag.launch --tf_file {tf_static} --gps_file {gps} --imu_file {imu} --total_station_file {total_station} --use_sim_time:=true&' ")
    sleep(30)