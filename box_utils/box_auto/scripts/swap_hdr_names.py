import rosbag
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from ap20_driver_ros.msg import ImuDebug, PositionDebug, TimestampDebug
import rosbag
from collections import deque
import rospy
from pathlib import Path
import os
from copy import deepcopy

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

def get_bag(directory, pattern):
    return [str(s) for s in Path(directory).rglob(pattern)][0]

def read_bag_file(bag_path, bag_o_path, in_key, out_key):
    with rosbag.Bag(bag_o_path, 'w', compression='lz4') as bag_out:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                msg.header.frame_id = msg.header.frame_id.replace(in_key, out_key)
                bag_out.write(topic.replace(in_key, out_key), msg, t)
                
                

bags = [ (get_bag(MISSION_DATA, "*_jetson_hdr_front_swap.bag"), get_bag(MISSION_DATA, "*_jetson_hdr_front_swap.bag").replace("_hdr_front_swap.bag", "_hdr_right.bag"),  "hdr_front", "hdr_right"),
        (get_bag(MISSION_DATA, "*_jetson_hdr_right_swap.bag"), get_bag(MISSION_DATA, "*_jetson_hdr_right_swap.bag").replace("_hdr_right_swap.bag", "_hdr_front.bag"),  "hdr_right", "hdr_front")]

# Usage example
for bag, bag_o, in_key, out_key in bags:
    print(bag, bag_o, in_key, out_key)
    read_bag_file(bag, bag_o, in_key, out_key)
