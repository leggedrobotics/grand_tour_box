#!/usr/bin/env python3

import rosbag
import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from pathlib import Path
import rospkg
import os
import argparse

def get_package_path(package_name):
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.common.ResourceNotFound:
        return None
    
def get_bag(directory, pattern):
    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False
    
    return files[0], True

def process_bags(reference_bag_path, tf_static_bag_path, output_bag_path):
    # Get start and end time from reference bag
    with rosbag.Bag(reference_bag_path, 'r') as ref_bag:
        start_time = ref_bag.get_start_time()
        end_time = ref_bag.get_end_time()
    
    # Get tf_static message from tf_static bag
    tf_static_msg = None
    with rosbag.Bag(tf_static_bag_path, 'r') as tf_bag:
        for topic, msg, t in tf_bag.read_messages(topics=['/tf_static']):
            tf_static_msg = msg
            break
    
    if tf_static_msg is None:
        raise ValueError("No tf_static message found in the tf_static bag")
    
    # Create output bag with repeated tf_static messages
    with rosbag.Bag(output_bag_path, 'w') as out_bag:
        current_time = start_time
        
        while current_time <= end_time:
            t = rospy.Time.from_sec( current_time)
            for i in range( len(tf_static_msg.transforms)):
                tf_static_msg.transforms[i].header.stamp = t
            out_bag.write('/tf_static', tf_static_msg, t)
            current_time = current_time + 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--directory", "-d",
        type=str,
        default="/mission_data",
        help="Directory to search for active bag files (default: current directory)."
    )
    args = parser.parse_args()

    reference_bag_path, suc = get_bag(args.directory, "*_jetson_adis.bag")
    tf_static_bag_path = os.path.join( get_package_path("box_calibration"), "calibration/tf_static.bag")               
    output_bag_path =  reference_bag_path.replace("_jetson_adis.bag", "_tf_static.bag")
    
    try:
        process_bags(reference_bag_path, tf_static_bag_path, output_bag_path)
        print(f"Successfully created {output_bag_path}")
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == '__main__':
    main()