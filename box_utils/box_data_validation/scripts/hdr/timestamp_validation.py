#!/usr/bin/env python3

import rospy
import rosbag
import argparse
import re
import numpy as np


def analyze_rosbag(bag_name):
    # Extract the hdr_x part from the bag name
    match = re.search(r"hdr_\w+", bag_name)
    if not match:
        rospy.logerr("Bag name must contain 'hdr_x' to determine the topic names.")
        return
    hdr_topic = match.group(0)

    # Define the topic names based on the extracted hdr_x
    kernel_topic = f"/gt_box/{hdr_topic}/kernel_timestamp"
    v4l2_topic = f"/gt_box/{hdr_topic}/v4l2_timestamp"

    kernel_timestamps = []
    v4l2_timestamps = []

    # Open and read the bag
    with rosbag.Bag(bag_name, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[kernel_topic, v4l2_topic]):
            if topic == kernel_topic:
                kernel_timestamps.append(msg.time_ref.to_sec())
            elif topic == v4l2_topic:
                v4l2_timestamps.append(msg.time_ref.to_sec())

    # Ensure both lists are non-empty and have alternating entries
    if not kernel_timestamps or not v4l2_timestamps:
        rospy.logerr("One or both timestamp topics are empty.")
        return

    # Pair the kernel and v4l2 timestamps, checking for correct alternating sequence
    deltas = []
    i, j = 0, 0
    error_found = False

    while i < len(kernel_timestamps) and j < len(v4l2_timestamps):
        # Check for correct alternating order
        if i < len(kernel_timestamps) - 1 and kernel_timestamps[i + 1] < v4l2_timestamps[j]:
            rospy.logerr(f"Error: Two consecutive kernel_timestamps detected at index {i}.")
            error_found = True
            i += 1
        elif j < len(v4l2_timestamps) - 1 and v4l2_timestamps[j + 1] < kernel_timestamps[i]:
            rospy.logerr(f"Error: Two consecutive v4l2_timestamps detected at index {j}.")
            error_found = True
            j += 1
        else:
            delta = (v4l2_timestamps[j] - kernel_timestamps[i]) * 1000  # Convert to milliseconds
            deltas.append(delta)
            i += 1
            j += 1

    if not error_found:
        rospy.loginfo("No consitency issues found!")

    # Calculate statistics
    if deltas:
        mean_delta = np.mean(deltas)
        std_dev_delta = np.std(deltas)
        rospy.loginfo(f"Mean delta: {mean_delta:.3f} ms, Std dev: {std_dev_delta:.3f} ms")
    else:
        rospy.logwarn("No valid deltas found between timestamps.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze ROS bag timestamp topics.")
    parser.add_argument("bag_name", type=str, help="Name of the ROS bag file to analyze")
    args = parser.parse_args()

    rospy.init_node("rosbag_timestamp_analyzer", anonymous=True)
    analyze_rosbag(args.bag_name)
