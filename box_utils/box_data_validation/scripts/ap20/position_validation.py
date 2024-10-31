#!/usr/bin/env python3

import rospy
import rosbag
import numpy as np
import argparse
from math import sqrt


def calculate_cartesian_distance(bag_name, topic_name="/ap20/tps"):
    first_positions = []
    last_positions = []

    # Open the ROS bag
    with rosbag.Bag(bag_name, "r") as bag:
        # Get all messages in the specified topic
        messages = [msg for _, msg, _ in bag.read_messages(topics=[topic_name])]

        if len(messages) < 20:
            rospy.logwarn("Not enough messages to compute first and last 10 positions.")
            return

        # Extract the first 10 and last 10 messages
        first_positions = messages[10:20]
        last_positions = messages[-20:-10]

    # Helper function to calculate mean x, y, z
    def mean_position(positions):
        x_values = [msg.point.x for msg in positions]
        y_values = [msg.point.y for msg in positions]
        z_values = [msg.point.z for msg in positions]
        return np.mean(x_values), np.mean(y_values), np.mean(z_values)

    # Calculate mean positions for the first and last 10 messages
    mean_first = mean_position(first_positions)
    mean_last = mean_position(last_positions)

    # Calculate Cartesian distance between the mean positions
    distance = sqrt(
        (mean_first[0] - mean_last[0]) ** 2 + (mean_first[1] - mean_last[1]) ** 2 + (mean_first[2] - mean_last[2]) ** 2
    )

    distance_cm = distance * 100
    print(f"Cartesian distance between first 10 and last 10 mean positions: {distance_cm:.3f} cm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calculate distance between mean positions in a ROS bag.")
    parser.add_argument("bag_name", type=str, help="Path to the ROS bag file")
    args = parser.parse_args()

    rospy.init_node("position_mean_distance_calculator", anonymous=True)
    calculate_cartesian_distance(args.bag_name)
