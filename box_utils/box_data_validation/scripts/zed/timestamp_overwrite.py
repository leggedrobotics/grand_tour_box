#!/usr/bin/env python3

import sys
import rosbag
import os


def restamp_bag(input_bag_path):
    output_bag_path = os.path.splitext(input_bag_path)[0] + "_restamped.bag"

    with rosbag.Bag(output_bag_path, "w", compression="lz4") as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            # Replace tf timestamps if applicable
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python restamp_bag.py <input_bag>")
        sys.exit(1)

    input_bag = sys.argv[1]
    restamp_bag(input_bag)
    print(f"Restamped bag saved as {os.path.splitext(input_bag)[0]}_restamped.bag")
