#!/usr/bin/env python3

import rosbag
import argparse
from sensor_msgs.msg import Imu
from tqdm import tqdm

def extract_imu_submessage(input_bag, output_bag):
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in tqdm(inbag.read_messages(topics=["/gt_box/ap20/imu_debug/"]), total=inbag.get_message_count()):
                imu_msg = msg.imu  # Extract the `imu` field
                imu_msg.header.stamp = msg.header.stamp  # Ensure timestamp consistency
                outbag.write("/gt_box/ap20/imu", imu_msg, t)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract IMU submessage from /gt_box/ap20/imu_debug/ topic")
    parser.add_argument("--input", type=str, required=True, help="Path to input rosbag file")
    parser.add_argument("--output", type=str, required=True, help="Path to output rosbag file")

    args = parser.parse_args()
    extract_imu_submessage(args.input, args.output)
    print(f"Extracted IMU messages to {args.output}")
