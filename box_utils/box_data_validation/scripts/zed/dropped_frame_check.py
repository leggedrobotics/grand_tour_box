#!/usr/bin/env python3

import sys
import rosbag
import numpy as np
from collections import deque


def check_dropped_frames(bag_file):
    topic = "/gt_box/zed2i/zed_node/left/image_rect_color/compressed"

    deltas = []
    dropped_frames = []
    previous_msg = None
    recent_deltas = deque(maxlen=5)  # Stores the last 10 deltas

    with rosbag.Bag(bag_file, "r") as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            if previous_msg:
                current_stamp = msg.header.stamp.to_sec()
                previous_stamp = previous_msg.header.stamp.to_sec()
                delta = current_stamp - previous_stamp
                deltas.append(delta)

                if len(recent_deltas) == 5:
                    avg_recent_deltas = np.mean(recent_deltas)
                    if delta > 1.2 * avg_recent_deltas:
                        dropped_frames.append((msg.header.seq, delta))

                recent_deltas.append(delta)
            previous_msg = msg

    # Calculate statistics
    avg_delta = np.mean(deltas)
    std_delta = np.std(deltas)

    # Output results
    print(f"Average delta: {avg_delta:.6f} seconds")
    print(f"Standard deviation of deltas: {std_delta:.6f} seconds")

    if dropped_frames:
        print("Dropped frames detected at the following seq IDs:")
        for seq, delta in dropped_frames:
            print(f"Seq ID: {seq}, Delta: {delta:.6f} seconds")
    else:
        print("No dropped frames detected.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python dropped_frame_check.py <input_bag>")
        sys.exit(1)

    bag_file = sys.argv[1]
    check_dropped_frames(bag_file)
