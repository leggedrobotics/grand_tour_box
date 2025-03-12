#!/usr/bin/env python3

import rosbag
import os

from box_auto.utils import (
    get_bag,
)


def process_bag(topic="/lidar/time_sync_status"):
    total_msgs = 0
    delayed_msgs = 0

    raw_bag = get_bag("*_npc_velodyne.bag")
    try:
        with rosbag.Bag(raw_bag, "r") as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                total_msgs += 1

                if msg.delayed_pulse_counter > 1:
                    delayed_msgs += 1
                    print("Warning: High delayed pulse counter: {} at time {}".format(msg.delayed_pulse_counter, t))

                if total_msgs % 100 == 0:
                    print(
                        "Processed {} messages so far; {} with delayed pulse counter > 1.".format(
                            total_msgs, delayed_msgs
                        )
                    )
    except Exception as e:
        print("Error processing bag file:", e)

    print("Finished processing.")
    print("Total messages processed: {}".format(total_msgs))
    print("Messages with delayed pulse counter > 1: {}".format(delayed_msgs))
    # Rename bag file if delayed messages were detected
    if delayed_msgs > 0:
        base_name = os.path.splitext(raw_bag)[0]
        new_name = base_name + "_FAILED.bag"
        try:
            os.rename(raw_bag, new_name)
            print(f"Renamed bag file to {new_name} due to delayed messages")
        except Exception as e:
            print(f"Failed to rename bag file: {e}")


if __name__ == "__main__":
    process_bag()
