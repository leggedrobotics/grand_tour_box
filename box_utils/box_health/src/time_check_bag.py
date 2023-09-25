#!/usr/bin/env python3

import sys
import rosbag
    
threshold = 100000

def check_clock_offsets(bag):
    counters = {
        "offset_mgbe0_systemclock": 0,
        "offset_mgbe0_mgbe1": 0,
        "offset_enp45s0_systemclock": 0,
        "offset_enp45s0_enp46s0": 0,
        "offset_mgbe0_enp45s0": 0
    }

    for topic, msg, t in rosbag.Bag(bag).read_messages():
        if topic == "/gt_box/health_status/jetson":
            offset_mgbe0_systemclock = int(msg.offset_mgbe0_systemclock)
            offset_mgbe0_mgbe1 = int(msg.offset_mgbe0_mgbe1)
            if offset_mgbe0_systemclock > threshold:
                counters["offset_mgbe0_systemclock"] += 1
            if offset_mgbe0_mgbe1 > threshold:
                counters["offset_mgbe0_mgbe1"] += 1

        elif topic == "/gt_box/health_status/nuc":
            offset_enp45s0_systemclock = int(msg.offset_enp45s0_systemclock)
            offset_enp45s0_enp46s0 = int(msg.offset_enp45s0_enp46s0)
            offset_mgbe0_enp45s0 = int(msg.offset_mgbe0_enp45s0)
            if offset_enp45s0_systemclock > threshold:
                counters["offset_enp45s0_systemclock"] += 1
            if offset_enp45s0_enp46s0 > threshold:
                counters["offset_enp45s0_enp46s0"] += 1
            if offset_mgbe0_enp45s0 > threshold:
                counters["offset_mgbe0_enp45s0"] += 1

    print("With threshold " + str(threshold) + ": ")
    print("mbge0 - systemclock:",  counters["offset_mgbe0_systemclock"])
    print("mgbe0 - mgbe1:", counters["offset_mgbe0_mgbe1"])
    print("enp45s0 - systemclock:",  counters["offset_enp45s0_systemclock"])
    print("enp45s0 - enp46s0:", counters["offset_enp45s0_enp46s0"])
    print("mgbe0 - enp45s0:",  counters["offset_mgbe0_enp45s0"])

if __name__ == "__main__":
    bag_path = sys.argv[1]
    check_clock_offsets(bag_path)