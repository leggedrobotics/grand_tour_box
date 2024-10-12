#!/usr/bin/env python3

import rosbag
import sys
import os 
import pathlib
import numpy as np
import contextlib

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

class TeeFile:
    def __init__(self, filename):
        self.file = open(filename, 'w')
        self.stdout = sys.stdout

    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)

    def flush(self):
        self.file.flush()
        self.stdout.flush()

def check_clock_offsets(bags):
    val = {
        "offset_ptp4l_enp45s0": [],
        "offset_phc2sys_enp45s0": [],
        "offset_phc2sys_enp46s0": [],
        "offset_ptp4l_mgbe0": [],
        "offset_phc2sys_mgbe0": [],
        "nr_messages_jetson": 0,
        "nr_messages_nuc": 0,
    }

    results_filename = os.path.join(MISSION_DATA, f"time_check_bag.txt")
    
    with contextlib.redirect_stdout(TeeFile(results_filename)):
        for bag in bags:
            for topic, msg, t in rosbag.Bag(bag).read_messages():
                if topic == "/gt_box/health_status/jetson/status":
                    val["offset_ptp4l_mgbe0"].append( int(msg.offset_ptp4l_mgbe0))
                    val["offset_phc2sys_mgbe0"].append( int(msg.offset_phc2sys_mgbe0))
                    val["nr_messages_jetson"] += 1

                elif topic == "/gt_box/health_status/nuc/status":
                    val["offset_ptp4l_enp45s0"].append( int(msg.offset_ptp4l_enp45s0))
                    val["offset_phc2sys_enp45s0"].append( int(msg.offset_phc2sys_enp45s0))
                    val["offset_phc2sys_enp46s0"].append( int(msg.offset_phc2sys_enp46s0))

                    val["nr_messages_nuc"] += 1

        # Calculate and format statistics
        print("Statistics for Network Offsets:")
        print("-" * 80)
        print(f"{'Offset Type':<25} {'Mean [ns]':>15} {'Std Dev [ns]':>15} {'Max [ns]':>15}")
        print("-" * 80)

        for key, values in val.items():
            if type(values) == list:
                if values:  # Only process if we have data
                    mean = np.mean(values)
                    std = np.std(values)
                    max_val = np.max(values)
                    print(f"{key:<25} {mean:>15.2f} {std:>15.2f} {max_val:>15.0f}")
                else:
                    print(f"{key:<25} {'No data':>15} {'No data':>15} {'No data':>15}")
            else:
                print(f"{key:<25} {values:>15} ")
                
        print("-" * 80)


if __name__ == "__main__":
    check_clock_offsets( [str(s) for s in pathlib.Path(MISSION_DATA).rglob("*_nuc_utils.bag")] + 
                         [str(s) for s in pathlib.Path(MISSION_DATA).rglob("*_jetson_utils.bag")]
                        )
