# Verify that there is only a single LOG file per directory
# Verify that the files timestamps match with the _nuc_livox

import rosbag
from datetime import datetime
from box_auto.utils import MISSION_DATA, get_bag
from pathlib import Path


def get_rosbag_times(rosbag_path):
    """Extract start and end times from rosbag."""
    with rosbag.Bag(rosbag_path, "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()

    # Convert ROS time to datetime
    start_datetime = datetime.fromtimestamp(start_time)
    end_datetime = datetime.fromtimestamp(end_time)
    return start_datetime, end_datetime


def verify_log_file(mission_data):
    # Find .LOG files
    log_file = [str(s) for s in Path(MISSION_DATA).rglob("*.LOG") if "/ie/" not in str(s)]
    suc = True if len(log_file) == 1 else False

    if not suc:
        raise ValueError(f"Expected 1 .LOG file, found {len(log_file)}")

    # Find rosbag files
    log_reference_bag = get_bag("*_cpt7_raw_imu.bag")

    suc = False
    with rosbag.Bag(log_reference_bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=["/gt_box/cpt7/offline_from_novatel_logs/info"]):
            # Remove "RAWIMUX" from the string and check if it matches the log file path
            cleaned_info = msg.data
            cleaned_info = cleaned_info.replace("_RAWIMUSX", "")
            suc = True
            break

    if not suc:
        raise ValueError("cpt7_raw_imu_bag does not contain debug info")
    else:

        if cleaned_info not in log_file:
            print(log_file)
            raise ValueError(f"cpt7_raw_imu_bag created based on different .LOG file {cleaned_info}, {log_file}")

    livox_bag = get_bag("*_nuc_livox.bag")

    # Check timestamp compatibility with rosbags
    start_time, end_time = get_rosbag_times(livox_bag)
    start_time_ref, end_time_ref = get_rosbag_times(log_reference_bag)

    # Check if log file modification time is within rosbag time range
    if start_time <= start_time_ref <= end_time:
        print(f"LOG file timestamp matches rosbag {livox_bag} {log_reference_bag} {log_file}")
        print(f"  livox_start_time: {start_time} ")
        print(f"  gps_start_time:   {start_time_ref} ")
        return True
    else:
        print(f"  livox__time: {start_time} {end_time} ")
        print(f"  gps__time:   {start_time_ref} {end_time_ref}")
        raise ValueError("Times mismatch!")


def main():
    try:
        verify_log_file(MISSION_DATA)
    except Exception as e:
        print(e)
        exit(-1)

    print("Verification successful!")
    exit(0)


if __name__ == "__main__":
    main()
