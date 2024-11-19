import os
import re
from datetime import datetime, timedelta
import shutil
import rosbag
import argparse


def parse_datetime(date_str):
    """Parse datetime from folder/file names."""
    return datetime.strptime(date_str, "%Y-%m-%d-%H-%M-%S")


def get_rosbag_times(rosbag_path):
    """Extract start and end times from rosbag."""
    try:
        bag = rosbag.Bag(rosbag_path)
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        bag.close()

        # Convert ROS time to datetime
        start_datetime = datetime.fromtimestamp(start_time)
        end_datetime = datetime.fromtimestamp(end_time)

        return start_datetime, end_datetime
    except Exception as e:
        print(f"Error reading rosbag: {e}")
        return None, None


def organize_data_folders(data_folder):
    # Find all subfolders and GPS folders
    subfolders = [
        f
        for f in os.listdir(data_folder)
        if re.match(r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}", f) and os.path.isdir(os.path.join(data_folder, f))
    ]

    gps_folders = [
        f
        for f in os.listdir(data_folder)
        if re.match(r"NMZT\d+N_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}", f) and os.path.isdir(os.path.join(data_folder, f))
    ]

    matched_gps = []
    unmatched_gps = []

    # Sort subfolders by start time
    subfolders.sort(key=parse_datetime)

    for gps_folder in gps_folders:
        # Parse GPS folder time and add 1 hour
        gps_time = datetime.strptime(gps_folder.split("_")[1] + " " + gps_folder.split("_")[2], "%Y-%m-%d %H-%M-%S")
        gps_time_adjusted = gps_time + timedelta(hours=1)

        # Find best matching mission start folder
        best_match = None
        smallest_offset = float("inf")

        for start_folder in subfolders:
            start_time = parse_datetime(start_folder)

            # Check if GPS time is within 25 minutes of mission start
            offset = abs((gps_time_adjusted - start_time).total_seconds() / 60)

            if offset <= 25 and offset < smallest_offset:
                # Ensure end time (GPS time) is after start time
                if gps_time_adjusted > start_time:
                    best_match = start_folder
                    smallest_offset = offset

        if best_match:
            # Move GPS folder and log file
            gps_source = os.path.join(data_folder, gps_folder)
            gps_dest = os.path.join(data_folder, best_match, gps_folder)

            log_file = gps_folder + ".LOG"
            log_source = os.path.join(data_folder, log_file)
            log_dest = os.path.join(data_folder, best_match, log_file)

            # Verify rosbag times
            rosbag_path = os.path.join(data_folder, best_match, f"{best_match}_tf_static.bag")
            if os.path.exists(rosbag_path):
                rosbag_start, rosbag_end = get_rosbag_times(rosbag_path)

                if rosbag_start and rosbag_end:
                    mission_duration = rosbag_end - rosbag_start
                    print(f"{best_match} --- Duration: {mission_duration}")
                    print(f"   Rosbag Start: {rosbag_start}")
                    print(f"   Rosbag End: {rosbag_end}")
                    print(f"   GPS Time (adjusted): {gps_time_adjusted}")

                    # Check time difference
                    time_diff = abs((rosbag_end - gps_time_adjusted).total_seconds())
                    print(f"   Time Difference: {time_diff} seconds")

                    if time_diff < 10:
                        matched_gps.append(gps_folder)

                        # Move GPS folder and log file
                        shutil.move(gps_source, gps_dest)
                        shutil.move(log_source, log_dest)

                else:
                    ValueError(f"{best_match} --- Error reading rosbag times")
                    unmatched_gps.append(gps_folder)

            else:
                ValueError(f"{best_match} --- Rosbag not found")
                unmatched_gps.append(gps_folder)
        else:
            unmatched_gps.append(gps_folder)
        print(" ")
        print(" ")

    # Print summary
    print("Matched GPS Folders:", matched_gps)
    print("Unmatched GPS Folders:", unmatched_gps)


# Usage
def main():
    parser = argparse.ArgumentParser(description="Organize mission folders and GPS data")
    parser.add_argument("--data_folder", type=str, help="Path to the mission folder")

    args = parser.parse_args()
    organize_data_folders(args.data_folder)


if __name__ == "__main__":
    main()
