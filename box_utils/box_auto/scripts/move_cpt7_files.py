import os
import re
from datetime import datetime
import shutil
import rosbag
import argparse
from pathlib import Path


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
        if re.match(r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}(_.+)?$", f) and os.path.isdir(os.path.join(data_folder, f))
    ]
    gps_to_start = {}

    for bagf in Path(data_folder).rglob("*_RAWIMUSX.bag"):
        rosbag_start, rosbag_end = get_rosbag_times(bagf)
        gps_to_start[bagf] = rosbag_start

    mission_to_start = {}
    for folder in subfolders:

        for p in [f"{folder}_jetson_utils.bag", "alphasense_front_center.bag"]:
            rosbag_path = os.path.join(data_folder, folder, p)
            if os.path.exists(rosbag_path):
                rosbag_start, rosbag_end = get_rosbag_times(rosbag_path)
                mission_to_start[folder] = rosbag_start
                break

    for mission, start in mission_to_start.items():
        delta = float("inf")

        for gps, gps_start in gps_to_start.items():
            dt = abs((start - gps_start).seconds)
            if dt < delta:
                best = gps
                best_dt = dt

            if best_dt < 60:
                print(f"Move {gps} to {mission} - Best dt {best_dt}")
                shutil.copy(str(best).replace(".bag", ".ASCII"), os.path.join(data_folder, mission))
                shutil.copy(
                    os.path.join(data_folder, Path(best).name.replace("_RAWIMUSX.bag", ".LOG")),
                    os.path.join(data_folder, mission),
                )
    print(mission_to_start)


# Usage
def main():
    parser = argparse.ArgumentParser(description="Organize mission folders and GPS data")
    parser.add_argument(
        "--data_folder", default="/media/jonfrey/Data/deployment_day_16", type=str, help="Path to the mission folder"
    )

    args = parser.parse_args()
    organize_data_folders(args.data_folder)


if __name__ == "__main__":
    main()
