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
        bag = rosbag.Bag(rosbag_path, "r")
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


def organize_data_folders(data_folder, cpt7_folder):
    # Find all subfolders and GPS folders
    subfolders = [
        f
        for f in os.listdir(data_folder)
        if re.match(r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}(_.+)?$", f) and os.path.isdir(os.path.join(data_folder, f))
    ]
    gps_to_start = {}

    for bagf in Path(cpt7_folder).rglob("*_RAWIMUSX.bag"):
        rosbag_start, rosbag_end = get_rosbag_times(bagf)
        gps_to_start[bagf] = rosbag_start

    mission_to_start = {}
    for folder in subfolders:

        for p in [f"{folder}_jetson_utils.bag", "alphasense_front_center.bag"]:
            rosbag_path = os.path.join(data_folder, folder, p)
            if os.path.exists(rosbag_path):
                rosbag_start, rosbag_end = get_rosbag_times(rosbag_path)

                if rosbag_start is not None:
                    mission_to_start[folder] = rosbag_start
                    break

    suc = True
    for mission, start in mission_to_start.items():
        best_dt = float("inf")
        for gps_rosbag_path, gps_start in gps_to_start.items():
            dt = abs((start - gps_start).total_seconds())
            if dt < best_dt:
                best_gps_rosbag_path = gps_rosbag_path
                best_dt = dt
                best_gps_start = gps_start

        if best_dt < 60:
            gps_rawimusx_ascii_file = Path(str(best_gps_rosbag_path).replace(".bag", ".ASCII"))
            pattern = "*" + gps_rawimusx_ascii_file.stem.replace("_RAWIMUSX", "") + "*.LOG"
            gps_log_file = next(Path(cpt7_folder).rglob(pattern))

            print(f"Move {best_gps_rosbag_path} to {mission} - Best dt {best_dt}")

            mission_folder = Path(data_folder) / mission
            if False:
                shutil.copy(str(gps_log_file), str(mission_folder))
                shutil.copy(str(gps_rawimusx_ascii_file), str(mission_folder))

        else:
            print(f"No good match found: Best dt {best_dt} - GPS {best_gps_start} - MISSION {start}")
            suc = False
    return suc


# Usage
def main():
    parser = argparse.ArgumentParser(description="Organize mission folders and GPS data")
    parser.add_argument(
        "--data_folder",
        default="/media/jonfrey/BoxiS2-2TB/deployment_day_8",
        type=str,
        help="Path to the mission folder",
    )
    parser.add_argument(
        "--cpt7_folder",
        default="/media/jonfrey/Data/CPT7/2024-11-27_post_leica",
        type=str,
        help="Path to the mission folder",
    )

    args = parser.parse_args()
    suc = organize_data_folders(args.data_folder, args.cpt7_folder)

    if not suc:
        exit(1)
    exit(0)


if __name__ == "__main__":
    main()
