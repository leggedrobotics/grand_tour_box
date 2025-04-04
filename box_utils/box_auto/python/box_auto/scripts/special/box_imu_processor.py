#!/usr/bin/env python3
import gspread
import pathlib
from time import sleep
from box_auto.utils import (
    WS,
    get_bag,
    MISSION_DATA,
    kill_roscore,
    upload_bag,
    run_ros_command,
    BOX_AUTO_DIR,
)
from pathlib import Path
import argparse
import shutil
import os


def read_stim_time_offset(date_str):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Open the Google Sheet
    sheet = gc.open_by_key("1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg")
    worksheet = sheet.worksheet("mission_overview")

    # Read data from A1 to Z500 (all data)
    data = worksheet.get_all_values("A1:Z500")

    # Find the row where the first column entry matches date_str
    target_row = None
    for _, row in enumerate(data):
        if row[0] == date_str:
            target_row = row
            break

    if target_row is None:
        raise ValueError(f"No row found with date_str: {date_str}")

    header_row = data[0]
    try:
        column_index = header_row.index("stim320-imu_mean")
    except ValueError:
        raise ValueError("Column 'stim320-imu_mean' not found in header row")

    return float(target_row[column_index])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script configuration")
    parser.add_argument(
        "--config_name",
        type=str,
        default="boxi_imu_processor",  # boxi_imu_processor
        help="The name of the configuration file. Default is 'boxi_imu_processor'.",
    )
    args = parser.parse_args()
    print(f"Using configuration: {args.config_name}")

    config_path = str(Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg" / (args.config_name + ".yaml"))
    bag_pattern = "*_jetson_stim.bag"
    mode = "stim"

    # Iterate through each output pattern to ensure it is located where its expected.
    global_bag_path = get_bag(pattern=bag_pattern, auto_download=False, rglob=False)
    global_outbag_path = global_bag_path.replace("_jetson_stim.bag", "_jetson_stim_ready.bag")
    # Remove global_outbag_path if it exists
    if os.path.exists(global_outbag_path):
        os.remove(global_outbag_path)
        print(f"Removed existing file: {global_outbag_path}")

    try:
        date_str = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob(bag_pattern)][0]
    except:
        raise FileNotFoundError(f"No bag found with pattern: {bag_pattern}")

    # Read timeoffset from mission data
    topic_name = "/gt_box/stim320/imu"  # Topic name to extract

    # Read the timeoffset from the Google Sheet
    timeoffset = read_stim_time_offset(date_str)

    kill_roscore()
    run_ros_command(
        f"roslaunch box_auto box_imu_processor.launch time_offset_to_apply:={timeoffset} input_rosbag_file:={global_bag_path} output_rosbag_file:={global_outbag_path} mode:={mode}",
        background=False,
    )

    failed_file = Path("/tmp/stim320_failed")
    if failed_file.exists():
        raise RuntimeError("File 'stim320_failed' found in mission data.")

    # Replace the original bag with the processed bag
    shutil.move(global_outbag_path, global_bag_path)
    upload_bag([global_bag_path])

    print("\033[92mBox IMU processing finished.\033[0m")
    sleep(1)
