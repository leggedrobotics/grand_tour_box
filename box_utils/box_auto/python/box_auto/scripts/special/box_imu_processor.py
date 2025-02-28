#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    get_bag,
    MISSION_DATA,
    kill_roscore,
    upload_bag,
    run_ros_command,
)
from pathlib import Path
import argparse
import shutil
import yaml

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

    # Iterate through each output pattern to ensure it is located where its expected.
    global_bag_path = get_bag(pattern=bag_pattern, auto_download=False, rglob=False)
    global_outbag_path = global_bag_path.replace(".bag", "_jetson_stim_ready.bag")

    # Set mode based on bag pattern
    if "stim" in bag_pattern:
        mode = "stim"
    else:
        mode = "default"  # Replace with appropriate default if needed

    # Read timeoffset from mission data
    topic_name = "/gt_box/stim320/imu"  # Topic name to extract
    timeoffset = None
    try:
        timeoffset_file = Path(MISSION_DATA) / "verification/all_missions_summary_skip_30.yaml"
        if timeoffset_file.exists():
            with open(timeoffset_file, "r") as f:
                timeoffset_data = yaml.safe_load(f)
                # Find the timeoffset for the specific topic
                for timestamp_key, topics in timeoffset_data.items():
                    if topic_name in topics:
                        timeoffset = topics[topic_name]
                        print(f"Loaded timeoffset for {topic_name}: {timeoffset}")
                        break
                if timeoffset is None:
                    print(f"Warning: Topic {topic_name} not found in timeoffset file")
        else:
            print(f"Warning: Timeoffset file {timeoffset_file} not found")
    except Exception as e:
        print(f"No timeoffset available: {e}")

    # Convert timeoffset from nanoseconds to milliseconds if it exists
    if timeoffset is not None:
        timeoffset = timeoffset / 1e6  # Convert from nanoseconds to milliseconds
        print(f"Timeoffset converted to milliseconds: {timeoffset}")

    kill_roscore()
    run_ros_command(
        f"roslaunch box_auto box_imu_processor.launch time_offset_to_apply:={timeoffset} input_rosbag_file:={global_bag_path} output_rosbag_file:={global_outbag_path} mode:={mode}",
        background=False,
    )
    kill_roscore()

    failed_file = Path("/tmp/stim320_failed")
    if failed_file.exists():
        raise RuntimeError("File 'stim320_failed' found in mission data.")

    shutil.move(global_outbag_path, global_bag_path)
    upload_bag([global_bag_path])

    print("\033[92mBox IMU processing finished.\033[0m")
    sleep(1)
