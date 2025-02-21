#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    MISSION_DATA,
    get_bag,
    kill_roscore,
    upload_bag,
    run_ros_command,
)
from pathlib import Path
import yaml


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


import argparse
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script configuration")
    parser.add_argument(
        "--config_name",
        type=str,
        default="boxi_processor_topics",  # boxi_processor_topics #boxi_tf_full
        help="The name of the configuration file. Default is 'boxi_processor_topics'.",
    )
    args = parser.parse_args()
    print(f"Using configuration: {args.config_name}")

    config_path = str(Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg" / (args.config_name + ".yaml"))

    bag_pattern = "*_jetson_stim.bag"
    # Iterate through each output pattern to ensure it is located where its expected.
    global_bag_path = get_bag(pattern=bag_pattern, auto_download=False, rglob=False)

    kill_roscore()
    # Run the evo preparation.
    run_ros_command(
        f"roslaunch box_auto box_post_processor.launch output_folder:={MISSION_DATA} input_filepath:={global_bag_path} config_path:={config_path}",
        background=False,
    )
    kill_roscore()

    failed_file = os.path.join(MISSION_DATA, "stim320_failed")
    if failed_file.exists():
        raise RuntimeError("File 'stim320_failed' found in mission data.")

    bag_pattern = bag_pattern.replace(".bag", "_post_processed.bag")
    outputbag_path = get_bag(pattern=bag_pattern, auto_download=False, rglob=False)

    upload_bag([outputbag_path])

    print("\033[92mBox Processor finished.\033[0m")
    sleep(1)
