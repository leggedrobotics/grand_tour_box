#!/usr/bin/env python3

from time import sleep
from box_auto.utils import (
    WS,
    MISSION_DATA,
    get_bag,
    kill_roscore,
    run_ros_command,
)
from pathlib import Path
import yaml


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script configuration")
    parser.add_argument(
        "--config_name",
        type=str,
        default="box_tf_pure_perception",  # box_tf_pure_perception #boxi_tf_full
        help="The name of the configuration file. Default is 'box_tf_pure_perception'.",
    )
    args = parser.parse_args()
    print(f"Using configuration: {args.config_name}")
    patterns = ["*_nuc_tf.bag", "*_tf_static.bag", "*_lpc_tf.bag", "*_dlio_new_formulation_minus_1ms.bag"]

    path = Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg" / (args.config_name + ".yaml")

    # Configuration path.
    config_path = str(path)

    inputs = []
    # Iterate through each output pattern to ensure it is located where its expected.
    for pattern in patterns:
        f = get_bag(pattern=pattern, auto_download=False, rglob=False)
        inputs.append(f)

    tf_bag_paths = ",".join(inputs)
    tf_bag_paths = "[" + tf_bag_paths + "]"

    kill_roscore()
    # Run the evo preparation.
    run_ros_command(
        f"roslaunch box_auto box_tf_processor.launch output_folder:={MISSION_DATA} tf_bag_paths:={tf_bag_paths} bag_post_fix:={args.config_name} config_path:={config_path}",
        background=False,
    )
    kill_roscore()
    print("\033[92mTF and TF_Static Successfully curated.\033[0m")
    sleep(1)

    # Deploy .zip file reader e.g.
