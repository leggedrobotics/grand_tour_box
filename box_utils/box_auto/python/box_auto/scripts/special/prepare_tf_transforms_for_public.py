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

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script configuration")
    parser.add_argument(
        "--config_name",
        type=str,
        default="tf_minimal",  # tf_model, tf_minimal
        help="The name of the configuration file. Default is 'boxi_tf_pure_perception'.",
    )
    args = parser.parse_args()
    print(f"Using configuration: {args.config_name}")

    # The first pattern is used to infer the date tag of the output bag.
    patterns = ["*_lpc_tf.bag", "*_tf_static.bag", "*_cpt7_ie_tc.bag", "*_hesai_dlio.bag"]

    # Patterns for optional applications.
    # ,"*_hesai_dlio.bag"
    # ,"*_gt_tf.bag"
    # ,"*_open3d_slam.bag"

    path = Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg" / (args.config_name + ".yaml")

    # Configuration path.
    config_path = str(path)

    inputs = []
    # Iterate through each output pattern to ensure it is located where its expected.
    for pattern in patterns:
        try:
            f = get_bag(pattern=pattern, auto_download=False, rglob=False)
            inputs.append(f)
        except Exception as e:
            print(e)

    tf_bag_paths = ",".join(inputs)
    tf_bag_paths = "[" + tf_bag_paths + "]"

    kill_roscore()
    run_ros_command(
        f"roslaunch box_auto box_tf_processor.launch output_folder:={MISSION_DATA} tf_bag_paths:={tf_bag_paths} bag_post_fix:={args.config_name} config_path:={config_path}",
        background=False,
    )
    kill_roscore()
    sleep(1)
