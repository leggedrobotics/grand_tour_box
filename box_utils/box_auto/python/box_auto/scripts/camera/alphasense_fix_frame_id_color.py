#!/usr/bin/env python3
"""Retain the historical entry point for the two color-camera frame corrections."""
from pathlib import Path
import rosbag
from box_auto.utils import get_bag, upload_bag
from box_auto.utils.atomic_file import atomic_output

FRAME_ID_MAP = {
    "/gt_box/alphasense_driver_node/cam1/color/camera_info": "alphasense_front_center",
    "/gt_box/alphasense_driver_node/cam1/color/image/compressed": "alphasense_front_center",
    "/gt_box/alphasense_driver_node/cam3/color/camera_info": "alphasense_front_left",
    "/gt_box/alphasense_driver_node/cam3/color/image/compressed": "alphasense_front_left",
}


def correct_frames(source, destination):
    if Path(source).resolve() == Path(destination).resolve():
        raise ValueError("Output must differ from the input bag")
    corrected = 0
    with atomic_output(destination) as temporary:
        with rosbag.Bag(str(source), "r") as input_bag, rosbag.Bag(
            str(temporary), "w", compression="lz4"
        ) as output_bag:
            for topic, msg, stamp in input_bag.read_messages():
                if topic in FRAME_ID_MAP:
                    msg.header.frame_id = FRAME_ID_MAP[topic]
                    corrected += 1
                output_bag.write(topic, msg, stamp)
            if not corrected:
                raise ValueError("No expected color-camera messages found; existing output is preserved")
    return corrected


def main():
    source = get_bag("*_nuc_alphasense_tmp.bag")
    destination = source.replace("_nuc_alphasense_tmp.bag", "_nuc_alphasense_color.bag")
    correct_frames(source, destination)
    upload_bag(destination)


if __name__ == "__main__":
    main()
