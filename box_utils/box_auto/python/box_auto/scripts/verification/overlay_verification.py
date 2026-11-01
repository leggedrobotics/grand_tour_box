#!/usr/bin/env python3
"""Replay processed sensor bags for visual calibration checks on a private master.

The model bag supplies transforms. Passing a recording through this viewer is a
software check; it does not establish physical calibration accuracy.
"""

import argparse
import math
from pathlib import Path
import shlex
import subprocess

import rosbag
from box_auto.utils.ros_session import private_ros_session

PATTERNS = (
    "*_tf_model.bag",
    "*_alphasense.bag",
    "*_zed2i.bag",
    "*_zed.bag",
    "*_anymal_depth_cameras.bag",
    "*_anymal_velodyne_undist.bag",
    "*_hdr_front.bag",
    "*_hdr_left.bag",
    "*_hdr_right.bag",
    "*_hesai_undist.bag",
    "*_livox_undist.bag",
)


def prepare_inputs(paths):
    paths = list(dict.fromkeys(Path(path).resolve() for path in paths))
    if not paths:
        raise ValueError("No processed overlay bags found")
    topics = {}
    has_static_transforms = False
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            for topic, info in bag.get_type_and_topic_info().topics.items():
                if info.message_count:
                    if topic in topics and topics[topic] != info.msg_type:
                        raise ValueError("Conflicting message types for " + topic)
                    topics[topic] = info.msg_type
            for _, message, _ in bag.read_messages(topics=["/tf_static"]):
                has_static_transforms |= bool(message.transforms)
    if not has_static_transforms:
        raise ValueError("Overlay requires a nonempty /tf_static model bag")
    if not any(
        kind in ("sensor_msgs/PointCloud2", "sensor_msgs/Image", "sensor_msgs/CompressedImage")
        for kind in topics.values()
    ):
        raise ValueError("Overlay requires camera or point-cloud messages")
    flags = {
        "enable_zed": any(t.startswith("/boxi/zed2i/") for t in topics),
        "enable_hdr": any(t.startswith("/boxi/hdr/") for t in topics),
        "enable_alphasense": any(t.startswith("/boxi/alphasense/") for t in topics),
        "enable_pointcloud": any(t.startswith("/anymal/depth_camera/") for t in topics),
    }
    return paths, flags


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission-data", type=Path)
    parser.add_argument("--bag", type=Path, action="append", help="Explicit input, repeatable")
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--rviz-config", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Validate inputs and print launch command")
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")
    if args.bag:
        paths = args.bag
    else:
        if args.mission_data is None:
            from box_auto.utils import MISSION_DATA

            args.mission_data = Path(MISSION_DATA)
        paths = [path for pattern in PATTERNS for path in sorted(args.mission_data.glob(pattern))]
    paths, flags = prepare_inputs(paths)
    command = [
        "roslaunch",
        "box_auto",
        "box_overlay_verification.launch",
        "data_containing_rosbag_paths:=" + shlex.join([str(path) for path in paths]),
        "enable_viz:=" + str(not args.no_viz).lower(),
        "playback_rate:=" + str(args.rate),
    ]
    command.extend(name + ":=" + str(value).lower() for name, value in flags.items())
    if args.rviz_config:
        if not args.rviz_config.is_file():
            parser.error("RViz configuration does not exist")
        command.append("rviz_config:=" + str(args.rviz_config.resolve()))
    if args.dry_run:
        print(shlex.join(command))
        return
    with private_ros_session() as env:
        subprocess.run(command, env=env, check=True)


if __name__ == "__main__":
    main()
