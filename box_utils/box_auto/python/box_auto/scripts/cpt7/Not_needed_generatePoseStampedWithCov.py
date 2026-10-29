#!/usr/bin/env python3
"""Add a pose view of existing odometry; current IE exporters already emit it.

Only the odometry covariance is in the pose's declared frame. Raw Euler standard
 deviations and per-fix ENU NavSat covariance cannot be substituted without the
 original attitude and origin transforms. Re-export the source IE file instead.
"""
import argparse
import copy
from pathlib import Path
import numpy as np
import rosbag
from geometry_msgs.msg import PoseWithCovarianceStamped
from box_auto.utils.atomic_file import atomic_output

ODOM_TOPIC = "/boxi/inertial_explorer/tc/odometry"
NEW_TOPIC = "/boxi/inertial_explorer/tc/posestampedWithCovariance"


def write_bag_with_posecov(input_bag_path, output_bag_path, odom_topic=ODOM_TOPIC, new_topic=NEW_TOPIC):
    if Path(input_bag_path).resolve() == Path(output_bag_path).resolve():
        raise ValueError("Input and output must differ")
    count = 0
    with rosbag.Bag(str(input_bag_path)) as source:
        topics = source.get_type_and_topic_info().topics
        if odom_topic not in topics or new_topic in topics:
            raise ValueError("Source odometry is missing or output topic already exists")
        with atomic_output(output_bag_path, overwrite=False) as temporary:
            with rosbag.Bag(str(temporary), "w", compression="lz4") as output:
                for topic, msg, time in source.read_messages():
                    output.write(topic, msg, time)
                    if topic != odom_topic:
                        continue
                    covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
                    if (
                        not np.isfinite(covariance).all()
                        or not np.any(covariance)
                        or not np.allclose(covariance, covariance.T, atol=1e-10)
                        or np.linalg.eigvalsh(covariance).min() < -1e-10
                    ):
                        raise ValueError("Source covariance is unknown or invalid; re-export IE source data")
                    if not msg.header.frame_id or msg.header.stamp.to_nsec() <= 0:
                        raise ValueError("Source odometry requires an explicit frame and acquisition timestamp")
                    pose = PoseWithCovarianceStamped(header=copy.deepcopy(msg.header), pose=copy.deepcopy(msg.pose))
                    output.write(new_topic, pose, time)
                    count += 1
                if not count:
                    raise ValueError("Source odometry is empty")
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--pose-topic", default=NEW_TOPIC)
    args = parser.parse_args(argv)
    write_bag_with_posecov(args.input, args.output, args.odom_topic, args.pose_topic)


if __name__ == "__main__":
    main()
