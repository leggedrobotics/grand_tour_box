#!/usr/bin/env python3
"""Legacy filename for Open3D SLAM replay with ANYmal or DLIO pose priors.

This wrapper has always launched open3d_slam_ros. The colorless mapper itself is
launched with colorless_uniform_mapper/replay_colorlessMapper.launch.
"""

import argparse
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import rosbag
import numpy as np
from box_auto.utils import MISSION_DATA, get_bag, upload_bag
from box_auto.utils.atomic_file import atomic_output
from box_auto.utils.ros_session import private_ros_session
from box_auto.scripts.general.merge_mission import merge_bags_single


def validate_replay_inputs(inputs, pose_topic):
    """Match the shipped asynchronous Open3D replay consumer before launch."""
    cloud_topic = "/dlio/deskewed_point_cloud"
    found_cloud = found_pose = found_static = False
    for path in inputs:
        with rosbag.Bag(str(path)) as bag:
            for topic, message, _ in bag.read_messages(topics=[cloud_topic, pose_topic, "/tf_static"]):
                if topic == "/tf_static":
                    found_static |= bool(message.transforms)
                    continue
                if not message.header.frame_id or message.header.stamp.to_sec() <= 0:
                    raise ValueError("Replay input requires a frame and positive acquisition time: " + topic)
                if topic == cloud_topic:
                    if message._type != "sensor_msgs/PointCloud2":
                        raise ValueError("Replay cloud topic must contain PointCloud2")
                    # The shipped Open3D converter reads fields[3] and native
                    # FLOAT32 XYZ iterators. Reject unsupported layouts here:
                    # an XYZ-only cloud otherwise segfaults in rosToOpen3d.
                    fields = {field.name: field for field in message.fields}
                    if len(message.fields) < 4 or len(fields) != len(message.fields):
                        raise ValueError("Open3D replay requires the DLIO cloud layout (XYZ plus auxiliary fields)")
                    if any(
                        name not in fields
                        or fields[name].datatype != 7
                        or fields[name].count != 1
                        or fields[name].offset < 0
                        or fields[name].offset + 4 > message.point_step
                        for name in ("x", "y", "z")
                    ):
                        raise ValueError("Open3D replay requires valid scalar FLOAT32 XYZ fields")
                    if message.is_bigendian or message.row_step != message.width * message.point_step:
                        raise ValueError("Open3D replay requires little-endian, tightly packed cloud rows")
                    if message.width * message.height == 0 or len(message.data) != message.row_step * message.height:
                        raise ValueError("Open3D replay requires nonempty, complete point-cloud payloads")
                    dtype = np.dtype(
                        {
                            "names": ["x", "y", "z"],
                            "formats": ["<f4"] * 3,
                            "offsets": [fields[name].offset for name in ("x", "y", "z")],
                            "itemsize": message.point_step,
                        }
                    )
                    points = np.frombuffer(message.data, dtype=dtype)
                    if any(not np.isfinite(points[name]).all() for name in ("x", "y", "z")):
                        raise ValueError("Open3D replay requires finite XYZ coordinates")
                    found_cloud = True
                else:
                    if message._type not in ("geometry_msgs/PoseWithCovarianceStamped", "nav_msgs/Odometry"):
                        raise ValueError("Unsupported asynchronous pose type: " + message._type)
                    pose = message.pose.pose
                    values = (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    )
                    if (
                        not all(math.isfinite(value) for value in values)
                        or abs(sum(v * v for v in values[3:]) - 1) > 1e-3
                    ):
                        raise ValueError("Replay pose requires finite values and a unit quaternion")
                    found_pose = True
    if not (found_cloud and found_pose and found_static):
        raise ValueError("Replay requires nonempty cloud, supported pose-prior and static-transform topics")


def launch_nodes(mission_data=MISSION_DATA, pose_source="anymal", output=None, upload=False):
    directory = Path(mission_data)
    patterns = ["*_hesai_dlio.bag", "*_tf_static.bag"]
    if pose_source == "anymal":
        patterns.append("*_lpc_state_estimator.bag")
        prior_frame, pose_topic = "base", "/state_estimator/pose_in_odom"
    elif pose_source == "dlio":
        prior_frame, pose_topic = "hesai_lidar", "/dlio/lidar_map_odometry"
    else:
        raise ValueError("pose_source must be anymal or dlio")
    inputs = [Path(get_bag(pattern, directory=directory)) for pattern in patterns]
    validate_replay_inputs(inputs, pose_topic)
    timestamp = inputs[0].name.split("_")[0]
    destination = Path(output) if output else directory / (timestamp + "_open3d_slam.bag")
    if destination.exists() or destination.resolve() in [p.resolve() for p in inputs]:
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(prefix="open3d_replay_") as temporary:
        root = Path(temporary)
        merged = root / "input.bag"
        merge_bags_single([str(path) for path in inputs], str(merged))
        artifacts = root / "artifacts"
        artifacts.mkdir()
        with private_ros_session() as env:
            subprocess.run(
                [
                    "roslaunch",
                    "open3d_slam_ros",
                    "grandtour_replay.launch",
                    "rosbag_filepath:=" + str(merged),
                    "map_saving_folder:=" + str(artifacts),
                    "async_pose_topic:=" + pose_topic,
                    "assumed_external_odometry_tracked_frame:=" + prior_frame,
                ],
                env=env,
                check=True,
            )
        result = artifacts / "open3d_slam_replayed.bag"
        with rosbag.Bag(str(result)) as bag:
            for topic in ("/open3d_slam/optimized_poses", "/open3d_slam/point_cloud"):
                if bag.get_message_count(topic_filters=[topic]) == 0:
                    raise ValueError("Open3D replay produced no usable output on " + topic)
        with atomic_output(destination, overwrite=False) as pending:
            shutil.copyfile(result, pending)
    if upload:
        upload_bag(str(destination))
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission-data", type=Path, default=Path(MISSION_DATA))
    parser.add_argument("--pose-source", choices=("anymal", "dlio"), default="anymal")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--upload", action="store_true")
    args = parser.parse_args(argv)
    launch_nodes(args.mission_data, args.pose_source, args.output, args.upload)


if __name__ == "__main__":
    main()
