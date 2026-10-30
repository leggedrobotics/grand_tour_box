"""Run the Ceres camera–IMU workflow without modifying input bags or installed calibration."""
from datetime import datetime
from pathlib import Path
import os
import shlex
import subprocess
import tempfile
import warnings

import numpy as np
import rosbag
import yaml

IMU_FRAMES = {
    "stim320": "stim320_imu", "cpt7": "cpt7_imu", "zed2i": "zed2i_imu_link",
    "adis": "adis16475_imu", "alphasense": "imu_sensor_frame", "livox": "livox_imu", "ap20": "ap20_imu",
}


def topic_overrides(values):
    result = {}
    for value in values:
        name, separator, topic = value.partition("=")
        if not separator or not topic.startswith("/") or name in result:
            raise ValueError("Expected one --imu-topic NAME=/topic per IMU")
        result[name] = topic
    return result


def inspect_bags(paths):
    topics = {}
    for path in paths:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, info in bag.get_type_and_topic_info().topics.items():
                if topic in topics and topics[topic] != info.msg_type:
                    raise ValueError(f"Mixed message types for {topic}")
                topics[topic] = info.msg_type
    return topics


def select_imus(topics, allan_root, overrides=None):
    overrides = overrides or {}
    unknown = set(overrides) - set(IMU_FRAMES)
    if unknown:
        raise ValueError(f"Unknown IMU names: {sorted(unknown)}")
    selected = {}
    for name, frame in IMU_FRAMES.items():
        with (Path(allan_root) / name / "imu.yaml").open() as stream:
            topic = overrides.get(name, yaml.safe_load(stream)["rostopic"])
        if topics.get(topic) != "sensor_msgs/Imu":
            if name in overrides:
                raise ValueError(f"Explicit IMU topic {topic} is absent or is not sensor_msgs/Imu")
            warnings.warn(f"Skipping {name}: {topic} is not available as sensor_msgs/Imu", stacklevel=2)
            continue
        if topic in selected:
            raise ValueError(f"IMU topic {topic} assigned to multiple frames")
        selected[topic] = frame
    if not selected:
        raise ValueError("No configured sensor_msgs/Imu topics; supply corrected topics with --imu-topic NAME=TOPIC")
    return selected


def validate_transform(value):
    transform = np.asarray(value, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-6)):
        raise ValueError("Invalid camera–IMU rigid transform")
    return transform


def run_camera_imu_batch(folder, calibration, output, allan_root, overrides=None, dry_run=False, runner=None):
    paths = sorted(Path(folder).resolve().glob("*.bag"))
    if not paths:
        raise ValueError(f"No input bags in {folder}")
    topics = inspect_bags(paths)
    selected = select_imus(topics, allan_root, overrides)
    detection_paths = []
    image_paths = []
    for path in paths:
        bag_topics = inspect_bags([path])
        if "grand_tour_camera_detection_msgs/CameraDetections" in bag_topics.values():
            detection_paths.append(path)
        if "sensor_msgs/Image" in bag_topics.values():
            image_paths.append(path)
    if not detection_paths and not image_paths:
        raise ValueError("Camera–IMU calibration needs detection bags or decoded sensor_msgs/Image bags")
    output = Path(output).resolve()
    if output in paths:
        raise ValueError("Output must not replace an input bag")
    calibration = Path(calibration).resolve()
    execute = runner or (lambda command: subprocess.run(command, check=True))

    def solve(workspace, execute_commands):
        detections = detection_paths
        if not detections:
            detections = [workspace / "camera_detections.bag"]
            command = ["rosrun", "grand_tour_ceres_apps", "camera_imu_preprocess", "--bags",
                       *map(str, image_paths), "--output", str(detections[0])]
            print(shlex.join(command))
            if execute_commands:
                execute(command)
        merged = workspace / "imu.bag"
        if execute_commands:
            # Copy only selected IMU messages; preserve sensor headers and bag times.
            # Invalid/unindexed sources fail read-only instead of reindexing raw data.
            with rosbag.Bag(str(merged), "w") as target:
                for path in paths:
                    with rosbag.Bag(str(path), "r") as source:
                        for topic, message, stamp in source.read_messages(topics=list(selected)):
                            target.write(topic, message, stamp)
        else:
            print(f"Would copy IMU topics {list(selected)} into {merged}")
        result = {}
        for topic, frame in selected.items():
            individual = workspace / f"{frame}.yaml"
            command = ["rosrun", "grand_tour_ceres_apps", "camera_imu_offline_calibration",
                       "-c", str(calibration), "--camera_detection_bags", *map(str, detections),
                       "-b", str(merged), "-t", topic, "-o", str(individual)]
            print(shlex.join(command))
            if execute_commands:
                execute(command)
                with individual.open() as stream:
                    data = yaml.safe_load(stream)[topic]
                validate_transform(data["T_camerabundle_imu"])
                result[frame] = data
        return result

    if dry_run:
        solve(output.parent / "<temporary-camera-imu>", False)
        return None
    # Inputs and destination are separate. Failed stages and successful runs both
    # remove intermediate bags; publish the aggregate only after every solve passes.
    with tempfile.TemporaryDirectory(prefix=".camera-imu-", dir=output.parent) as temporary:
        workspace = Path(temporary)
        result = solve(workspace, True)
        with rosbag.Bag(str(paths[0]), "r") as source:
            stamp = datetime.fromtimestamp(source.get_start_time()).strftime("%Y-%m-%d-%H-%M-%S")
        staged = workspace / "imu_calibration.yaml"
        staged.write_text(f"#{stamp}\n" + yaml.safe_dump(result))
        os.replace(staged, output)
    return stamp
