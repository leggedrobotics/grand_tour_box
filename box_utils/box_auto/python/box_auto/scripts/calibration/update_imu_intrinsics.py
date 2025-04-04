#!/usr/bin/env python3

import rosbag
from tqdm import tqdm
from pathlib import Path
from box_auto.utils import get_bag, upload_bag, WS
import yaml

CALIB_PATH = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration/allan_variances"
CONFIG = {
    "ap20": {
        "pattern": "_jetson_ap20_synced.bag",
        "topics": ["/gt_box/ap20/imu"],
        "out_pattern": "_jetson_ap20_intrinsics.bag",
        "frequency": 100,
    },
    "stim320": {
        "pattern": "_jetson_stim.bag",
        "topics": ["/gt_box/stim320/imu"],
        "out_pattern": "_jetson_stim_intrinsics.bag",
        "frequency": 500,
    },
    "livox": {
        "pattern": "_nuc_livox.bag",
        "topics": ["/gt_box/livox/imu", "/gt_box/livox/imu_si_compliant"],
        "out_pattern": "_nuc_livox_intrinsics.bag",
        "frequency": 200,
    },
    "adis": {
        "pattern": "_jetson_adis.bag",
        "topics": ["/gt_box/adis16475_node/imu"],
        "out_pattern": "_jetson_adis_intrinsics.bag",
        "frequency": 200,
    },
    "cpt7": {
        "pattern": ["_nuc_cpt7_post_processed.bag", "_cpt7_raw_imu.bag"],
        "topics": [
            "/gt_box/cpt7/imu/data_raw",
            "/gt_box/cpt7/gps/imu",
            "/gt_box/cpt7/offline_from_novatel_logs/imu",
        ],
        "out_pattern": ["_nuc_cpt7_intrinsics.bag", "_cpt7_raw_intrinsics.bag"],
        "frequency": 100,
    },
    "alphasense": {
        "pattern": "_nuc_alphasense.bag",
        "topics": ["/gt_box/alphasense_driver_node/imu"],
        "out_pattern": "_nuc_alphasense_intrinsics.bag",
        "frequency": 200,
    },
    "zed2i": {
        "pattern": "_jetson_zed2i_prop.bag",
        "topics": ["/gt_box/zed2i/zed_node/imu/data"],
        "out_pattern": "_jetson_zed2i_prop_intrinsics.bag",
        "frequency": 400,
    },
}


def process_bags(calibrations) -> None:
    for k, v in CONFIG.items():
        calib = calibrations[k]

        accel_variance = calib["accelerometer_noise_density"] ** 2 * v["frequency"]
        gyro_variance = calib["gyroscope_noise_density"] ** 2 * v["frequency"]
        linear_acceleration_covariance = [accel_variance, 0.0, 0.0, 0.0, accel_variance, 0.0, 0.0, 0.0, accel_variance]

        # Angular velocity covariance
        angular_velocity_covariance = [gyro_variance, 0.0, 0.0, 0.0, gyro_variance, 0.0, 0.0, 0.0, gyro_variance]

        if not type(v["pattern"]) is list:
            v["pattern"] = [v["pattern"]]
            v["out_pattern"] = [v["out_pattern"]]

        for pattern, out_pattern in zip(v["pattern"], v["out_pattern"]):
            input_bag_path = get_bag("*" + pattern)
            output_bag_path = input_bag_path.replace(pattern, out_pattern)

            with rosbag.Bag(input_bag_path, "r") as input_bag, rosbag.Bag(
                output_bag_path, "w", compression="lz4"
            ) as output_bag:
                with tqdm(total=input_bag.get_message_count(), desc=f"Processing {k}") as pbar:
                    for topic, msg, t in input_bag.read_messages(topics=v["topics"]):
                        msg.linear_acceleration_covariance = linear_acceleration_covariance
                        msg.angular_velocity_covariance = angular_velocity_covariance
                        output_bag.write(topic, msg, t)
                        pbar.update(1)

            upload_bag(output_bag_path)


def load_calibrations():
    calibrations = {}
    for k, v in CONFIG.items():
        with open(CALIB_PATH / k / "imu.yaml", "r") as f:
            calibrations[k] = yaml.safe_load(f)
    return calibrations


if __name__ == "__main__":
    calibrations = load_calibrations()
    process_bags(calibrations)
    exit(0)
