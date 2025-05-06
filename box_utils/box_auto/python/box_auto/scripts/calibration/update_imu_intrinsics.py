#!/usr/bin/env python3

import rosbag
from tqdm import tqdm
from pathlib import Path
from box_auto.utils import get_bag, upload_bag, WS, BOX_AUTO_DIR
import yaml
import gspread
import rospy

CALIB_PATH = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration/allan_variances"
CONFIG = {
    "stim320": {
        "pattern": "_jetson_stim.bag",
        "topics": ["/gt_box/stim320/imu"],
        "out_pattern": "_jetson_stim_intrinsics.bag",
        "frequency": 500,
    },
    "ap20": {
        "pattern": "_jetson_ap20_synced.bag",
        "topics": ["/gt_box/ap20/imu"],
        "out_pattern": "_jetson_ap20_intrinsics.bag",
        "frequency": 100,
    },
    "livox": {
        "pattern": "_nuc_livox.bag",
        "topics": ["/gt_box/livox/imu_si_compliant"],
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
        "pattern": "_cpt7_raw_imu.bag",
        "topics": [
            "/gt_box/cpt7/offline_from_novatel_logs/imu",
        ],
        "out_pattern": "_cpt7_raw_intrinsics.bag",
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


def read_stim_time_offset(date_str):
    gc = gspread.service_account(
        filename=Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Open the Google Sheet
    sheet = gc.open_by_key("1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg")
    worksheet = sheet.worksheet("mission_overview")

    # Read data from A1 to Z500 (all data)
    data = worksheet.get_all_values("A1:Z500")

    # Find the row where the first column entry matches date_str
    target_row = None
    for _, row in enumerate(data):
        if row[0] == date_str:
            target_row = row
            break

    if target_row is None:
        raise ValueError(f"No row found with date_str: {date_str}")

    header_row = data[0]
    try:
        column_index = header_row.index("stim320-imu_mean")
    except ValueError:
        raise ValueError("Column 'stim320-imu_mean' not found in header row")

    return float(target_row[column_index])


def process_bags(calibrations) -> None:
    for k, v in CONFIG.items():
        calib = calibrations[k]

        accel_variance = calib["accelerometer_noise_density"] ** 2 * v["frequency"]
        gyro_variance = calib["gyroscope_noise_density"] ** 2 * v["frequency"]
        linear_acceleration_covariance = [accel_variance, 0.0, 0.0, 0.0, accel_variance, 0.0, 0.0, 0.0, accel_variance]

        # Angular velocity covariance
        angular_velocity_covariance = [gyro_variance, 0.0, 0.0, 0.0, gyro_variance, 0.0, 0.0, 0.0, gyro_variance]
        STIM320_TIME_OFFSET = None
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

                        if "stim320" in topic:
                            if STIM320_TIME_OFFSET is None:
                                mission_name = Path(input_bag_path).stem.replace(v["pattern"][0][:-4], "")
                                STIM320_TIME_OFFSET = read_stim_time_offset(mission_name)

                                STIM320_TIME_OFFSET = rospy.Duration.from_sec(STIM320_TIME_OFFSET / 1000.0)
                                print(f"STIM320_TIME_OFFSET: {STIM320_TIME_OFFSET}")
                            msg.header.stamp += STIM320_TIME_OFFSET

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
