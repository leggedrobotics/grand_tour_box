#!/usr/bin/env python3
"""Convert NovAtel RAWIMUSX ASCII to an atomic ROS IMU bag."""
import argparse
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import genpy
import numpy as np
import pandas as pd
import rosbag
from box_auto.utils import MISSION_DATA
from boxi import ColorLogger

CPT7_TOPIC_DEFAULT = "/gt_box/cpt7/offline_from_novatel_logs/imu"


def configure_logger(debug: bool) -> logging.Logger:
    logger = ColorLogger.get_logger()
    logger.propagate = False

    # Deduplicate handlers (ColorLogger may add a handler per get_logger() call elsewhere)
    seen = set()
    for h in list(logger.handlers):
        key = (type(h), getattr(h, "stream", None))
        if key in seen:
            logger.removeHandler(h)
        else:
            seen.add(key)

    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    return logger


def as_utc_str_from_ros_time(t: genpy.Time) -> str:
    return datetime.fromtimestamp(t.to_sec(), tz=timezone.utc).strftime("%Y-%m-%d-%H-%M-%S")


def parse_count_series(series: pd.Series) -> np.ndarray:
    if series.dtype == object:
        vals = series.astype(str).to_numpy()
        out = np.empty(vals.shape[0], dtype=np.int64)
        for i, s in enumerate(vals):
            s = s.strip()
            if "*" in s:
                s = s.split("*", 1)[0]
            out[i] = int(s)
        return out
    return series.to_numpy(dtype=np.int64)


def find_single_rawimusx_ascii(root: Path) -> Path:
    files = [p for p in root.rglob("*RAWIMUSX.ASCII")]
    if len(files) == 0:
        raise FileNotFoundError(f"No RAWIMUSX.ASCII found in: {root}")
    if len(files) > 1:
        raise RuntimeError(f"Too many ({len(files)}) RAWIMUSX.ASCII files found in: {root}")
    return files[0]


class RAWIMUDataParser:
    def __init__(self, logger=None, output_imu_msg_name=CPT7_TOPIC_DEFAULT, utc_offset=-18.0):
        # Retain the legacy RAWIMUDataParser(topic) and keyword-topic API.
        if isinstance(logger, str):
            output_imu_msg_name, logger = logger, None
        self.logger = logger if logger is not None else configure_logger(False)

        # Keep the established GPS-to-UTC offset of -18 s. The experimental
        # +19 s setting shifts both header and bag timestamps forward by exactly
        # 37 s for identical input; it requires an independent UTC reference
        # before adoption as a dataset default.
        self.expected_utc_offset = float(utc_offset)
        if not np.isfinite(self.expected_utc_offset):
            raise ValueError("UTC offset must be finite")
        self.expected_gps_offset = 0.0
        self.weeks_to_seconds = 604_800.0

        posix_start = datetime(1970, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
        gps_start = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
        self.gps_epoch_start_in_POSIX_seconds = (gps_start - posix_start).total_seconds()

        self.x_accel = []
        self.minusy_accel = []
        self.z_accel = []
        self.x_gyro = []
        self.minusy_gyro = []
        self.z_gyro = []
        self.ros_times = []

        self.output_imu_msg_name = output_imu_msg_name
        self.reference_logfile_name = ""

    def process_imu_data(self, path: str) -> bool:
        self.logger.info(f"[convert] Reading RAWIMUSX ASCII: {path}")
        rawimu_df = self.fetch_dataframe_from_csv(path)
        if rawimu_df is None:
            self.logger.error(f"[convert] Could not process IMU file named: {path}")
            return False

        self.logger.info(f"[convert] Parsed CSV rows={len(rawimu_df)} cols={len(rawimu_df.columns)}")
        if not self.load_imu_times(rawimu_df):
            return False
        if not self.load_imu_data(rawimu_df):
            return False

        start_time = as_utc_str_from_ros_time(self.ros_times[0])
        end_time = as_utc_str_from_ros_time(self.ros_times[-1])
        self.logger.info(f"[convert] CPT7 RAWIMU time span (UTC): {start_time} -> {end_time}")

        self.reference_logfile_name = str(Path(path).stem)
        return True

    def load_imu_times(self, imu_df: pd.DataFrame) -> bool:
        if imu_df.empty:
            self.logger.error("RAWIMU file contains no samples")
            return False
        EXPECTED_IMU_FIELDNAME = "%RAWIMUSXA"
        if not (imu_df.iloc[:, 0] == EXPECTED_IMU_FIELDNAME).all():
            self.logger.error(
                "Unexpected IMU message type."
                f"\nExpected: {EXPECTED_IMU_FIELDNAME}"
                f"\nGot: {imu_df.iloc[0, 0]}"
            )
            return False

        WEEK_INDEX = 4
        SOW_INDEX = 5

        raw_week = np.asarray(imu_df.iloc[:, WEEK_INDEX], dtype=np.float64)
        sow = np.asarray(imu_df.iloc[:, SOW_INDEX], dtype=np.float64)
        if (not np.isfinite(raw_week).all() or not np.isfinite(sow).all()
                or np.any(raw_week < 0) or np.any(raw_week != np.floor(raw_week))
                or np.any(sow < 0) or np.any(sow >= self.weeks_to_seconds)):
            raise ValueError("Invalid GPS week or seconds-of-week in RAWIMU input")
        week = raw_week.astype(np.int64)
        sow = sow - float(self.expected_gps_offset) + float(self.expected_utc_offset)

        secs_sow = np.floor(sow).astype(np.int64)
        nsecs = np.round((sow - secs_sow) * 1e9).astype(np.int64)

        # snap to 1000 ns and normalize
        nsecs = (np.round(nsecs / 1000.0).astype(np.int64)) * 1000
        carry = nsecs // 1_000_000_000
        secs_sow = secs_sow + carry
        nsecs = nsecs % 1_000_000_000

        seconds_since_gps_start = week * int(self.weeks_to_seconds) + secs_sow
        seconds_unix = seconds_since_gps_start + int(self.gps_epoch_start_in_POSIX_seconds)

        self.logger.info(
            f"[convert] RAWIMU starts at {datetime.fromtimestamp(int(seconds_unix[0]), tz=timezone.utc)}"
        )
        self.ros_times = [genpy.Time(secs=int(s), nsecs=int(ns)) for s, ns in zip(seconds_unix, nsecs)]
        if any(b <= a for a, b in zip(self.ros_times, self.ros_times[1:])):
            raise ValueError("RAWIMU acquisition timestamps must be strictly increasing")
        return True

    def load_imu_data(self, imu_df: pd.DataFrame) -> bool:
        IMUTYPE_INDEX = 3
        if not (imu_df.iloc[:, IMUTYPE_INDEX] == 68).all():
            self.logger.error(f"IMU type {imu_df.iloc[0, IMUTYPE_INDEX]} doesn't match expected type 68")
            return False

        INDICES = {"Z Accel": -6, "-(Y Accel)": -5, "X Accel": -4, "Z Gyro": -3, "-Y Gyro": -2, "X Gyro": -1}
        CPT7RATE = 100.0
        GYROSCALEFACTOR = np.power(2.0, -33)
        ACCELSCALEFACTOR = np.power(2.0, -29)

        x_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["X Accel"]])
        my_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["-(Y Accel)"]])
        z_accel_cnt = parse_count_series(imu_df.iloc[:, INDICES["Z Accel"]])

        self.x_accel = x_accel_cnt * ACCELSCALEFACTOR * CPT7RATE
        self.minusy_accel = my_accel_cnt * ACCELSCALEFACTOR * CPT7RATE
        self.z_accel = z_accel_cnt * ACCELSCALEFACTOR * CPT7RATE

        x_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["X Gyro"]])
        my_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["-Y Gyro"]])
        z_gyro_cnt = parse_count_series(imu_df.iloc[:, INDICES["Z Gyro"]])

        self.x_gyro = x_gyro_cnt * GYROSCALEFACTOR * CPT7RATE
        self.minusy_gyro = my_gyro_cnt * GYROSCALEFACTOR * CPT7RATE
        self.z_gyro = z_gyro_cnt * GYROSCALEFACTOR * CPT7RATE

        self.logger.info(f"[convert] Loaded IMU samples: {len(self.x_accel)}")
        return True

    def write_to_rosbag(self, path: str) -> bool:
        processed_data = [
            self.ros_times,
            self.x_accel,
            self.minusy_accel,
            self.z_accel,
            self.x_gyro,
            self.minusy_gyro,
            self.z_gyro,
        ]
        lengths = [len(x) for x in processed_data]
        if len(set(lengths)) != 1 or not lengths[0]:
            self.logger.error(f"Not all timestamp and IMU data have the same lengths: {lengths}")
            return False

        from sensor_msgs.msg import Imu

        topic_name = self.output_imu_msg_name
        latest_message = Imu()
        latest_message.header.frame_id = "cpt7_imu"
        latest_message.header.seq = 0
        # RAWIMU supplies no orientation estimate. ROS uses -1 in this entry
        # to distinguish absent orientation from a zero-uncertainty estimate.
        latest_message.orientation_covariance[0] = -1.0
        if any(not np.isfinite(values).all() for values in processed_data[1:]):
            self.logger.error("RAWIMU contains nonfinite accelerometer or gyro values")
            return False

        try:
            from box_auto.utils.atomic_file import atomic_output
            with atomic_output(path) as temporary, rosbag.Bag(str(temporary), "w", compression="lz4") as bag:
                self.logger.info(f"[convert] Writing {lengths[0]} CPT7 samples to {path}")

                for stamp, x_a, minusy_a, z_a, x_g, minusy_g, z_g in zip(*processed_data):
                    latest_message.header.seq += 1
                    latest_message.header.stamp = stamp

                    latest_message.linear_acceleration.x = float(x_a)
                    latest_message.linear_acceleration.y = float(-minusy_a)
                    latest_message.linear_acceleration.z = float(z_a)

                    latest_message.angular_velocity.x = float(x_g)
                    latest_message.angular_velocity.y = float(-minusy_g)
                    latest_message.angular_velocity.z = float(z_g)

                    bag.write(topic=topic_name, msg=latest_message, t=stamp)

        except Exception as e:
            self.logger.error(f"[convert] Writing CPT7 IMU bag failed:\n{e}")
            return False

        self.logger.info(f"[convert] Wrote CPT7 IMU bag: {path}")
        return True

    def fetch_dataframe_from_csv(self, input_path: str) -> Optional[pd.DataFrame]:
        if not os.path.exists(input_path):
            self.logger.error(f"The path: {input_path} does not exist")
            return None
        try:
            # NovAtel ASCII may be headerless; pandas' default otherwise consumes
            # the first measurement as column names and silently loses a sample.
            with open(input_path, encoding="utf-8-sig") as stream:
                first_line = stream.readline().lstrip()
            return pd.read_csv(input_path, header=None if first_line.startswith("%RAWIMUSXA") else "infer")
        except Exception as e:
            self.logger.error(f"Failed to read CSV: {e}")
            return None


def convert_all_rawimu(root, logger, utc_offset=-18.0):
    """Legacy --all writes each ASCII's bag alongside it for move_cpt7_files.py."""
    inputs = sorted(Path(root).rglob("*RAWIMUSX.ASCII"))
    if not inputs:
        raise FileNotFoundError(f"No RAWIMUSX.ASCII files in {root}")
    outputs = [source.with_name(source.name.split(".")[0] + ".bag") for source in inputs]
    if len(set(outputs)) != len(outputs):
        raise ValueError("Multiple RAWIMU inputs resolve to the same output bag")
    for source, output in zip(inputs, outputs):
        converter = RAWIMUDataParser(logger, CPT7_TOPIC_DEFAULT, utc_offset=utc_offset)
        if not converter.process_imu_data(str(source)) or not converter.write_to_rosbag(str(output)):
            raise RuntimeError(f"RAWIMU conversion failed: {source}")
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--all", action="store_true", help="Convert every ASCII beside its source")
    parser.add_argument("--mission-data", type=Path, default=Path(MISSION_DATA))
    parser.add_argument("--output_bag", type=Path)
    parser.add_argument("--skip_verify", action="store_true", help="Compatibility flag: this command only converts")
    parser.add_argument("--utc-offset", type=float, default=-18.0, help="Seconds added to GPS time (default: -18)")
    args = parser.parse_args()
    if not -86400 < args.utc_offset < 86400:
        parser.error("--utc-offset must be finite and within one day")
    if args.all and args.output_bag:
        parser.error("--all cannot be combined with --output_bag")
    logger = configure_logger(debug=args.debug)
    root = args.mission_data
    if args.all:
        convert_all_rawimu(root, logger, args.utc_offset)
        raise SystemExit(0)
    try:
        source = find_single_rawimusx_ascii(root)
    except FileNotFoundError as error:
        logger.error(str(error))
        raise SystemExit(1)
    except RuntimeError as error:
        logger.error(str(error))
        raise SystemExit(2)
    output = args.output_bag
    if output is None:
        # Keep mission output naming while allowing ASCII-only conversion.
        candidates = sorted(root.glob("*_nuc_imu*.bag")) or sorted(root.glob("*_nuc_livox*.bag"))
        output = (root / (candidates[0].name.split("_")[0] + "_cpt7_raw_imu.bag")
                  if candidates else source.with_name(source.name.split(".")[0] + ".bag"))
    converter = RAWIMUDataParser(logger=logger, output_imu_msg_name=CPT7_TOPIC_DEFAULT, utc_offset=args.utc_offset)
    if not converter.process_imu_data(str(source)) or not converter.write_to_rosbag(str(output.expanduser())):
        raise SystemExit(3)
    raise SystemExit(0)


if __name__ == "__main__":
    main()
