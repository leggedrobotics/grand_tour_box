import logging
import math
import os.path
from datetime import datetime

import numpy as np
import pandas as pd
import rosbag
import rospy
from boxi import ColorLogger
from sensor_msgs.msg import Imu

logger = ColorLogger.get_logger()
logger.setLevel(logging.INFO)


class RAWIMUData:
    def __init__(self):
        self.raw_imu = None
        self.times = None
        self.expected_utc_offset = -18.0  # UTC time = GPS System Time + UTC offset
        self.expected_gps_offset = 0.0
        self.weeks_to_seconds = 604_800.0
        posix_start_in_utc = datetime(year=1970, month=1, day=1, hour=0, minute=0, second=0)
        gps_start_in_utc = datetime(year=1980, month=1, day=6, hour=0, minute=0, second=0)
        self.gps_epoch_start_in_POSIX_seconds = (gps_start_in_utc - posix_start_in_utc).total_seconds()
        self.x_accel = list()
        self.minusy_accel = list()
        self.z_accel = list()
        self.x_gyro = list()
        self.minusy_gyro = list()
        self.z_gyro = list()
        self.ros_times = list()

    def load_imu_times(self, imu_df: pd.DataFrame):
        """
        According to Novatel the UTC logged times are converted from the
        CPT7 recorded times through:
        UTC time = GPS reference time - offset + UTC offset
        https://docs.novatel.com/OEM7/Content/Logs/TIME.html
        """
        EXPECTED_IMU_FIELDNAME = "%RAWIMUSXA"
        logger.debug(f"Checking RAWIMU fieldnames...")
        if not (imu_df.iloc[:, 0] == EXPECTED_IMU_FIELDNAME).all():
            logger.error(f"Unexpected IMU message type."
                         f"\nExpected: {EXPECTED_IMU_FIELDNAME}"
                         f"\nGot: {imu_df.iloc[0, 0]}")
        else:
            logger.debug(f"IMU field name {imu_df.iloc[0, 0]} is correct {ColorLogger.GREEN_CHECK}")
        WEEK_INDEX = 4
        SOW_INDEX = 5  # Seconds of week
        week = imu_df.iloc[:, WEEK_INDEX]
        seconds_of_week = imu_df.iloc[:, SOW_INDEX] - self.expected_gps_offset + self.expected_utc_offset
        seconds_since_gps_start_in_utc = week * self.weeks_to_seconds + seconds_of_week
        seconds_unix = seconds_since_gps_start_in_utc + self.gps_epoch_start_in_POSIX_seconds
        logger.info(f"Log time starts at {datetime.utcfromtimestamp(seconds_unix[0])}")
        self.ros_times = [rospy.Time(secs=int(x), nsecs=int(math.modf(x)[0] * 1e9)) for x in seconds_unix]

    def load_imu_data(self, imu_df: pd.DataFrame):
        """
        Expect HG4930_AN04 type = 68 (https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm#IMUType)
        Args:
            imu_df:

        Returns:

        """
        IMUTYPE_INDEX = 3
        if imu_df.iloc[0, IMUTYPE_INDEX] != 68:
            logger.error(f"IMU type {imu_df.iloc[0, IMUTYPE_INDEX]} doesn't match the expected type 68")
        else:
            logger.debug(f"IMU type is correct {ColorLogger.GREEN_CHECK}")
        INDICES = {"Z Accel": -6,
                   "-(Y Accel)": -5,
                   "X Accel": -4,
                   "Z Gyro": -3,
                   "-Y Gyro": -2,
                   "X Gyro": -1}
        CPT7RATE = 100.0
        """
        Values below come from: https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUSX.htm#RawIMUScaleFactors
        """

        GYROSCALEFACTOR = np.power(2.0, -33)  # rad/LSB
        ACCELSCALEFACTOR = np.power(2.0, -29)  # m/s/LSB

        """
        According to Novatel: https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMU.htm
        The change in velocity (acceleration) and angle (rotation rate) scale factors for each IMU type can be found in Table: Raw IMU Scale Factors. Multiply the appropriate scale factor by the count value for the velocity (field 5-7) and angle (field 8-10) increments.
        To obtain acceleration in m/s/s or rotation rate in rad/s, multiply the velocity/rotation increments by the output rate of the IMU:
        100 Hz for UIMU-HG1700-AG58, UIMU-HG1700-AG62, IMU-HG1900, OEM-HG1900, OEM-HG1930, OEM-IMU-HG4930, CPT7 and CPT7700
        125 Hz for IMU-IGM-S1, OEM-IMU-STIM300, OEM-IMU-EG320N, PwrPak7-E1, PwrPak7D-E1 and SMART7-S
        200 Hz for IMU-ISA-100C, IMU-FSAS, IMU-LN200, IMU-LN200C, IMU-µIMU-IC, OEM-IMU-µIMU-IC, IMU-KVH1750, IMU-P1750, IMU-IGM-A1, OEM-IMU-ADIS-16488, OEM-IMU-EG370N, OEM-IMU-EG320N_200Hz, PwrPak7-E2 and PwrPak7D-E2
        400 Hz for OEM-IMU-HG4930_400Hz, CPT7_400Hz, CPT7700_400Hz and OEM-IMU-µIMU-IC-UART
        The units of acceleration and rotation rate will depend on the IMU Scale Factors.
        This log is output in the IMU Body frame.
        """
        self.x_accel = imu_df.iloc[:, INDICES["X Accel"]] * ACCELSCALEFACTOR * CPT7RATE
        self.minusy_accel = imu_df.iloc[:, INDICES["-(Y Accel)"]] * ACCELSCALEFACTOR * CPT7RATE
        self.z_accel = imu_df.iloc[:, INDICES["Z Accel"]] * ACCELSCALEFACTOR * CPT7RATE

        x_gyro_data = imu_df.iloc[:, INDICES["X Gyro"]]
        x_gyro_data = np.array([int(x.split("*")[0]) for x in x_gyro_data])
        self.x_gyro = x_gyro_data * GYROSCALEFACTOR * CPT7RATE
        self.minusy_gyro = imu_df.iloc[:, INDICES["-Y Gyro"]] * GYROSCALEFACTOR * CPT7RATE
        self.z_gyro = imu_df.iloc[:, INDICES["Z Gyro"]] * GYROSCALEFACTOR * CPT7RATE
        average_z_accel = np.mean(self.z_accel)
        if self.check_floating_point_diff(average_z_accel, -9.81, tolerance=1e-1):
            logger.warning(f"Average Z-acceleration of {average_z_accel:.2f} is unexpected")
        else:
            logger.debug(f"Average Z-accel: {average_z_accel:.2f} is reasonable {ColorLogger.GREEN_CHECK}")

    def write_to_rosbag(self, path):
        logger.debug(f"Checking that output data lengths are consistent")
        processed_data = [self.ros_times,
                          self.x_accel, self.minusy_accel, self.z_accel,
                          self.x_gyro, self.minusy_gyro, self.z_gyro]
        lengths = [len(x) for x in processed_data]
        unique_lengths = set(lengths)
        if len(unique_lengths) > 1:
            logger.error(f"Not all timestamp and IMU data have the same lengths")
        else:
            logger.debug(f"All data lengths are consistent.")

        topic_name = "/gt_box/cpt7/offline_from_novatel_logs/imu"
        latest_message = Imu()
        latest_message.header.frame_id = "cpt7_imu"
        latest_message.header.seq = 0
        try:
            with rosbag.Bag(path, "w") as bag:
                logger.debug(f"Opened {path} for writing")
                logger.info(f"Writing {lengths[0]} samples")
                for stamp, x_a, minusy_a, z_a, x_g, minusy_g, z_g in zip(*processed_data):
                    latest_message.header.seq += 1
                    latest_message.header.stamp = stamp

                    latest_message.linear_acceleration.x = x_a
                    latest_message.linear_acceleration.y = -minusy_a
                    latest_message.linear_acceleration.z = z_a

                    latest_message.angular_velocity.x = x_g
                    latest_message.angular_velocity.y = -minusy_g
                    latest_message.angular_velocity.z = z_g

                    bag.write(topic=topic_name, msg=latest_message, t=stamp)
        except Exception as e:
            logger.error(f"Writing IMU data to rosbag failed with:"
                         f"\n {e}")
        logger.info(f"Done writing to:"
                    f"\n{path}")

    def check_floating_point_diff(self, a, b, tolerance=1e-8):
        return np.abs(a - b) > tolerance

    def check_time_message_consistency(self, times_df: pd.DataFrame):
        logger.debug(f"Checking that CPT7 recorded the expected time offsets...")
        UTC_OFFSET_INDEX = -8
        utc_offsets = times_df.iloc[:, UTC_OFFSET_INDEX]
        if self.check_floating_point_diff(utc_offsets.mean(), self.expected_utc_offset):
            logger.warning(f"{ColorLogger.YELLOW_WARNING} "
                           f"UTC offset of {utc_offsets.mean()} if different from expected.")
        else:
            logger.debug(f"UTC offset is reasonable {ColorLogger.GREEN_CHECK}")

        GPS_SYSTEM_OFFSET_INDEX = 10  # GPS System Time = GPS reference time - offset.
        gps_offset = times_df.iloc[:, GPS_SYSTEM_OFFSET_INDEX]
        if self.check_floating_point_diff(gps_offset.mean(), self.expected_gps_offset):
            logger.warning(f"{ColorLogger.YELLOW_WARNING} "
                           f"GPS system offset {gps_offset.mean()} is different from expected")
        else:
            logger.debug(f"GPS offset is reasonable {ColorLogger.GREEN_CHECK}")


def fetch_dataframe_from_csv(input_path):
    logger.debug(f"Processing {input_path}")
    df = None
    try:
        df = pd.read_csv(input_path)
    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
    except pd.errors.EmptyDataError as e:
        logger.error(f"Empty file: {e}")
    except pd.errors.ParserError as e:
        logger.error(f"Parsing error: {e}")
    except UnicodeDecodeError as e:
        logger.error(f"Encoding error: {e}")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")
    return df


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--imu_ascii_file", "-i", help="Path to the ascii csv file with RAWIMUSXA messages",
                        required=False)
    parser.add_argument("--time_ascii_file", "-t", help="Path to the ascii csv file with RAWIMUSXA messages",
                        required=False,
                        default="")
    parser.add_argument("--output", "-o", help="Output bag path", default="")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    if args.debug:
        logger.setLevel(logging.DEBUG)
    return parser


def main(args):
    imu_path = args.imu_ascii_file
    times_path = args.time_ascii_file
    rawimu_df = fetch_dataframe_from_csv(imu_path)
    if times_path != "":
        time_df = fetch_dataframe_from_csv(times_path)
    else:
        time_df = None
    imu_data = RAWIMUData()
    if time_df is not None:
        imu_data.check_time_message_consistency(times_df=time_df)
    imu_data.load_imu_times(imu_df=rawimu_df)
    imu_data.load_imu_data(rawimu_df)
    if args.output == "":
        basename = os.path.basename(imu_path).split(".")[0]
        output_path = os.path.join(os.path.dirname(imu_path), basename + ".bag")
    else:
        output_path = args.output
    imu_data.write_to_rosbag(output_path)
