#!/usr/bin/env python3

import os
from time import sleep
import shutil
import argparse
import math
import rosbag
from box_auto.utils import (
    get_bag,
    upload_bag,
    BOX_AUTO_SCRIPTS_DIR,
    MISSION_DATA,
    kill_roscore,
    start_roscore,
    run_ros_command,
    check_duplicate_timestamps,
)


def compute_gravity(lat_deg, altitude_m):
    """
    Compute the local gravitational acceleration using the International Gravity Formula 1980.

    Parameters:
        lat_deg (float): Latitude in degrees.
        altitude_m (float): Altitude in meters above sea level.

    Returns:
        float: Gravitational acceleration in m/s².
    """
    # Convert latitude from degrees to radians
    lat_rad = math.radians(lat_deg)

    # Calculate sine of latitude and sine of twice the latitude
    sin_lat = math.sin(lat_rad)
    sin2_lat = math.sin(2 * lat_rad)

    # International Gravity Formula 1980
    g = 9.780327 * (1 + 0.0053024 * sin_lat**2 - 0.0000058 * sin2_lat**2) - 3.086e-6 * altitude_m
    return g


def launch_nodes(
    patterns, output_bag_name, lidar_topic, imu_topic, tag, post_fix, rotational_offset, translation_offset
):

    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    inputs = []
    for pattern in patterns:
        f = get_bag(pattern)
        inputs.append(f)
        timestamp = str(f).split("/")[-1]
        timestamp = timestamp.split("_")[0]

    assert len(inputs) == len(patterns), "Failure did not find all the bags required"

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_dlio_prep.bag")

    # Check if merged_rosbag_path already exists
    if os.path.exists(merged_rosbag_path):
        print(f"Removing existing merged rosbag at {merged_rosbag_path}")
        os.system(f"rm {merged_rosbag_path}")

    print(f"Merging bags into {merged_rosbag_path}")
    os.system(f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}")

    kill_roscore()

    # Check gravity

    lat, lon, alt = None, None, None
    try:
        gps_pattern = "*_cpt7_ie_tc.bag"
        gps_bag_path = get_bag(gps_pattern)

        with rosbag.Bag(gps_bag_path, "r") as bag:
            for _, msg, _ in bag.read_messages(topics=["/gt_box/inertial_explorer/tc/navsatfix_origin"]):
                if msg._type == "sensor_msgs/NavSatFix":
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.altitude
                    # Just need the first message to get origin
                    break
        print(f"GPS Origin - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
    except Exception as e:
        print(f"Error reading GPS data: {e}")
        print("No GPS data will be used for gravity calculation.")

    if lat is not None:
        gravity = compute_gravity(lat, alt)
        print(f"Local gravity at GPS origin: {gravity} m/s²")
    else:
        print("No GPS data found in the bag.")
        gravity = 9.80665  # Use standard gravity as fallback
        print("Using standard gravity value: 9.80665 m/s²")
        print("WARNING: Without accurate GPS data, gravity calculations may be less precise.")
        # Alternatively, uncomment the line below to stop execution if GPS data is required
        # raise ValueError("No GPS data found. Cannot proceed without GPS origin information.")

    check_duplicate_timestamps(merged_rosbag_path, lidar_topic)

    start_roscore()
    sleep(1)
    run_ros_command(
        f"roslaunch direct_lidar_inertial_odometry dlio_replay.launch gravity:={gravity} abliation_rotation_offset:={rotational_offset} abliation_translation_offset:={translation_offset} input_rosbag_path:={merged_rosbag_path}  output_rosbag_folder_path:={MISSION_DATA} output_rosbag_name:={output_bag_name} imu_topic:={imu_topic} pointcloud_topic:={lidar_topic}",
        background=True,
    )
    sleep(5)
    run_ros_command(f"rosbag play -r 1 --clock {merged_rosbag_path}")
    print("Waiting for 10s before uploading!")
    sleep(10)
    print("Moving and uploading bag!")
    kill_roscore()

    output_bag_path = os.path.join(MISSION_DATA, f"{timestamp}_{tag}dlio{post_fix}.bag")
    shutil.move(f"{MISSION_DATA}/{output_bag_name}.bag", output_bag_path)

    check_duplicate_timestamps(output_bag_path, "/dlio/deskewed_point_cloud")

    # Clean up the merged bag file
    if os.path.exists(merged_rosbag_path):
        os.remove(merged_rosbag_path)
        print(f"Deleted merged bag file: {merged_rosbag_path}")

    upload_bag(output_bag_path)


def parse_arguments():
    parser = argparse.ArgumentParser(description="Configure LiDAR and IMU options.")
    parser.add_argument(
        "--lidar", choices=["hesai", "livox", "velodyne"], default="hesai", help="Select the LiDAR configuration (default: hesai)."
    )
    parser.add_argument(
        "--imu",
        choices=["stim320", "cpt7", "ap20", "adis", "alphasense", "livox", "zed2i"],
        default="cpt7",
        help="Select the IMU configuration (default: cpt7).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()

    # Resolve IMU information based on command line argument
    if args.imu == "stim320":
        imu_pattern = "*_jetson_stim.bag"
        imu_topic = "/boxi/stim320/imu"
    elif args.imu == "cpt7":
        imu_pattern = "*_cpt7_raw_imu.bag"
        imu_topic = "/gt_box/cpt7/offline_from_novatel_logs/imu"
    elif args.imu == "ap20":
        imu_pattern = "*_jetson_ap20_synced.bag"
        imu_topic = "/gt_box/ap20/imu"
    elif args.imu == "adis":
        imu_pattern = "*_jetson_adis.bag"
        imu_topic = "/gt_box/adis16475_node/imu"
    elif args.imu == "zed2i":
        imu_pattern = "*_jetson_zed2i_prop.bag"
        imu_topic = "/gt_box/zed2i/zed_node/imu/data"
    elif args.imu == "alphasense":
        imu_pattern = "*_alphasense_updated.bag"
        imu_topic = "/gt_box/alphasense_driver_node/imu"
    elif args.imu == "livox":
        imu_pattern = "*_livox.bag"
        imu_topic = "/gt_box/livox/imu_si_compliant"
    else:
        print(f"IMU not found: {args.imu}")
        exit(1)

    # Resolve LiDAR information prefer filtered pointclouds
    if args.lidar == "hesai":
        get_bag("*_nuc_hesai_ready.bag")
        patterns = [imu_pattern, "*_tf_static_start_end.bag", "*_nuc_hesai_ready.bag"]
        lidar_topic = "/gt_box/hesai/points"
        tag = "hesai_"
    elif args.lidar == "livox":
        get_bag("*_nuc_livox_ready.bag")
        patterns = [imu_pattern, "*_tf_static_start_end.bag", "*_nuc_livox_ready.bag"]
        lidar_topic = "/gt_box/livox/lidar"
        tag = "livox_"
    elif args.lidar == "velodyne":
        get_bag("*_npc_velodyne_ready.bag")
        patterns = [imu_pattern, "*_tf_minimal.bag", "*_npc_velodyne_ready.bag"]
        lidar_topic = "/anymal/velodyne/points"
        tag = "velodyne_"
    else:
        print(f"LiDAR not supported: {args.lidar}")
        exit(2)

    # Ablation study
    post_fix = ""
    rotational_offset = 0.0  # degree
    if not rotational_offset == 0.0:
        if rotational_offset < 0:
            rotational_offset_str = str(abs(rotational_offset)).replace(".", "_point_")
            post_fix = "_rot_minus_" + rotational_offset_str + "deg"
        else:
            rotational_offset_str = str(rotational_offset).replace(".", "_point_")
            post_fix = "_rot_" + rotational_offset_str + "deg"

    translation_offset = 0.0  # millimeter
    if not translation_offset == 0.0:
        if translation_offset < 0:
            post_fix = "_trans_minus_" + str(abs(translation_offset)) + "mm"
        else:
            post_fix = "_trans_" + str(translation_offset) + "mm"

    output_bag_name = f"dlio_replayed_{args.lidar}_{args.imu}"
    launch_nodes(
        patterns, output_bag_name, lidar_topic, imu_topic, tag, post_fix, rotational_offset, translation_offset
    )
    exit(0)
