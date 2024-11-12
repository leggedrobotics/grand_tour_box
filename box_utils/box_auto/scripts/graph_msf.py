#!/usr/bin/env python3

import rospy
import os
from time import sleep
import psutil
from pathlib import Path
import shutil

# General Packages
from scipy.spatial.transform import Rotation as R
import copy
import rosbag
from tf.msg import tfMessage
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped


MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")
WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"
PATTERNS = ["*_jetson_ap20_synced.bag", "*_cpt7_raw_imu.bag", "*_cpt7_gps_ie.bag", "*_tf_static_start_end.bag"]
OUT = MISSION_DATA  # "/out"


def invert_transform(tf_msg):
    # Invert the orientation and translation of a transform
    q_old = [
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w,
    ]
    r_old = R.from_quat(q_old)
    r_new = r_old.inv()
    q_new = r_new.as_quat()

    t_old = [
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z,
    ]
    t_new = -r_new.apply(t_old)

    tf_msg_new = TransformStamped()
    tf_msg_new.header = copy.deepcopy(tf_msg.header)
    tf_msg_new.header.frame_id = tf_msg.child_frame_id
    tf_msg_new.child_frame_id = tf_msg.header.frame_id

    tf_msg_new.transform.translation.x = t_new[0]
    tf_msg_new.transform.translation.y = t_new[1]
    tf_msg_new.transform.translation.z = t_new[2]
    tf_msg_new.transform.rotation.x = q_new[0]
    tf_msg_new.transform.rotation.y = q_new[1]
    tf_msg_new.transform.rotation.z = q_new[2]
    tf_msg_new.transform.rotation.w = q_new[3]

    return tf_msg_new


def create_transform_stamped(
    timestamp: float,
    x: float,
    y: float,
    z: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    fixed_frame_id: str,
    child_frame_id: str,
):
    # Create a TransformStamped message
    t = TransformStamped()
    t.header.stamp = rospy.Time.from_sec(float(timestamp))
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x = float(qx)
    t.transform.rotation.y = float(qy)
    t.transform.rotation.z = float(qz)
    t.transform.rotation.w = float(qw)
    t.header.frame_id = fixed_frame_id
    t.child_frame_id = child_frame_id
    return t


def write_tf_static_bag(input_file_path, output_bag_path, fixed_frame_id, child_frame_id):
    # Write the transforms to a bag file
    first_line = True
    with open(input_file_path, "r") as file, rosbag.Bag(output_bag_path, "w") as bag:
        for line in file:
            if first_line:
                first_line = False
                continue
            parts = line.strip().split(",")
            if len(parts) < 11:
                continue  # Skip malformed lines
            timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw = parts
            tf_msg = create_transform_stamped(
                timestamp=timestamp,
                x=x,
                y=y,
                z=z,
                qw=qw,
                qx=qx,
                qy=qy,
                qz=qz,
                fixed_frame_id=fixed_frame_id,
                child_frame_id=child_frame_id,
            )
            # Invert the transform
            inv_tf = invert_transform(tf_msg)
            tfm = tfMessage([inv_tf])
            bag.write("/tf", tfm, tf_msg.header.stamp)

    print(f"Conversion complete. Output saved to {output_bag_path}")


def convert_csv_to_bag(
    input_file_cov_path, input_file_pose_path, tf_gt_path, pose_gt_path, fixed_frame_id, child_frame_id
):
    """
    Convert pose and covariance CSV files to a ROS bag file.

    Args:
        input_file_cov_path (str): Path to the covariance CSV file
        input_file_pose_path (str): Path to the pose CSV file
        output_bag_path (str): Path where the output bag file will be saved
        fixed_frame_id (str): The fixed frame ID for the transform
        child_frame_id (str): The child frame ID for the transform
    """
    first_line_cov = True
    first_line_pose = True

    with open(input_file_cov_path, "r") as cov_file, open(input_file_pose_path, "r") as pose_file, rosbag.Bag(
        tf_gt_path, "w"
    ) as tf_gt_bag, rosbag.Bag(pose_gt_path, "w") as pose_gt_bag:

        # Use zip to process both files simultaneously
        for cov_line, pose_line in zip(cov_file, pose_file):
            # Skip headers
            if first_line_cov and first_line_pose:
                first_line_cov = False
                first_line_pose = False
                continue

            # Process covariance data
            cov_parts = cov_line.strip().split(",")
            if len(cov_parts) < 37:
                continue  # Skip malformed covariance lines

            # Process pose data
            pose_parts = pose_line.strip().split(",")
            if len(pose_parts) < 11:
                continue  # Skip malformed pose lines

            # Extract timestamps and verify they match
            timestamp_cov = float(cov_parts[0])
            timestamp_pose = float(pose_parts[0])

            if abs(timestamp_cov - timestamp_pose) > 1e-6:  # Allow small floating-point differences
                print(f"Warning: Timestamps don't match! Cov: {timestamp_cov}, Pose: {timestamp_pose}")
                continue

            # Create covariance matrix
            cov = np.array(
                [
                    [float(cov_parts[i]) for i in range(1, 7)],
                    [float(cov_parts[i]) for i in range(7, 13)],
                    [float(cov_parts[i]) for i in range(13, 19)],
                    [float(cov_parts[i]) for i in range(19, 25)],
                    [float(cov_parts[i]) for i in range(25, 31)],
                    [float(cov_parts[i]) for i in range(31, 37)],
                ]
            )

            # Extract pose data
            x, y, z = map(float, pose_parts[1:4])
            qx, qy, qz, qw = map(float, pose_parts[4:8])

            # Create ROS timestamp
            stamp = rospy.Time.from_sec(timestamp_pose)

            # Create PoseWithCovarianceStamped message
            pose_with_cov = PoseWithCovarianceStamped()
            pose_with_cov.header.stamp = stamp
            pose_with_cov.header.frame_id = fixed_frame_id

            # Set position and orientation
            pose_with_cov.pose.pose.position.x = x
            pose_with_cov.pose.pose.position.y = y
            pose_with_cov.pose.pose.position.z = z
            pose_with_cov.pose.pose.orientation.x = qx
            pose_with_cov.pose.pose.orientation.y = qy
            pose_with_cov.pose.pose.orientation.z = qz
            pose_with_cov.pose.pose.orientation.w = qw

            # Set covariance matrix
            pose_with_cov.pose.covariance = cov.flatten().tolist()

            # Write PoseWithCovarianceStamped to the bag
            pose_gt_bag.write("/gt_box/ground_truth/pose_with_covariance", pose_with_cov, stamp)

            # Create and write PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header = pose_with_cov.header
            pose_stamped.pose = pose_with_cov.pose.pose
            pose_gt_bag.write("/gt_box/ground_truth/pose_stamped", pose_stamped, stamp)

            # Create and write TransformStamped message
            transform = TransformStamped()
            transform.header = pose_with_cov.header
            transform.child_frame_id = child_frame_id

            # Use the actual pose data for the transform
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            tf_gt_bag.write("/tf", invert_transform(transform), stamp)


def get_bag(directory, pattern):
    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False

    return files[0], True


def kill_rosmaster():
    # Find all processes with name 'rosmaster'
    for proc in psutil.process_iter(["pid", "name"]):
        if "rosmaster" in proc.info["name"]:
            try:
                # Terminate the process
                print(f"Terminating rosmaster with PID {proc.info['pid']}")
                proc.terminate()
                proc.wait(timeout=60)  # Wait up to 60 seconds for the process to terminate
                print("rosmaster process terminated successfully.")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired) as e:
                print(f"Failed to terminate rosmaster process: {e}")


def launch_nodes():
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
    kill_rosmaster()

    inputs = []

    for pattern in PATTERNS:
        f, suc = get_bag(MISSION_DATA, pattern)
        if suc:
            inputs.append(f)
            timestamp = str(f).split("/")[-1]
            timestamp = timestamp.split("_")[0]

        else:
            raise ValueError("Pattern not found: ", pattern, " in Directory")

    assert len(inputs) == len(PATTERNS), "Failure did not find all the bags required"

    os.makedirs("/out", exist_ok=True)

    inputs = ",".join(inputs)

    merged_rosbag_path = os.path.join(MISSION_DATA, "merged.bag")
    os.system(
        f"python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
    )

    os.system("bash -c '" + PRE + "roscore&' ")
    sleep(1)

    os.system(
        "bash -c '" + PRE + f"roslaunch atn_position3_fuser position3_fuser.launch  logging_dir_location:={OUT} &' "
    )
    sleep(5)
    os.system("bash -c '" + PRE + f"rosbag play -r 1 --clock {merged_rosbag_path}' ")
    print("Waiting 10s before starting optimization!")
    sleep(10)
    os.system('rosservice call /graph_msf/trigger_offline_optimization "max_optimization_iterations: 1000"')

    tf_static_path, _ = get_bag(MISSION_DATA, "*_tf_static_start_end.bag")
    gt_tf_static_path = tf_static_path.replace("_tf_static_start_end.bag", "_gt_tf_static.bag")
    csv_file_world, _ = get_bag(OUT, "*_transform_enu_origin_to_world.csv")

    write_tf_static_bag(csv_file_world, gt_tf_static_path, "enu_origin", "world")

    tf_gt = tf_static_path.replace("_tf_static_start_end.bag", "_gt_tf.bag")
    pose_gt = tf_static_path.replace("_tf_static_start_end.bag", "_gt_pose.bag")
    csv_cov_file, _ = get_bag(OUT, "*_X_state_6D_pose_covariance.csv")
    csv_pose_file, _ = get_bag(OUT, "*_X_state_6D_pose.csv")
    convert_csv_to_bag(csv_cov_file, csv_pose_file, tf_gt, pose_gt, "world", "box_base")

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(
            f"klein mission upload --mission-uuid {uuid} --path {tf_gt} --path {pose_gt} --path {gt_tf_static_path}"
        )
    else:
        print(f"Finished processing - Generated: {tf_gt}, {pose_gt}, {gt_tf_static_path} - and multiple csv files.")


def fetch_multiple_files_kleinkram(patterns):
    tmp_dir = os.path.join(MISSION_DATA, "tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    for pattern in patterns:
        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein mission download --mission-uuid {uuid} --local-path {tmp_dir} --pattern {pattern}")

            # Move all files from /mission_data/tmp to /mission_data/
            for file_name in os.listdir(tmp_dir):
                source_file = os.path.join(tmp_dir, file_name)
                destination_file = os.path.join("/mission_data", file_name)
                shutil.move(source_file, destination_file)


if __name__ == "__main__":
    fetch_multiple_files_kleinkram(PATTERNS)
    try:
        launch_nodes()
    except rospy.ROSInterruptException:
        pass
