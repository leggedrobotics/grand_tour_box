#!/usr/bin/env python3

import rospy
import os
from time import sleep
import psutil
from pathlib import Path
import copy
import rosbag
from tf.msg import tfMessage
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf_bag import BagTfTransformer
from rosbag import Bag
import matplotlib.pyplot as plt


MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")
WS = "/home/catkin_ws"
PRE = f"source /opt/ros/noetic/setup.bash; source {WS}/devel/setup.bash;"
PATTERNS = ["*_jetson_ap20_robot.bag", "*_cpt7_raw_imu.bag", "*_cpt7_gps_ie.bag", "*_tf_static.bag"]
OUT = "/out"


class BagTfTransformerWrapper:
    def __init__(self, bag):
        self.tf_listener = BagTfTransformer(bag)

    def waitForTransform(self, parent_frame, child_frame, time, duration):
        return self.tf_listener.waitForTransform(parent_frame, child_frame, time)

    def lookupTransform(self, parent_frame, child_frame, time, latest=False):
        if latest:
            time = self.tf_listener.getStartTime()
        try:
            return self.tf_listener.lookupTransform(parent_frame, child_frame, time)
        except Exception:
            return (None, None)


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
        n = len(files)
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory} - {n}")
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

    # GPS bag is optional
    for pattern in PATTERNS:
        f, suc = get_bag(MISSION_DATA, pattern)
        if suc:
            inputs.append(f)
            timestamp = str(f).split("/")[-1]
            timestamp = timestamp.split("_")[0]

        else:
            if pattern == "*_cpt7_gps_ie.bag":
                print("The GPS postprocessed Bag is optional!")
                continue
            else:
                raise ValueError("Pattern not found: ", pattern, " in Directory ", MISSION_DATA)

    os.makedirs(OUT, exist_ok=True)

    inputs = ",".join(inputs)
    merged_rosbag_path = os.path.join(MISSION_DATA, "merged.bag")

    if False:
        os.system(
            f"python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
        )
        os.system("bash -c '" + PRE + "roscore &' ")
        sleep(1)

        os.system(
            "bash -c '" + PRE + f"roslaunch atn_position3_fuser position3_fuser.launch  logging_dir_location:={OUT} &' "
        )
        print("Waiting 3s for graph_msf to startup")
        sleep(3)
        os.system("bash -c '" + PRE + f"rosbag play -r 4 --clock {merged_rosbag_path}' ")
        print("Waiting 3s for all messages to be consumed by graph_msf before starting optimization!")  #

        sleep(3)
        os.system(
            'rosservice call /graph_msf/trigger_offline_optimization "max_optimization_iterations: 100\nsave_covariance: true"'
        )

    # Prepare data for plotting
    tf_static_path, _ = get_bag(MISSION_DATA, "*_tf_static.bag")
    tf_static_gt_path = tf_static_path.replace("_tf_static.bag", "_tf_static_gt.bag")

    csv_file_world, gps_is_available = get_bag(OUT, "*_transform_enu_origin_to_world.csv")
    if gps_is_available:
        write_tf_static_bag(csv_file_world, tf_static_gt_path, "enu_origin", "world")

    tf_gt = tf_static_path.replace("_tf_static.bag", "_gt_tf.bag")
    pose_gt = tf_static_path.replace("_tf_static.bag", "_gt_pose.bag")
    csv_cov_file, _ = get_bag(OUT, "*_X_state_6D_pose_covariance.csv")
    csv_pose_file, _ = get_bag(OUT, "*_X_state_6D_pose.csv")
    convert_csv_to_bag(csv_cov_file, csv_pose_file, tf_gt, pose_gt, "world", "box_base")

    tf_static_path, _ = get_bag(MISSION_DATA, "*_tf_static.bag")
    tf_static_gt_path = tf_static_path.replace("_tf_static.bag", "_tf_static_gt.bag")

    pose_gt_path = pose_gt
    ap20_gps_bag = merged_rosbag_path

    ap20 = np.array([t.point for _, t, _ in Bag(ap20_gps_bag).read_messages(topics="/gt_box/ap20/prism_position")])
    pose_gt = np.array(
        [t.pose.pose for _, t, _ in Bag(pose_gt_path).read_messages(topics="/gt_box/ground_truth/pose_with_covariance")]
    )

    if gps_is_available:
        tf_transformer = BagTfTransformerWrapper(tf_static_gt_path)
        t_enu_origin_world, q_enu_origin_world = tf_transformer.lookupTransform("enu_origin", "world", 0, latest=True)
        del tf_transformer

        tf_transformer = BagTfTransformerWrapper(merged_rosbag_path)
        t_prism_enu_origin, q_prism_enu_origin = tf_transformer.lookupTransform("prism", "enu_origin", 0, latest=True)
        del tf_transformer

        # Read in the full paths
        gps_ie = np.array(
            [
                t.pose.pose
                for _, t, _ in Bag(ap20_gps_bag).read_messages(topics="/gt_box/inertial_explorer/gt_poses_novatel")
            ]
        )
        # Convert the paths to 4x4 numpy arrays
        gps_ie_path_arr = np.array([np.eye(4) for _ in range(len(gps_ie))])
        for i in range(len(gps_ie)):
            gps_ie_path_arr[i, :3, :3] = R.from_quat(
                [gps_ie[i].orientation.x, gps_ie[i].orientation.y, gps_ie[i].orientation.z, gps_ie[i].orientation.w]
            ).as_matrix()
            gps_ie_path_arr[i, :3, 3] = [gps_ie[i].position.x, gps_ie[i].position.y, gps_ie[i].position.z]

    ap20_path_arr = np.array([np.eye(4) for _ in range(len(ap20))])
    pose_gt_path_arr = np.array([np.eye(4) for _ in range(len(pose_gt))])

    for i in range(len(ap20)):
        ap20_path_arr[i, :3, 3] = [ap20[i].x, ap20[i].y, ap20[i].z]

    for i in range(len(pose_gt)):
        pose_gt_path_arr[i, :3, :3] = R.from_quat(
            [pose_gt[i].orientation.x, pose_gt[i].orientation.y, pose_gt[i].orientation.z, pose_gt[i].orientation.w]
        ).as_matrix()
        pose_gt_path_arr[i, :3, 3] = [pose_gt[i].position.x, pose_gt[i].position.y, pose_gt[i].position.z]

    # Transform the paths to the correct frames
    T_enu_origin_world = np.eye(4)
    T_enu_origin_world[:3, :3] = R.from_quat(q_enu_origin_world).as_matrix()
    T_enu_origin_world[:3, 3] = t_enu_origin_world

    T_prism_enu_origin = np.eye(4)
    T_prism_enu_origin[:3, :3] = R.from_quat(q_prism_enu_origin).as_matrix()
    T_prism_enu_origin[:3, 3] = t_prism_enu_origin

    colors = {
        "gray": "#cecece",
        "purple": "#a559aa",
        "teal": "#59a89c",
        "gold": "#f0c571",
        "red": "#e02b35",
        "dark_blue": "#082a54",
    }

    if gps_is_available:
        gps_ie_path_world = gps_ie_path_arr
    else:
        gps_ie_path_world = ap20_path_arr

    ap20_path_world = ap20_path_arr
    pose_gt_path_world = pose_gt_path_arr

    x_min = min(
        np.min(gps_ie_path_world[:, 0, 3]), np.min(ap20_path_world[:, 0, 3]), np.min(pose_gt_path_world[:, 0, 3])
    )
    x_max = max(
        np.max(gps_ie_path_world[:, 0, 3]), np.max(ap20_path_world[:, 0, 3]), np.max(pose_gt_path_world[:, 0, 3])
    )
    y_min = min(
        np.min(gps_ie_path_world[:, 1, 3]), np.min(ap20_path_world[:, 1, 3]), np.min(pose_gt_path_world[:, 1, 3])
    )
    y_max = max(
        np.max(gps_ie_path_world[:, 1, 3]), np.max(ap20_path_world[:, 1, 3]), np.max(pose_gt_path_world[:, 1, 3])
    )

    # Calculate aspect ratio based on data ranges
    x_range = x_max - x_min
    y_range = y_max - y_min
    aspect_ratio = y_range / x_range

    # Set base width and calculate height
    width = 10
    height = width * aspect_ratio

    # Create figure with computed size
    fig, ax = plt.subplots(figsize=(width, height))

    # Plot trajectories

    if gps_is_available:
        ax.plot(
            gps_ie_path_world[:, 0, 3],
            gps_ie_path_world[:, 1, 3],
            "-",
            color=colors["red"],
            label="GPS-IE",
            linewidth=2,
        )
    ax.plot(ap20_path_world[:, 0, 3], ap20_path_world[:, 1, 3], "-", color=colors["purple"], label="AP20", linewidth=2)
    ax.plot(
        pose_gt_path_world[:, 0, 3],
        pose_gt_path_world[:, 1, 3],
        "-",
        color=colors["dark_blue"],
        label="Ground Truth",
        linewidth=2,
    )

    # Style settings
    ax.set_xlabel("Distance in [m]", fontsize=12)
    ax.set_ylabel("Distance in [m]", fontsize=12)
    ax.set_title("Comparision of AP20 - GPS - GraphMSF", fontsize=14, pad=20)
    ax.legend(fontsize=10)

    # Add padding and round to nearest 10m for grid alignment
    padding = 10  # 10m padding
    x_min_grid = np.floor((x_min - padding) / 10.0) * 10
    x_max_grid = np.ceil((x_max + padding) / 10.0) * 10
    y_min_grid = np.floor((y_min - padding) / 10.0) * 10
    y_max_grid = np.ceil((y_max + padding) / 10.0) * 10

    # Set grid lines every 10 meters
    ax.set_xticks(np.arange(x_min_grid, x_max_grid + 1, 10))
    ax.set_yticks(np.arange(y_min_grid, y_max_grid + 1, 10))
    ax.grid(True, which="major", alpha=0.3)

    # Set equal aspect ratio for x and y
    ax.set_aspect("equal")

    # Add some padding to the limits
    padding = 0.05  # 5% padding
    x_padding = x_range * padding
    y_padding = y_range * padding
    ax.set_xlim(x_min - x_padding, x_max + x_padding)
    ax.set_ylim(y_min - y_padding, y_max + y_padding)

    # Style spines
    for spine in ax.spines.values():
        spine.set_color("gray")
        spine.set_linewidth(0.5)

    # Adjust layout
    plt.tight_layout()
    plt.savefig("/out/graph_msf_result.png")

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein upload --mission {uuid} {tf_gt} {pose_gt} {tf_static_gt_path}")
    else:
        print(f"Finished processing - Generated: {tf_gt}, {pose_gt}, {tf_static_gt_path} - and multiple csv files.")


def fetch_multiple_files_kleinkram(patterns):
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        paths = " ".join([f"'{p}'" for p in patterns])
        os.system(f"klein download --mission {uuid} --dest /mission_data {paths}")


if __name__ == "__main__":
    fetch_multiple_files_kleinkram(PATTERNS)
    try:
        launch_nodes()
    except rospy.ROSInterruptException:
        pass
