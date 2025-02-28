#!/usr/bin/env python3

import rospy
import os
from time import sleep
import copy
import rosbag
from tf.msg import tfMessage
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf_bag import BagTfTransformer
from rosbag import Bag
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion

from box_auto.utils import (
    get_bag,
    upload_bag,
    MISSION_DATA,
    BOX_AUTO_SCRIPTS_DIR,
    kill_roscore,
    get_file,
    start_roscore,
    run_ros_command,
    ARTIFACT_FOLDER,
)

PATTERNS = ["*_jetson_ap20_robot.bag", "*_cpt7_raw_imu.bag", "*_cpt7_ie_tc.bag", "*_tf_static.bag"]

"""
Exit Codes:

EXIT CODE 0: Successful Verification
EXIT CODE 1: Essential Bag for Conversion are Missing
"""


def is_gnss_in_bag(bag_file_path):
    def get_bag_start_time(bag_file_path):
        try:
            with rosbag.Bag(bag_file_path, "r") as bag:
                start_time = bag.get_start_time()
                print(f"The start time of the bag is: {start_time} seconds (Unix epoch time).")
                return start_time
        except rosbag.bag.ROSBagException as e:
            print(f"Error reading the rosbag file: {e}")
        except FileNotFoundError:
            print("The specified rosbag file was not found.")

    def check_topic_in_rosbag(bag_file_path, topic_name):
        try:
            # Open the bag file
            with rosbag.Bag(bag_file_path, "r") as bag:
                # Check if the topic is present
                topic_first_appearance = None
                for topic, _, t in bag.read_messages():
                    if topic == topic_name:
                        topic_first_appearance = t.to_sec()
                        break

                if topic_first_appearance is not None:
                    print(f"The topic '{topic_name}' first appears at {topic_first_appearance} seconds in the bag.")
                    return topic_first_appearance
                else:
                    print(f"The topic '{topic_name}' is not present in the bag.")
                    return None

        except rosbag.bag.ROSBagException as e:
            print(f"Error reading the rosbag file: {e}")
        except FileNotFoundError:
            print("The specified rosbag file was not found.")

    topic_name = "/gt_box/inertial_explorer/tc/odometry"
    # Check when the topic first appears in the bag
    topic_first_appearance = check_topic_in_rosbag(bag_file_path, topic_name)
    if topic_first_appearance is None:
        print("The topic is not present in the bag.")
        return False

    # Get the start time of the bag
    bag_start_time = get_bag_start_time(bag_file_path)
    # Check
    if bag_start_time is None or topic_first_appearance < bag_start_time:
        print("Error computing relative time.")
        return False

    # Compute relative time of first appearance of the topic
    relative_time = topic_first_appearance - bag_start_time
    print(f"The topic '{topic_name}' first appears at {relative_time} seconds after the start of the bag.")

    # If it appears within the first 10 seconds, return exit code 0
    if relative_time < 10:
        print("The topic appears within the first 10 seconds, hence the test is successful.")
        return True
    else:
        print("The topic does not appear within the first 10 seconds, hence the test is unsuccessful.")
        return False


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
) -> TransformStamped:
    """
    Create a ROS TransformStamped message.

    Args:
        timestamp (float): Timestamp of the transform
        x, y, z (float): Translation components
        qw, qx, qy, qz (float): Quaternion rotation components
        fixed_frame_id (str): Parent frame ID
        child_frame_id (str): Child frame ID

    Returns:
        TransformStamped: ROS transform message
    """
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


def csv_to_TransformStamped(input_file_path: str, fixed_frame_id: str, child_frame_id: str) -> TransformStamped:
    """
    Convert a CSV file to a TransformStamped message.

    Args:
        input_file_path (str): Path to the input CSV file
        fixed_frame_id (str): Parent frame ID
        child_frame_id (str): Child frame ID

    Returns:
        TransformStamped: Converted transform message
    """
    try:
        with open(input_file_path, "r") as file:
            # Skip header
            next(file)

            # Read first data line
            line = file.readline().strip()
            parts = line.split(",")

            if len(parts) < 11:
                print(f"Error: Insufficient data in line for {input_file_path}")
                return None

            timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw = parts

            return create_transform_stamped(
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
    except Exception as e:
        print(f"Error processing {input_file_path}: {e}")
        return None


def write_tf_static_bag(output_bag_path: str, transforms: list):
    """
    Write transforms to a ROS bag file.

    Args:
        output_bag_path (str): Path to output bag file
        transforms (list): List of TransformStamped messages to write
    """
    try:
        with rosbag.Bag(output_bag_path, "w") as bag:
            for tf_msg in transforms:
                if tf_msg is not None:
                    # inv_tf = invert_transform(tf_msg)
                    tfm = tfMessage([tf_msg])
                    bag.write("/tf_static", tfm, tf_msg.header.stamp)
                    tf_msg.header.stamp.secs += 1

                    bag.write("/tf_static", tfm, tf_msg.header.stamp)

        print(f"Conversion complete. Output saved to {output_bag_path}")
    except Exception as e:
        print(f"Error writing bag file: {e}")


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
    ) as tf_gt_bag, rosbag.Bag(pose_gt_path, "w", compression="lz4") as pose_gt_bag:

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

            # Create and write nav_msgs/Odometry message
            odometry = Odometry()
            odometry.header = pose_with_cov.header
            odometry.pose.pose = pose_with_cov.pose.pose
            odometry.child_frame_id = child_frame_id
            pose_gt_bag.write("/gt_box/ground_truth/odometry", odometry, stamp)

            # Create and populate the TransformStamped message
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

            # Invert the transform to allow multi-child 1 parent tree.
            transformed_data = invert_transform(transform)

            # Create a TFMessage and add the TransformStamped object
            tf_message = TFMessage(transforms=[transformed_data])

            # Write the TFMessage to the bag
            tf_gt_bag.write("/tf", tf_message, stamp)


def launch_nodes():
    kill_roscore()

    inputs = []

    # GPS bag is optional
    for pattern in PATTERNS:
        f, suc = get_file(pattern)
        if suc:
            inputs.append(f)
            timestamp = str(f).split("/")[-1]
            timestamp = timestamp.split("_")[0]

        else:
            if pattern == "*_cpt7_ie_tc.bag":
                print("The GPS postprocessed Bag is optional!")
                continue
            else:
                print("Pattern not found: ", pattern, " in Directory ", MISSION_DATA)
                exit(1)

    GRAPH_MSF_ARTIFACT_FOLDER = os.path.join(ARTIFACT_FOLDER, "graph_msf")
    os.makedirs(GRAPH_MSF_ARTIFACT_FOLDER, exist_ok=True)

    inputs = ",".join(inputs)
    merged_rosbag_path = os.path.join(MISSION_DATA, "merged_for_graph_msf.bag")

    if True:
        # Check if merged_rosbag_path already exists
        if os.path.exists(merged_rosbag_path):
            print(f"Using existing merged rosbag at {merged_rosbag_path}")
        else:
            print(f"Merging bags into {merged_rosbag_path}")
            os.system(
                f"python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={inputs} --output={merged_rosbag_path}"
            )
        start_roscore()
        sleep(1)

        initialize_using_gnss = "true" if is_gnss_in_bag(merged_rosbag_path) else "false"
        print(f"Evaluate if initialize_using_gnss returns: {initialize_using_gnss}")

        run_ros_command(
            f"roslaunch atn_position3_fuser position3_fuser_replay.launch  logging_dir_location:={GRAPH_MSF_ARTIFACT_FOLDER} initialize_using_gnss:={initialize_using_gnss}",
            background=True,
        )

        print("Waiting 3s for graph_msf to startup")
        sleep(3)
        run_ros_command(f"rosbag play -r 1 --clock {merged_rosbag_path}")

        print("Waiting 3s for all messages to be consumed by graph_msf before starting optimization!")  #
        sleep(3)
        run_ros_command(
            'rosservice call /graph_msf/trigger_offline_optimization "max_optimization_iterations: 1000\nsave_covariance: true"'
        )
        kill_roscore()

    print("GMSF processing finished. Converting the data.")

    # Convert tf_statics to debug bag
    tf_static_path = get_bag("*_tf_static.bag")
    tf_static_gt_path = tf_static_path.replace("_tf_static.bag", "_tf_static_gt.bag")
    tf_statics = []

    GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER = max(
        [
            os.path.join(GRAPH_MSF_ARTIFACT_FOLDER, d)
            for d in os.listdir(GRAPH_MSF_ARTIFACT_FOLDER)
            if os.path.isdir(os.path.join(GRAPH_MSF_ARTIFACT_FOLDER, d))
        ],
        key=os.path.getmtime,
    )

    print(GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER)
    csv_R_6D_transform_world_to_leica_total_station, suc1 = get_file(
        "R_6D_transform_world_to_leica_total_station.csv", GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER
    )
    csv_6D_transform_leica_total_station_to_prism, suc2 = get_file(
        "R_6D_transform_leica_total_station_to_leica_total_stationOld.csv", GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER
    )
    csv_file_world_to_enu_orign, gps_is_available = get_file(
        "R_6D_transform_world_to_enu_origin.csv", GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER
    )
    assert suc1 and suc2, "Totalstation has to be available!"
    R_world_to_helper = csv_to_TransformStamped(
        csv_R_6D_transform_world_to_leica_total_station, "world", "leica_total_station_helper"
    )

    R_helper_to_prism = csv_to_TransformStamped(
        csv_6D_transform_leica_total_station_to_prism, "leica_total_station_helper", "prism"
    )
    tf_statics.append(R_world_to_helper)
    tf_statics.append(R_helper_to_prism)
    if gps_is_available:
        R_world_to_enu_origin = csv_to_TransformStamped(csv_file_world_to_enu_orign, "world", "enu_origin")
        tf_statics.append(R_world_to_enu_origin)

    # else:
    #     R_world_to_enu_origin = create_transform_stamped(
    #         timestamp=R_world_to_helper.header.stamp.to_sec(),
    #         x=0,
    #         y=0,
    #         z=0,
    #         qw=1,
    #         qx=0,
    #         qy=0,
    #         qz=0,
    #         fixed_frame_id="world",
    #         child_frame_id="enu_origin",
    #     )
    #     gps_is_available = True

    write_tf_static_bag(tf_static_gt_path, tf_statics)  # Important this bag cannot be replayed given

    # Convert world to box_base to rosbag
    tf_gt_path = tf_static_path.replace("_tf_static.bag", "_gt_tf.bag")
    pose_gt_path = tf_static_path.replace("_tf_static.bag", "_gt_pose.bag")
    csv_cov_file, _ = get_file("X_state_6D_pose_covariance.csv", GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER)
    csv_pose_file, _ = get_file("X_state_6D_pose.csv", GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER)
    convert_csv_to_bag(csv_cov_file, csv_pose_file, tf_gt_path, pose_gt_path, "world", "box_base")

    # Load all data from rosbags and plot it
    ap20_gps_bag = merged_rosbag_path
    ap20 = np.array([t.point for _, t, _ in Bag(ap20_gps_bag).read_messages(topics="/gt_box/ap20/prism_position")])
    pose_gt = np.array(
        [t.pose.pose for _, t, _ in Bag(pose_gt_path).read_messages(topics="/gt_box/ground_truth/pose_with_covariance")]
    )

    tf_transformer = BagTfTransformer(tf_static_gt_path)
    t_prism_world, q_prism_world = tf_transformer.lookupTransform("prism", "world", 0, latest=True)

    if gps_is_available:
        t_enu_origin_world, q_enu_origin_world = tf_transformer.lookupTransform("enu_origin", "world", 0, latest=True)
        # Transform the paths to the correct frames
        T_enu_origin_world = np.eye(4)
        T_enu_origin_world[:3, :3] = R.from_quat(q_enu_origin_world).as_matrix()
        T_enu_origin_world[:3, 3] = np.array(t_enu_origin_world)

        # Read in the full paths
        gps_ie = np.array(
            [
                t.pose.pose
                for _, t, _ in Bag(ap20_gps_bag).read_messages(topics="/gt_box/inertial_explorer/tc/gt_poses_novatel")
            ]
        )

        gps_ie_path_arr = np.array([np.eye(4) for _ in range(len(gps_ie))])
        for i in range(len(gps_ie)):
            gps_ie_path_arr[i, :3, :3] = R.from_quat(
                [gps_ie[i].orientation.x, gps_ie[i].orientation.y, gps_ie[i].orientation.z, gps_ie[i].orientation.w]
            ).as_matrix()
            gps_ie_path_arr[i, :3, 3] = [gps_ie[i].position.x, gps_ie[i].position.y, gps_ie[i].position.z]

            gps_ie_path_world = T_enu_origin_world @ gps_ie_path_arr

    ap20_path_arr = np.array([np.eye(4) for _ in range(len(ap20))])
    pose_gt_path_arr = np.array([np.eye(4) for _ in range(len(pose_gt))])

    for i in range(len(ap20)):
        ap20_path_arr[i, :3, 3] = [ap20[i].x, ap20[i].y, ap20[i].z]

    for i in range(len(pose_gt)):
        pose_gt_path_arr[i, :3, :3] = R.from_quat(
            [pose_gt[i].orientation.x, pose_gt[i].orientation.y, pose_gt[i].orientation.z, pose_gt[i].orientation.w]
        ).as_matrix()
        pose_gt_path_arr[i, :3, 3] = [pose_gt[i].position.x, pose_gt[i].position.y, pose_gt[i].position.z]
    T_prism_world = np.eye(4)
    T_prism_world[:3, :3] = R.from_quat(q_prism_world).as_matrix()
    T_prism_world[:3, 3] = np.array(t_prism_world)

    colors = {
        "gray": "#cecece",
        "purple": "#a559aa",
        "teal": "#59a89c",
        "gold": "#f0c571",
        "red": "#e02b35",
        "dark_blue": "#082a54",
    }

    ap20_path_world = np.linalg.inv(T_prism_world) @ ap20_path_arr
    pose_gt_path_world = pose_gt_path_arr

    if not gps_is_available:
        gps_ie_path_world = np.copy(ap20_path_world)  # For debugging only

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

    # Instead of single plot command, we'll plot segments
    MAX_DISTANCE = 0.2  # 10cm threshold

    # Get x and y coordinates
    x_coords = ap20_path_world[:, 0, 3]
    y_coords = ap20_path_world[:, 1, 3]

    # Find segments with distances less than threshold
    for i in range(len(x_coords) - 1):
        dist = np.sqrt((x_coords[i + 1] - x_coords[i]) ** 2 + (y_coords[i + 1] - y_coords[i]) ** 2)
        if dist < MAX_DISTANCE:
            ax.plot(
                [x_coords[i], x_coords[i + 1]], [y_coords[i], y_coords[i + 1]], "-", color=colors["purple"], linewidth=2
            )

    # Add a single entry for the legend
    ax.plot([], [], "-", color=colors["purple"], label="AP20", linewidth=3)

    ax.plot(
        pose_gt_path_world[:, 0, 3],
        pose_gt_path_world[:, 1, 3],
        ":",
        color=colors["dark_blue"],
        label="Ground Truth",
        linewidth=3,
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
    plt.savefig(os.path.join(GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER, "graph_msf_result.png"))

    def plot_ground_truth_data(pose_gt_path, output_folder):
        """Plot ground truth pose and covariance data."""
        # Initialize lists to store data
        timestamps = []
        positions = {"x": [], "y": [], "z": []}
        position_covariances = []
        orientations = {"roll": [], "pitch": [], "yaw": []}
        orientation_covariances = []

        # Read the rosbag
        with rosbag.Bag(pose_gt_path, "r") as bag:
            for _, msg, t in bag.read_messages(topics=["/gt_box/ground_truth/pose_with_covariance"]):
                # Extract timestamp
                timestamps.append(t.to_sec())

                # Extract position
                positions["x"].append(msg.pose.pose.position.x)
                positions["y"].append(msg.pose.pose.position.y)
                positions["z"].append(msg.pose.pose.position.z)

                # Extract covariance
                cov = np.array(msg.pose.covariance).reshape(6, 6)
                position_covariances.append([cov[0, 0], cov[1, 1], cov[2, 2]])

                # Extract orientation
                quat = msg.pose.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                orientations["roll"].append(roll)
                orientations["pitch"].append(pitch)
                orientations["yaw"].append(yaw)

                # Extract orientation covariance
                orientation_covariances.append([cov[3, 3], cov[4, 4], cov[5, 5]])

        # Convert to numpy arrays
        timestamps = np.array(timestamps)
        positions = {key: np.array(values) for key, values in positions.items()}
        position_covariances = np.array(position_covariances)
        orientations = {key: np.array(values) for key, values in orientations.items()}
        orientation_covariances = np.array(orientation_covariances)

        # Create plots
        fig, axs = plt.subplots(4, 1, figsize=(10, 16))

        # Plot 1: Positions
        for key, color in zip(["x", "y", "z"], ["r", "g", "b"]):
            axs[0].plot(timestamps, positions[key], label=f"{key} position", color=color)
        axs[0].set_title("Position (x, y, z)")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (m)")
        axs[0].legend()

        # Plot 2: Position covariances
        for i, (key, color) in enumerate(zip(["x", "y", "z"], ["r", "g", "b"])):
            axs[1].plot(timestamps, position_covariances[:, i], label=f"{key} covariance", color=color)
        axs[1].set_title("Position Covariance (x, y, z)")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Covariance (m²)")
        axs[1].legend()

        # Plot 3: Orientations
        for key, color in zip(["roll", "pitch", "yaw"], ["r", "g", "b"]):
            axs[2].plot(timestamps, orientations[key], label=f"{key}", color=color)
        axs[2].set_title("Orientation (Roll, Pitch, Yaw)")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Angle (rad)")
        axs[2].legend()

        # Plot 4: Orientation covariances
        for i, (key, color) in enumerate(zip(["roll", "pitch", "yaw"], ["r", "g", "b"])):
            axs[3].plot(timestamps, orientation_covariances[:, i], label=f"{key} covariance", color=color)
        axs[3].set_title("Orientation Covariance (Roll, Pitch, Yaw)")
        axs[3].set_xlabel("Time (s)")
        axs[3].set_ylabel("Covariance (rad²)")
        axs[3].legend()

        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, "ground_truth_analysis.png"))
        plt.close()

    plot_ground_truth_data(pose_gt_path, GRAPH_MSF_ARTIFACT_FOLDER_SUBFOLDER)

    # Upload results
    upload_bag([tf_gt_path, pose_gt_path, tf_static_gt_path])


if __name__ == "__main__":
    launch_nodes()
    exit(0)
