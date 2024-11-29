import csv
import rospy
import rosbag  # For ROS1. Replace with `rosbag2_py` if using ROS2
import argparse
import tf.transformations as tft
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import tf2_msgs.msg
import geometry_msgs.msg


def write_tf_static_bag(output_bag_path, R, t, timestamp):
    """
    Write the computed SE(3) transformation as a static transform into a ROS bag.

    Parameters:
        output_bag_path (str): Path to the output bag file.
        R (np.ndarray): 3x3 rotation matrix.
        t (np.ndarray): 3x1 translation vector.
        timestamp (rospy.Time): The timestamp to use for the tf_static message.
    """
    # Create the transform message
    static_transform = tf2_msgs.msg.TFMessage()

    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = timestamp
    transform.header.frame_id = "lasertracker_frame"  # Adjust as needed
    transform.child_frame_id = "ap20_frame"  # Adjust as needed

    # Fill in the translation
    transform.transform.translation.x = t[0]
    transform.transform.translation.y = t[1]
    transform.transform.translation.z = t[2]

    # Convert the rotation matrix to a quaternion
    quaternion = tft.quaternion_from_matrix(np.vstack([np.hstack([R, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Add the transform to the TFMessage
    static_transform.transforms.append(transform)

    # Write to a ROS bag
    with rosbag.Bag(output_bag_path, "w") as bag:
        bag.write("/tf_static", static_transform, t=timestamp)


def read_lasertracker(lasertracker_csv):
    measurments = {}
    with open(lasertracker_csv, mode="r") as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=";")
        # Create the ROS bag
        keys = ["room_1", "room_2", "room_3", "room_4", "prism", "t-probe"]
        s = 0
        for row in csv_reader:
            # Extract data
            try:
                # Parse position and orientation data
                position = np.array([float(row["X"]), float(row["Y"]), float(row["Z"])])

                orientation = {
                    "rx": float(row["RX"]),
                    "ry": float(row["RY"]),
                    "rz": float(row["RZ"]),
                }

                quaternion = tft.quaternion_from_euler(orientation["rx"], orientation["ry"], orientation["rz"])
                measurments[keys[s]] = {
                    "position": position,
                    "q_xyzw": quaternion,
                }
                s = s + 1
            except Exception as e:
                rospy.logwarn(f"Failed to process row: {row}. Error: {e}")

    return measurments


def read_ap20(ap20_bag):
    timestamp = None
    # Read the input bag file
    with rosbag.Bag(ap20_bag) as input_bag:
        prism_positions = []

        # Iterate over the /gt_box/ap20/prism_position messages
        for topic, msg, t in input_bag.read_messages(topics=["/gt_box/ap20/prism_position"]):
            if timestamp is None:
                timestamp = msg.header.stamp

            prism_positions.append({"position": np.array([msg.point.x, msg.point.y, msg.point.z]), "time": t.to_sec()})

        # Variables to store processed positions
        threshold_distance = 1.0  # 1 meter
        keys = ["room_1", "room_2", "room_3", "room_4"]
        s = -1
        last_position = None
        measurments_raw = {}

        for measurement in prism_positions:
            position = measurement["position"]

            # Check if this is the start of a new point
            if last_position is None or np.linalg.norm(position - last_position) > threshold_distance:
                s += 1
                if s >= len(keys):
                    rospy.loginfo("All points processed.")
                    break

                measurments_raw[keys[s]] = []

            measurments_raw[keys[s]].append(position)
            last_position = position

        measurments = {}
        for k, v in measurments_raw.items():
            if len(v) < 20:
                print("Invalid")
                exit - 1
            measurments[k] = {"position": np.mean(v[10:-10], axis=0)}

    return measurments, timestamp


def compute_se3_transformation(source_points, target_points):
    """
    Computes the SE(3) transformation (rotation and translation) between two sets of points.

    Parameters:
        source_points (np.ndarray): Nx3 array of points in the source frame.
        target_points (np.ndarray): Nx3 array of corresponding points in the target frame.

    Returns:
        R (np.ndarray): 3x3 rotation matrix.
        t (np.ndarray): 3x1 translation vector.
    """
    assert source_points.shape == target_points.shape, "Source and target points must have the same shape"
    assert source_points.shape[1] == 3, "Points must be 3D"

    # Compute centroids
    source_centroid = np.mean(source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    # Center the points
    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid

    # Compute the cross-covariance matrix
    H = source_centered.T @ target_centered

    # Compute the Singular Value Decomposition (SVD)
    U, S, Vt = np.linalg.svd(H)

    # Compute the rotation matrix
    R = Vt.T @ U.T

    # Ensure a proper rotation (det(R) == 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute the translation vector
    t = target_centroid - R @ source_centroid

    return R, t


def visualize_and_compute_error(source_points, target_points, R, t):
    """
    Visualizes the points and computes the projection error after applying the SE(3) transformation.

    Parameters:
        source_points (np.ndarray): Nx3 array of source points.
        target_points (np.ndarray): Nx3 array of target points.
        R (np.ndarray): 3x3 rotation matrix.
        t (np.ndarray): 3x1 translation vector.
    """
    # Transform the source points
    transformed_points = (R @ source_points.T).T + t

    # Compute projection error
    errors = np.linalg.norm(target_points - transformed_points, axis=1)
    mean_error = np.mean(errors)

    # Visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(source_points[:, 0], source_points[:, 1], source_points[:, 2], c="red", label="Source Points")
    ax.scatter(target_points[:, 0], target_points[:, 1], target_points[:, 2], c="blue", label="Target Points")
    ax.scatter(
        transformed_points[:, 0],
        transformed_points[:, 1],
        transformed_points[:, 2],
        c="green",
        label="Transformed Points",
        marker=matplotlib.markers.CARETDOWNBASE,
    )

    for i in range(source_points.shape[0]):
        ax.plot(
            [target_points[i, 0], transformed_points[i, 0]],
            [target_points[i, 1], transformed_points[i, 1]],
            [target_points[i, 2], transformed_points[i, 2]],
            "k--",
            linewidth=0.8,
        )

    ax.set_title("3D Visualization of Points and Transform")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()

    print(f"Projection Error for each point: {errors}")
    print(f"Mean Projection Error: {mean_error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert CSV to PoseStamped ROS bag.")
    parser.add_argument(
        "--lasertracker_csv",
        default="/media/jonfrey/Data/deployment_day_16/laser_tracker/1_4ReferencePrismMeasurment1_Initial.csv",
        help="Path to the input CSV file",
    )
    parser.add_argument(
        "--ap20_bag",
        default="/media/jonfrey/Data/deployment_day_16/calibration/2024-11-25-10-57-07_prism_calibration/2024-11-25-10-57-07_jetson_ap20_synced.bag",
        help="AP20",
    )

    args = parser.parse_args()
    measurments_lasertracker = read_lasertracker(args.lasertracker_csv)
    measurments_ap20, timestamp = read_ap20(args.ap20_bag)

    # Extract the points into arrays
    source_points = np.array(
        [
            measurments_lasertracker["room_1"]["position"],
            measurments_lasertracker["room_2"]["position"],
            measurments_lasertracker["room_3"]["position"],
            measurments_lasertracker["room_4"]["position"],
        ]
    )
    target_points = np.array(
        [
            measurments_ap20["room_1"]["position"],
            measurments_ap20["room_2"]["position"],
            measurments_ap20["room_3"]["position"],
            measurments_ap20["room_4"]["position"],
        ]
    )
    # Compute transformation
    R, t = compute_se3_transformation(source_points, target_points)
    visualize_and_compute_error(source_points, target_points, R, t)
    write_tf_static_bag(args.ap20_bag.replace("_jetson_ap20_synced", "_lasertracker_tf_static"), R, t, timestamp)
