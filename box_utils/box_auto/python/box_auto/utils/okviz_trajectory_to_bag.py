#!/usr/bin/env python

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_msgs.msg
import csv
import numpy as np
from geometry_msgs.msg import Transform, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def create_pose_stamped(timestamp, x, y, z, qx, qy, qz, qw, world):
    """Create a PoseStamped message from position and orientation data."""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = timestamp
    pose_msg.header.frame_id = world

    # Set position
    pose_msg.pose.position.x = float(x)
    pose_msg.pose.position.y = float(y)
    pose_msg.pose.position.z = float(z)

    # Set orientation
    pose_msg.pose.orientation.x = float(qx)
    pose_msg.pose.orientation.y = float(qy)
    pose_msg.pose.orientation.z = float(qz)
    pose_msg.pose.orientation.w = float(qw)

    return pose_msg


def create_transform_stamped(timestamp, x, y, z, qx, qy, qz, qw, world, sensor):
    """Create a TransformStamped message from position and orientation data."""
    transform_msg = TransformStamped()
    transform_msg.header.stamp = timestamp
    transform_msg.header.frame_id = world
    transform_msg.child_frame_id = sensor

    # Set translation
    transform_msg.transform.translation.x = float(x)
    transform_msg.transform.translation.y = float(y)
    transform_msg.transform.translation.z = float(z)

    # Set rotation
    transform_msg.transform.rotation.x = float(qx)
    transform_msg.transform.rotation.y = float(qy)
    transform_msg.transform.rotation.z = float(qz)
    transform_msg.transform.rotation.w = float(qw)

    return transform_msg


def create_tf_message(transform_stamped):
    """Create a TF message containing the transform."""
    tf_msg = tf2_msgs.msg.TFMessage()
    tf_msg.transforms.append(transform_stamped)
    return tf_msg


def invert_transform(transform_msg):
    """
    Inverts a ROS Transform or TransformStamped message.
    For TransformStamped messages, also swaps the frame_ids appropriately.

    Args:
        transform_msg: geometry_msgs/Transform or geometry_msgs/TransformStamped message

    Returns:
        The inverted transform message of the same type as the input
    """
    # Handle both Transform and TransformStamped messages
    if isinstance(transform_msg, TransformStamped):
        transform = transform_msg.transform
        is_stamped = True
    else:
        transform = transform_msg
        is_stamped = False

    # Extract the quaternion and translation
    quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]

    trans = [transform.translation.x, transform.translation.y, transform.translation.z]

    # Convert to 4x4 homogeneous transformation matrix
    mat = quaternion_matrix(quat)
    mat[0:3, 3] = trans

    # Invert the matrix
    inv_mat = np.linalg.inv(mat)

    # Extract the inverted rotation and translation
    inv_trans = inv_mat[0:3, 3]
    inv_quat = quaternion_from_matrix(inv_mat)

    # Create the output message
    if is_stamped:
        inv_transform_msg = TransformStamped()
        inv_transform_msg.header.stamp = transform_msg.header.stamp
        # Swap frame_ids for the inverse transform
        inv_transform_msg.header.frame_id = transform_msg.child_frame_id
        inv_transform_msg.child_frame_id = transform_msg.header.frame_id
        inv_transform = inv_transform_msg.transform
    else:
        inv_transform_msg = Transform()
        inv_transform = inv_transform_msg

    # Fill in the inverted values
    inv_transform.translation.x = inv_trans[0]
    inv_transform.translation.y = inv_trans[1]
    inv_transform.translation.z = inv_trans[2]

    inv_transform.rotation.x = inv_quat[0]
    inv_transform.rotation.y = inv_quat[1]
    inv_transform.rotation.z = inv_quat[2]
    inv_transform.rotation.w = inv_quat[3]

    return inv_transform_msg


def okviz_trajectory_to_bag(input_csv, output_bag, world, sensor):
    """Convert CSV file to ROS bag with pose and tf messages."""
    with rosbag.Bag(output_bag, "w") as bag:
        with open(input_csv, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                timestamp_sec = float(row["timestamp"]) / 1e9  # Convert ns to sec
                ros_time = rospy.Time(timestamp_sec)

                timestamp = int(row["timestamp"])
                seconds = timestamp // 10**9
                nanoseconds = timestamp % 10**9
                ros_time = rospy.Time(seconds, nanoseconds)

                # Create pose stamped message
                pose_msg = create_pose_stamped(
                    ros_time,
                    row[" p_WS_W_x"],
                    row[" p_WS_W_y"],
                    row[" p_WS_W_z"],
                    row[" q_WS_x"],
                    row[" q_WS_y"],
                    row[" q_WS_z"],
                    row[" q_WS_w"],
                    world,
                )

                transform_msg = create_transform_stamped(
                    ros_time,
                    float(row[" p_WS_W_x"]),
                    float(row[" p_WS_W_y"]),
                    float(row[" p_WS_W_z"]),
                    float(row[" q_WS_x"]),
                    float(row[" q_WS_y"]),
                    float(row[" q_WS_z"]),
                    float(row[" q_WS_w"]),
                    world,
                    sensor,
                )

                # Create TF message
                tf_msg = create_tf_message(invert_transform(transform_msg))
                bag.write("/gt_box/okviz/pose", pose_msg, ros_time)
                bag.write("/tf", tf_msg, ros_time)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert CSV file to ROS bag with pose and tf messages")
    parser.add_argument("--input_csv", help="Input CSV file path")
    parser.add_argument("--output_bag", help="Output ROS bag file path")
    parser.add_argument("--world", default="okviz_map", required=False)  # choices=["okviz_map", "okviz_odom"])
    parser.add_argument("--sensor", default="imu_sensor_frame", required=False)

    args = parser.parse_args()
    print(args.input_csv, args.output_bag, args.world, args.sensor)
    try:
        okviz_trajectory_to_bag(args.input_csv, args.output_bag, args.world, args.sensor)
        print(f"Successfully converted {args.input_csv} to {args.output_bag}")
    except Exception as e:
        print(f"Error converting CSV to ROS bag: {str(e)}")
