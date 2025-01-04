#!/usr/bin/env python

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_msgs.msg
import csv
import tf.transformations as tf_trans


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


def convert_csv_to_rosbag(input_csv, output_bag, world, sensor):
    """Convert CSV file to ROS bag with pose and tf messages."""
    with rosbag.Bag(output_bag, "w") as bag:
        with open(input_csv, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:

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

                # Invert quaternion
                inv_quaternion = tf_trans.quaternion_conjugate(
                    [float(row[" q_WS_x"]), float(row[" q_WS_y"]), float(row[" q_WS_z"]), float(row[" q_WS_w"])]
                )

                transform_msg = create_transform_stamped(
                    ros_time,
                    -float(row[" p_WS_W_x"]),
                    -float(row[" p_WS_W_y"]),
                    -float(row[" p_WS_W_z"]),
                    inv_quaternion[0],
                    inv_quaternion[1],
                    inv_quaternion[2],
                    inv_quaternion[3],
                    sensor,
                    world,
                )

                # transform_msg = create_transform_stamped(
                #     ros_time,
                #     float(row[' p_WS_W_x']), float(row[' p_WS_W_y']), float(row[' p_WS_W_z']),
                #     float(row[' q_WS_x']), float(row[' q_WS_y']), float(row[' q_WS_z']), float(row[' q_WS_w']) , sensor, world
                # )

                # Create TF message
                tf_msg = create_tf_message(transform_msg)
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
        convert_csv_to_rosbag(args.input_csv, args.output_bag, args.world, args.sensor)
        print(f"Successfully converted {args.input_csv} to {args.output_bag}")
    except Exception as e:
        print(f"Error converting CSV to ROS bag: {str(e)}")
