import csv
from geometry_msgs.msg import PoseStamped
import rospy
import rosbag  # For ROS1. Replace with `rosbag2_py` if using ROS2
import argparse
from datetime import datetime


def parse_csv_to_posestamped(input_csv, output_rosbag):
    """
    Parses a CSV file and converts its data into PoseStamped messages saved in a ROS bag.
    :param input_csv: Path to the input CSV file.
    :param output_rosbag: Path to the output ROS bag file.
    """
    # Open the input CSV file
    with open(input_csv, mode="r") as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=";")
        # Create the ROS bag
        with rosbag.Bag(output_rosbag, "w") as bag:
            for row in csv_reader:
                # Extract data
                try:
                    # Parse position and orientation data
                    position = {
                        "x": float(row["X"]),
                        "y": float(row["Y"]),
                        "z": float(row["Z"]),
                    }
                    orientation = {
                        "rx": float(row["RX"]),
                        "ry": float(row["RY"]),
                        "rz": float(row["RZ"]),
                    }

                    # Parse timestamp
                    timestamp_str = row["Time Stamp"].strip("'")
                    timestamp = datetime.strptime(timestamp_str, "%d.%m.%Y %H:%M:%S.%f")

                    # # Create PoseStamped message
                    secs = int(timestamp.timestamp())  # Whole seconds since epoch
                    nsecs = timestamp.microsecond * 1000  # Convert microseconds to nanoseconds

                    # Create PoseStamped message
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp.secs = secs
                    pose_stamped.header.stamp.nsecs = nsecs

                    pose_stamped.header.frame_id = "lasertracker"  # Adjust as needed
                    pose_stamped.pose.position.x = position["x"]
                    pose_stamped.pose.position.y = position["y"]
                    pose_stamped.pose.position.z = position["z"]

                    # Placeholder for quaternion calculation from Euler angles (rx, ry, rz)
                    # Convert orientation (Euler angles) to quaternion
                    import tf.transformations as tft

                    quaternion = tft.quaternion_from_euler(orientation["rx"], orientation["ry"], orientation["rz"])
                    pose_stamped.pose.orientation.x = quaternion[0]
                    pose_stamped.pose.orientation.y = quaternion[1]
                    pose_stamped.pose.orientation.z = quaternion[2]
                    pose_stamped.pose.orientation.w = quaternion[3]

                    # Write the PoseStamped message to the bag
                    bag.write("/gt_box/lasertracker/pose_stamped", pose_stamped, pose_stamped.header.stamp)

                except Exception as e:
                    rospy.logwarn(f"Failed to process row: {row}. Error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert CSV to PoseStamped ROS bag.")
    parser.add_argument(
        "--input_csv",
        default="/media/jonfrey/Data/deployment_day_16/laser_tracker/9_DynamicMeasurementWithAnyMal2.csv",
        help="Path to the input CSV file",
    )
    parser.add_argument(
        "--output_rosbag",
        default="/media/jonfrey/Data/deployment_day_16/2024-11-25-11-56-26/2024-11-25-11-56-26_lasertracker_raw.bag",
        help="Path to the output ROS bag file",
    )
    args = parser.parse_args()

    parse_csv_to_posestamped(args.input_csv, args.output_rosbag)
