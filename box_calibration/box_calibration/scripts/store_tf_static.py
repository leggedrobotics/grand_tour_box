#!/usr/bin/env python3
import os
import sys
import shutil
from datetime import datetime

import rospy
import rosbag
import yaml
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def main():
    rospy.init_node('store_tf_static')

    # Get parameters from the ROS launch or command-line
    bag_file = rospy.get_param('~bag_file', 'output.bag')
    validity_end_str = rospy.get_param('~validity_end', datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    validity_start_str = rospy.get_param('~validity_start', datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

    # Validate the timestamp format
    try:
        validity_end = datetime.strptime(validity_end_str, "%Y-%m-%d-%H-%M-%S").strftime("%Y-%m-%d-%H-%M-%S")
    except ValueError:
        rospy.logerr(f"Invalid validity_end format: {validity_end_str}. Expected format: YYYY-MM-DD-HH-MM-SS")
        sys.exit(1)

    rospy.loginfo(f"Storing /tf_static to bag file: {bag_file}")
    rospy.loginfo(f"Validity end timestamp: {validity_end}")

    # Get output folder and define secondary bag file name
    output_folder = os.path.dirname(os.path.abspath(bag_file))

    # Open the main bag file for writing
    bag = rosbag.Bag(bag_file, 'w')
    callback_called = [False]  # Use a mutable type to modify it inside the callback

    def callback(msg):
        if callback_called[0]:
            return

        # Write the /tf_static messages to the bag
        bag.write('/tf_static', msg)
        callback_called[0] = True

        # Define the fixed structure of expected files
        expected_files = {
            "alphasense": ["cam1.yaml", "cam2.yaml", "cam3.yaml", "cam4.yaml", "cam5.yaml"],
            "hdr": ["hdr_front.yaml", "hdr_left.yaml", "hdr_right.yaml"],
            "zed2i": ["zed_left.yaml", "zed_right.yaml"],
        }
        package_name = "box_calibration"

        try:
            # Resolve the package path
            import rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            package_path = os.path.join(package_path, "calibration")
        except rospkg.ResourceNotFound:
            rospy.logerr(f"Package '{package_name}' not found.")
            rospy.signal_shutdown(f"Package '{package_name}' not found.")
            bag.close()
            sys.exit(1)

        metadata_path = os.path.join(package_path, "tf", "calibration_latest.yaml")
        try:
            with open(metadata_path, "r") as f:
                extrinsic_calibration_data = yaml.safe_load(f)
                calibration_metadata = extrinsic_calibration_data["calibration_metadata"]
                valid_dates = [
                    datetime.strptime(value, "%Y-%m-%d-%H-%M-%S")
                    for value in calibration_metadata.values()
                    if value.lower() != "cad"
                ]
                earliest_date = min(valid_dates).strftime("%Y-%m-%d-%H-%M-%S")

                if validity_start_str == 0:
                    earliest_date = "2024-01-01-00-00-00"

                # Generate second bag file name
                second_bag_filename = f"tf_static_metadata_{earliest_date}__{validity_end}.bag"
                second_bag_path = os.path.join(output_folder, second_bag_filename)

                rospy.loginfo(f"Secondary bag will be: {second_bag_path}")

                # Store calibration metadata in main bag
                calibration_summary = {
                    "validity_start": earliest_date,
                    "validity_end": validity_end
                }
                calibration_summary.update(calibration_metadata)
                msg = String()
                msg.data = yaml.dump(calibration_summary)
                bag.write("/calibration_metadata", msg)

        except Exception as e:
            rospy.logerr(f"Failed to process calibration metadata: {e}")

        for folder, files in expected_files.items():
            folder_path = os.path.join(package_path, folder)
            for file_name in files:
                file_path = os.path.join(folder_path, file_name)
                if not os.path.exists(file_path):
                    rospy.logwarn(f"File not found: {file_path}. Skipping.")
                    continue

                try:
                    # Load YAML content
                    with open(file_path, 'r') as yaml_file:
                        yaml_data = yaml.safe_load(yaml_file)

                    # Ensure 'rostopic' field exists
                    if 'rostopic' in yaml_data:
                        topic = yaml_data['rostopic'] + "/calibration_data"
                    else:
                        rospy.logwarn(f"Skipping {file_path}: Missing 'rostopic' field.")
                        continue

                    # Read the raw YAML content to preserve formatting
                    with open(file_path, 'r') as yaml_file:
                        yaml_content = yaml_file.read()

                    # Write to ROS bag as std_msgs/String
                    msg = String()
                    msg.data = yaml_content
                    bag.write(topic, msg)
                    rospy.loginfo(f"Written: {file_path} to topic: {topic}")
                except Exception as e:
                    rospy.logerr(f"Failed to process {file_path}: {e}")

        # Close the original bag before copying
        bag.close()
        rospy.loginfo(f"Closed {bag_file}")

        # Copy the bag after closing
        try:
            rospy.loginfo(f"Copying {bag_file} to {second_bag_path}...")
            shutil.copy2(bag_file, second_bag_path)
            rospy.loginfo("Copy completed successfully.")

            # Reindex the copied bag if necessary
            os.system(f"rosbag reindex {second_bag_path}")

        except Exception as e:
            rospy.logerr(f"Failed to copy or reindex bag: {e}")

        rospy.signal_shutdown("Bag writing completed successfully.")
        sys.exit(0)

    # Wait for a single /tf_static message instead of spinning
    rospy.Subscriber('/tf_static', TFMessage, callback)
    rospy.wait_for_message('/tf_static', TFMessage)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
