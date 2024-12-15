#!/usr/bin/env python3
import os
from datetime import datetime

import rospy
import rosbag
import yaml
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def main():
    rospy.init_node('store_tf_static')

    # Get the output bag file name from a parameter
    bag_file = rospy.get_param('~bag_file', 'output.bag')
    rospy.loginfo(f"Storing /tf_static to bag file: {bag_file}")

    # Open the bag file for writing
    with rosbag.Bag(bag_file, 'w') as bag:
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
            except rospkg.ResourceNotFound as e:
                print(f"Package '{package_name}' not found.")
                rospy.signal_shutdown(f"Package '{package_name}' not found.")
                return

            metadata_path = os.path.join(package_path, "tf", "calibration_latest.yaml")
            with open(metadata_path, "r") as f:
                extrinsic_calibration_data = yaml.safe_load(f)
                calibration_metadata = extrinsic_calibration_data["calibration_metadata"]
                valid_dates = [
                    datetime.strptime(value, "%Y-%m-%d-%H-%M-%S")
                    for value in calibration_metadata.values()
                    if value.lower() != "cad"
                ]
                earliest_date = min(valid_dates).strftime("%Y-%m-%d-%H-%M-%S")
                calibration_summary = {
                    "validity_start" : earliest_date,
                    "validity_end" : datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                }
                calibration_summary.update(calibration_metadata)
                msg = String()
                msg.data = yaml.dump(calibration_summary)
                bag.write("calibration_metadata", msg)

            for folder, files in expected_files.items():
                folder_path = os.path.join(package_path, folder)
                for file_name in files:
                    file_path = os.path.join(folder_path, file_name)
                    if not os.path.exists(file_path):
                        print(f"File not found: {file_path}. Skipping.")
                        continue

                    try:
                        # Load YAML content
                        with open(file_path, 'r') as yaml_file:
                            yaml_data = yaml.safe_load(yaml_file)

                        # Ensure 'rostopic' field exists
                        if 'rostopic' in yaml_data:
                            topic = yaml_data['rostopic'] + "/calibration_data"
                        else:
                            print(f"Skipping {file_path}: Missing 'rostopic' field.")
                            continue

                        # Read the raw YAML content to preserve formatting
                        with open(file_path, 'r') as yaml_file:
                            yaml_content = yaml_file.read()

                        # Write to ROS bag as std_msgs/String
                        msg = String()
                        msg.data = yaml_content
                        bag.write(topic, msg)
                        print(f"Written: {file_path} to topic: {topic}")
                    except Exception as e:
                        print(f"Failed to process {file_path}: {e}")

            rospy.signal_shutdown("Callback executed successfully.")

        rospy.Subscriber('/tf_static', TFMessage, callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
