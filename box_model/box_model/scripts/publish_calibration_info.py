#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import String


def publish_calibration_metadata():
    # Initialize the ROS node
    rospy.init_node("calibration_metadata_publisher", anonymous=True)

    # Get the calibration file path from the parameter server
    calibration_file = rospy.get_param("~calibration_file")

    # Publisher for calibration metadata
    pub = rospy.Publisher("/gt_box/calibration_metadata", String, queue_size=10)

    # Rate object to control publishing frequency
    rate = rospy.Rate(1)  # 1 Hz (once per second)

    while not rospy.is_shutdown():
        try:
            # Read the YAML file
            with open(calibration_file, "r") as file:
                calibration_data = yaml.safe_load(file)

            # Convert the metadata to a string
            metadata_str = str(calibration_data.get("calibration_metadata", {}))

            # Publish the metadata
            pub.publish(metadata_str)

            rospy.loginfo(f"Published calibration metadata: {metadata_str}")

        except Exception as e:
            rospy.logerr(f"Error reading calibration file: {e}")

        # Sleep to maintain 1 Hz publishing rate
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_calibration_metadata()
    except rospy.ROSInterruptException:
        pass
