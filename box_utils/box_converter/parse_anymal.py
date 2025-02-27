#!/usr/bin/env python

import rosbag
import rospy
from anymal_msgs.msg import AnymalState
from anymal_msgs.msg import ReadingsExtended, ReadingExtended
from box_auto.utils import get_bag


def convert_anymal_state(msg):
    new_msg = AnymalState()
    new_msg.header = msg.header
    new_msg.pose = msg.pose
    new_msg.twist = msg.twist
    new_msg.joints = msg.joints
    new_msg.contacts = msg.contacts
    new_msg.frame_transforms = msg.frame_transforms
    return new_msg


def convert_readings_extended(msg):
    new_msg = ReadingsExtended()
    new_msg.readings = [ReadingExtended(reading) for reading in msg.readings]
    return new_msg


if __name__ == "__main__":
    in_bag = get_bag("*_lpc_state_estimator.bag")
    out_bag = in_bag.replace("_estimator.bag", "_estimator_pretty.bag")

    # STATE ESTIMATOR
    with rosbag.Bag(in_bag, "r") as inbag:
        with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
            for topic, msg, t in inbag.read_messages():
                # Convert AnymalState messages
                if topic == "/state_estimator/anymal_state":
                    try:
                        new_msg = convert_anymal_state(msg)
                        outbag.write(topic, new_msg, t)
                    except Exception as e:
                        rospy.logwarn(f"Failed to convert AnymalState message: {e}")
                        continue
                # Write all other messages as-is
                else:
                    outbag.write(topic, msg, t)

    in_bag = get_bag("*_lpc_general.bag")
    out_bag = in_bag.replace("_lpc_general.bag", "_lpc_general_pretty.bag")
    # STATE ESTIMATOR
    with rosbag.Bag(in_bag, "r") as inbag:
        with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
            for topic, msg, t in inbag.read_messages():
                # Convert AnymalState messages
                if topic == "/anymal_low_level_controller/actuator_readings":
                    try:
                        new_msg = convert_readings_extended(msg)
                        outbag.write(topic, new_msg, t)
                    except Exception as e:
                        rospy.logwarn(f"Failed to convert ReadingsExtended message: {e}")
                        continue
                # Write all other messages as-is
                else:
                    outbag.write(topic, msg, t)
