#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import String

def string_publisher_node():
    rospy.init_node('string_publisher_node', anonymous=True)

    # Read the list of strings from the ROS parameter server
    string_list = rospy.get_param("strings", [])

    for idx, s in enumerate(string_list):
        topic_name = "string_topic_" + str(idx)
        pub = rospy.Publisher(topic_name, String, queue_size=10)
        while pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        rospy.loginfo("Publishing string '%s' on topic '%s'", s, topic_name)
        pub.publish(s)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        string_publisher_node()
    except rospy.ROSInterruptException:
        pass
