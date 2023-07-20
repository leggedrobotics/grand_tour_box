#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publish_once():

    rospy.init_node('string_publisher', anonymous=True)

    topic_name = rospy.get_param("~topic_name")
    msg_string = rospy.get_param("~msg_string")

    pub = rospy.Publisher(topic_name, String, queue_size=1)

    # Wait until the publisher is connected to the topic
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # Publish the message
    pub.publish(msg_string)

    # Sleep for a short duration to ensure the message is transmitted
    rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        publish_once()
    except rospy.ROSInterruptException:
        pass




