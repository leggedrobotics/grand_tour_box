#!/usr/bin/env python3

import rospy, rostopic
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import time
import yaml
import socket

class FrequencyPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.rt = rostopic.ROSTopicHz(100)
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.rt.callback_hz, callback_args=self.topic) 
        self.publisher = rospy.Publisher(self.topic + "/rate", Float32, queue_size=10)
        rospy.sleep(1)   

    def publish_hz(self):
        hz_status = self.rt.get_hz(self.topic)
        if hz_status:
            self.publisher.publish(hz_status[0])
        else:
            self.publisher.publish(0.0)
        

if __name__ == '__main__':
    rospy.init_node('frequency_publisher')

    # jetson or nuc
    hostname = socket.gethostname() 

    # Read camera topics from ROS parameter
    topics = rospy.get_param(hostname, [])

    publishers = []
    for topic in topics:
        publisher = FrequencyPublisher(topic)
        publishers.append(publisher)

    while not rospy.is_shutdown():
        time.sleep(1)
        for pub in publishers:
            pub.publish_hz()

    rospy.spin()
