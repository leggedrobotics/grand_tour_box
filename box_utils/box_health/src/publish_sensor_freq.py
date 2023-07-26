#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import time
import yaml
import socket

class FrequencyPublisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.window_duration = 1.0  # Time window for calculating instantaneous frequency (in seconds)

        self.message_counter = 0
        self.window_start_time = time.time()
        self.frequency = 0.0

        rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.callback)
        self.publisher = rospy.Publisher(self.topic_name + "/rate", Float32, queue_size=10)

    def callback(self, data):
        self.message_counter += 1
        current_time = time.time()

        # Check if the time window has passed
        if current_time - self.window_start_time >= self.window_duration:
            elapsed_time = current_time - self.window_start_time
            self.frequency = float(self.message_counter) / elapsed_time

            # Publish frequency as a Float32
            self.publisher.publish(self.frequency)

            # Reset counters and start time for the next window
            self.message_counter = 0
            self.window_start_time = current_time


if __name__ == '__main__':
    rospy.init_node('frequency_publisher')

    # jetson or nuc
    hostname = socket.gethostname() 

    # Read camera topics from ROS parameter
    camera_topics = rospy.get_param(hostname, [])
    print(camera_topics)

    publishers = []
    for topic in camera_topics:
        publisher = FrequencyPublisher(topic)
        publishers.append(publisher)

    rospy.spin()
