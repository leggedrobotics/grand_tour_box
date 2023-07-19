#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo
import time

class FrequencyPublisher:
    def __init__(self, topic_name, publish_topic, input_type):
        self.topic_name = topic_name
        self.publish_topic = publish_topic

        self.message_counter = 0
        self.start_time = None
        self.frequency = 0.0

        rospy.Subscriber(self.topic_name, input_type, self.callback)
        self.publisher = rospy.Publisher(self.publish_topic, Float32, queue_size=10)

    def callback(self, data):
        if self.start_time is None:
            self.start_time = time.time()

        self.message_counter += 1
        current_time = time.time()

        elapsed_time = current_time - self.start_time
        self.frequency = float(self.message_counter) / elapsed_time

        # Publish frequency as a Float32
        self.publisher.publish(self.frequency)

if __name__ == '__main__':
    rospy.init_node('frequency_publisher')

    camera3_topic = "/alphasense_driver_node/cam3"
    camera3_publish_topic = "/alphasense_driver_node/cam3/rate"
    camera3_publisher = FrequencyPublisher(camera3_topic, camera3_publish_topic, input_type=Image)

    camera4_topic = "/alphasense_driver_node/cam4"
    camera4_publish_topic = "/alphasense_driver_node/cam4/rate"
    camera4_publisher = FrequencyPublisher(camera4_topic, camera4_publish_topic, input_type=Image)

    camera5_topic = "/alphasense_driver_node/cam5"
    camera5_publish_topic = "/alphasense_driver_node/cam5/rate"
    camera5_publisher = FrequencyPublisher(camera5_topic, camera5_publish_topic, input_type=Image)

    camera6_topic = "/usb_cam/camera_info"
    camera6_publish_topic = "/usb_cam/camera_info/rate"
    camera6_publisher = FrequencyPublisher(camera6_topic, camera6_publish_topic, input_type=CameraInfo)

    rospy.spin()