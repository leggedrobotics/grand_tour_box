#!/usr/bin/env python3

import socket
#import rospkg
from os.path import join
#import yaml
#import subprocess
#import re

import rospy
from box_health.msg import healthStatus

class BoxStatusMerger:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        self.external_hostnames = ['jetson', 'nuc']
        self.subscribers = []
        for external_host in self.external_hostnames:
            topic = self.namespace + 'health_status/' + external_host
            sub = rospy.Subscriber(topic, healthStatus, self.callback, callback_args=external_host)
            self.subscribers.append(sub)

        rospy.init_node('health_status_merger')
        self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/merged' , healthStatus, queue_size=10)
        self.rate = rospy.Rate(1)
        
        self.complete_health_msg = healthStatus()

    def update_health_data(self, complete_health_data, partial_healt_data, host):


        return complete_health_data

    def callback(self, partial_health_data, host):
        print("Host: ", host)
        print("Healthdata: ", partial_health_data)

        self.complete_health_msg = self.update_health_data(self.complete_health_msg, partial_health_data, host)
    
    def publish_complete_health_status(self):
        while not rospy.is_shutdown():
            self.health_status_publisher.publish(self.complete_health_msg)
            self.complete_health_msg = healthStatus()
            self.rate.sleep()  

if __name__ == '__main__':
    try:
        box_status = BoxStatusMerger()
        box_status.publish_complete_health_status()

    except rospy.ROSInitException:
        pass
