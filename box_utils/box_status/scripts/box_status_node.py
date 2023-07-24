#! /usr/bin/env python3

import rospy
import rostopic
import rospkg
import socket
from os.path import join
import yaml

import subprocess

def load_yaml(path: str) -> dict:
    """Loads yaml file
    Args:
        path (str): File path
    Returns:
        (dict): Returns content of file
    """
    with open(path) as file:
        res = yaml.load(file, Loader=yaml.FullLoader)
    if res is None:
        res = {}
    return res

class BoxStatusNode:
    def __init__(self):
        print("init")
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node(f'box_status_node_{self.hostname}')
        print(self.hostname)

        #self.r = rostopic.ROSTopicHz(1)

        rp = rospkg.RosPack()
        self.topics_yaml = join( str(rp.get_path('box_status')), "cfg/box_status_topics.yaml")
        self.topics = load_yaml(self.topics_yaml)

        self.services_yaml = join( str(rp.get_path('box_status')), "cfg/box_status_services.yaml")
        self.services = load_yaml(self.services_yaml)
        rospy.loginfo("[BoxStatusNode] Setup.")
        


    
    def check_clock(interface):
        print("Check clock")

    def check_topic(self, topic):
        print(self.r.get_hz('/alphasense_driver_node/cam0')) 
        print("Check: ", topic)

    def check_topics(self):
        for topic in self.topics[self.hostname]:
            self.check_topic(topic)

    def check_service(self, service):
        print("check service: ", service)
        p = subprocess.Popen(["systemctl", "is-active",  service], stdout=subprocess.PIPE)
        (output, err) = p.communicate()
        output = output.decode('utf-8').strip()
        print("out: ", output)
        print("-----")
        print("err: ", err)


    def check_services(self):
        for service in self.services[self.hostname]:
            self.check_service(service)
    def check_status(self):
        print("Check status")
        #self.check_topics()
        self.check_services()

if __name__ == '__main__':
    rospy.loginfo("[BoxStatusNode] Starting the box status node")
    box_status = BoxStatusNode()
    r = rospy.Rate(10) # 10Hz
    i = 0
    while not rospy.is_shutdown() and i < 3:
        box_status.check_status()
        r.sleep()
        i = i + 1
    