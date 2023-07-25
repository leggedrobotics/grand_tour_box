#! /usr/bin/env python3

import rospy
import rostopic
import rospkg
import socket
from os.path import join, splitext
import yaml
from std_msgs.msg import Int64
import re

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

def last_line(text: str) -> str:
    """Returns last line of a text
    Args:
        text (str): some string, may or may not 
        contain one or multiple newlines
    Returns:
        (str): Returns the last line
    """
    # find last newline, ignoring the one at the very end
    idx = text.rfind('\n', 0, len(text) - 1)
    return text[idx+1:]

class BoxStatusNode:
    def __init__(self):
        print("init")
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node(f'box_status_node_{self.hostname}')
        print(self.hostname)

        rp = rospkg.RosPack()
        self.services_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_services.yaml")
        self.services = load_yaml(self.services_yaml)
        rospy.loginfo("[BoxStatusNode] Setup.")

        self.publishers = {}
        for service in self.services[self.hostname]:
            self.publishers[service] = rospy.Publisher("/health_check/" + splitext(service)[0], Int64, queue_size=10)
        print(self.publishers)

    
    def check_clock(interface):
        print("Check clock")


    def check_service(self, service):
        
        # check if service is active
        p = subprocess.Popen(["systemctl", "is-active",  service], stdout=subprocess.PIPE)
        (output_active, error_active) = p.communicate()
        output_active = output_active.decode('utf-8').strip()
        # print("out: ", output_active)
        # print("err: ", error_active)

        # check status of service
        p = subprocess.Popen(["systemctl", "status",  service], stdout=subprocess.PIPE)
        (output_status, error_status) = p.communicate()
        # print("out: ", output.decode('utf-8'))
        recent_line = last_line(output_status.decode('utf-8'))

        if self.hostname == "jetson":
            if "ptp4l" in service:
                if "assuming the grand master role" in recent_line:
                    self.publishers[service].publish(1)
                else:
                    self.publishers[service].publish(0)
            elif "phc2sys" in service:
                idx = recent_line.find("offset")
                numbers_in_line = [int(d) for d in re.findall(r'-?\d+', recent_line[idx:])]
                offset = numbers_in_line[0]
                freq = numbers_in_line[2]
                delay = numbers_in_line[3]
                self.publishers[service].publish(offset)

        #elif self.hostname == "nuc":
        #    # todo        
        #    print("ptp4l")
        


        


    def check_services(self):
        for service in self.services[self.hostname]:
            self.check_service(service)

if __name__ == '__main__':
    rospy.loginfo("[BoxStatusNode] Starting the box status node")
    box_status = BoxStatusNode()
    r = rospy.Rate(10) # 10Hz
    i = 0
    while not rospy.is_shutdown():
        box_status.check_services()
        r.sleep()
        i = i + 1
    