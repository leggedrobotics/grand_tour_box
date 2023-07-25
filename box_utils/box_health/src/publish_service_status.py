#! /usr/bin/env python3

import rospy
import rostopic
import rospkg
import socket
from os.path import join, splitext
import yaml
from std_msgs.msg import Int64, Float32
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

def offset_from_status(line: str) -> int:
    idx = line.find("offset")
    numbers_in_line = [int(d) for d in re.findall(r'-?\d+', line[idx:])]
    offset = numbers_in_line[0]
    return offset

class BoxServiceStatus:
    def __init__(self):
        print("init")
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node(f'box_service_status_{self.hostname}')
        print(self.hostname)

        rp = rospkg.RosPack()
        self.services_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_services.yaml")
        self.services = load_yaml(self.services_yaml)
        rospy.loginfo("[BoxServiceStatus] Setup.")

        self.publishers = {}
        for service in self.services[self.hostname]:
            self.publishers[service] = rospy.Publisher("/health_check/" + splitext(service)[0], Float32, queue_size=10)
        print(self.publishers)

    
    def check_clock(interface):
        print("Check clock")


    def check_service(self, service):
        
        # check status of service
        p = subprocess.Popen(["systemctl", "status",  service], stdout=subprocess.PIPE)
        (output_status, error_status) = p.communicate()
        recent_line = last_line(output_status.decode('utf-8'))

        if self.hostname == "jetson":
            if "ptp4l" in service:
                if "assuming the grand master role" in recent_line:
                    self.publishers[service].publish(1.0)
                else:
                    self.publishers[service].publish(0.0)
            elif "phc2sys" in service:                
                offset = offset_from_status(recent_line)
                self.publishers[service].publish(offset)
            else:
                rospy.logerr("[BoxServiceStatus] This service is unknown on the " + self.hostname + ": " + str(service))
        
        elif self.hostname == "nuc":
            if "ptp4l" in service:
                # enp45s is the slave clock, gets synchronized to jetson:mgbe0
                if "enp45s0" in service:
                    offset = offset_from_status(recent_line)
                    self.publishers[service].publish(offset)
                if "enp46s0" in service:
                    if "assuming the grand master role" in recent_line:
                        self.publishers[service].publish(1)
                    else:
                        self.publishers[service].publish(0)
            elif "phc2sys" in service:                
                offset = offset_from_status(recent_line)
                self.publishers[service].publish(offset)
            else:
                rospy.logerr("[BoxServiceStatus] This service is unknown on the " + self.hostname + ": " + str(service))
        else:
            rospy.logerr("[BoxServiceStatus] This hostname is unknown: " + self.hostname)
        
    def check_services(self):
        for service in self.services[self.hostname]:
            self.check_service(service)

if __name__ == '__main__':
    rospy.loginfo("[BoxServiceStatus] Starting the box status node")
    box_service_status = BoxServiceStatus()
    r = rospy.Rate(1) # 1Hz, the service status itself is published with 1 HZ
    while not rospy.is_shutdown():
        box_service_status.check_services()
        r.sleep()
    