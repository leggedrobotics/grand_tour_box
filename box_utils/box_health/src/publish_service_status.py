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

        #self.r = rostopic.ROSTopicHz(1)

        rp = rospkg.RosPack()
        self.services_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_services.yaml")
        self.services = load_yaml(self.services_yaml)
        rospy.loginfo("[BoxStatusNode] Setup.")
        


    
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
        # print("last: ", last_line(output.decode('utf-8')))
        # print("err: ", err)

        if "ptp4l" in service:
            print("ptp4l")
        elif "phc2sys" in service:
            print("phc2sys")

        


    def check_services(self):
        for service in self.services[self.hostname]:
            self.check_service(service)

if __name__ == '__main__':
    rospy.loginfo("[BoxStatusNode] Starting the box status node")
    box_status = BoxStatusNode()
    r = rospy.Rate(10) # 10Hz
    i = 0
    while not rospy.is_shutdown() and i < 1:
        box_status.check_services()
        r.sleep()
        i = i + 1
    