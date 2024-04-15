#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from functools import partial 

class FanControllerNode(object):
    def __init__(self):
        rospy.init_node("fan_controller")
        self.get_params()
        servers = {}

        for m in ["normal", "silent", "slow", "fast"]:
            servers[m] = rospy.Service(f"~{m}", Trigger, partial(self.set_mode, mode=m))
        
        rospy.loginfo(f"[FanControllerNode] Started in mode {self.mode}.")

        rospy.spin() 

    def get_params(self):
        self.mode = rospy.get_param("~mode", "normal")

    def set_mode(self, request, mode):
        response = TriggerResponse()
        response.success = True
        response.message = f"Switched from {self.mode} mode to {mode} mode."
        self.mode = mode  

        # Further actions here.

        rospy.loginfo(f"[FanControllerNode] Switched from {self.mode} mode to {mode} mode.")
        return response

if __name__ == "__main__":
    node = FanControllerNode()
    