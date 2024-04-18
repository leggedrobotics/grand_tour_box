#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from functools import partial


class FanControlNode(object):
    def __init__(self):
        # Not tested at all
        rospy.init_node("fan_controller")
        self.get_params()
        servers = {}

        
        self.desired_temperature = 50.0
        self.desired_fan_speed_in_percentage = 50.0
        self.last_error = 0.0

        for m in ["normal", "silent", "slow", "fast"]:
            servers[m] = rospy.Service(f"~{m}", Trigger, partial(self.set_mode, mode=m))
            
        rospy.loginfo(f"[FanControllerNode] Started in mode {self.mode}.")


        rate = rospy.Rate(self.control_rate)

        
        self.last_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self.desired_fan_speed_in_percentage != -1:
                # Constant Fan Speed
                control_signal = self.desired_fan_speed_in_percentage
            else:
                # Implement PD Controller
                current_temperature = self.get_current_temperature()
                error = self.desired_temperature - current_temperature
                current_time = rospy.get_time()
                delta_time = current_time - self.last_time
                derivative = (error - self.last_error) / delta_time
                
                # Calculate control action
                control_signal = self.Kp * error + self.Kd * derivative

                self.last_error = error
                self.last_time = current_time
                rospy.logdebug(f"Temperature error: {error}, Control signal: {control_signal}")


            self.adjust_fan_speed(control_signal)
            rate.sleep()
    
    def get_current_temperature(self):
        #TODO
        pass

    def adjust_fan_speed(self, control_signal):
        #TODO
        pass

    def get_params(self):
        self.mode = rospy.get_param("~mode", "normal")
        self.control_rate = rospy.get_param("~control_rate", 1)
        
        # Initialize PD controller variables
        self.Kp = rospy.get_param('Kp', 1.0)  # Proportional gain
        self.Kd = rospy.get_param('Kd', 0.1)  # Derivative gain

    def set_mode(self, request, mode):
        response = TriggerResponse()
        response.success = True
        response.message = f"Switched from {self.mode} mode to {mode} mode."
        self.mode = mode


        # Set here correctly
        self.last_error = 0.0 # Reset PD error
        self.desired_temperature 
        self.desired_fan_speed_in_percentage = -1

        # Further actions here.

        rospy.loginfo(f"[FanControllerNode] Switched from {self.mode} mode to {mode} mode.")
        return response


if __name__ == "__main__":
    node = FanControlNode()
