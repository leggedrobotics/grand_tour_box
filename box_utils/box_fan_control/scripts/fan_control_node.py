#!/usr/bin/env python3
import rospy
import pigpio
from std_srvs.srv import Trigger, TriggerResponse
from functools import partial
from box_fan_control.srv import SetFanSpeed, SetFanSpeedRequest


FAN_GPIO = 18
FAN_FEEDBACK = 16


class FanControlNode(object):
    def __init__(self):
        rospy.init_node("fan_controller")
        self.get_params()
        self.pi = pigpio.pi()

        self.pi.set_mode(FAN_GPIO, pigpio.OUTPUT)
        self.pi.set_mode(FAN_FEEDBACK, pigpio.INPUT)

        # We are controlling FAN: Sanyo Denki - 9TD12P6G001 - PWM Frequency 25kHz
        self.pi.set_PWM_frequency(FAN_GPIO, 25000)
        servers = {}

        self.last_error = 0.0

        for m in ["normal", "hot", "cold"]:
            servers[m] = rospy.Service(f"~{m}", Trigger, partial(self.set_mode, mode=m))

        servers["set_fan_speed"] = rospy.Service(f"~set_fan_speed", SetFanSpeed, self.set_fan_speed)
        rospy.loginfo(
            f"[FanControllerNode] Started in mode {self.mode} / Fan speed {self.desired_fan_speed_in_percentage}%."
        )

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

                if self.desired_temperature - 10 > current_temperature:
                    control_signal = 0
                elif self.desired_temperature < current_temperature:
                    control_signal = 40
                else:
                    control_signal = 60

                # current_time = rospy.get_time()
                # delta_time = current_time - self.last_time
                # derivative = (error - self.last_error) / delta_time

                # Calculate control action
                # control_signal = self.Kp * error + self.Kd * derivative

                # self.last_error = error
                # self.last_time = current_time
                # rospy.logdebug(f"Temperature error: {error}, Control signal: {control_signal}")

            if self.get_current_temperature() > self.critical_temperature:
                control_signal = 100

            self.adjust_fan_speed(control_signal)
            rate.sleep()

    def get_current_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as file:
                temp_str = file.read()
            return float(temp_str) / 1000  # Convert from millidegree Celsius to degree Celsius
        except FileNotFoundError:
            print("Could not read temperature from file - Return critical_temperature")
            return self.critical_temperature

    def adjust_fan_speed(self, control_signal):
        control_signal = max(0, min(100, control_signal))
        self.pi.set_PWM_dutycycle(FAN_GPIO, int(float(control_signal) * 2.55))

    def get_params(self):
        self.mode = rospy.get_param("~mode", "normal")
        self.control_rate = rospy.get_param("~control_rate", 5)
        self.desired_fan_speed_in_percentage = rospy.get_param("~desired_fan_speed_in_percentage", 40)
        self.critical_temperature = rospy.get_param("~critical_temperature", 70)

        # Initialize PD controller variables
        # self.Kp = rospy.get_param('Kp', 1.0)  # Proportional gain
        # self.Kd = rospy.get_param('Kd', 0.1)  # Derivative gain

    def set_fan_speed(self, request: SetFanSpeedRequest):
        response = TriggerResponse()
        response.success = True
        response.message = f"Fan speed set to {request.fan_speed_in_percentage}%."
        self.desired_fan_speed_in_percentage = request.fan_speed_in_percentage.data
        rospy.loginfo(f"[FanControllerNode] Fan speed set to {request.fan_speed_in_percentage}%.")
        return response

    def set_mode(self, request, mode):
        response = TriggerResponse()
        response.success = True
        response.message = f"Switched from {self.mode} mode to {mode} mode."
        self.mode = mode

        # Set here correctly
        self.last_error = 0.0  # Reset PD error
        self.desired_fan_speed_in_percentage = -1

        if mode == "normal":
            self.desired_temperature = 50.0
        elif mode == "hot":
            self.desired_temperature = 65.0
        elif mode == "cold":
            self.desired_temperature = 40.0

        rospy.loginfo(f"[FanControllerNode] Switched from {self.mode} mode to {mode} mode.")
        return response


if __name__ == "__main__":
    node = FanControlNode()
