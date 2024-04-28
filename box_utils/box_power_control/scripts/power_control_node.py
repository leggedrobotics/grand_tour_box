#!/usr/bin/env python3
import rospy
import os
import requests
import subprocess
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class PowerControlNode(object):
    def __init__(self):
        rospy.init_node("box_power_controller")
        # Get the parameters
        self.get_params()

        # Get the current working directory
        self.cwd = os.path.join(os.getcwd(), "/scripts")

        # Set the servers.
        servers = {}
        servers["toggle_livox_lidar"] = rospy.Service("~toggle_livoxMID360", SetBool, self.set_livox_mode)
        servers["toggle_hesai_lidar"] = rospy.Service("~toggle_hesaiXT32", SetBool, self.set_hesai_mode)

        rospy.loginfo(
            f"[PowerControllerNode] Started. By default, both LiDARs are enabled."
        )

        # Double to rate conversion.
        rate = rospy.Rate(self.ros_rate)

        while not rospy.is_shutdown():
            # No recursive action, we have the services.
            rate.sleep()

    def get_params(self):
        self.ros_rate = rospy.get_param("~ros_rate", 10)
    
    def set_hesai_mode(self, request: SetBoolRequest):
        """
        {
            "Head": {
                "ErrorCode": "0",
                "Message": "Success"
            },
            "Body": {
                "standbymode": "0"
            }
        }

        standbymode [string] Standby mode
        "0" — In operation
        "1" — Standby
        """

        response = SetBoolResponse()

        # Here get the current state of the LiDAR
        # TODO(Get the IP from /etc/hosts file or from the launch file?)
        htmlResponse = requests.get("http://192.168.2.201/pandar.cgi?action=get&object=lidar_data&key=standbymode")
        htmlResponse.raise_for_status()
        hesaiCurrentdata = htmlResponse.json()
        print(hesaiCurrentdata)

        # Parse the current state of the LiDAR
        currentState = (hesaiCurrentdata["Body"]["standbymode"] == "0")

        # Early return if the request is to enable the LiDAR and it is already enabled
        if (request.data and currentState):
            response.success = True
            response.message = "Hesai-XT32 is already enabled."
            rospy.loginfo(f"[PowerControllerNode] Hesai-XT32 is already enabled.")
            return response
        
        # Early return if the request is to disable the LiDAR and it is already disabled
        if (not request.data and not currentState):
            response.success = True
            response.message = "Hesai-XT32 is already disabled."
            rospy.loginfo(f"[PowerControllerNode] Hesai-XT32 is already disabled.")
            return response

        if (request.data):
            setEnableResponse = requests.get("http://192.168.2.201/pandar.cgi?action=set&object=lidar_data&key=standbymode&value=0")
            setEnableResponse.raise_for_status()
            setEnableResponseData = setEnableResponse.json()
            print(setEnableResponseData)

            response.success = True
            response.message = "Hesai-XT32 is enabled."
            rospy.loginfo(f"[PowerControllerNode] Hesai-XT32 is enabled.")

        if (not request.data):
            setDisableResponse = requests.get("http://192.168.2.201/pandar.cgi?action=set&object=lidar_data&key=standbymode&value=1")
            setDisableResponse.raise_for_status()
            setDisableResponseData = setDisableResponse.json()
            print(setDisableResponseData)

            response.success = True
            response.message = "Hesai-XT32 is disabled."
            rospy.loginfo(f"[PowerControllerNode] Hesai-XT32 is disabled.")

        return response

    def set_livox_mode(self, request: SetBoolRequest):
        response = SetBoolResponse()

        # Here get the current state of the LiDAR
        if request.data:
            try:
                pathToScript = os.path.join(self.cwd, "/enable_livox.sh")
                print(pathToScript)
                output = subprocess.check_output(['/bin/bash', pathToScript], stderr=subprocess.STDOUT)
                print("Enable_livox output:\n", output.decode('utf-8'))
            except subprocess.CalledProcessError as e:
                print("Failed to run script:", e.output.decode('utf-8'))

            response.success = True
            response.message = "Livox-MID360 is enabled."
            rospy.loginfo(f"[PowerControllerNode] Livox-MID360 is enabled.")

        if (not request.data):
            try:
                pathToScript = os.path.join(self.cwd, "/disable_livox.sh")
                output = subprocess.check_output(['/bin/bash', pathToScript], stderr=subprocess.STDOUT)
                print("Enable_livox output:\n", output.decode('utf-8'))
            except subprocess.CalledProcessError as e:
                print("Failed to run script:", e.output.decode('utf-8'))

            response.success = True
            response.message = "Livox-MID360 is disabled."
            rospy.loginfo(f"[PowerControllerNode] Livox-MID360 is disabled.")

        return response


if __name__ == "__main__":
    node = PowerControlNode()
