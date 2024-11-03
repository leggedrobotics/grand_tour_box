#!/usr/bin/env python3

import rospy
from pdb_msgs.srv import SetPowerLine


class PowerLineActivationService:
    def __init__(self):
        rospy.init_node("power_line_activation_service")

        # Define the power lines to be activated
        self.power_lines = [1, 5, 13]

        rospy.loginfo("Power Line Activation Service is ready.")

    def wait_for_power_line_services(self):
        """
        Wait for all power line services to be available
        """
        for line in self.power_lines:
            service_name = "/pdb_driver/set_power_line"
            rospy.loginfo(f"Waiting for service: {service_name}")
            try:
                rospy.wait_for_service(service_name, timeout=5.0)
            except rospy.ROSException:
                rospy.logerr(f"Service {service_name} not available")
                return False
        return True

    def activate_power_lines(self):
        """
        Attempt to activate all specified power lines
        """
        try:
            # Create service proxies for each power line
            set_power_line_service = rospy.ServiceProxy("/pdb_driver/set_power_line", SetPowerLine)

            # Activate each power line
            for line in self.power_lines:
                rospy.loginfo(f"Activating power line {line}")
                response = set_power_line_service(state=True, powerLine=line)
                # Check if the service call was successful
                if not response.is_set:
                    rospy.logerr(f"Failed to activate power line {line}")
                    return False

            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def handle_activate_power_lines(self):
        """
        Service handler for activating power lines
        """
        # Check if services are available
        if not self.wait_for_power_line_services():
            return False

        # Attempt to activate power lines
        success = self.activate_power_lines()

        return success


def main():
    plas = PowerLineActivationService()
    suc = False
    while not suc:
        suc = plas.handle_activate_power_lines()


if __name__ == "__main__":
    main()
