#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Bool
# from smbus2 import SMBus


class UserInterfaceNode:
    def __init__(self):
        rospy.init_node('user_interface_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz for 100ms period

        # Initialize GPIO
        self.pi = pigpio.pi()
        self.left_button_pin = 21
        self.right_button_pin = 7
        self.pi.set_mode(self.left_button_pin, pigpio.INPUT)
        self.pi.set_mode(self.right_button_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.left_button_pin, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.right_button_pin, pigpio.PUD_UP)

        # Initialize publishers
        self.left_button_pub = rospy.Publisher('/gt_box/user_interface/button_left_status', Bool, queue_size=10)
        self.right_button_pub = rospy.Publisher('/gt_box/user_interface/button_right_status', Bool, queue_size=10)

        # Initialize I2C
        # self.bus = SMBus(0)  # Use I2C_0
        # self.i2c_address = 0x20  # TCA6408ARSVR address (adjust if different)

        # Initialize LED states
        # self.left_led_state = [0, 0, 0]  # R, G, B
        # self.right_led_state = [0, 0, 0]  # R, G, B

    def read_buttons(self):
        left_status = not self.pi.read(self.left_button_pin)  # Invert because of pull-up
        right_status = not self.pi.read(self.right_button_pin)  # Invert because of pull-up
        return left_status, right_status

    def publish_button_status(self):
        left_status, right_status = self.read_buttons()
        self.left_button_pub.publish(Bool(left_status))
        self.right_button_pub.publish(Bool(right_status))

    # def set_led_color(self, led, color):
    #     if led not in ['left', 'right'] or len(color) != 3:
    #         rospy.logerr("Invalid LED or color specification")
    #         return

    #     start_pin = 0 if led == 'left' else 3
    #     for i, val in enumerate(color):
    #         if val not in [0, 1]:
    #             rospy.logerr("LED color values must be 0 or 1")
    #             return
    #         self.set_i2c_pin(start_pin + i, val)

    #     if led == 'left':
    #         self.left_led_state = color
    #     else:
    #         self.right_led_state = color

    # def set_i2c_pin(self, pin, value):
    #     try:
    #         current = self.bus.read_byte(self.i2c_address)
    #         if value:
    #             new = current | (1 << pin)
    #         else:
    #             new = current & ~(1 << pin)
    #         self.bus.write_byte(self.i2c_address, new)
    #     except Exception as e:
    #         rospy.logerr(f"Error setting I2C pin: {e}")

    def run(self):
        while not rospy.is_shutdown():
            self.publish_button_status()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = UserInterfaceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass