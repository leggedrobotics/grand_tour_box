#!/usr/bin/env python3

import socket
import rospy

from box_health.msg import healthStatus
from publish_local_health_status import default_healthstatus

# TODO(beni) add mutex?
class BoxStatusMerger:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node('health_status_merger')

        # todo(beni) remove
        self.external_hostnames = ['jetson', 'nuc']
        self.subscribers = []

        for external_host in self.external_hostnames:
            topic = self.namespace + 'health_status/' + external_host
            sub = rospy.Subscriber(topic, healthStatus, self.callback)
            self.subscribers.append(sub)
        
        # names have to exactly match healthStatus.msg
        # TODO(beni): read directly from file
        self.message_fields = [
            "offset_mgbe0_systemclock",
            "offset_mgbe0_mgbe1",
            "status_mgbe0_ptp4l",
            "status_mgbe1_ptp4l",
            "offset_enp45s0_systemclock",
            "offset_enp45s0_enp46s0",
            "offset_mgbe0_enp45s0",
            "status_enp46s0_ptp4l",
            "gt_box_alphasense_driver_node_cam3_hz",
            "gt_box_alphasense_driver_node_cam4_hz",
            "gt_box_alphasense_driver_node_cam5_hz",
            "gt_box_hesai_pandar_hz",
            "gt_box_livox_lidar_hz",
            "gt_box_livox_imu_hz",
            "gt_box_alphasense_driver_node_imu_hz",
            "gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz",
            "gt_box_adis16475_hz",
            "gt_box_usb_cam_image_hz",
            "gps_num_sat",
            "gps_rtk_mode_fix",
            "gps_fix_mode",
            "gps_utc_time_ready",
        ]

        self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/merged' , healthStatus, queue_size=10)
        self.rate = rospy.Rate(1)
        
        self.complete_health_msg = default_healthstatus()

    def return_if_set(self, value):
        if isinstance(value, int):
            if value != -1:
                return value
            else:
                return -1             
        if isinstance(value, str):
            if value != "empty":
                return value
            else:
                return "empty" 

    def callback(self, partial_health_data):
        for field in self.message_fields:
            setattr(self.complete_health_msg, field, self.return_if_set(getattr(partial_health_data, field)))
    
    def publish_complete_health_status(self):
        while not rospy.is_shutdown():
            self.health_status_publisher.publish(self.complete_health_msg)
            self.complete_health_msg = healthStatus()
            self.rate.sleep()  

if __name__ == '__main__':
    try:
        box_status = BoxStatusMerger()
        box_status.publish_complete_health_status()

    except rospy.ROSInitException:
        pass
