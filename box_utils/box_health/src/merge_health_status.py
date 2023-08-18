#!/usr/bin/env python3

import socket
import rospy

from box_health.msg import healthStatus, healthStatus_jetson, healthStatus_nuc

# TODO(beni) add mutex?
class BoxStatusMerger:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node('health_status_merger')

        self.subscriber_jetson = rospy.Subscriber('/' + "health_status/jetson", healthStatus_jetson, self.callback, "jetson")
        self.subscriber_nuc = rospy.Subscriber('/' + "health_status/nuc", healthStatus_nuc, self.callback, "nuc")
        # names have to exactly match healthStatus.msg
        # TODO(beni): read directly from file
        self.message_fields = {
            "jetson" : [
                "offset_mgbe0_systemclock",
                "offset_mgbe0_mgbe1",
                "status_mgbe0_ptp4l",
                "status_mgbe1_ptp4l",
                "gt_box_alphasense_driver_node_cam3_hz",
                "gt_box_alphasense_driver_node_cam4_hz",
                "gt_box_alphasense_driver_node_cam5_hz",
                "gt_box_hesai_pandar_hz",
                "gt_box_livox_lidar_hz",
                "gt_box_livox_imu_hz",
                "gt_box_alphasense_driver_node_imu_hz",
                "gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz",
                "gps_num_sat",
                "gps_rtk_mode_fix",
                "gps_fix_mode",
                "gps_utc_time_ready"
            ],
            "nuc" : [
                "offset_enp45s0_systemclock",
                "offset_enp45s0_enp46s0",
                "offset_mgbe0_enp45s0",
                "status_enp46s0_ptp4l",
                "gt_box_adis16475_hz",
                "gt_box_image_raw_hz",
            ]
        }
        self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/merged' , healthStatus, queue_size=10)
        self.rate = rospy.Rate(1)
        self.complete_health_msg = healthStatus()

    def callback(self, partial_health_data, sender):
        rospy.loginfo("[BoxStatusMerger]" + sender)
        for field in self.message_fields[sender]:
            setattr(self.complete_health_msg, field, getattr(partial_health_data, field))
    
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
