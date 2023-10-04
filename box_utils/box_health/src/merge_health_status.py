#!/usr/bin/env python3

import socket
import rospy
from threading import Lock
import numpy as np

from box_health.msg import healthStatus, healthStatus_jetson, healthStatus_nuc, healthStatus_opc
from std_msgs.msg import Float32, ColorRGBA
from jsk_rviz_plugins.msg import *

mutex = Lock()

def array_to_color(color_list):
    lst = str(list(color_list))
    return "rgb(" + lst[1:-1] + ")"

def color_wrapper(text, perfect, ok, bad, biggerbetter):
    green = np.array([34,139,34])
    orange = np.array([255,140,0])
    red = np.array([139,0,0])
    try:
        value = float(text)
        # TODO add smooth color change
        if biggerbetter:
            if value > perfect:
                color = array_to_color(green)
            elif value > ok:
                color = array_to_color(orange)
            elif value > bad:
                color = array_to_color(red)
            else:
                color = array_to_color(red)
        else:
            if value < perfect:
                color = array_to_color(green)
            elif value < ok:
                color = array_to_color(orange)
            elif value < bad:
                color = array_to_color(red)
            else:
                color = array_to_color(red)

        return '<span style="color: ' + color + ';">' + text + '</span>'
    except:
        color = array_to_color(red)
        return '<span style="color: ' + color + ';">' + text + '</span>'

class visualizationPublisher:
    def __init__(self):
        self.namespace = rospy.get_namespace()

        # names have to exactly match healthStatus.msg
        topics_to_publish_float = [
            "gt_box_alphasense_driver_node_cam3_hz",
            "gt_box_alphasense_driver_node_cam4_hz",
            "gt_box_alphasense_driver_node_cam5_hz",
            "gt_box_hesai_pandar_packets_hz",
            "gt_box_livox_lidar_hz",
            "gt_box_livox_imu_hz",
            "gt_box_alphasense_driver_node_imu_hz",
            "gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz",
            "gt_box_adis16475_hz",
            "gt_box_v4l2_camera_left_image_raw_hz",
            "gt_box_v4l2_camera_middle_image_raw_hz",
            "gt_box_v4l2_camera_right_image_raw_hz",
            "gt_box_leica_position_hz",
            "offset_mgbe0_systemclock",
            "offset_mgbe0_mgbe1",
            "offset_mgbe0_enp45s0",
            "offset_enp45s0_systemclock",
            "offset_chrony_opc_jetson",
        ]

        self.publishers_float = {}
        for topic in topics_to_publish_float:
            self.publishers_float[topic] = rospy.Publisher(
                self.namespace + "visualization/" + topic, Float32, queue_size=10
            )

        self.text_publisher = rospy.Publisher("visualization/clock_status", OverlayText, queue_size=1)


class BoxStatusMerger:
    def __init__(self, publisher):
        self.publisher = publisher
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node("health_status_merger")

        self.subscriber_jetson = rospy.Subscriber(
            self.namespace + "health_status/jetson", healthStatus_jetson, self.callback, "jetson"
        )
        self.subscriber_nuc = rospy.Subscriber(
            self.namespace + "health_status/nuc", healthStatus_nuc, self.callback, "nuc"
        )
        self.subscriber_nuc = rospy.Subscriber(
            self.namespace + "health_status/opc", healthStatus_opc, self.callback, "opc"
        )
        # names have to exactly match healthStatus.msg
        # TODO(beni): read directly from file
        self.message_fields = {
            "jetson": [
                "offset_mgbe0_systemclock",
                "offset_mgbe0_mgbe1",
                "status_mgbe0_ptp4l",
                "status_mgbe1_ptp4l",
                "gt_box_alphasense_driver_node_cam3_hz",
                "gt_box_alphasense_driver_node_cam4_hz",
                "gt_box_alphasense_driver_node_cam5_hz",
                "gt_box_hesai_pandar_packets_hz",
                "gt_box_livox_lidar_hz",
                "gt_box_livox_imu_hz",
                "gt_box_alphasense_driver_node_imu_hz",
                "gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz",
                "gps_num_sat",
                "gps_rtk_mode_fix",
                "gps_fix_mode",
                "gps_utc_time_ready",
                "cpu_usage_jetson",
                "avail_memory_jetson",
            ],
            "nuc": [
                "gt_box_v4l2_camera_left_image_raw_hz",
                "gt_box_v4l2_camera_middle_image_raw_hz",
                "gt_box_v4l2_camera_right_image_raw_hz",
                "offset_enp45s0_systemclock",
                "offset_enp45s0_enp46s0",
                "offset_mgbe0_enp45s0",
                "status_enp46s0_ptp4l",
                "gt_box_adis16475_hz",
                "cpu_usage_nuc",
                "avail_memory_nuc",
            ],
            "opc": [
                "offset_chrony_opc_jetson",
                "gt_box_leica_position_hz",
                "cpu_usage_opc",
                "avail_memory_opc",
                "chrony_status",
            ]
        }
        self.health_status_publisher = rospy.Publisher(
            self.namespace + "health_status/merged", healthStatus, queue_size=10
        )
        self.rate = rospy.Rate(1)
        self.complete_health_msg = healthStatus()

    def callback(self, partial_health_data, sender):
        with mutex:
            for field in self.message_fields[sender]:
                setattr(self.complete_health_msg, field, getattr(partial_health_data, field))

    def publish_frequency_visualization(self):
        for key, value in self.publisher.publishers_float.items():
            msg = getattr(self.complete_health_msg, key)
            if msg:
                try:
                    value.publish(abs(float(msg)))
                except:
                    value.publish(0.0)
                    rospy.logerr(
                        "[BoxStatusMerger] The following value for topic "
                        + str(key)
                        + " is not a offset (i.e a float): "
                        + str(msg)
                    )
            else:
                value.publish(0.0)

    def publish_text_visualization(self):
        text = OverlayText()
        text.width = 250
        text.height = 300
        text.left = 10
        text.top = 370
        text.text_size = 12
        text.line_width = 2
        text.font = "Arial"
        text.fg_color = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        text.text = """Clock status:
        Clock mgbe0: %s
        Clock mgbe1: %s
        Clock enp46s0: %s

        Jetson mgbe0 to mgbe1: %s

        GPS status:
        RTK mode fix: %i
        GPS fix mode: %s
        Num sat: %i 
        
        Avail memory Jetson: %s
        Avail memory Nuc: %s
        CPU usage Jetson: %.2f%%
        CPU usage Nuc: %.2f%%""" % (
            getattr(self.complete_health_msg, "status_mgbe0_ptp4l"),
            getattr(self.complete_health_msg, "status_mgbe1_ptp4l"),
            getattr(self.complete_health_msg, "status_enp46s0_ptp4l"),
            color_wrapper(getattr(self.complete_health_msg, "offset_mgbe0_mgbe1"), 0, 100, 1000, False),
            getattr(self.complete_health_msg, "gps_rtk_mode_fix"),
            getattr(self.complete_health_msg, "gps_fix_mode"),
            getattr(self.complete_health_msg, "gps_num_sat"),
            getattr(self.complete_health_msg, "avail_memory_jetson"),
            getattr(self.complete_health_msg, "avail_memory_nuc"),
            getattr(self.complete_health_msg, "cpu_usage_jetson"),
            getattr(self.complete_health_msg, "cpu_usage_nuc"),
        )
        self.publisher.text_publisher.publish(text)

    def publish_complete_health_status(self):
        while not rospy.is_shutdown():
            with mutex:
                self.health_status_publisher.publish(self.complete_health_msg)
                self.publish_frequency_visualization()
                self.publish_text_visualization()

                self.complete_health_msg = healthStatus()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        vizualization = visualizationPublisher()
        box_status = BoxStatusMerger(vizualization)
        box_status.publish_complete_health_status()

    except rospy.ROSInitException:
        pass
