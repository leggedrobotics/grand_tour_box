#!/usr/bin/env python3

import socket
import os
import rospy
from threading import Lock
import numpy as np

from box_health.msg import healthStatus, healthStatus_jetson, healthStatus_nuc, healthStatus_opc, healthStatus_rpi
from std_msgs.msg import Float32, Bool, ColorRGBA
from jsk_rviz_plugins.msg import *

mutex = Lock()
recording_mutex = Lock()

def array_to_color(color_list):
    lst = str(list(color_list))
    return "rgb(" + lst[1:-1] + ")"

def color_wrapper(text, perfect, ok, bad, biggerbetter):
    green = np.array([34,139,34])
    orange = np.array([255,140,0])
    red = np.array([139,0,0])
    try:
        value = abs(float(text))
        green = np.array([34,139,34])
        orange = np.array([255,140,0])
        red = np.array([139,0,0])

        if biggerbetter:
            if value > perfect:
                color = array_to_color(green)
            elif value > ok:
                color = array_to_color((((value - ok) / (perfect - ok)) * (green - orange)) + orange)
            elif value > bad:
                color = array_to_color((((value - bad) / (ok - bad)) * (green - orange)) + orange)
            else:
                color = array_to_color(red)
        else:
            if value < perfect:
                color = array_to_color(green)
            elif value < ok:
                color = array_to_color((((value - perfect) / (ok - perfect)) * (orange - green)) + green)
            elif value < bad:
                color = array_to_color((((value - ok) / (bad - ok)) * (red - orange)) + orange)
            else:
                color = array_to_color(red)

        return '<span style="color: ' + color + ';">' + str(text) + '</span>'
    except:
        color = array_to_color(red)
        return '<span style="color: ' + color + ';">' + str(text) + '</span>'

class visualizationPublisher:
    def __init__(self):
        self.namespace = rospy.get_namespace()

        # names have to exactly match healthStatus.msg
        topics_to_publish_float = [
            "gt_box_alphasense_driver_node_cam0_hz",
            "gt_box_alphasense_driver_node_cam1_hz",
            "gt_box_alphasense_driver_node_cam2_hz",
            "gt_box_alphasense_driver_node_cam3_hz",
            "gt_box_alphasense_driver_node_cam4_hz",
            "gt_box_hesai_pandar_packets_hz",
            "gt_box_hesai_pandar_hz",
            "gt_box_livox_lidar_hz",
            "gt_box_livox_imu_hz",
            "gt_box_alphasense_driver_node_imu_hz",
            "gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz",
            "gt_box_adis16475_imu_hz",
            "gt_box_stim320_imu_hz",
            "gt_box_camera_trigger_timestamps_hz",
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
        self.namespace = rospy.get_namespace()

        rospy.init_node("health_status_merger")
        self.recording_status = {
            "jetson": False,
            "nuc": False,
            "opc": False,
        }

        self.subscriber_jetson = rospy.Subscriber(
            self.namespace + "health_status/jetson", healthStatus_jetson, self.callback, "jetson"
        )
        self.subscriber_nuc = rospy.Subscriber(
            self.namespace + "health_status/nuc", healthStatus_nuc, self.callback, "nuc"
        )
        self.subscriber_opc = rospy.Subscriber(
            self.namespace + "health_status/opc", healthStatus_opc, self.callback, "opc"
        )
        self.subscriber_opc = rospy.Subscriber(
            self.namespace + "health_status/rpi", healthStatus_rpi, self.callback, "rpi"
        )
        self.recording_jetson = rospy.Subscriber(
            self.namespace + "health_status/recording_jetson", Bool, self.recording_callback, "jetson"
        )
        self.recording_nuc = rospy.Subscriber(
            self.namespace + "health_status/recording_nuc", Bool, self.recording_callback, "nuc"
        )
        self.recording_opc = rospy.Subscriber(
            self.namespace + "health_status/recording_opc", Bool, self.recording_callback, "opc"
        )

        self.message_fields = {
            "jetson": [],
            "nuc": [],
            "opc": [],
            "rpi": [],
        }

        for host in self.message_fields:
            filename = os.path.dirname(__file__) + "/../msg/healthStatus_" + host + ".msg"
            for line in open(filename):
                li=line.strip()
                if li and not li.startswith("#") and not li.isspace():
                    health_topic = li.split()[1]
                    self.message_fields[host].append(health_topic)

        self.health_status_publisher = rospy.Publisher(
            self.namespace + "health_status/merged", healthStatus, queue_size=10
        )
        self.rate = rospy.Rate(1)
        self.complete_health_msg = healthStatus()

    def callback(self, partial_health_data, sender):
        with mutex:
            for field in self.message_fields[sender]:
                setattr(self.complete_health_msg, field, getattr(partial_health_data, field))

    def recording_callback(self, recording_status, sender):
        with recording_mutex:
            self.recording_status[sender] = recording_status

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
        text.width = 300
        text.height = 575
        text.left = 10
        text.top = 430
        text.text_size = 13
        text.line_width = 0
        text.font = "Lato"
        text.fg_color = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.1)
        text.text = """Jetson mgbe0: %s
            Jetson mgbe1: %s
            Nuc enp46s0: %s
            OPC chrony: %s

            ptp mgbe0->enp45s0: %sns
            ptp mgbe0->eth0: %sns
            p2s mgbe0->mgbe1: %sns
            p2s mgbe0->jetson sys: %sns
            p2s enp45s0->enp46s0: %sns
            p2s enp45s0->nuc sys: %sns
            p2s eth0->pi sys: %sns
            chrony jetson->opc: %sns
            alphasense_frames_no_ptp: %s

            RTK mode fix: %s
            GPS fix mode: %s
            Num sat: %s

            Jetson avail memory: %s
            Jetson CPU usage: %s%%
            Nuc avail memory: %s
            Nuc CPU usage: %s%%
            Pi avail memory: %s
            Pi CPU usage: %s%%
            
            Jetson: %s
            Nuc: %s
            Opc: %s""" % (
            getattr(self.complete_health_msg, "status_mgbe0_ptp4l"),
            getattr(self.complete_health_msg, "status_mgbe1_ptp4l"),
            getattr(self.complete_health_msg, "status_enp46s0_ptp4l"),
            getattr(self.complete_health_msg, "chrony_status"),
            color_wrapper(getattr(self.complete_health_msg, "offset_mgbe0_enp45s0"), 100, 200, 1000, False),
            color_wrapper("0", 100, 200, 1000, False), # ptp mgbe0 -> pi eth0
            color_wrapper(getattr(self.complete_health_msg, "offset_mgbe0_mgbe1"), 100, 200, 1000, False),
            color_wrapper(getattr(self.complete_health_msg, "offset_mgbe0_systemclock"), 100, 200, 1000, False),
            color_wrapper(getattr(self.complete_health_msg, "offset_enp45s0_enp46s0"), 100, 200, 1000, False),
            color_wrapper(getattr(self.complete_health_msg, "offset_enp45s0_systemclock"), 100, 100, 1000, False),
            color_wrapper("0", 100, 200, 1000, False), # p2s eth0 -> pi sys:
            color_wrapper(getattr(self.complete_health_msg, "offset_chrony_opc_jetson"), 1000, 3000, 10000, False),
            color_wrapper(getattr(self.complete_health_msg, "alphasense_frames_no_ptp"), 1, 2, 3, False),
            color_wrapper(getattr(self.complete_health_msg, "gps_rtk_mode_fix"), 1, 0, 0, True),
            getattr(self.complete_health_msg, "gps_fix_mode"),
            color_wrapper(getattr(self.complete_health_msg, "gps_num_sat"), 10, 5, 0, True),
            getattr(self.complete_health_msg, "avail_memory_jetson"),
            color_wrapper("{:.2f}".format(getattr(self.complete_health_msg, "cpu_usage_jetson")), 50, 80, 90, False),
            getattr(self.complete_health_msg, "avail_memory_nuc"),
            color_wrapper("{:.2f}".format(getattr(self.complete_health_msg, "cpu_usage_nuc")), 50, 80, 90, False),
            "0",
            color_wrapper("{:.2f}".format(22.22222), 50, 80, 90, False),
            "<span style='color: rgb(34,139,34);'>Recording</span>" if self.recording_status["jetson"] else "Not recording",
            "<span style='color: rgb(34,139,34);'>Recording</span>" if self.recording_status["nuc"] else "Not recording",
            "<span style='color: rgb(34,139,34);'>Recording</span>" if self.recording_status["opc"] else "Not recording",
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
