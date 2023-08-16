#!/usr/bin/env python3

import socket
import rospkg
from os.path import join
import yaml
import subprocess
import re

import rospy, rostopic
from box_health.msg import healthStatus
from piksi_rtk_msgs.msg import ReceiverState_V2_6_5

def load_yaml(path: str) -> dict:
    with open(path) as file:
        res = yaml.load(file, Loader=yaml.FullLoader)
    if res is None:
        res = {}
    return res

def last_line(text: str) -> str:
    idx = text.rfind('\n', 0, len(text) - 1)
    return text[idx+1:]

def offset_from_status(line: str) -> int:
    idx = line.find("offset")
    numbers_in_line = [int(d) for d in re.findall(r'-?\d+', line[idx:])]
    if numbers_in_line:
        offset = numbers_in_line[0]
        return offset
    else:
        rospy.logerr("[BoxStatus] Error reading offset from line: " + line)
        return "error reading offset"
    
class FrequencyFinder:
    def __init__(self, topic):
        self.topic = topic
        self.rt = rostopic.ROSTopicHz(100)
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.rt.callback_hz, callback_args=self.topic)
        rospy.sleep(1)   

    def find_frequency(self):
        hz_status = self.rt.get_hz(self.topic)
        if hz_status:
            return hz_status[0]
        else:
            rospy.logerr("[BoxStatus] Error reading frequency of " + self.topic)
            return 0.0

class BoxStatus:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node(f'health_status_publisher_{self.hostname}')

        rp = rospkg.RosPack()
        services_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_services.yaml")
        self.services = load_yaml(services_yaml)

        topics_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_topics.yaml")
        topics_allPCs = load_yaml(topics_yaml)
        self.topics = []
        if self.hostname in topics_allPCs:
            self.topics = topics_allPCs[self.hostname]

        self.finders = {}
        for topic in self.topics:
            finder = FrequencyFinder(topic)
            self.finders[topic] = finder

        # check GPS status on PC which checks the GPS topic frequency
        self.check_gps_status = "rover" in "".join(self.topics)
        if self.check_gps_status:
            self.GPS_subscriber = rospy.Subscriber("/gt_box/rover/piksi/position_receiver_0/ros/receiver_state", ReceiverState_V2_6_5, self.gps_callback)
            self.gps_num_sat = 0
            self.gps_rtk_mode_fix = False
            self.gps_fix_mode = ""
            self.gps_utc_time_ready = False

        self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/' + self.hostname, healthStatus, queue_size=10)
        self.rate = rospy.Rate(1)

    def gps_callback(self, data):
        self.gps_num_sat = data.num_sat
        self.gps_rtk_mode_fix = data.rtk_mode_fix
        self.gps_fix_mode = data.fix_mode
        self.gps_utc_time_ready = data.utc_time_ready


    def get_clock_offsets(self, health_msg):
        if self.hostname in self.services:
            for service in self.services[self.hostname]:
                health_msg = self.check_service(health_msg, service)
        return health_msg
    
    def check_if_grandmaster(self, recent_line):
        if "assuming the grand master role" in recent_line:
            return "grand master"
        else:
            return "not grand master"
        
    def check_clock_offset(self, recent_line):
        if "Waiting for ptp4l..." in recent_line:
            return "waiting for ptp4l"
        else:      
            return str(offset_from_status(recent_line))
    
    def check_service(self, health_msg, service):

        # check status of service
        p = subprocess.Popen(["systemctl", "status",  service], stdout=subprocess.PIPE)
        (output_status, error_status) = p.communicate()
        recent_line = last_line(output_status.decode('utf-8'))

        if self.hostname == "jetson":
            if "ptp4l_mgbe0" in service:
                health_msg.status_mgbe0_ptp4l = self.check_if_grandmaster(recent_line)
            elif "ptp4l_mgbe1" in service:
                health_msg.status_mgbe1_ptp4l = self.check_if_grandmaster(recent_line)
            elif "phc2sys_mgbe0" in service:
                health_msg.offset_mgbe0_systemclock = self.check_clock_offset(recent_line)
            elif "phc2sys_mgbe1" in service:                
                health_msg.offset_mgbe0_mgbe1 = self.check_clock_offset(recent_line)
            else:
                rospy.logerr("[BoxStatus] This service is unknown on the " + self.hostname + ": " + str(service))
        
        elif self.hostname == "nuc":
            # enp45s0 gets time from the jetson mgbe0 port, hence enp45s0 is a client, not a master
            if "ptp4l_enp45s0" in service:
                health_msg.offset_mgbe0_enp45s0 = self.check_clock_offset(recent_line)
            elif "ptp4l_enp46s0" in service:
                health_msg.status_enp46s0_ptp4l = self.check_if_grandmaster(recent_line)
            elif "phc2sys_NIC" in service:
                health_msg.offset_enp45s0_enp46s0 = self.check_clock_offset(recent_line)
            elif "phc2sys_system" in service:
                health_msg.offset_enp45s0_systemclock =self.check_clock_offset(recent_line)
            else:
                rospy.logerr("[BoxStatus] This service is unknown on the " + self.hostname + ": " + str(service))
        else:
            rospy.logerr("[BoxStatus] This hostname is unknown: " + self.hostname)

        return health_msg
    
    def get_frequency_if_available(self, topic):
        if topic in self.finders:
            return self.finders[topic].find_frequency()
        else:
            return 0.0
    
    def get_topic_frequency(self, health_msg):
        health_msg.gt_box_alphasense_driver_node_cam3_hz = self.get_frequency_if_available("/gt_box/alphasense_driver_node/cam3")
        health_msg.gt_box_alphasense_driver_node_cam4_hz = self.get_frequency_if_available("/gt_box/alphasense_driver_node/cam4")
        health_msg.gt_box_alphasense_driver_node_cam5_hz = self.get_frequency_if_available("/gt_box/alphasense_driver_node/cam5")
        health_msg.gt_box_hesai_pandar_hz = self.get_frequency_if_available("/gt_box/hesai/pandar")
        health_msg.gt_box_livox_lidar_hz = self.get_frequency_if_available("/gt_box/livox/lidar")
        health_msg.gt_box_livox_imu_hz = self.get_frequency_if_available("/gt_box/livox/imu")
        health_msg.gt_box_alphasense_driver_node_imu_hz = self.get_frequency_if_available("/gt_box/alphasense_driver_node/imu")
        health_msg.gt_box_adis16475_hz = self.get_frequency_if_available("/gt_box/adis16475")
        health_msg.gt_box_usb_cam_image_hz = self.get_frequency_if_available("/gt_box/usb_cam/image")
        health_msg.gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz = self.get_frequency_if_available("/gt_box/rover/piksi/position_receiver_0/ros/pos_enu")
        return health_msg

    def get_GPS_status(self, health_msg):
        health_msg.gps_num_sat = self.gps_num_sat
        health_msg.gps_rtk_mode_fix = self.gps_rtk_mode_fix
        health_msg.gps_fix_mode = self.gps_fix_mode
        health_msg.gps_utc_time_ready = self.gps_utc_time_ready
        return health_msg

    def publish_health_status(self):

        while not rospy.is_shutdown():
            health_msg = healthStatus()

            health_msg = self.get_clock_offsets(health_msg)
            health_msg = self.get_topic_frequency(health_msg)
            if self.check_gps_status:
                health_msg = self.get_GPS_status(health_msg)
            #health_msg = self.get_PC_status(health_msg) -> cpu, power consumption, empty disk space, etc.

            self.health_status_publisher.publish(health_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        box_status = BoxStatus()
        box_status.publish_health_status()

    except rospy.ROSInitException:
        pass
