#!/usr/bin/env python3

import socket
import rospkg
from os.path import join

import rospy
from box_health.msg import healthStatus
from publish_local_health_status import load_yaml

class BoxStatusMerger:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rp = rospkg.RosPack()
        services_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_services.yaml")
        services = load_yaml(services_yaml)
        print(services)

        topics_yaml = join( str(rp.get_path('box_health')), "cfg/health_check_topics.yaml")
        topics = load_yaml(topics_yaml)

        self.external_hostnames = ['jetson', 'nuc']
        self.subscribers = []
        self.fields = { }

        for external_host in self.external_hostnames:
            topic = self.namespace + 'health_status/' + external_host
            sub = rospy.Subscriber(topic, healthStatus, self.callback, callback_args=external_host)
            self.subscribers.append(sub)

            self.fields[external_host] = topics[external_host] + services[external_host]


        rospy.init_node('health_status_merger')
        self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/merged' , healthStatus, queue_size=10)
        self.rate = rospy.Rate(1)
        
        self.complete_health_msg = healthStatus()




    def callback(self, partial_health_data, host):
        print("Host: ", host)
        print("Healthdata: ", partial_health_data)


        if host == "jetson": self.complete_health_msg.offset_mgbe0_systemclock = partial_health_data.offset_mgbe0_systemclock
        if host == "jetson": self.complete_health_msg.offset_mgbe0_mgbe1 = partial_health_data.offset_mgbe0_mgbe1
        if host == "jetson": self.complete_health_msg.status_mgbe0_ptp4l = partial_health_data.status_mgbe0_ptp4l
        if host == "jetson": self.complete_health_msg.status_mgbe1_ptp4l = partial_health_data.status_mgbe1_ptp4l

        if host == "nuc": self.complete_health_msg.offset_enp45s0_systemclock = partial_health_data.offset_enp45s0_systemclock
        if host == "nuc": self.complete_health_msg.offset_enp45s0_enp46s0 = partial_health_data.offset_enp45s0_enp46s0
        if host == "nuc": self.complete_health_msg.offset_mgbe0_enp45s0 = partial_health_data.offset_mgbe0_enp45s0
        if host == "nuc": self.complete_health_msg.status_enp46s0_ptp4l = partial_health_data.status_enp46s0_ptp4l


        if "/gt_box/alphasense_driver_node/cam3" in self.fields[host]: self.complete_health_msg.gt_box_alphasense_driver_node_cam3_hz =  partial_health_data.gt_box_alphasense_driver_node_cam3_hz
        if "/gt_box/alphasense_driver_node/cam4" in self.fields[host]: self.complete_health_msg.gt_box_alphasense_driver_node_cam4_hz =  partial_health_data.gt_box_alphasense_driver_node_cam4_hz
        if "/gt_box/alphasense_driver_node/cam5" in self.fields[host]: self.complete_health_msg.gt_box_alphasense_driver_node_cam5_hz =  partial_health_data.gt_box_alphasense_driver_node_cam5_hz
        if "/gt_box/hesai/pandar" in self.fields[host]: self.complete_health_msg.gt_box_hesai_pandar_hz =  partial_health_data.gt_box_hesai_pandar_hz
        if "/gt_box/livox/lidar" in self.fields[host]: self.complete_health_msg.gt_box_livox_lidar_hz =  partial_health_data.gt_box_livox_lidar_hz
        if "/gt_box/livox/imu" in self.fields[host]: self.complete_health_msg.gt_box_livox_imu_hz =  partial_health_data.gt_box_livox_imu_hz
        if "/gt_box/alphasense_driver_node/imu" in self.fields[host]: self.complete_health_msg.gt_box_alphasense_driver_node_imu_hz =  partial_health_data.gt_box_alphasense_driver_node_imu_hz
        if "/gt_box/adis16475" in self.fields[host]: self.complete_health_msg.gt_box_adis16475_hz =  partial_health_data.gt_box_adis16475_hz
        
        if "/gt_box/usb_cam/image" in self.fields[host]: self.complete_health_msg.gt_box_usb_cam_image_hz =  partial_health_data.gt_box_usb_cam_image_hz
        if "/gt_box/rover/piksi/position_receiver_0/ros/pos_enu" in self.fields[host]: self.complete_health_msg.gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz =  partial_health_data.gt_box_rover_piksi_position_receiver_0_ros_pos_enu_hz

        if "/gt_box/rover/piksi/position_receiver_0/ros/pos_enu" in self.fields[host]: self.complete_health_msg.gps_num_sat =  partial_health_data.gps_num_sat
        if "/gt_box/rover/piksi/position_receiver_0/ros/pos_enu" in self.fields[host]: self.complete_health_msg.gps_rtk_mode_fix =  partial_health_data.gps_rtk_mode_fix
        if "/gt_box/rover/piksi/position_receiver_0/ros/pos_enu" in self.fields[host]: self.complete_health_msg.gps_fix_mode =  partial_health_data.gps_fix_mode    
        if "/gt_box/rover/piksi/position_receiver_0/ros/pos_enu" in self.fields[host]: self.complete_health_msg.gps_utc_time_ready =  partial_health_data.gps_utc_time_ready              
    
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
