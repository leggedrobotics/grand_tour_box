#!/usr/bin/env python3

import socket
import rospkg
from os.path import join
import yaml
import subprocess
import re

import rospy, rostopic
from box_health.msg import healthStatus_jetson, healthStatus_nuc
from piksi_rtk_msgs.msg import ReceiverState_V2_6_5

def load_yaml(path: str) -> dict:
    with open(path) as file:
        res = yaml.load(file, Loader=yaml.FullLoader)
    if res is None:
        rospy.logwarn("[BoxStatus] Yaml file " + path + " was empty.")
        res = {}
    return res

def last_line(text: str) -> str:
    idx = text.rfind('\n', 0, len(text) - 1)
    return text[idx+1:]

def offset_from_status(line: str) -> str:
    idx = line.find("offset")
    numbers_in_line = [int(d) for d in re.findall(r'-?\d+', line[idx:])]
    if numbers_in_line:
        offset = numbers_in_line[0]
        return str(offset)
    else:
        rospy.logerr("[BoxStatus] Error reading offset from line: " + line)
        return "error reading offset"
    
class FrequencyFinder:
    def __init__(self, topic):
        self.topic = topic
        self.rt = rostopic.ROSTopicHz(100)
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.rt.callback_hz, callback_args=self.topic)
        rospy.sleep(0.2)   

    def find_frequency(self):
        hz_status = self.rt.get_hz(self.topic)
        if hz_status:
            return hz_status[0]
        else:
            rospy.logdebug("[BoxStatus] Error reading frequency of " + self.topic)
            return 0.0

class BoxStatus:
    def __init__(self):
        self.hostname = socket.gethostname()
        self.namespace = rospy.get_namespace()

        rospy.init_node(f'health_status_publisher_{self.hostname}')

        rp = rospkg.RosPack()
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
            rospy.loginfo("[BoxStatus] Check GPS stats on host " + self.hostname)
            self.GPS_subscriber = rospy.Subscriber("/gt_box/rover/piksi/position_receiver_0/ros/receiver_state", ReceiverState_V2_6_5, self.set_GPS_status)
            self.gps_num_sat = 0
            self.gps_rtk_mode_fix = False
            self.gps_fix_mode = "uninitialized"
            self.gps_utc_time_ready = False

        if self.hostname == "jetson":
            self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/' + self.hostname, healthStatus_jetson, queue_size=2)
        elif self.hostname == "nuc":
            self.health_status_publisher = rospy.Publisher(self.namespace + 'health_status/' + self.hostname, healthStatus_nuc, queue_size=2)
        else:
            rospy.logerr("[BoxStatus] Hostname " + self.hostname + " is unknown.")
        self.rate = rospy.Rate(1.2)

    def set_GPS_status(self, data):
        self.gps_num_sat = data.num_sat
        self.gps_rtk_mode_fix = data.rtk_mode_fix
        self.gps_fix_mode = data.fix_mode
        self.gps_utc_time_ready = data.utc_time_ready
    
    def check_if_grandmaster(self, recent_line):
        if "assuming the grand master role" or "LISTENING to GRAND_MASTER on RS_GRAND_MASTER" in recent_line:
            return "grand master"
        else:
            return "not grand master"
        
    def check_clock_offset(self, recent_line):
        if "Waiting for ptp4l..." in recent_line:
            return "waiting for ptp4l"
        else:      
            return offset_from_status(recent_line)
        
    def read_clock_status(self, service):
        p = subprocess.Popen(["systemctl", "status",  service], stdout=subprocess.PIPE)
        (output_status, error) = p.communicate()
        if error:
            rospy.logerr("[BoxStatus] Error subprocess reading clocks: " + str(error))
        return last_line(output_status.decode('utf-8'))
    
    def check_clocks(self, health_msg):
        if self.hostname == "jetson":
            health_msg.status_mgbe0_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l_mgbe0.service"))
            health_msg.status_mgbe1_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l_mgbe1.service"))
            health_msg.offset_mgbe0_systemclock = self.check_clock_offset(self.read_clock_status("phc2sys_mgbe0.service"))               
            health_msg.offset_mgbe0_mgbe1 = self.check_clock_offset(self.read_clock_status("phc2sys_mgbe1.service"))
    
        elif self.hostname == "nuc":
            # enp45s0 gets time from the jetson mgbe0 port, hence enp45s0 is a client, not a master        
            health_msg.offset_mgbe0_enp45s0 = self.check_clock_offset(self.read_clock_status("ptp4l_enp45s0.service"))
            health_msg.status_enp46s0_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l_enp46s0.service"))
            health_msg.offset_enp45s0_enp46s0 = self.check_clock_offset(self.read_clock_status("phc2sys_NIC.service"))
            health_msg.offset_enp45s0_systemclock = self.check_clock_offset(self.read_clock_status("phc2sys_system.service"))
        else:
            rospy.logerr("[BoxStatus] This hostname is unknown: " + self.hostname)
        return health_msg
    
    def get_frequency_if_available(self, topic):
        if topic in self.finders:
            return self.finders[topic].find_frequency()
        else:
            return 0.0
    
    def get_topic_frequencies(self, health_msg):
        for topic in self.topics:
            setattr(health_msg, topic[1:].replace('/', '_') + "_hz", self.get_frequency_if_available(topic))
        return health_msg

    def get_GPS_status(self, health_msg):
        health_msg.gps_num_sat = self.gps_num_sat
        health_msg.gps_rtk_mode_fix = self.gps_rtk_mode_fix
        health_msg.gps_fix_mode = self.gps_fix_mode
        health_msg.gps_utc_time_ready = self.gps_utc_time_ready
        return health_msg
    
    def healthstatus(self):
        if self.hostname == "jetson":
            return healthStatus_jetson()
        elif self.hostname == "nuc":
            return healthStatus_nuc()
        else:
            rospy.logerr("[BoxStatus] Hostname " + self.hostname + " is unknown.")

    def get_PC_status(self, health_msg):
        process = subprocess.Popen("grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage}'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        if stderr:
            rospy.logerr(stderr)
        if stdout:
            setattr(health_msg, "cpu_usage_" + self.hostname, float(stdout))
        else:
            rospy.warn("[BoxStatus] CPU usage could not be determined. ")

        process = subprocess.Popen("df -H --output=avail ${HOME} | awk 'NR==2 {print $1}'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        if stderr:
            rospy.logerr(stderr)
        if stdout:
            setattr(health_msg, "avail_memory_" + self.hostname, stdout)
        else:
            rospy.warn("[BoxStatus] Available memory could not be determined. ")
        return health_msg    
      
    def publish_health_status(self):
        while not rospy.is_shutdown():
            health_msg = self.healthstatus()

            health_msg = self.check_clocks(health_msg)
            health_msg = self.get_topic_frequencies(health_msg)
            if self.check_gps_status:
                health_msg = self.get_GPS_status(health_msg)
            health_msg = self.get_PC_status(health_msg)
            self.health_status_publisher.publish(health_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        box_status = BoxStatus()
        box_status.publish_health_status()

    except rospy.ROSInitException:
        pass
