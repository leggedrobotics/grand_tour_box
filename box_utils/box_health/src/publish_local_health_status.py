#!/usr/bin/env python3

import socket
import rospkg
from os.path import join
import yaml
import subprocess
import re

import rospy
import rostopic
from std_msgs.msg import String
from box_health.msg import (
    healthStatus_jetson,
    healthStatus_nuc,
    healthStatus_opc,
    healthStatus_pi,
)


def load_yaml(path: str) -> dict:
    with open(path) as file:
        res = yaml.load(file, Loader=yaml.FullLoader)
    if res is None:
        rospy.logwarn("[BoxStatus] Yaml file " + path + " was empty.")
        res = {}
    return res


def last_line(text: str) -> str:
    idx = text.rfind("\n", 0, len(text) - 1)
    return text[idx + 1 :]


def chrony_line(text: str) -> str:
    jetson_idx = text.find("jetson")
    line_start_idx = text.rfind("\n", 0, jetson_idx)
    line_end_idx = text.find("\n", jetson_idx, len(text))
    return text[line_start_idx + 1 : line_end_idx]


def offset_from_status(line: str) -> str:
    idx = line.find("offset")
    numbers_in_line = [int(d) for d in re.findall(r"-?\d+", line[idx:])]
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

        rospy.init_node(f"health_status_publisher_{self.hostname}")

        rp = rospkg.RosPack()
        topics_yaml = join(str(rp.get_path("box_health")), "cfg/health_check_topics.yaml")
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
            from piksi_rtk_msgs.msg import ReceiverState_V2_6_5

            rospy.loginfo("[BoxStatus] Check GPS stats on host " + self.hostname)
            self.GPS_subscriber = rospy.Subscriber(
                "/gt_box/rover/piksi/position_receiver_0/ros/receiver_state",
                ReceiverState_V2_6_5,
                self.set_GPS_status,
            )
            self.set_GPS_status_default()

        self.check_alphasense_ptp = "alphasense" in "".join(self.topics)
        if self.check_alphasense_ptp:
            rospy.loginfo("[BoxStatus] Check Alphasense PTP status on host " + self.hostname)
            self.alphasense_subscriber = rospy.Subscriber(
                "/gt_box/alphasense_driver_node/debug_info",
                String,
                self.set_alphasense_ptp_status,
            )
            self.set_alphasense_ptp_status_default()

        if self.hostname == "jetson":
            self.health_status_publisher = rospy.Publisher(
                self.namespace + "health_status/" + self.hostname,
                healthStatus_jetson,
                queue_size=2,
            )
        elif self.hostname == "nuc":
            self.health_status_publisher = rospy.Publisher(
                self.namespace + "health_status/" + self.hostname,
                healthStatus_nuc,
                queue_size=2,
            )
        elif self.hostname == "opc":
            self.health_status_publisher = rospy.Publisher(
                self.namespace + "health_status/" + self.hostname,
                healthStatus_opc,
                queue_size=2,
            )
        elif self.hostname == "pi":
            self.health_status_publisher = rospy.Publisher(
                self.namespace + "health_status/" + self.hostname,
                healthStatus_pi,
                queue_size=2,
            )
        else:
            rospy.logerr("[BoxStatus] Hostname " + self.hostname + " is unknown.")
        self.rate = rospy.Rate(1.2)

    def set_GPS_status_default(self):
        self.gps_num_sat = 0
        self.gps_rtk_mode_fix = False
        self.gps_fix_mode = "unknown"
        self.gps_utc_time_ready = False

    def set_GPS_status(self, data):
        self.gps_num_sat = data.num_sat
        self.gps_rtk_mode_fix = data.rtk_mode_fix
        self.gps_fix_mode = data.fix_mode
        self.gps_utc_time_ready = data.utc_time_ready

    def set_alphasense_ptp_status_default(self):
        self.alphasense_frames_no_ptp = -1

    def set_alphasense_ptp_status(self, data):
        string = data.data
        idx = string.find("uptime")
        string = string[idx + 8 :]
        idx_minus = string.find("\n")
        uptime = int(string[:idx_minus])

        idx = string.find("last_ptp_sync_uptime")
        string = string[idx + 22 :]
        idx_minus = string.find("\n")
        last_ptp_sync_uptime = int(string[:idx_minus])

        frames_not_synced = uptime - last_ptp_sync_uptime
        self.alphasense_frames_no_ptp = frames_not_synced

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

    def check_chrony_offset(self, recent_line):
        offset_idx_start = recent_line.find("[") + 1
        offset_idx_end = recent_line.find("]")
        offset = recent_line[offset_idx_start:offset_idx_end].strip()
        unit_idx = offset.find(next(filter(str.isalpha, offset)))
        offset_num = int(offset[:unit_idx])
        unit = offset[unit_idx:]
        if unit == "ns":
            pass
        elif unit == "us":
            offset_num *= 1e3
        elif unit == "ms":
            offset_num *= 1e6
        elif unit == "s":
            offset_num *= 1e9
        else:
            rospy.logerr("[BoxStatus] Unknown unit:", unit)
        return str(int(offset_num))

    def get_chrony_status(self, recent_line):
        states = {
            "*": "current synced",
            "+": "combined",
            "-": "not combined",
            "?": "unreachable",
            "x": "time may be in error",
            "~": "time too variable",
        }
        state_symbol = recent_line[1]
        return states[state_symbol]

    def read_clock_status(self, service):
        p = subprocess.Popen(["systemctl", "status", service], stdout=subprocess.PIPE)
        (output_status, error) = p.communicate()
        if error:
            rospy.logerr("[BoxStatus] Error subprocess reading clocks: " + str(error))
        return last_line(output_status.decode("utf-8"))

    def read_chrony_status(self):
        p = subprocess.Popen(["chronyc", "sources"], stdout=subprocess.PIPE)
        (output_status, error) = p.communicate()
        if error:
            rospy.logerr("[BoxStatus] Error subprocess reading clocks: " + str(error))
        return chrony_line(output_status.decode("utf-8"))

    def check_clocks(self, health_msg):
        try:
            if self.hostname == "jetson":
                health_msg.status_mgbe0_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l_mgbe0.service"))
                health_msg.status_mgbe1_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l_mgbe1.service"))
                health_msg.offset_mgbe0_systemclock = self.check_clock_offset(
                    self.read_clock_status("phc2sys_mgbe0.service")
                )
                health_msg.offset_mgbe0_mgbe1 = self.check_clock_offset(self.read_clock_status("phc2sys_mgbe1.service"))

            elif self.hostname == "nuc":
                # enp45s0 gets time from the jetson mgbe0 port, hence enp45s0 is a client, not a master
                health_msg.offset_mgbe0_enp45s0 = self.check_clock_offset(
                    self.read_clock_status("ptp4l_enp45s0.service")
                )
                health_msg.status_enp46s0_ptp4l = self.check_if_grandmaster(
                    self.read_clock_status("ptp4l_enp46s0.service")
                )
                health_msg.offset_enp45s0_enp46s0 = self.check_clock_offset(
                    self.read_clock_status("phc2sys_NIC.service")
                )
                health_msg.offset_enp45s0_systemclock = self.check_clock_offset(
                    self.read_clock_status("phc2sys_system.service")
                )
            elif self.hostname == "opc":
                try:
                    chrony_line = self.read_chrony_status()
                    health_msg.offset_chrony_opc_jetson = self.check_chrony_offset(chrony_line)
                    health_msg.chrony_status = self.get_chrony_status(chrony_line)
                except:
                    health_msg.offset_chrony_opc_jetson = "-1"
                    health_msg.chrony_status = "error reading status"
            elif self.hostname == "pi":
                health_msg.offset_mgbe0_eth0 = self.check_clock_offset(self.read_clock_status("ptp4l.service"))
                health_msg.status_eth0_ptp4l = self.check_if_grandmaster(self.read_clock_status("ptp4l.service"))
                health_msg.offset_eth0_systemclock = self.check_clock_offset(self.read_clock_status("phc2sys.service"))
            else:
                rospy.logerr("[BoxStatus] This hostname is unknown: " + self.hostname)
        except Exception as error:
            rospy.logerr("[BoxStatus] Error reading clock offset")
            rospy.logerr("[BoxStatus]", error)
        return health_msg

    def get_frequency_if_available(self, topic):
        if topic in self.finders:
            return self.finders[topic].find_frequency()
        else:
            return 0.0

    def get_topic_frequencies(self, health_msg):
        for topic in self.topics:
            setattr(
                health_msg,
                topic[1:].replace("/", "_") + "_hz",
                self.get_frequency_if_available(topic),
            )
        return health_msg

    def get_GPS_status(self, health_msg):
        health_msg.gps_num_sat = self.gps_num_sat
        health_msg.gps_rtk_mode_fix = self.gps_rtk_mode_fix
        health_msg.gps_fix_mode = self.gps_fix_mode
        health_msg.gps_utc_time_ready = self.gps_utc_time_ready
        self.set_GPS_status_default()
        return health_msg

    def get_alphasense_ptp_status(self, health_msg):
        health_msg.alphasense_frames_no_ptp = self.alphasense_frames_no_ptp
        return health_msg

    def healthstatus(self):
        if self.hostname == "jetson":
            return healthStatus_jetson()
        elif self.hostname == "nuc":
            return healthStatus_nuc()
        elif self.hostname == "opc":
            return healthStatus_opc()
        elif self.hostname == "pi":
            return healthStatus_pi()
        else:
            rospy.logerr("[BoxStatus] Hostname " + self.hostname + " is unknown.")

    def get_PC_status(self, health_msg):
        process = subprocess.Popen(
            "grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage}'",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            stdout, stderr = process.communicate()
            if stderr:
                rospy.logerr(stderr)
            if stdout:
                setattr(
                    health_msg,
                    "cpu_usage_" + self.hostname,
                    float(stdout.decode().replace(",", ".")),
                )
            else:
                setattr(health_msg, "cpu_usage_" + self.hostname, -1.0)
                rospy.logerr("[BoxStatus] CPU usage could not be determined. ")
        except:
            setattr(health_msg, "cpu_usage_" + self.hostname, -1.0)
            rospy.logerr("[BoxStatus] CPU usage could not be determined. ")
            rospy.logerr(stdout.decode())

        try:
            process = subprocess.Popen(
                "df -H --output=avail ${HOME} | awk 'NR==2 {print $1}'",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = process.communicate()
            if stderr:
                rospy.logerr(stderr)
            if stdout:
                avail_memory = stdout.decode("utf-8").strip()
                setattr(health_msg, "avail_memory_" + self.hostname, avail_memory)
            else:
                setattr(health_msg, "avail_memory_" + self.hostname, "unknown")
                rospy.logerr("[BoxStatus] Available memory could not be determined. ")
        except:
            setattr(health_msg, "avail_memory_" + self.hostname, "unknown")
            rospy.logerr("[BoxStatus] Available memory could not be determined. ")
        return health_msg

    def publish_health_status(self):
        while not rospy.is_shutdown():
            health_msg = self.healthstatus()

            health_msg = self.check_clocks(health_msg)
            health_msg = self.get_topic_frequencies(health_msg)
            if self.check_gps_status:
                health_msg = self.get_GPS_status(health_msg)
            if self.check_alphasense_ptp:
                health_msg = self.get_alphasense_ptp_status(health_msg)
            health_msg = self.get_PC_status(health_msg)
            self.health_status_publisher.publish(health_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        box_status = BoxStatus()
        box_status.publish_health_status()

    except rospy.ROSInitException:
        pass
