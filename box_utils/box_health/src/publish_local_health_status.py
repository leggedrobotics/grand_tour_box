#!/usr/bin/env python3

import socket
import rospkg
from os.path import join
import yaml
import subprocess
import re

import rostopic
from std_msgs.msg import String
import box_health.msg as box_health_msg
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
from datetime import datetime
from pathlib import Path
import os
import rospy
import rosnode
from subprocess import Popen, PIPE


def is_node_alive(node_name):
    try:
        nodes = rosnode.get_node_names()
        return node_name in nodes
    except rosnode.ROSNodeIOException:
        print("failed")
        return False


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
        idx = line.find("rms")
        numbers_in_line = [int(d) for d in re.findall(r"-?\d+", line[idx:])]
        if numbers_in_line:
            offset = numbers_in_line[0]
            return str(offset)

    rospy.logerr("[BoxStatus] Error reading offset from line: " + line)
    return "error_reading_offset"


def get_chronyc_tracking_info():
    try:
        # Run the `chronyc tracking` command and capture its output
        result = subprocess.run(["chronyc", "tracking"], capture_output=True, text=True, check=True)
        output = result.stdout

        # Parse the output to find the status and last offset
        tracking_successful = False
        last_offset = None

        for line in output.split("\n"):
            if "Leap status" in line and "Normal" in line:
                tracking_successful = True
            if "Last offset" in line:
                last_offset = line.split(":")[1].strip()

        return tracking_successful, last_offset

    except subprocess.CalledProcessError as e:
        print(f"An error occurred while running `chronyc tracking`: {e}")
        return None, None


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


def get_file_size(file_path):
    """Return the size of the file at the given path."""
    si = 0
    n = file_path.split("/")[-1]
    ls = [str(s) for s in Path(file_path).parent.rglob(f"{n}*.bag*")]

    for p in ls:
        si += os.path.getsize(p)

    active = len([str(s) for s in Path(file_path).parent.rglob(f"{n}*.bag.active")]) != 0
    suc = si != 0
    return si, suc, active


def add_color(text, rgb):

    return f'<span style="color: rgb({rgb[0]},{rgb[1]},{rgb[2]});">' + text + "</span>\n"


class PiReader:
    def __init__(self, services):
        self.files = {}
        self.k = {}
        for service in services:
            self.files[service] = open(os.path.join("/tmp", service + ".log"), "r+")
            self.k[service] = 0

    def read_clock_status(self, service):
        # Move the pointer to the beginning of the file and read all lines
        self.k[service] += 1
        if self.k[service] == 100:
            # remove content from file
            self.files[service].seek(0)
            self.files[service].truncate()

        try:
            f = f"/tmp/{service}.log"
            p = Popen(["tail", "-1", f], shell=False, stderr=PIPE, stdout=PIPE)
            res, err = p.communicate()

            res = str(res.decode("utf-8"))
            return res
        except:
            rospy.logerr(f"[BoxStatus] Error reading clock status: {service}")
            return "No data"


class BoxStatus:
    def __init__(self):
        self.hostname = socket.gethostname()

        self.hostname = self.hostname.replace("anymal-d039-", "")
        self.namespace = rospy.get_namespace()

        rospy.init_node(f"health_status_publisher_{self.hostname}")

        rp = rospkg.RosPack()
        cfg_file = join(str(rp.get_path("box_health")), "cfg/health_check_topics.yaml")

        pretty = self.hostname.capitalize()
        self.msg = getattr(box_health_msg, f"HealthStatus{pretty}")
        self.cfg = load_yaml(cfg_file)[self.hostname]

        self.finders = {}
        for topic in [k for k in self.cfg["topics"].keys()]:
            finder = FrequencyFinder(topic)
            self.finders[topic] = finder

        self.health_status_publisher = rospy.Publisher(
            self.namespace + "health_status/" + self.hostname + "/status",
            self.msg,
            queue_size=2,
        )
        self.overlay_text_publisher = rospy.Publisher(
            self.namespace + "health_status/" + self.hostname + "/overlay_text",
            OverlayText,
            queue_size=2,
        )

        if self.cfg["recording"]:
            self.recording_strings = ""
            self.recording_lines = 0
            self.subs = rospy.Subscriber(
                f"/gt_box/rosbag_record_node_{self.hostname}/recording_info", String, self.recording_info_callback
            )

        if self.namespace.find("pi"):
            self.pi_reader = PiReader(self.cfg.get("ptp4l", []) + self.cfg.get("phc2sys", []))
            self.read_clock_status = self.read_clock_status_pi

        self.rate = rospy.Rate(1.2)

    def recording_info_callback(self, msg):
        self.recording_strings = ""
        self.recording_lines = 0
        for bag in msg.data.split(","):
            o = bag.split("----")

            if len(o) != 2:
                continue

            recorder_node_name = o[0]
            recorder_path = o[1]
            size, suc, active = get_file_size(recorder_path)
            size_mb = size / 1e6
            if suc:
                if size / 1e3 < 5:
                    self.recording_strings += add_color(
                        f"{recorder_node_name}: {size_mb}MB - Bag Exists but very small", (255, 165, 0)
                    )
                else:
                    self.recording_strings += add_color(f"{recorder_node_name}: {size_mb}MB", (0, 255, 0))
            else:
                self.recording_strings += add_color(f"{recorder_node_name}: Failed - File not found", (255, 0, 0))

            self.recording_lines += 1

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

    def read_clock_status_pi(self, service):
        return self.pi_reader.read_clock_status(service.replace(".service", ""))

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

    def check_services(self, health_msg):
        for ptp4l_service in self.cfg.get("ptp4l", []):
            setattr(
                health_msg,
                f"offset_{ptp4l_service}",
                self.check_clock_offset(self.read_clock_status(f"{ptp4l_service}.service")),
            )

        for phc2sys_service in self.cfg.get("phc2sys", []):
            print(self.read_clock_status(f"{phc2sys_service}.service"))
            setattr(
                health_msg,
                f"offset_{phc2sys_service}",
                self.check_clock_offset(self.read_clock_status(f"{phc2sys_service}.service")),
            )
        return health_msg

    def get_frequency_if_available(self, topic):
        if topic in self.finders:
            return self.finders[topic].find_frequency()
        else:
            return 0.0

    def get_topic_frequencies(self, health_msg):
        for topic in self.cfg["topics"]:
            setattr(
                health_msg,
                topic[1:].replace("/", "_") + "_hz",
                self.get_frequency_if_available(topic),
            )
        return health_msg

    def get_pc_status(self, health_msg):
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
                    "cpu_usage",
                    float(stdout.decode().replace(",", ".")),
                )
            else:
                setattr(health_msg, "cpu_usage", -1.0)
                rospy.logerr("[BoxStatus] CPU usage could not be determined. ")
        except:
            setattr(health_msg, "cpu_usage", -1.0)
            rospy.logerr("[BoxStatus] CPU usage could not be determined. ")
            rospy.logerr(stdout.decode())

        try:
            monitor_dir = self.cfg["memory"]
            process = subprocess.Popen(
                f"df -H --output=avail {monitor_dir}" + " | awk 'NR==2 {print $1}'",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = process.communicate()
            if stderr:
                rospy.logerr(stderr)
            if stdout:
                avail_memory = stdout.decode("utf-8").strip()
                setattr(health_msg, "avail_memory", avail_memory)
            else:
                setattr(health_msg, "avail_memory", "unknown")
                rospy.logerr("[BoxStatus] Available memory could not be determined. ")
        except:
            setattr(health_msg, "avail_memory", "unknown")
            rospy.logerr("[BoxStatus] Available memory could not be determined. ")
        return health_msg

    def publish_overlay_text(self, health_msg):
        text = OverlayText()
        fsk = self.cfg["fsk"]["text"]
        text.width = fsk["width"]
        text.height = fsk["height"]
        text.left = fsk["left"]
        text.top = fsk["top"]
        text.text_size = fsk["text_size"]
        text.line_width = 1
        text.font = "FreeMono"
        offset = fsk["height"]

        lineheight = 14
        text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.8)
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

        text.text = '<span style="color: rgb(0,255,0);">' + self.hostname.capitalize() + "</span>\n"
        text.text += f"""Time: {dt_string}
            CPU: {health_msg.cpu_usage} /%
            Storage: {health_msg.avail_memory}
            """
        offset += lineheight * 3

        sync_ele = self.cfg.get("ptp4l", []) + self.cfg.get("phc2sys", [])
        for service in sync_ele:
            val = getattr(health_msg, f"offset_{service}")
            text.text += f"{service}:" + f"{val}ns\n"
            offset += lineheight

        if self.cfg.get("chrony", False):
            tracking_successful, last_offset = get_chronyc_tracking_info()

            if tracking_successful is not None:
                if tracking_successful:
                    text.text += add_color(f"Chrony is tracking. Last offset: {last_offset}", (0, 255, 0))
                else:
                    text.text += add_color("Chrony is not tracking successfully.", (255, 0, 0))
            else:
                text.text += add_color("Failed to retrieve tracking information.", (255, 0, 0))
            offset += lineheight

        added = False
        for k, v in self.cfg["topics"].items():
            val = getattr(health_msg, k[1:].replace("/", "_") + "_hz")
            if abs(val - v["rate"]) > 0.01 * v["rate"]:
                if not added:
                    added = True
                    text.text += '\n<span style="color: rgb(0,255,0);">' + "Topics:" + "</span>\n"
                    offset += lineheight * 2

                t = f"{k}: {round(val,2)}Hz != {v['rate']}Hz\n"
                t = '<span style="color: rgb(209,134,0);">' + str(t) + "</span>"
                text.text += t
                offset += lineheight

        if self.cfg.get("recording", False):
            text.text += '\n<span style="color: rgb(0,255,0);">' + "Recording:" + "</span>\n"
            if self.hostname == "jetson":
                for pc in ["jetson", "nuc"]:
                    ready = is_node_alive(f"/gt_box/rosbag_record_node_{pc}")
                    pretty = pc.capitalize()
                    text.text += f"{pretty} ready: {ready}\n"
                    offset += lineheight
            text.text += self.recording_strings
            offset += self.recording_lines * lineheight
            offset += lineheight * 2

        text.height = offset
        self.overlay_text_publisher.publish(text)

    def publish_health_status(self):
        while not rospy.is_shutdown():
            health_msg = self.msg()
            health_msg = self.check_services(health_msg)
            health_msg = self.get_topic_frequencies(health_msg)
            health_msg = self.get_pc_status(health_msg)
            self.health_status_publisher.publish(health_msg)
            self.publish_overlay_text(health_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        box_status = BoxStatus()
        box_status.publish_health_status()

    except rospy.ROSInitException:
        pass
