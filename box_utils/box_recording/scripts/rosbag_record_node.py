#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import subprocess
import socket
import rospkg
import psutil
import signal
import os
import rosparam
import shutil
import subprocess
import time
from pathlib import Path
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from box_recording.srv import StartRecordingInternalResponse, StartRecordingInternal
from box_recording.srv import StopRecordingInternalResponse, StopRecordingInternal, StopRecordingInternalRequest
from zed2i_recording_driver_msgs.srv import StartRecordingSVO, StartRecordingSVORequest
import time


class RosbagRecordNode(object):
    def __init__(self):
        # Get the host name of the machine
        self.node = socket.gethostname()
        rospy.init_node(f"rosbag_record_node_{self.node}")

        # Set up services
        servers = {}
        self.processes = []
        self.bag_base_path = None
        self.namespace = rospy.get_namespace()
        servers["start"] = rospy.Service("~start_recording", StartRecordingInternal, self.start_recording)
        servers["stop"] = rospy.Service("~stop_recording", StopRecordingInternal, self.stop_recording)

        self.bag_running = False
        self.recording_zed = False
        default_path = rospkg.RosPack().get_path("box_recording") + "/data"
        self.data_path = rospy.get_param("~data_path", default_path)

        self.pub_recording_status = rospy.Publisher("~recording_status", Bool, queue_size=3)
        self.pub_disk_space_free_in_gb = rospy.Publisher("~disk_space_free_in_gb", Float32, queue_size=3)

        if not os.path.exists(self.data_path):
            self.data_path = default_path
            rospy.logwarn(
                "[RosbagRecordNode("
                + self.node
                + ")] Data path "
                + self.data_path
                + " does not exist. Will write to "
                + default_path
            )

        rp = rospkg.RosPack()
        self.rosbag_recorder_bash_script = os.path.join(rp.get_path("box_recording"), "bin/record_bag.sh")
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Set up to record to " + self.data_path)

        rate = rospy.Rate(1)
        free_disk_space_in_gb = 0

        while rospy.is_shutdown() is False:
            if self.bag_running:
                p = self.bag_base_path
            else:
                p = self.data_path

            if os.path.exists(p):
                free_disk_space_in_gb = shutil.disk_usage(p)[2] / 1000000000
            else:
                rospy.logerr(f"[RosbagRecordNode({self.node})] Failed to check free disk space on {p}.")

            if free_disk_space_in_gb < 5 and self.bag_running:
                rospy.logerr(f"[RosbagRecordNode({self.node})] Reached low disk space. Stopping recording.")
                request = StopRecordingInternalRequest()
                self.stop_recording(request)
                time.sleep(30)
                exit - 1
            else:
                rospy.loginfo(f"[RosbagRecordNode({self.node})] Free disk space: " + str(free_disk_space_in_gb) + " GB in " + p + ".")

            self.pub_disk_space_free_in_gb.publish(free_disk_space_in_gb)
            self.pub_recording_status.publish(self.bag_running)

            rate.sleep()

    def terminate_process_and_children(self, p):
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()
        
    def terminate_process_inside_docker(self, process_name="ros2 bag"):
        docker_container_name = "isaac_ros_dev-aarch64-container-recording"
        # Command to find the PID of the "ros2 bag" process within the Docker container
        command_find_pid = f"docker exec {docker_container_name} bash -c 'ps aux | grep \"{process_name}\" | grep -v grep | awk \"{{print \\$2}}\"'"
        # Execute the command to find the PID
        proc = subprocess.Popen(command_find_pid, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = proc.communicate()
        pid = stdout.decode().strip()

        if pid:
            # Command to kill the process with the found PID in the Docker container
            command_kill_pid = f"docker exec {docker_container_name} bash -c 'kill {pid}'"

            # Execute the kill command
            kill_proc = subprocess.Popen(command_kill_pid, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            _, kill_stderr = kill_proc.communicate()

            if kill_proc.returncode == 0:
                rospy.loginfo(f"[RosbagRecordNode({self.node})] {process_name} Docker process killed successfully.")
            else:
                rospy.logerr(f"[RosbagRecordNode({self.node})] Failed to kill process {process_name}. Error: {kill_stderr.decode()}")
        else:
            rospy.loginfo(f"[RosbagRecordNode({self.node})] No PID found for process {process_name}. Error: {stderr.decode()}")
        
    def toggle_zed_recording(self, start, timestamp, response):
        service_name = self.namespace + '/zed2i_recording_driver/start_recording_svo'
        rospy.loginfo(f"[RosbagRecordNode({self.node} zed2i)] Trying to start svo recording process on zed2i")
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
        except rospy.ROSException as e:
            response.message += f"zed2i [FAILED] service not found, "
            response.suc = False
            return response
        
        try:
            start_recording_svo_srv = rospy.ServiceProxy(service_name, StartRecordingSVO)
            req = StartRecordingSVORequest()
            req.start_recording = start
            req.video_filename = self.bag_base_path + f"/{timestamp}_{self.node}_zed2i.svo2"

            start_recording_svo_srv(req)
            response.message += f"zed2i [SUC], "
            self.recording_zed = start
            rospy.loginfo(f"[RosbagRecordNode({self.node} zed2i)] {'Started' if start else 'Stopped'} svo recording process on zed2i")
        except rospy.ServiceException as e:
            response.suc = False
            response.message += f"zed2i [FAILED] Exception: " + str(e) + ", "
            rospy.logerr(f"Failed to {'start' if start else 'stop'} rosbag recording process on zed2i: {e}")
        return response

    def start_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to start rosbag recording process.")
        response = StartRecordingInternalResponse()
        timestamp = request.timestamp
        self.bag_base_path = os.path.join(self.data_path, timestamp)

        Path(self.bag_base_path).mkdir(parents=True, exist_ok=True)

        # Check if we're on lpc. If so, dump rosparams to yaml file.
        if self.node == "jetson":
            yaml_file_path = os.path.join(self.bag_base_path, f"{timestamp}_{self.node}.yaml")
            rosparam.dump_params(yaml_file_path, "/")

        topic_cfgs = request.topics.split(" ")
        print()
        bag_configs = {}
        for topic_cfg in topic_cfgs:
            print(topic_cfg.split("----"))
            bag_name = topic_cfg.split("----")[0]
            topic_name = topic_cfg.split("----")[1]
            if bag_name not in bag_configs:
                bag_configs[bag_name] = ""
            bag_configs[bag_name] += topic_name + " "

        self.bag_configs = bag_configs
        response.suc = True
        response.message = f"Starting rosbag recording process."
        
        for bag_name, topics in bag_configs.items():
            if bag_name == "zed2i" and "svo" in topics:
                # If we are recording svo files instead of rosbags for the zed, we need to call the svo recording service.
                response = self.toggle_zed_recording(True, timestamp, response)
                continue
            bag_path = os.path.join(self.bag_base_path, timestamp + "_" + self.node + "_" + bag_name)
            bash_command = (
                f"rosrun box_recording record_bag.sh {bag_path} {topics} __name:=record_{self.node}_{bag_name}"
            )
            # TODO: Replace with proper system after testing
            if bag_name == "hdr":
                docker_script_path = "/data/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/run_recording.sh"
                bash_command = f"{docker_script_path} -c \"start_recording {timestamp} {topics}\""

            self.processes.append(subprocess.Popen(bash_command, shell=True, stderr=subprocess.PIPE))
            self.bag_running = True
            
            response.message += f"{bag_name} [SUC], "
            
            rospy.loginfo(f"[RosbagRecordNode({self.node} {bag_name})] Starting rosbag recording process.")

        return response

    def stop_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to send SIGINT to recording process.")
        response = StopRecordingInternalResponse()
        response.result = ""

        if self.bag_running:
            response.suc = True
            response.message = "Sending SIGINT to recording process."
            rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Sent SIGINT to recording process.")
        else:
            response.suc = False
            response.message = "No recording process running yet."
            rospy.logwarn("[RosbagRecordNode(" + self.node + ")] No recording process running yet.")
            
        # First kill the recording process inside Docker. This ugliness is required due to https://github.com/moby/moby/issues/9098
        self.terminate_process_inside_docker()

        for p in self.processes:
            self.terminate_process_and_children(p)
        self.processes = []
        
        if self.recording_zed:
            response = self.toggle_zed_recording(False, "", response)

        if request.verbose:
            # output = subprocess.check_output([f"rosbag info --freq {self.bag_path}*.bag"], shell=True)
            response.result += ". Other output not implemented yet"  # str(output)[2:-1]

        self.bag_running = False
        return response


if __name__ == "__main__":
    rospy.loginfo("Starting ROS bag record node")
    RosbagRecordNode()
