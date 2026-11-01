#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import subprocess
import socket
import rospkg
import signal
import os
import rosparam
import shutil
from pathlib import Path
from std_msgs.msg import Float32, String, Bool
from std_srvs.srv import Trigger
from box_recording.srv import StartRecordingInternalResponse, StartRecordingInternal
from box_recording.srv import StopRecordingInternalResponse, StopRecordingInternal, StopRecordingInternalRequest
from zed2i_recording_driver_msgs.srv import StartRecordingSVO, StartRecordingSVORequest
from box_recording_helper.cpt7_helper import cpt7_start_recording, cpt7_stop_recording
from box_recording_helper.store_debug_logs_to_folder import store_debug_logs_to_folder
from box_recording_helper.recording_directory import prepare_recording_directory
from box_recording_helper.recording_names import recording_node_name
import time
import threading


RUN_RECORDING = "/data/workspaces/ros2_ws/src/isaac_ros_common/scripts/run_recording.sh"
IMAGE_NAME = "isaac_ros_dev-aarch64:grand_tour_box"


def start_hdr():
    subprocess.Popen(
        f"{RUN_RECORDING} hdr_start",
        shell=True,
        stderr=subprocess.PIPE,
    )


class RosbagRecordNode(object):
    def __init__(self):
        # Get the host name of the machine
        self.node = socket.gethostname()
        self.node = self.node.replace("anymal-d039-", "")
        self.node.replace("anymal-d039-", "")
        rospy.init_node(recording_node_name(self.node))

        # Set up services
        servers = {}
        self.processes = []
        self.bag_base_path = None
        self.namespace = rospy.get_namespace()
        self.store_debug_logs = rospy.get_param("~store_debug_logs", False)
        self._recording_lock = threading.RLock()

        self.bag_running = False
        self.recording_zed = False
        self.zed_recording_error = ""
        self.recording_cpt7 = False
        self.recording_hdr = False
        default_path = "/data"
        self.data_path = rospy.get_param("~data_path", default_path)

        self.pub_recording_status = rospy.Publisher("~recording_status", Bool, queue_size=3)

        self.info_string = ""
        self.pub_recording_info = rospy.Publisher("~recording_info", String, queue_size=3)

        self.pub_disk_space_free_in_gb = rospy.Publisher("~disk_space_free_in_gb", Float32, queue_size=3)

        self.data_path = prepare_recording_directory(self.data_path)

        rp = rospkg.RosPack()
        self.rosbag_recorder_bash_script = os.path.join(rp.get_path("box_recording"), "bin/record_bag.sh")
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Set up to record to " + self.data_path)
        servers["start"] = rospy.Service("~start_recording", StartRecordingInternal, self.start_recording)
        servers["stop"] = rospy.Service("~stop_recording", StopRecordingInternal, self.stop_recording)

        rate = rospy.Rate(1)
        free_disk_space_in_gb = 0
        msg = String()

        if self.node == "jetson":
            start_hdr()

        while rospy.is_shutdown() is False:
            if self.recording_zed:
                with self._recording_lock:
                    if self.recording_zed:
                        self.check_zed_recording()

            if self.bag_running:
                p = self.bag_base_path
            else:
                p = self.data_path

            self.p = p
            if os.path.exists(p):
                free_disk_space_in_gb = shutil.disk_usage(p)[2] / 1000000000
            else:
                rospy.logerr(f"[RosbagRecordNode({self.node})] Failed to check free disk space on {p}.")

            if free_disk_space_in_gb < 5 and self.bag_running:
                rospy.logerr(f"[RosbagRecordNode({self.node})] Reached low disk space. Stopping recording.")
                request = StopRecordingInternalRequest()
                self.stop_recording(request)
                time.sleep(30)
                rospy.signal_shutdown("Recording stopped: low disk space")
                break
            else:
                rospy.loginfo(
                    f"[RosbagRecordNode({self.node})] Free disk space: "
                    + str(free_disk_space_in_gb)
                    + " GB in "
                    + p
                    + "."
                )

            self.pub_disk_space_free_in_gb.publish(free_disk_space_in_gb)
            # A running rosbag process does not mean the accompanying SVO is healthy.
            self.pub_recording_status.publish(self.bag_running and not self.zed_recording_error)

            msg.data = self.info_string
            if self.zed_recording_error:
                msg.data += " ZED FAILED: " + self.zed_recording_error
            self.pub_recording_info.publish(msg)

            rate.sleep()

    def stop_bag_process(self, process):
        # record_bag.sh execs the writer; wait for its index flush before ACK.
        process.send_signal(signal.SIGINT)
        return process.wait()

    def terminate_process_inside_docker(self, process_name="ros2 bag"):
        docker_container_name = "isaac_ros_dev-aarch64-container-recording"
        # Command to find the PID of the "ros2 bag" process within the Docker container
        command_find_pid = f'docker exec {docker_container_name} bash -c \'ps aux | grep "{process_name}" | grep -v grep | awk "{{print \\$2}}"\''
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
                rospy.logerr(
                    f"[RosbagRecordNode({self.node})] Failed to kill process {process_name}. Error: {kill_stderr.decode()}"
                )
        else:
            rospy.loginfo(
                f"[RosbagRecordNode({self.node})] No PID found for process {process_name}. Error: {stderr.decode()}"
            )

    def toggle_zed_recording(self, start, timestamp, response):
        service_name = self.namespace + "/zed2i_recording_driver/start_recording_svo"
        rospy.loginfo(f"[RosbagRecordNode({self.node} zed2i)] Trying to start svo recording process on zed2i")
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
        except rospy.ROSException as e:
            response.message += f"zed2i FAILED Service not found: {e}"
            response.suc = False
            self.zed_recording_error = str(e)
            return response

        try:
            start_recording_svo_srv = rospy.ServiceProxy(service_name, StartRecordingSVO)
            req = StartRecordingSVORequest()
            req.start_recording = start
            req.video_filename = self.bag_base_path + f"/{timestamp}_{self.node}_zed2i.svo2"

            result = start_recording_svo_srv(req)
            if not result.success:
                response.suc = False
                self.zed_recording_error = "Driver reported failure during " + ("start" if start else "recording/stop")
                if not start:
                    # The file may already be closed after a write error. Query
                    # actual state so a failed recording does not block a new one.
                    self.check_zed_recording()
                response.message += "zed2i FAILED " + self.zed_recording_error + " "
                return response
            response.message += "zed2i OK "
            self.recording_zed = start
            if start:
                self.zed_recording_error = ""
            rospy.loginfo(
                f"[RosbagRecordNode({self.node} zed2i)] {'Started' if start else 'Stopped'} svo recording process on zed2i"
            )
        except rospy.ServiceException as e:
            response.suc = False
            self.zed_recording_error = str(e)
            response.message += f"zed2i FAILED Exception: {e} "
            rospy.logerr(f"Failed to {'start' if start else 'stop'} rosbag recording process on zed2i: {e}")
        return response

    def check_zed_recording(self):
        """Propagate failures after start while preserving the other sensor recordings."""
        service_name = self.namespace + "/zed2i_recording_driver/get_recording_status"
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
            status = rospy.ServiceProxy(service_name, Trigger)()
            if not status.success:
                self.zed_recording_error = status.message or "SVO recording stopped unexpectedly"
                self.recording_zed = False
            elif not self.zed_recording_error:
                return
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.zed_recording_error = "Cannot verify SVO recording: " + str(error)
        # Latch the error until a new recording. A temporary recovery must not hide
        # an unverified interval in the captured data.
        rospy.logerr_throttle(5.0, self.zed_recording_error)

    def start_recording(self, request):
        with self._recording_lock:
            return self._start_recording(request)

    def _start_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to start rosbag recording process.")
        response = StartRecordingInternalResponse()
        if self.bag_running:
            response.suc = False
            response.message = "Recording already running; stop it before starting another session"
            return response
        self.zed_recording_error = ""
        self.start_recording_time = rospy.Time.now()
        timestamp = request.timestamp
        self.bag_base_path = os.path.join(self.data_path, timestamp)

        free_disk_space_in_gb = shutil.disk_usage(self.data_path)[2] / 1000000000
        if free_disk_space_in_gb < 1:
            response.suc = False
            response.message = f"No disk space available. Do not start recording on {self.node}"
            return response

        Path(self.bag_base_path).mkdir(parents=True, exist_ok=True)

        # Check if we're on lpc. If so, dump rosparams to yaml file.
        if self.node == "jetson" or self.node.find("lpc") != -1:
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
        response.message = "Starting rosbag recording process."

        for bag_name, topics in bag_configs.items():
            self.bag_running = True

            if bag_name == "cpt7_local" and "cpt7_local" in topics:
                suc = cpt7_start_recording()
                if suc:
                    response.message += "cpt7_local [SUC], "
                else:
                    response.message += "cpt7_local [FAILED], "
                    response.suc = False

                self.recording_cpt7 = True
                continue

            if bag_name == "zed2i" and "svo" in topics:
                # If we are recording svo files instead of rosbags for the zed, we need to call the svo recording service.
                response = self.toggle_zed_recording(True, timestamp, response)
                if self.recording_zed:
                    bag_path = os.path.join(self.bag_base_path, timestamp + "_" + self.node + "_" + bag_name + ".svo2")
                    self.info_string += f"record_{self.node}_{bag_name}----{bag_path},"
                continue

            # TODO: Replace with proper system after testing
            if bag_name == "hdr":
                bash_command_hdr = f"{RUN_RECORDING} start_recording {timestamp} {topics}"
                self.recording_hdr = True
                rospy.loginfo(f"[RosbagRecordNode({self.node} HDR" + bash_command_hdr)
                subprocess.Popen(bash_command_hdr, shell=True, stderr=subprocess.PIPE)
                response.message += "hdr_ros2_started [SUC], "
                for bag_name in ["hdr_left", "hdr_right", "hdr_front"]:
                    bag_path = os.path.join(
                        self.bag_base_path,
                        f"{timestamp}_{self.node}_{bag_name}",
                        f"{timestamp}_{self.node}_{bag_name}_0.mcap",
                    )
                    self.info_string += f"record_{self.node}_{bag_name}----{bag_path},"
                continue

            bag_path = os.path.join(self.bag_base_path, timestamp + "_" + self.node + "_" + bag_name)
            # Use the resolved package script and argument boundaries on every host.
            bash_command = [
                "bash",
                self.rosbag_recorder_bash_script,
                bag_path,
                *topics.split(),
                "__name:=" + recording_node_name(self.node, bag_name),
            ]
            self.info_string += f"record_{self.node}_{bag_name}----{bag_path},"

            self.processes.append(subprocess.Popen(bash_command))

            response.message += f"{bag_name} [SUC], "
            rospy.loginfo(f"[RosbagRecordNode({self.node} {bag_name})] {bash_command}.")

            rospy.loginfo(f"[RosbagRecordNode({self.node} {bag_name})] Starting rosbag recording process.")

        return response

    def stop_recording(self, request):
        with self._recording_lock:
            return self._stop_recording(request)

    def _stop_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to send SIGINT to recording process.")
        self.info_string = ""
        response = StopRecordingInternalResponse()
        response.result = ""

        if self.store_debug_logs and self.bag_base_path:
            store_debug_logs_to_folder(
                self.start_recording_time,
                copy_to=os.path.join(self.bag_base_path, "ros_logs_" + self.node),
            )

        if self.bag_running:
            response.suc = True
            response.message = "Bags Stopped - "
            rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Sent SIGINT to recording process.")
        else:
            response.suc = False
            response.message = "Bags Nothing to Stop - "
            rospy.logwarn("[RosbagRecordNode(" + self.node + ")] No recording process running yet.")

        for p in self.processes:
            if self.stop_bag_process(p) != 0:
                response.suc = False
                response.message += " rosbag writer exited with an error; inspect its log."
        self.processes = []

        if self.recording_zed:
            response = self.toggle_zed_recording(False, "", response)

        if self.recording_hdr:
            bash_command_hdr = f"{RUN_RECORDING} stop_recording"
            self.recording_hdr = False
            subprocess.Popen(bash_command_hdr, shell=True, stderr=subprocess.PIPE)

            rospy.loginfo(f"[RosbagRecordNode({self.node} HDR)] Call service to stop HDR.")
            time.sleep(10)
            rospy.loginfo(f"[RosbagRecordNode({self.node} HDR)] Call docker to kill HDR container.")

            import docker

            # Initialize Docker client
            client = docker.from_env()
            # List all running containers
            containers = client.containers.list()
            # Filter containermessages by image name and stop them
            for container in containers:
                if container.image.tags and IMAGE_NAME in container.image.tags[0]:
                    print(f"Stopping container {container.id} ({container.name}) with image {container.image.tags[0]}")
                    container.stop()
            time.sleep(1)
            rospy.loginfo(f"[RosbagRecordNode({self.node} HDR)] Call docker to restart HDR container.")

            # This was only needed in old version
            # First kill the recording process inside Docker. This ugliness is required due to https://github.com/moby/moby/issues/9098
            # self.terminate_process_inside_docker()

            start_hdr()

        if self.recording_cpt7:
            suc = cpt7_stop_recording()
            if suc:
                response.message += "  CPT7 OK"
                self.recording_cpt7 = False
            else:
                response.message += "  CPT7 Failed"

            response.suc = suc and response.suc

        if request.verbose:
            # output = subprocess.check_output([f"rosbag info --freq {self.bag_path}*.bag"], shell=True)
            response.result += "Verbose not working"  # str(output)[2:-1]

        if self.zed_recording_error:
            response.suc = False
            response.message += " ZED recording was incomplete: " + self.zed_recording_error
        if response.suc:
            response.message = "[SUC] " + response.message
        else:
            response.message = "[FAILED] " + response.message
        self.bag_running = self.recording_zed or self.recording_cpt7
        return response


if __name__ == "__main__":
    rospy.loginfo("Starting ROS bag record node")
    RosbagRecordNode()
