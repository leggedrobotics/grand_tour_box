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
from pathlib import Path
from concurrent import futures
import grpc
import start_recording_pb2
import start_recording_pb2_grpc
import stop_recording_pb2
import stop_recording_pb2_grpc
from box_recording_helper.store_debug_logs_to_folder import store_debug_logs_to_folder


class StartRecordingServicer(start_recording_pb2_grpc.StartRecordingServicer):
    def __init__(self, recorder_node):
        self.recorder_node = recorder_node

    def SendMessage(self, request, context):
        self.recorder_node.start_recording_time = rospy.Time.now()
        if self.recorder_node.bag_running:
            return start_recording_pb2.StartRecordingResponse(response="Failed recorded was already running.")
        message = ""

        timestamp = request.timestamp
        self.recorder_node.bag_base_path = os.path.join(self.recorder_node.data_path, timestamp)
        Path(self.recorder_node.bag_base_path).mkdir(parents=True, exist_ok=True)

        # Check if we're on lpc. If so, dump rosparams to yaml file.
        if self.recorder_node.node.find("lpc") != -1:
            yaml_file_path = os.path.join(
                self.recorder_node.bag_base_path, f"{timestamp}_{self.recorder_node.node}.yaml"
            )
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

        for bag_name, topics in bag_configs.items():
            bag_path = os.path.join(
                self.recorder_node.bag_base_path, timestamp + "_" + self.recorder_node.node + "_" + bag_name
            )
            bash_command = f"rosrun box_recording record_bag.sh {bag_path} {topics} __name:=record_{self.recorder_node.node}_{bag_name}"
            self.recorder_node.processes.append(subprocess.Popen(bash_command, shell=True, stderr=subprocess.PIPE))
        self.recorder_node.bag_running = True

        rospy.loginfo("[RosbagRecordNodeGRPC(" + self.recorder_node.node + ")] Started recording all.")

        return start_recording_pb2.StartRecordingResponse(response=message)


class StopRecordingServicer(stop_recording_pb2_grpc.StopRecordingServicer):
    def __init__(self, recorder_node):
        self.recorder_node = recorder_node

    def SendMessage(self, request, context):

        if self.recorder_node.store_debug_logs:
            store_debug_logs_to_folder(
                self.recorder_node.start_recording_time,
                rospy.Time.now(),
                directory=os.path.expanduser("~/.ros"),
                copy_to=os.path.join(self.recorder_node.bag_base_path, "ros_logs_" + self.recorder_node.node),
            )

        def terminate_process_and_children(p):
            process = psutil.Process(p.pid)
            for sub_process in process.children(recursive=True):
                sub_process.send_signal(signal.SIGINT)
            p.wait()

        for p in self.recorder_node.processes:
            terminate_process_and_children(p)
        self.recorder_node.processes = []
        self.recorder_node.bag_running = False
        rospy.loginfo("[RosbagRecordNodeGRPC(" + self.recorder_node.node + ")] Stopped recording all.")

        return stop_recording_pb2.StopRecordingResponse(response="Message received")


class RosbagRecordNodeGrpc(object):
    def __init__(self):
        # Get the host name of the machine
        self.node = socket.gethostname()
        self.node = self.node.replace("anymal-d039-", "")
        self.node.replace("anymal-d039-", "")
        rospy.init_node(f"rosbag_record_node_{self.node}")
        self.store_debug_logs = rospy.get_param("~store_debug_logs", False)

        # Set up services
        self.processes = []
        self.bag_base_path = None
        self.namespace = rospy.get_namespace()

        self.bag_running = False
        default_path = "/data"
        # TODO: maybe customizable better
        # try:
        #     default_path = rospkg.RosPack().get_path("box_recording") + "/data"
        # except rospkg.ResourceNotFound:
        #     # fallback path if package is not built/installed
        #     rospy.logwarn("[RosbagRecordNode(" + self.node + ")] Package 'box_recording' not found. It may not be built or installed correctly.")
        #     default_path = "/data"

        self.data_path = rospy.get_param("~data_path", default_path)
        self.port = rospy.get_param("~port")

        self.info_string = ""

        if not os.path.exists(self.data_path):
            try:
                os.makedirs(self.data_path, exist_ok=True)
                rospy.loginfo("[RosbagRecordNodeGRPC(" + self.node + ")] Created data path: " + self.data_path)
            except Exception as e:
                rospy.logwarn(
                    "[RosbagRecordNodeGRPC("
                    + self.node
                    + ")] Failed to create data path "
                    + self.data_path
                    + ": "
                    + str(e)
                    + ". Will write to "
                    + default_path
                )
                self.data_path = default_path

        rp = rospkg.RosPack()
        self.rosbag_recorder_bash_script = os.path.join(rp.get_path("box_recording"), "bin/record_bag.sh")
        rospy.loginfo("[RosbagRecordNodeGRPC(" + self.node + ")] Set up to record to " + self.data_path)

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        stop_recording_pb2_grpc.add_StopRecordingServicer_to_server(StopRecordingServicer(self), server)
        start_recording_pb2_grpc.add_StartRecordingServicer_to_server(StartRecordingServicer(self), server)

        server.add_insecure_port(f"[::]:{self.port}")
        server.start()
        rospy.spin()
        # server.wait_for_termination()


if __name__ == "__main__":
    rospy.loginfo("Starting ROS bag record node")
    RosbagRecordNodeGrpc()
