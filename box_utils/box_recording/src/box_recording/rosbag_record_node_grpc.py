#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import subprocess
import socket
import rospkg
import signal
import threading
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
from box_recording_helper.recording_directory import prepare_recording_directory
from box_recording_helper.recording_names import recording_node_name


class StartRecordingServicer(start_recording_pb2_grpc.StartRecordingServicer):
    def __init__(self, recorder_node):
        self.recorder_node = recorder_node

    def SendMessage(self, request, context):
        with self.recorder_node.recording_lock:
            return self._send_message(request, context)

    def _send_message(self, request, context):
        if self.recorder_node.bag_running:
            # Existing clients already handle RPC errors. A success transport with
            # a "Failed" string was otherwise reported as a successful recording.
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, "Recording already running")
        self.recorder_node.start_recording_time = rospy.Time.now()
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
            bash_command = [
                "bash",
                self.recorder_node.rosbag_recorder_bash_script,
                bag_path,
                *topics.split(),
                "__name:=" + recording_node_name(self.recorder_node.node, bag_name),
            ]
            self.recorder_node.processes.append(subprocess.Popen(bash_command))
            # Retain stop capability if launching a later bag raises an exception.
            self.recorder_node.bag_running = True

        rospy.loginfo("[RosbagRecordNodeGRPC(" + self.recorder_node.node + ")] Started recording all.")

        return start_recording_pb2.StartRecordingResponse(response=message)


class StopRecordingServicer(stop_recording_pb2_grpc.StopRecordingServicer):
    def __init__(self, recorder_node):
        self.recorder_node = recorder_node

    def SendMessage(self, request, context):
        with self.recorder_node.recording_lock:
            return self._send_message(request, context)

    def _send_message(self, request, context):
        if self.recorder_node.store_debug_logs and self.recorder_node.bag_base_path:
            store_debug_logs_to_folder(
                self.recorder_node.start_recording_time,
                copy_to=os.path.join(self.recorder_node.bag_base_path, "ros_logs_" + self.recorder_node.node),
            )

        failed = False
        for p in self.recorder_node.processes:
            # The script execs the writer, so completion includes bag finalization.
            p.send_signal(signal.SIGINT)
            failed = (p.wait() != 0) or failed
        self.recorder_node.processes = []
        self.recorder_node.bag_running = False
        if failed:
            context.abort(grpc.StatusCode.INTERNAL, "A rosbag writer exited with an error; inspect its log")
        rospy.loginfo("[RosbagRecordNodeGRPC(" + self.recorder_node.node + ")] Stopped recording all.")

        return stop_recording_pb2.StopRecordingResponse(response="Message received")


class RosbagRecordNodeGrpc(object):
    def __init__(self):
        # Get the host name of the machine
        self.node = socket.gethostname()
        self.node = self.node.replace("anymal-d039-", "")
        self.node.replace("anymal-d039-", "")
        rospy.init_node(recording_node_name(self.node))
        self.store_debug_logs = rospy.get_param("~store_debug_logs", False)

        # Set up services
        self.processes = []
        # gRPC has multiple worker threads; start/stop must own the same session.
        self.recording_lock = threading.RLock()
        self.bag_base_path = None
        self.namespace = rospy.get_namespace()

        self.bag_running = False
        default_path = rospkg.RosPack().get_path("box_recording") + "/data"
        self.data_path = rospy.get_param("~data_path", default_path)
        self.port = rospy.get_param("~port")

        self.info_string = ""

        self.data_path = prepare_recording_directory(self.data_path)

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


# tomorrow start out by working on wrapping up the correct ports and addresses for the grpc server and client for lpc and npc.
# push the code to a branch on boxi opc - use boxi opc to update the box.
# change all of the fkie setting to the minimum.


if __name__ == "__main__":
    rospy.loginfo("Starting ROS bag record node")
    RosbagRecordNodeGrpc()
