#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import rospkg
import datetime
from os.path import join
import yaml
import os
from box_recording.srv import StopRecordingInternal, StopRecordingInternalRequest
from box_recording.srv import StartRecordingInternal, StartRecordingInternalRequest
from box_recording.srv import (
    StartRecording,
    StartRecordingResponse,
    StartRecordingRequest,
)
from box_recording.srv import StopRecording, StopRecordingResponse, StopRecordingRequest
from pathlib import Path
import shutil


def load_yaml(path: str) -> dict:
    """Loads yaml file

    Args:
        path (str): File path

    Returns:
        (dict): Returns content of file
    """
    with open(path) as file:
        res = yaml.load(file, Loader=yaml.FullLoader)
    if res is None:
        res = {}
    return res


class RosbagRecordCoordinator(object):
    def __init__(self):
        self.namespace = rospy.get_namespace()

        # Create services
        servers = {}
        servers["start"] = rospy.Service("~start_recording", StartRecording, self.start_recording)
        servers["stop"] = rospy.Service("~stop_recording", StopRecording, self.stop_recording)

        self.rp = rospkg.RosPack()
        self.default_yaml = join(str(self.rp.get_path("box_recording")), "cfg/box_default.yaml")
        rospy.loginfo("[RosbagRecordCoordinator] Setup.")
        self.timestamp = ""
        self.cfg = {}

    def start_recording(self, request: StartRecordingRequest):
        rospy.loginfo("[RosbagRecordCoordinator] Trying to start rosbag recording process.")
        timestamp = "{date:%Y-%m-%d-%H-%M-%S}".format(date=datetime.datetime.now())
        self.timestamp = timestamp
        response = StartRecordingResponse()
        response.timestamp = timestamp
        response.suc = True

        if len(request.yaml_file) == 0:
            request.yaml_file = self.default_yaml
        else:
            if not os.path.isabs(request.yaml_file):
                request.yaml_file = join(
                    str(self.rp.get_path("box_recording")),
                    "cfg",
                    request.yaml_file + ".yaml",
                )

        if not os.path.exists(request.yaml_file):
            rospy.loginfo("[RosbagRecordCoordinator] Failed to start recording given that yaml cannot be found!")
            response.suc = False
            response.message = "Failed to start recording given that yaml cannot be found!"
            p = join(str(self.rp.get_path("box_recording")), "cfg/")
            options = [str(s)[:-5].split("/")[-1] for s in Path(p).rglob("*.yaml")]
            options = ", ".join(options)
            response.message = f"Available options [{options}]"
            return response

        self.cfg = load_yaml(request.yaml_file)

        # Copy yaml file to data folder
        default_path = rospkg.RosPack().get_path("box_recording") + "/data"
        data_path = rospy.get_param("~data_path", default_path)
        bag_base_path = os.path.join(data_path, timestamp)
        Path(bag_base_path).mkdir(parents=True, exist_ok=True)
        shutil.copyfile(request.yaml_file, os.path.join(bag_base_path, request.yaml_file.split("/")[-1]))

        # Go through all the nodes (PCs) and start recording
        for node, node_cfg in self.cfg.items():
            topic_cfgs = ""
            for bag_name, topics in node_cfg["bags"].items():
                for topic in topics:
                    topic_cfgs += f"{bag_name}----{topic} "
            topic_cfgs = topic_cfgs[:-1]

            if node_cfg.get("grpc_cfg", None) is not None:
                self.send_request_grpc(node, topic_cfgs, node_cfg["grpc_cfg"], timestamp, response)
            else:
                self.send_request_ros(node, topic_cfgs, timestamp, response)

        if response.suc:
            response.message += " - All nodes started recording."
            rospy.loginfo("[RosbagRecordCoordinator] Successfully started rosbag recording process on all nodes")
        return response

    def send_request_grpc(self, node, topic_cfgs, grpc_cfg, timestamp, response):
        import grpc
        import start_recording_pb2
        import start_recording_pb2_grpc

        try:
            with grpc.insecure_channel(f'{grpc_cfg["address"]}:{grpc_cfg["port"]}') as channel:
                stub = start_recording_pb2_grpc.StartRecordingStub(channel)
                req = start_recording_pb2.StartRecordingRequest(topics=topic_cfgs, timestamp=timestamp)
                grpc_response = stub.SendMessage(req)
                print(f"Server response: {grpc_response.response}")
                response.message += f"{node}-[SUC], "
        except Exception as e:
            response.suc = False
            response.message += f"{node} [FAILED] Exception: " + str(e) + ", "
            print("Service did not process request: " + str(e))
            rospy.logerr("Failed to start rosbag recording process on " + node)

    def send_request_ros(self, node, topic_cfgs, timestamp, response):
        service_name = self.namespace + "rosbag_record_node_" + node + "/start_recording"
        rospy.loginfo("[RosbagRecordCoordinator] Trying to start rosbag recording process on " + node)
        try:
            # Evaluate if service is offered
            rospy.wait_for_service(service_name, 2.0)
            start_recording_srv = rospy.ServiceProxy(service_name, StartRecordingInternal)
            int_request = StartRecordingInternalRequest()
            int_request.topics = topic_cfgs
            int_request.timestamp = timestamp
            start_recording_srv(int_request)
            rospy.loginfo("[RosbagRecordCoordinator] Starting rosbag recording process on " + node)
            response.message += f"{node}-[SUC], "

        except rospy.ROSException as exception:
            response.suc = False
            response.message += f"{node} [FAILED] Exception: " + str(exception) + ", "
            print("Service did not process request: " + str(exception))
            rospy.logerr("Failed to start rosbag recording process on " + node)

    def stop_recording(self, request: StopRecordingRequest):
        rospy.loginfo("[RosbagRecordCoordinator] Trying to stop rosbag recording for all nodes.")
        response = StopRecordingResponse()

        pretty_ls = {}

        response.suc = True
        response.message = ""
        for node, node_cfg in self.cfg.items():
            if node_cfg.get("grpc_cfg", None) is not None:
                import grpc
                import stop_recording_pb2
                import stop_recording_pb2_grpc

                try:
                    grpc_cfg = node_cfg["grpc_cfg"]
                    with grpc.insecure_channel(f'{grpc_cfg["address"]}:{grpc_cfg["port"]}') as channel:
                        stub = stop_recording_pb2_grpc.StopRecordingStub(channel)
                        req = stop_recording_pb2.StopRecordingRequest(debug_info="")
                        grpc_response = stub.SendMessage(req)
                        print(f"Server response: {grpc_response.response}")
                        response.message += f"{node}-[SUC], "
                except Exception as e:
                    response.suc = False
                    response.result = response.result + node + ": [FAILED] Exception " + str(e) + ", "
                    print("Service did not process request: " + str(e))
                    rospy.logerr("Failed to start rosbag recording process on " + node)
                continue
            else:
                service_name = self.namespace + "/rosbag_record_node_" + node + "/stop_recording"
                try:
                    rospy.wait_for_service(service_name, 2.0)
                    stop_recording_srv = rospy.ServiceProxy(service_name, StopRecordingInternal)
                    int_req = StopRecordingInternalRequest()
                    int_req.verbose = request.verbose
                    int_res = stop_recording_srv(int_req)

                    response.result = response.result + node + ": " + int_res.message + ", "

                    pretty_ls[node] = int_res.result

                    rospy.loginfo("[RosbagRecordCoordinator] Stop rosbag recording process on " + node)

                except rospy.ROSException as e:
                    response.suc = False
                    response.result = response.result + node + ": [FAILED] Exception " + str(e) + ", "
                    print("Service did not process request: " + str(e))
                    rospy.logerr("Failed to stop rosbag recording process on " + node)

        if response.suc:
            response.message += f"boxi get_data --nuc --jetson --lpc --npc --directory {self.timestamp}"
        rospy.loginfo("[RosbagRecordCoordinator] Sent STOP to all nodes.")

        pretty = ""
        for k, v in pretty_ls.items():
            length = 82
            pretty += "-" * length
            pretty += k
            print("-" * length)
            print(k)
            v = v.replace(" \\ ", "")
            for a in v.split("\\n"):
                print(a)
                pretty += a
            print("-" * length)
            print("\n")
            pretty += "-" * length
            pretty += ""

        # response.result = pretty
        return response


if __name__ == "__main__":
    rospy.init_node("rosbag_record_robot_coordinator")
    RosbagRecordCoordinator()
    rospy.spin()
