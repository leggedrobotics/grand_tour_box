#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import rospkg
import datetime
from os.path import join
import yaml
from std_msgs.msg import UInt8
from box_recording.srv import StopRecordingInternal, StopRecordingInternalRequest
from box_recording.srv import StartRecordingInternal, StartRecordingInternalRequest
from box_recording.srv import StartRecording, StartRecordingResponse, StartRecordingRequest
from box_recording.srv import StopRecording, StopRecordingResponse, StopRecordingRequest


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
        servers['start'] = rospy.Service('~start_recording', StartRecording, self.start_recording)
        servers['stop'] = rospy.Service('~stop_recording', StopRecording, self.stop_recording)
        
        rp = rospkg.RosPack()
        self.default_yaml = join( str(rp.get_path('box_recording')), "cfg/box_default.yaml")
        self.bag_running = False
        self.publish_recording_status = rospy.Publisher(self.namespace + 'health_status/recording', UInt8, queue_size=3)
        self.publish_recording_status.publish(self.bag_running)
        rospy.loginfo("[RosbagRecordCoordinator] Setup.")

    def start_recording(self, request: StartRecordingRequest):
        rospy.loginfo("[RosbagRecordCoordinator] Trying to start rosbag recording process.")
        
        if len(request.yaml_file) == 0:
            request.yaml_file = self.default_yaml
            
        timestamp = "{date:%Y-%m-%d-%H-%M-%S}".format(date=datetime.datetime.now())
        response = StartRecordingResponse()
        response.timestamp = timestamp
        response.suc = True

        if self.bag_running:
            response.suc = False
            response.message = "Recording process already runs."
            rospy.logwarn("[RosbagRecordCoordinator] Recording process already runs.")
        else:
            response.message = "Failed to start recording process on:"
            self.cfg = load_yaml( request.yaml_file )

            # Go through all the nodes (PCs) and start recording
            for node, topics in self.cfg.items():
                service_name = self.namespace + 'rosbag_record_node_' + node + '/start_recording'

                rospy.loginfo("[RosbagRecordCoordinator] Trying to start rosbag recording process on " + node + " with topics: " + str(topics))
                try:
                    # Evaluate if service is offered
                    rospy.wait_for_service(service_name, 2.0)
                    start_recording_srv = rospy.ServiceProxy(service_name, StartRecordingInternal)
                    int_request = StartRecordingInternalRequest()
                    int_request.topics = " ".join(topics)
                    int_request.timestamp = timestamp
                    start_recording_srv(int_request)
                    rospy.loginfo("[RosbagRecordCoordinator] Starting rosbag recording process on " + node)
                    self.bag_running = True
                    self.publish_recording_status.publish(self.bag_running)

                except rospy.ROSException as exception:
                    response.suc = False
                    response.message = response.message + " " + node
                    print("Service did not process request: " + str(exception))
                    rospy.logerr("Failed to start rosbag recording process on " + node)
            if response.suc:
                response.message = "Successfully started all nodes"
                rospy.loginfo("[RosbagRecordCoordinator] Successfully started rosbag recording process on all nodes")
        return response

    def stop_recording(self,  request: StopRecordingRequest):
        rospy.loginfo("[RosbagRecordCoordinator] Trying to stop rosbag recording for all nodes.")
        response = StopRecordingResponse()

        pretty_ls = {}
        if self.bag_running:
            response.suc = True
            response.message = "Failed to stop recording process on:"
            for node, topics in self.cfg.items():
                service_name = self.namespace + '/rosbag_record_node_' + node + '/stop_recording'
                try:
                    rospy.wait_for_service(service_name, 2.0)
                    stop_recording_srv = rospy.ServiceProxy(service_name, StopRecordingInternal)
                    int_req = StopRecordingInternalRequest()
                    int_req.verbose = request.verbose
                    int_res = stop_recording_srv( int_req )
                    response.result += int_res.result
                    
                    pretty_ls[node] = int_res.result
                    
                    rospy.loginfo("[RosbagRecordCoordinator] Stop rosbag recording process on " + node)

                except rospy.ROSException as exception:
                    response.suc = False
                    response.message = response.message + " " + node
                    print("Service did not process request: " + str(exception))
                    rospy.logerr("Failed to stop rosbag recording process on " + node)

            if response.suc:
                self.bag_running = False
                self.publish_recording_status.publish(self.bag_running)
                response.message = "Successfully stop all nodes"
            rospy.loginfo("[RosbagRecordCoordinator] Sent STOP to all nodes.")
        else:
            response.suc = False
            response.message = "No recording process running yet."
            rospy.logwarn("[RosbagRecordCoordinator] No recording process running yet.")
            
        pretty = ""
        for k,v in pretty_ls.items():
            length = 82
            pretty += ("-"*82)
            pretty += k
            print("-"*82)
            print(k)
            v = v.replace(" \\ ", "")
            for a in v.split("\\n"):
                print(a)
                pretty += a
            print("-"*82)
            print("\n")
            pretty += ("-"*82)
            pretty += ("")
            
        response.result = pretty
        return response

if __name__ == '__main__':
    rospy.init_node('rosbag_record_robot_coordinator')
    RosbagRecordCoordinator()
    rospy.spin()