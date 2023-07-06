#! /usr/bin/env python3

# Parts ported from https://bitbucket.org/leggedrobotics/anymal_rsl/pull-requests/295

import rospy
import subprocess
import socket
import rospkg
import psutil
import signal
import os
from box_recording.srv import StartRecordingInternalResponse, StartRecordingInternal
from box_recording.srv import StopRecordingInternalResponse, StopRecordingInternal

class RosbagRecordNode(object):
    def __init__(self):

        # Get the host name of the machine
        self.node = socket.gethostname()

        self.namespace = rospy.get_namespace()

        rospy.init_node(f'rosbag_record_node_{self.node}')

        # Set up services
        servers = {}
        servers['start'] = rospy.Service('~start_recording', StartRecordingInternal, self.start_recording)
        servers['stop'] = rospy.Service('~stop_recording', StopRecordingInternal, self.stop_recording)

        self.bag_running = False
        default_path = rospkg.RosPack().get_path('box_recording') + '/data'
        self.data_path = rospy.get_param('~data_path', default_path)

        if not os.path.exists(self.data_path):
            self.data_path = default_path
            rospy.logwarn("[RosbagRecordNode(" + self.node + ")] Data path " + self.data_path + " does not exist. Will write to " + default_path)

        rp = rospkg.RosPack()
        self.rosbag_recorder_bash_script = os.path.join(rp.get_path('box_recording')  , 'bin/record_bag.sh')
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Set up to record to " + self.data_path)

    def terminate_process_and_children(self, p):
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()

    def start_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to start rosbag recording process.")
        response = StartRecordingInternalResponse()
        timestamp = request.timestamp
        self.bag_path = self.data_path + "/" + timestamp + "_" + self.node
        topics = request.topics
        topics.replace("\n", " ")
        bash_command = f"rosrun box_recording record_bag.sh {self.bag_path} {topics}"
        
        if self.bag_running:
            response.suc = False
            response.message = "Recording process already runs."
            rospy.logwarn("[RosbagRecordNode(" + self.node + ")] Recording process already runs.")
        else:
            self.process = subprocess.Popen(bash_command, shell=True, stderr=subprocess.PIPE)
            self.bag_running = True
            response.suc = True
            response.message = "Starting rosbag recording process."
            rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Starting rosbag recording process.")

        return response

    def stop_recording(self, request):
        rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Trying to send SIGINT to recording process.")
        response = StopRecordingInternalResponse()
        response.result = ""

        if self.bag_running:
            response.suc = True
            response.message = "Sending SIGINT to recording process."
            self.terminate_process_and_children(self.process)
            
            if request.verbose:
                output = subprocess.check_output([f'rosbag info --freq {self.bag_path}*.bag'], shell=True)
                response.result = str(output)[2:-1]
            self.bag_running = False
            rospy.loginfo("[RosbagRecordNode(" + self.node + ")] Sent SIGINT to recording process.")
        else:
            response.suc = False
            response.message = "No recording process running yet."
            rospy.logwarn("[RosbagRecordNode(" + self.node + ")] No recording process running yet.")

        
        return response

if __name__ == '__main__':
    rospy.loginfo("Starting ROS bag record node")
    RosbagRecordNode()
    rospy.spin()
