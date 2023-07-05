#! /usr/bin/env python3

import rospy
import subprocess
import socket
import rospkg
import psutil
import signal
import os
from os.path import join
from box_recording.srv import StartRecordingInternalRequest, StartRecordingInternalResponse, StartRecordingInternal
from box_recording.srv import StopRecordingInternalRequest, StopRecordingInternalResponse, StopRecordingInternal

class RosbagRecordNode(object):
    def __init__(self):
        self.node = socket.gethostname()

        self.node = "lpc"

        print(self.node)

        rospy.init_node(f'rosbag_record_robot_{self.node}')
        
        servers = {}
        servers['start'] = rospy.Service('~start_recording', StartRecordingInternal, self.start_recording)
        servers['stop'] = rospy.Service('~stop_recording', StopRecordingInternal, self.stop_recording)
        self.bag_running = False
        defaultpath = rospkg.RosPack().get_path('box_recording')+'/data'
        self.datapath = rospy.get_param('~data_path', defaultpath)

        if not os.path.exists(self.datapath):
            self.datapath = defaultpath
            rospy.logwarn("[RosbagRecordNode("+self.node+")] Data path "+self.datapath+" does not exist. Will write to "+defaultpath)


        rp = rospkg.RosPack()
        self.rosbag_recorder_bash_script = join(rp.get_path('box_recording')  , 'bin/record_bag.sh')
        rospy.loginfo("[RosbagRecordNode("+self.node+")] Set up to record to "+self.datapath)

    def terminate_process_and_children(self, p):
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()

    def start_recording(self, request):
        rospy.loginfo("[RosbagRecordNode("+self.node+")] Trying to start rosbag recording process.")
        response = StartRecordingInternalResponse()
        timestamp = request.timestamp
        self.bagpath = self.datapath+"/"+timestamp+"_"+self.node
        topics = request.topics
        topics.replace("\n", " ")
        bash_command = f"rosrun box_recording record_bag.sh {self.bagpath} {topics}" 
        
        if self.bag_running:
            response.suc = False
            response.message = "Recording process already runs."
            rospy.logwarn("[RosbagRecordNode("+self.node+")] Recording process already runs.")
        else:
            self.process = subprocess.Popen(bash_command, shell=True, stderr=subprocess.PIPE)
            self.bag_running = True
            response.suc = True
            response.message = "Starting rosbag recording process."
            rospy.loginfo("[RosbagRecordNode("+self.node+")] Starting rosbag recording process.")

        return response

    def stop_recording(self, request):
        rospy.loginfo("[RosbagRecordNode("+self.node+")] Trying to send SIGINT to recording process.")
        response = StopRecordingInternalResponse()
        response.result = ""

        if self.bag_running:
            response.suc = True
            response.message = "Sending SIGINT to recording process."
            self.terminate_process_and_children(self.process)
            
            if request.verbose:
                output = subprocess.check_output([f'rosbag info --freq {self.bagpath}*.bag'], shell=True)
                response.result = str(output)[2:-1]
            self.bag_running = False
            rospy.loginfo("[RosbagRecordNode("+self.node+")] Sent SIGINT to recording process.")
        else:
            response.suc = False
            response.message = "No recording process running yet."
            rospy.logwarn("[RosbagRecordNode("+self.node+")] No recording process running yet.")

        
        return response

if __name__ == '__main__':
    print("Starting rosbag record node.")
    RosbagRecordNode()
    rospy.spin()
