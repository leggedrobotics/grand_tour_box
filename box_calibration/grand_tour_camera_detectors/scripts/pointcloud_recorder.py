#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from grand_tour_camera_detection_msgs.srv import StartRecordingCalibrationDataService
import os


class PointCloudRecorder:
    def __init__(self, topic, service_timeout=5.0, query_interval=2.0):
        self.topic = topic
        self.service_timeout = service_timeout
        self.query_interval = query_interval
        self.recording_id = None
        self.bag = None
        self.subscriber = None
        self.recording_service_name = "camera_detection_recording_id"
        self.root_dir = "/data"

        # Start the timer to periodically query the recording ID
        self.timer = rospy.Timer(rospy.Duration(self.query_interval), self.query_recording_id)
        # Subscribe to the topic
        self.subscriber = rospy.Subscriber(self.topic, PointCloud2, self.callback)

    def request_recording_id(self):
        try:
            # Wait for the service with a specified timeout
            rospy.wait_for_service(self.recording_service_name, timeout=self.service_timeout)

            # Create a service proxy
            start_recording_service = rospy.ServiceProxy(self.recording_service_name,
                                                         StartRecordingCalibrationDataService)

            # Call the service and get the response
            response = start_recording_service()
            return response.recording_id.data
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(f"Service: '{self.recording_service_name}' not available within the timeout period. "
                          f"Closing bag with recording id: {self.recording_id}")
            self.stop_recording()
            return None

    def query_recording_id(self, event):
        # Try to request the recording ID
        recording_id = self.request_recording_id()
        if recording_id and recording_id != self.recording_id:
            self.recording_id = recording_id
            self.setup_recording(recording_id)

    def setup_recording(self, recording_id):
        topic_name_as_path = self.topic.replace("/", "_")
        # Use the recording ID to create the bag file path
        recording_id = recording_id + "_calibration"
        bag_dir = os.path.join(self.root_dir, f"{recording_id}")
        bag_file_path = os.path.join(bag_dir, f"{recording_id}_{topic_name_as_path}_points.bag")

        # Create the output directory if it doesn't exist
        os.makedirs(bag_dir, exist_ok=True)

        if self.bag:
            self.bag.close()
        # Open the bag file
        self.bag = rosbag.Bag(bag_file_path, 'w')
        rospy.loginfo(f"Started recording topic {self.topic} with ID {self.recording_id} to {bag_file_path}")

    def callback(self, msg):
        # Write the PointCloud2 message to the bag file
        if self.bag:
            self.bag.write(self.topic, msg)

    def stop_recording(self):
        if self.bag:
            self.bag.close()
            self.bag = None

    def shutdown(self):
        # Stop recording and shut down the timer on shutdown
        self.stop_recording()
        if self.timer:
            self.timer.shutdown()
        rospy.loginfo(f"Recorder for topic {self.topic} shut down")


def main():
    # Print out all parameters in the node's namespace
    rospy.init_node("pointcloud_recorder", anonymous=False)

    # Get the topic from the parameter server
    topic = rospy.get_param('~topic', '/pointcloud_topic')

    # Create the recorder object with a 5-second service timeout and 2-second query interval
    recorder = PointCloudRecorder(topic, service_timeout=2.0, query_interval=5.0)

    # Ensure the recording is stopped on shutdown
    rospy.on_shutdown(recorder.shutdown)

    # Spin to keep the script running
    rospy.spin()


if __name__ == '__main__':
    main()
