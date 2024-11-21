#!/usr/bin/env python

import rospy
from grand_tour_camera_detection_msgs.srv import StartRecordingCalibrationDataService
import os


class RecorderRelay:
    def __init__(self, yaml_file, service_timeout, query_interval):
        self.yaml_file = yaml_file
        self.service_timeout = service_timeout
        self.query_interval = query_interval
        self.recording_id = None
        self.recording_service_name = "camera_detection_recording_id"

        # Start the timer to periodically query the recording ID
        self.timer = rospy.Timer(rospy.Duration(self.query_interval), self.query_recording_id)

    def request_recording_id(self):
        try:
            # Wait for the service with a specified timeout
            rospy.wait_for_service(self.recording_service_name, timeout=self.service_timeout)

            # Create a service proxy
            start_recording_service = rospy.ServiceProxy(
                self.recording_service_name, StartRecordingCalibrationDataService
            )

            # Call the service and get the response
            response = start_recording_service()
            return response.recording_id.data
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(
                f"Service: '{self.recording_service_name}' not available within the timeout period: {e}"
                f"Closing bag with recording id: {self.recording_id}"
            )
            self.stop_recording()
            return None

    def query_recording_id(self, event):
        # Try to request the recording ID
        recording_id = self.request_recording_id()
        if recording_id and recording_id != self.recording_id:
            self.recording_id = recording_id
            self.setup_recording(recording_id)

    def setup_recording(self, recording_id):
        print("setup_recording")
        os.system(
            f"rosservice call /gt_box/rosbag_record_coordinator/start_recording \"yaml_file: '{self.yaml_file}' \nrecording_id: '{recording_id}'\""
        )

    def stop_recording(self):
        print("stop_recording")
        os.system('rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false"')

    def shutdown(self):
        # Stop recording and shut down the timer on shutdown
        self.stop_recording()
        if self.timer:
            self.timer.shutdown()
        rospy.loginfo(f"Recorder shut down for {self.yaml_file}")


def main():
    # Initialize the ROS node
    rospy.init_node("recorder_relay", anonymous=False)

    yaml_file = rospy.get_param("~yaml_file", "calib_imu")

    # Create the recorder object
    recorder = RecorderRelay(yaml_file, service_timeout=10.0, query_interval=2.0)

    # Ensure the recording is stopped on shutdown
    rospy.on_shutdown(recorder.shutdown)

    # Spin to keep the script running
    rospy.spin()


if __name__ == "__main__":
    main()
