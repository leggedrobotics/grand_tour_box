#!/bin/python3
import os
import uuid

import cv2
import numpy as np
import rerun as rr
import rospkg
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from grand_tour_camera_detection_msgs.msg import CameraDetections
from grand_tour_calibration_viewers.srv import FetchRecordingIDFromBlueprintNode, FetchRecordingIDFromBlueprintNodeRequest
from matplotlib import cm
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from diffcalibrate.geometry.distortion2D import Distorter


def load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)


class CameraLiDARAlignmentViewer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lidar_alignment_viewer')

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('grand_tour_calibration_viewers')
        config_root = os.path.join(package_path, "config")
        application_id = load_yaml(os.path.join(config_root, "viewer.yaml"))["application_id"]

        self.image_topic = rospy.get_param('~image_topic', '/gt_box/alphasense_driver_node/cam1')
        self.image_topic = rospy.get_param('~lidar_topic', '/gt_box/hesai/points')
        self.timer_interval = rospy.get_param('~timer_interval', 1)
        self.recording_id_service_name = 'camera_camera_calibration_viewer/recording_id_service'
        rospy.wait_for_service(self.recording_id_service_name)
        self.fetch_recording_service = rospy.ServiceProxy(self.recording_id_service_name,
                                                          FetchRecordingIDFromBlueprintNode)
        recording_id = self.get_recording_id()
        if recording_id is None:
            rospy.logerr("Failed to initialise viewer node")
            return
        rr.init(application_id=application_id, recording_id=recording_id, spawn=False)
        rr.spawn(memory_limit="5000MB")

        # Subscribe to the CameraDetections topic
        self.subscriber = rospy.Subscriber(self.detections_topic, CameraDetections,
                                           self.added_detection_callback)

        print(f"Suscribes once every: {self.timer_interval}")
        # Set up a timer for intervals (from ROS parameter)
        self.timer = rospy.Timer(rospy.Duration(self.timer_interval), self.timer_callback)

        # CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscriber refreshed by timer callback, otherwise unregistered
        self.image_sub = None
        self.lidar_sub = None
        self.setup_image_subscriber()
        self.setup_lidar_subscriber()
        self.image = None
        self.distorters = Distorter.from_config(
            "/home/fu/catkin_ws/src/grand_tour_box/box_calibration/box_calibration/calibration/raw_calibration_output/cameras-intrinsics-extrinsics_latest.yaml")
        self.distorter = [x for x in self.distorters if x[0] == self.image_topic][0]

    def get_recording_id(self):
        try:
            request = FetchRecordingIDFromBlueprintNodeRequest()
            response = self.fetch_recording_service(request)
            rospy.loginfo(f"Received recording ID: {response.recording_id}")
            return response.recording_id.data
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def run(self):
        # Keep the node running and listen for callbacks
        rospy.spin()

    def timer_callback(self, event):
        # Subscribe to the image topic once the timer fires
        if self.image_sub is None:
            rospy.loginfo("Subscribing to image topic...")
            self.setup_image_subscriber()

    def setup_image_subscriber(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def setup_lidar_subscriber(self):
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)

    def image_callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv_image = np.dstack([cv_image] * 3)  # Convert grayscale to RGB
            rospy.loginfo("Image received and processed")
            rr.set_time_nanos("camera_calibration_time", data.header.stamp.to_nsec())
            rr.log(f"{self.image_topic}/image", rr.Image(cv_image))

            # Unsubscribe after receiving and processing one image
            if self.image_sub is not None:
                rospy.loginfo("Unsubscribing from image topic...")
                self.image_sub.unregister()
                self.image_sub = None

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def lidar_callback(self, data):
        """Callback for processing incoming PointCloud2 messages."""
        rospy.loginfo("PointCloud2 message received")

        # Convert the PointCloud2 message to a numpy array
        points_xyzi = self.point_cloud_to_numpy(data)

        # Log the number of points in the point cloud
        rospy.loginfo(f"Processing point cloud data with {len(points_xyzi)} points")

        # Call a method to process the point cloud data
        self.process_point_cloud(points_xyzi)

    def point_cloud_to_numpy(self, cloud_msg):
        """Convert PointCloud2 message to a numpy array."""
        return np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))



if __name__ == '__main__':
    try:
        visualizer = CameraLiDARAlignmentViewer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
