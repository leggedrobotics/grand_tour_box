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
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_density_heatmap(image, points_in, subblock_size=(64, 64), alpha=0.5):
    """
    Generates a heatmap overlay representing the density of 2D points over an image,
    where the density is calculated using subblocks of a specific size.
    """
    # Calculate the number of bins based on the subblock size
    bins_x = image.shape[1] // subblock_size[0]
    bins_y = image.shape[0] // subblock_size[1]
    bins = [bins_x, bins_y]

    # Extract x and y coordinates from points
    x = points_in[:, 0]
    y = points_in[:, 1]

    # Generate 2D histogram for density based on the calculated number of bins
    heatmap, _, _ = np.histogram2d(x, y, bins=bins, range=[[0, image.shape[1]], [0, image.shape[0]]])

    heatmap = np.clip(heatmap, 0, 100)
    # Transpose and normalize heatmap
    heatmap = heatmap.T  # Transpose to match image orientation
    heatmap_normalized = heatmap / np.max(heatmap)

    # Use a predefined linear colormap ('RdYlGn') that goes from red to green
    cmap = cm.get_cmap('RdYlGn')

    # Map the normalized heatmap to RGB values using the colormap
    heatmap_rgb = cmap(heatmap_normalized)

    # Convert the RGBA image to RGB (drop the alpha channel)
    heatmap_rgb = (heatmap_rgb[:, :, :3] * 255).astype(np.uint8)

    heatmap_resized = cv2.resize(heatmap_rgb, (image.shape[1], image.shape[0]),
                                 interpolation=cv2.INTER_LINEAR)

    # Blend the heatmap with the original image using alpha blending
    blended_image = (alpha * heatmap_resized + (1 - alpha) * image).astype(np.uint8)

    return blended_image


class CornerVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('corner_visualizer')

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('grand_tour_calibration_viewers')
        config_root = os.path.join(package_path, "config")
        application_id = load_yaml(os.path.join(config_root, "viewer.yaml"))["application_id"]

        self.image_topic = rospy.get_param('~image_topic', '/gt_box/alphasense_driver_node/cam1')
        self.detections_topic = rospy.get_param('~detections_topic',
                                                '/gt_box/alphasense_driver_node/cam1_corner_detections')
        self.processed_detections_topic = rospy.get_param('~processed_detections_topic',
                                                '/gt_box/alphasense_driver_node/cam1_corner_detections_used')
        self.extrinsics_detections_topic = rospy.get_param('~processed_extrinsics_detections_topic',
                                                '/gt_box/alphasense_driver_node/cam1_corner_detections_used_extrinsics')
        self.heatmap_subblock_size = rospy.get_param('~heatmap_subblock_size', [64, 64])
        self.heatmap_alpha = rospy.get_param('~heatmap_alpha', 0.5)
        self.heatmap_clip_value = rospy.get_param('~heatmap_clip_value', 100)
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
        self.subscriber_post = rospy.Subscriber(self.processed_detections_topic, CameraDetections,
                                                self.intrinsics_residuals_callback)
        self.subscriber_post = rospy.Subscriber(self.extrinsics_detections_topic, CameraDetections,
                                                self.extrinsics_residuals_callback)

        # Store all detected points across messages
        self.accumulated_coords = dict()
        self.accumulated_residuals = dict()
        self.accumulated_extrinsics_residuals = dict()

        print(f"Suscribes once every: {self.timer_interval}")
        # Set up a timer for intervals (from ROS parameter)
        self.timer = rospy.Timer(rospy.Duration(self.timer_interval), self.timer_callback)

        # CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscriber refreshed by timer callback, otherwise unregistered
        self.image_sub = None
        self.setup_image_subscriber()
        self.image = None

    def get_recording_id(self):
        try:
            request = FetchRecordingIDFromBlueprintNodeRequest()
            response = self.fetch_recording_service(request)
            rospy.loginfo(f"Received recording ID: {response.recording_id}")
            return response.recording_id.data
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def intrinsics_residuals_callback(self, msg):
        header: Header = msg.header
        rr.set_time_nanos("camera_calibration_time", header.stamp.to_nsec())

        x_residuals = np.array([corner.x for corner in msg.residuals2d])[:, None]
        y_residuals = np.array([corner.y for corner in msg.residuals2d])[:, None]
        residuals = np.hstack((x_residuals, y_residuals))

        num_points = len(residuals)

        # If there are more than 1000 points, randomly sample 1000; otherwise, use all points
        if num_points > 1000:
            indices = np.random.choice(num_points, 1000, replace=False)
            sampled_residuals = residuals[indices]
        else:
            sampled_residuals = residuals

        x_coords = np.array([corner.x for corner in msg.corners2d])[:, None]
        y_coords = np.array([corner.y for corner in msg.corners2d])[:, None]
        points2d = np.hstack((x_coords, y_coords))
        self.accumulated_coords = points2d

        rr.log(f"{self.image_topic}_residuals",
               rr.Points2D(sampled_residuals, colors=[34, 138, 167]))
        rr.log(f"{self.image_topic}_residuals/1px", rr.Boxes2D(mins=[-1, -1], sizes=[2, 2], colors=[0, 255, 0]))
        rr.log(f"{self.image_topic}_residuals/3px", rr.Boxes2D(mins=[-3, -3], sizes=[6, 6], colors=[255, 0, 0]))

    def extrinsics_residuals_callback(self, msg):
        header: Header = msg.header
        rr.set_time_nanos("camera_calibration_time", header.stamp.to_nsec())

        x_residuals = np.array([corner.x for corner in msg.residuals2d])[:, None]
        y_residuals = np.array([corner.y for corner in msg.residuals2d])[:, None]
        residuals = np.hstack((x_residuals, y_residuals))

        rr.log(f"{self.image_topic}_residuals/extrinsic_alignment",
               rr.Points2D(residuals, colors=[255, 165, 0]))

    def added_detection_callback(self, msg):
        header: Header = msg.header
        rr.set_time_nanos("camera_calibration_time", header.stamp.to_nsec())
        # Extract the corners2d from the message
        x_coords = np.array([corner.x for corner in msg.corners2d])[:, None]
        y_coords = np.array([corner.y for corner in msg.corners2d])[:, None]
        points2d = np.hstack((x_coords, y_coords))

        # self.accumulated_coords[header.stamp.to_nsec()] = points2d

        rr.log(f"{self.image_topic}/image/keypoints",
               rr.Points2D(points2d, colors=[34, 138, 167]))

    def run(self):
        # Keep the node running and listen for callbacks
        rospy.spin()

    def timer_callback(self, event):
        # Subscribe to the image topic once the timer fires
        if self.image_sub is None:
            rospy.loginfo("Subscribing to image topic...")
            self.setup_image_subscriber()
        rr.log(f"{self.image_topic}/image/keypoints",
               rr.Clear(recursive=False))

    def setup_image_subscriber(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv_image = np.dstack([cv_image] * 3)  # Convert grayscale to RGB

            if len(self.accumulated_coords):
                coords = self.accumulated_coords
                # coords = np.concatenate(coords) if len(coords) > 1 else coords[0]
                heatmap_viz = generate_density_heatmap(
                    cv_image,
                    coords,
                    subblock_size=self.heatmap_subblock_size,
                    alpha=self.heatmap_alpha
                )
            else:
                heatmap_viz = cv_image
            rospy.loginfo("Image received and processed")
            rr.set_time_nanos("camera_calibration_time", data.header.stamp.to_nsec())
            rr.log(f"{self.image_topic}/image", rr.Image(heatmap_viz))

            # Unsubscribe after receiving and processing one image
            if self.image_sub is not None:
                rospy.loginfo("Unsubscribing from image topic...")
                self.image_sub.unregister()
                self.image_sub = None

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


if __name__ == '__main__':
    try:
        visualizer = CornerVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
