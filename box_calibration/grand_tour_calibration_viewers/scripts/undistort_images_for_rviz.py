#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os
import rospkg

class ImageUndistorter:
    def __init__(self):
        rospy.init_node('image_undistorter')

        # Fetch parameters
        self.camera_name = rospy.get_param("~camera_name", "cam1_sensor_frame")
        self.use_compressed = rospy.get_param("~use_compressed", True)

        # Load the calibration config file
        config_file = rospy.get_param("~config_file", "camera_calibration.yaml")
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("box_calibration")
        config_path = os.path.join(package_path, 'calibration/alphasense', config_file)

        # Load calibration data
        self.load_calibration(config_path)

        # Initialize CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = rospy.Publisher(f"{self.camera_name}/undistorted_image", Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(f"{self.camera_name}/camera_info", CameraInfo, queue_size=10)

        # Subscribe to the image/compressed image topic
        if self.use_compressed:
            self.sub = rospy.Subscriber(self.camera_name, CompressedImage, self.compressed_image_callback)
        else:
            self.sub = rospy.Subscriber(self.camera_name, Image, self.image_callback)

    def load_calibration(self, config_path):
        # Load YAML calibration file
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        # Extract relevant camera information
        self.camera_matrix = np.array(config["camera_matrix"]["data"]).reshape(3, 3)
        self.dist_coeffs = np.array(config["distortion_coefficients"]["data"])
        self.projection_matrix = np.array(config["projection_matrix"]["data"]).reshape(3, 4)
        self.rectification_matrix = np.array(config["rectification_matrix"]["data"]).reshape(3, 3)
        self.distortion_model = config["distortion_model"]
        self.image_width = config["image_width"]
        self.image_height = config["image_height"]

        # Compute undistortion maps based on the distortion model
        if self.distortion_model == "equidistant":
            self.new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix, self.dist_coeffs, (self.image_width, self.image_height), np.eye(3), balance=0.0
            )
            # Generate remap function
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, np.eye(3), self.new_camera_matrix,
                (self.image_width, self.image_height), cv2.CV_16SC2
            )
        else:
            self.new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (self.image_width, self.image_height), 1,
                (self.image_width, self.image_height)
            )
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix,
                (self.image_width, self.image_height), cv2.CV_16SC2
            )

            # Store the valid ROI for cropping later
            self.valid_pix_roi = valid_pix_roi

    def create_camera_info(self, frame_id):
        # Create CameraInfo message
        camera_info_msg = CameraInfo()

        # Set the header
        camera_info_msg.header.frame_id = "calibrated_front_center"
        camera_info_msg.width = self.image_width
        camera_info_msg.height = self.image_height

        # Set intrinsic matrix (K)
        camera_info_msg.K = [self.camera_matrix[0, 0], 0, self.camera_matrix[0, 2],
                             0, self.camera_matrix[1, 1], self.camera_matrix[1, 2],
                             0, 0, 1]

        # Set distortion model and coefficients
        camera_info_msg.distortion_model = self.distortion_model
        camera_info_msg.D = self.dist_coeffs.tolist()

        # Set rectified projection matrix (P)
        camera_info_msg.P = [self.new_camera_matrix[0, 0], 0, self.new_camera_matrix[0, 2], 0,
                             0, self.new_camera_matrix[1, 1], self.new_camera_matrix[1, 2], 0,
                             0, 0, 1, 0]

        # Set rectification matrix (R)
        camera_info_msg.R = self.rectification_matrix.flatten().tolist()

        return camera_info_msg

    def undistort_image(self, cv_image):
        # Apply undistortion using the precomputed maps
        undistorted_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        return undistorted_image

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Undistort the image
        undistorted_image = self.undistort_image(cv_image)

        # Publish the undistorted image
        undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
        undistorted_msg.header.stamp = rospy.Time.now()
        undistorted_msg.header.frame_id = "calibrated_front_center"
        self.image_pub.publish(undistorted_msg)

        # Publish camera info
        camera_info_msg = self.create_camera_info(msg.header.frame_id)
        camera_info_msg.header.stamp = undistorted_msg.header.stamp
        self.camera_info_pub.publish(camera_info_msg)

    def compressed_image_callback(self, msg):
        try:
            # Convert ROS compressed image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Failed to convert compressed image: {e}")
            return

        # Undistort the image
        undistorted_image = self.undistort_image(cv_image)

        # Publish the undistorted image
        undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
        undistorted_msg.header.stamp = rospy.Time.now()
        undistorted_msg.header.frame_id = msg.header.frame_id
        self.image_pub.publish(undistorted_msg)

        # Publish camera info
        camera_info_msg = self.create_camera_info(msg.header.frame_id)
        camera_info_msg.header.stamp = undistorted_msg.header.stamp
        self.camera_info_pub.publish(camera_info_msg)


if __name__ == '__main__':
    try:
        node = ImageUndistorter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
