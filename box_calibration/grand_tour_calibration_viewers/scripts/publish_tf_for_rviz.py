#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import yaml
from tf.transformations import quaternion_from_matrix


def read_transform_from_file(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    T_cam_lidar = np.array(data['cam0']['T_cam_lidar'])
    return T_cam_lidar


def publish_static_transform(file_path, parent_frame, child_frame):
    # Debug: Log the file path being passed
    rospy.loginfo(f"Loading transform from file: {file_path}")

    # Read transform from file
    T_cam_lidar = read_transform_from_file(file_path)

    # Extract translation (last column)
    translation = T_cam_lidar[0:3, 3]

    # Extract rotation (upper left 3x3 matrix)
    rotation_matrix = T_cam_lidar[0:3, 0:3]

    # Convert the rotation matrix to quaternion
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = rotation_matrix
    quaternion = quaternion_from_matrix(transform_matrix)

    # Create the broadcaster
    br = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message
    static_transform_stamped = geometry_msgs.msg.TransformStamped()

    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = parent_frame  # Parent frame
    static_transform_stamped.child_frame_id = child_frame  # Child frame

    # Set translation
    static_transform_stamped.transform.translation.x = translation[0]
    static_transform_stamped.transform.translation.y = translation[1]
    static_transform_stamped.transform.translation.z = translation[2]

    # Set rotation (quaternion)
    static_transform_stamped.transform.rotation.x = quaternion[0]
    static_transform_stamped.transform.rotation.y = quaternion[1]
    static_transform_stamped.transform.rotation.z = quaternion[2]
    static_transform_stamped.transform.rotation.w = quaternion[3]

    # Publish the static transform
    br.sendTransform(static_transform_stamped)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    try:
        # Initialize the ROS node first
        rospy.init_node('static_transform_publisher', anonymous=True)

        # Get the parameters after node initialization
        file_path = rospy.get_param('~file_path', 'transform.yaml')
        parent_frame = rospy.get_param('~parent_frame', 'lidar_frame')
        child_frame = rospy.get_param('~child_frame', 'camera_frame')

        # Call the static transform publisher
        publish_static_transform(file_path, parent_frame, child_frame)
    except rospy.ROSInterruptException:
        pass
