import rosbag
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import numpy as np
import sys
from pathlib import Path
import os

MISSION_FOLDER = os.environ.get("MISSION_FOLDER", "/mission_data")


def undistort_image(image, camera_info):
    # Get camera intrinsic parameters
    K = np.array(camera_info.K).reshape((3, 3))  # Intrinsic matrix
    D = np.array(camera_info.D)  # Distortion coefficients
    P = np.array(camera_info.P).reshape((3, 4))  # Projection matrix

    # Remove distortion from the image
    h, w = image.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, K, D, None, new_camera_matrix)

    # Return the undistorted image and the updated camera info
    new_camera_info = CameraInfo()
    new_camera_info.header = camera_info.header
    new_camera_info.width = camera_info.width
    new_camera_info.height = camera_info.height
    new_camera_info.K = new_camera_matrix.flatten().tolist()
    new_camera_info.D = [0, 0, 0, 0, 0]
    new_camera_info.R = camera_info.R
    new_camera_info.P = P.flatten().tolist()  # Usually, we don't change P for rectification

    return undistorted_image, new_camera_info


def process_rosbag(input_bag, image_topics, camera_info_topics):
    path = Path(input_bag)
    new_filename = f"{'_'.join(path.stem.split('_')[:-1])}_rectified_{path.stem.split('_')[-1]}{path.suffix}"
    # Combine the directory part with the new filename
    output_bag = str(path.with_name(new_filename))

    # Initialize ROS bag and CvBridge
    bag = rosbag.Bag(input_bag, "r")

    bridge = CvBridge()

    # Store camera info messages
    camera_info_dict = {}

    # First, get the camera_info messages
    for camera_info_topic in camera_info_topics:
        for topic, msg, t in bag.read_messages(topics=[camera_info_topic]):
            camera_info_dict[camera_info_topic] = msg
            break  # We only need the first camera_info message

    # Check if all camera_info messages are available
    for camera_info_topic in camera_info_topics:
        if camera_info_topic not in camera_info_dict:
            print(f"Error: Camera info topic '{camera_info_topic}' not found in the bag file.")
            bag.close()
            return

    print("Camera info obtained for all topics. Starting image undistortion...")

    out_bag = rosbag.Bag(output_bag, "w", compression='lz4')
    try:
        for topic, msg, t in bag.read_messages():
            if topic in image_topics:
                compressed = type(msg)._type.find("CompressedImage") != -1

                # Get the corresponding camera info
                camera_info_topic = camera_info_topics[image_topics.index(topic)]
                camera_info = camera_info_dict[camera_info_topic]

                if not compressed:
                    # Convert ROS Image message to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

                # Undistort the image
                undistorted_image, new_camera_info = undistort_image(cv_image, camera_info)

                if not compressed:
                    # Convert back to ROS Image message
                    rectified_image_msg = bridge.cv2_to_imgmsg(undistorted_image, encoding="passthrough")
                else:
                    rectified_image_msg = bridge.cv2_to_compressed_imgmsg(undistorted_image, dst_format="jpg")

                rectified_image_msg.header = msg.header

                # Write the new image and camera info to the new bag
                rectified_image_topic = topic.split("/")
                camera_info_topic = camera_info_topic.split("/")

                rectified_image_topic.insert(len(rectified_image_topic) - 1, "rectified")
                rectified_image_topic = "/".join(rectified_image_topic)

                camera_info_topic.insert(len(camera_info_topic) - 1, "rectified")
                camera_info_topic = "/".join(camera_info_topic)

                out_bag.write(rectified_image_topic, rectified_image_msg)
                new_camera_info.header = camera_info.header
                out_bag.write(camera_info_topic, new_camera_info)
    finally:
        bag.close()
        out_bag.close()

    print(f"Finished processing. Rectified bag saved as: {output_bag}")


if __name__ == "__main__":
    tasks = { 
        "hdr":{
            "camera_info_topics": ["/camera1/camera_info", "/camera2/camera_info"],
            "image_topics": ["/camera1/image_raw", "/camera2/image_raw"],
            "pattern": "*_jetson_hdr.bag",
        },
        "alphasense":{
            "camera_info_topics": [f"/gt_box/alphasense_driver_node/cam{n}/color_corrected/camera_info" for n in [1,2,3,4,5]],
            "image_topics": [f"/gt_box/alphasense_driver_node/cam{n}/color_corrected/image/compressed" for n in [1,2,3,4,5]],
            "pattern": "*_nuc_alphasense_color_corrected.bag",
        }
    }
    for name, task in tasks.items():
        bags = [str(s) for s in Path(MISSION_FOLDER).rglob(task["pattern"]) if str(s).find("rectified") == -1]
        print(f"\nProcess for {name} the following bags: \n", bags)
        for input_bag in bags:
            process_rosbag(str(input_bag), task["image_topics"], task["camera_info_topics"])
