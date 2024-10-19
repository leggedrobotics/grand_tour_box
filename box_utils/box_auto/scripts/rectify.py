import rosbag
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import numpy as np
from pathlib import Path
import os
MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

def undistort_image_fisheye(image, camera_info, new_camera_info=None):
    K = np.array(camera_info.K).reshape((3, 3))
    D = np.array(camera_info.D)
    
    h, w = image.shape[:2]
    
    if new_camera_info is None:
        new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (w, h), np.eye(3), balance=0.0, fov_scale=1.0
        )
        
        new_camera_info = CameraInfo()
        new_camera_info.header = camera_info.header
        new_camera_info.width = camera_info.width
        new_camera_info.height = camera_info.height
        new_camera_info.K = new_camera_matrix.flatten().tolist()
        new_camera_info.D = [0, 0, 0, 0]  # Fisheye distortion coefficients
        new_camera_info.R = camera_info.R
        new_camera_info.distortion_model = "plumb_bob"
        new_P = np.eye(4)
        new_P[:3, :3] = new_camera_matrix
        new_camera_info.P = new_P[:3,:4].flatten().tolist()
        # Initialize undistortion map

        
    else:
        new_camera_info.header = camera_info.header
        new_camera_matrix = np.array(new_camera_info.K).reshape((3, 3))

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2
    )
    # Apply undistortion
    undistorted_image = cv2.remap(
        image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
    )

    return undistorted_image, new_camera_info

def undistort_image(image, camera_info, new_camera_info = None):
    # Get camera intrinsic parameters
    K = np.array(camera_info.K).reshape((3, 3))  # Intrinsic matrix
    D = np.array(camera_info.D)  # Distortion coefficients
    P = np.array(camera_info.P).reshape((3, 4))  # Projection matrix

    # Remove distortion from the image
    h, w = image.shape[:2]
    if new_camera_info is None:
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
        # Return the undistorted image and the updated camera info
        new_camera_info = CameraInfo()
        new_camera_info.header = camera_info.header
        new_camera_info.width = camera_info.width
        new_camera_info.height = camera_info.height
        new_camera_info.K = new_camera_matrix.flatten().tolist()
        new_camera_info.D = [0, 0, 0, 0, 0]
        new_camera_info.R = camera_info.R
        new_camera_info.distortion_model = "plumb_bob"
        new_camera_info.P = P.flatten().tolist()  # Usually, we don't change P for rectification

    else:
        new_camera_info.header = camera_info.header
        new_camera_matrix = np.array(new_camera_info.K).reshape((3, 3))

    undistorted_image = cv2.undistort(image, K, D, None, new_camera_matrix)
    return undistorted_image, new_camera_info


def process_rosbag(input_bag, image_topics, camera_info_topics,  out_bag_path, out_image_topics, out_camera_info_topics ):
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

    out_bag = rosbag.Bag(out_bag_path, "w", compression='lz4')
    try:
        new_camera_info = None
        for topic, msg, t in bag.read_messages():
            if topic in image_topics:
                compressed = type(msg)._type.find("CompressedImage") != -1

                # Get the corresponding camera info
                idx = image_topics.index(topic)

                camera_info_topic = camera_info_topics[idx]
                camera_info = camera_info_dict[camera_info_topic]

                if not compressed:
                    # Convert ROS Image message to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

                # Undistort the image
                if "equidistant" in camera_info.distortion_model:
                    undistorted_image, new_camera_info = undistort_image_fisheye(cv_image, camera_info, new_camera_info)
                else:   
                    undistorted_image, new_camera_info = undistort_image(cv_image, camera_info, new_camera_info)


                if not compressed:
                    # Convert back to ROS Image message
                    rectified_image_msg = bridge.cv2_to_imgmsg(undistorted_image, encoding="passthrough")
                else:
                    rectified_image_msg = bridge.cv2_to_compressed_imgmsg(undistorted_image, dst_format="jpg")

                rectified_image_msg.header = msg.header

                out_bag.write(out_image_topics[idx], rectified_image_msg, t)
                out_bag.write(out_camera_info_topics[idx], new_camera_info, t)
    finally:
        bag.close()
        out_bag.close()

    if os.environ.get("KLEINKRAM_ACTIVE", False):
        os.system( f"klein mission upload --mission {os.environ["MISSION_UUID"]} --path {out_bag}")
        print(f"Rectified bag uploaded to kleinkram: {out_bag}")
    else:
        print(f"Finished processing. Rectified bag saved as: {out_bag_path}")


if __name__ == "__main__":
    tasks_hdr = { 
        "hdr_front":{
            "in": {
                "camera_info_topics": ["/gt_box/hdr_front/camera_info"],
                "image_topics": ["/gt_box/hdr_front/image_raw/compressed"],
                "pattern": "_jetson_hdr_front.bag",
            },
            "out": {
                "camera_info_topics": ["/gt_box/hdr_front_rect/camera_info"],
                "image_topics": ["/gt_box/hdr_front_rect/image_rect/compressed"],
                "pattern": "_jetson_hdr_front_rect.bag",
            }
        },
        "hdr_left":{
            "in": {
                "camera_info_topics": ["/gt_box/hdr_left/camera_info"],
                "image_topics": ["/gt_box/hdr_left/image_raw/compressed"],
                "pattern": "_jetson_hdr_left.bag",
            },
            "out": {
                "camera_info_topics": ["/gt_box/hdr_left_rect/camera_info"],
                "image_topics": ["/gt_box/hdr_left_rect/image_rect/compressed"],
                "pattern": "_jetson_hdr_left_rect.bag",
            }
        },
        "hdr_right":{
            "in": {
                "camera_info_topics": ["/gt_box/hdr_right/camera_info"],
                "image_topics": ["/gt_box/hdr_right/image_raw/compressed"],
                "pattern": "_jetson_hdr_right.bag",
            },
            "out": {
                "camera_info_topics": ["/gt_box/hdr_right_rect/camera_info"],
                "image_topics": ["/gt_box/hdr_right_rect/image_rect/compressed"],
                "pattern": "_jetson_hdr_right_rect.bag",
            }
        },
    }

    tasks_alphasense= {
        "alphasense":{
            "in": {
                "camera_info_topics": [f"/gt_box/alphasense_driver_node/cam{i}/color/camera_info" for i in range(1,6)],
                "image_topics": [f"/gt_box/alphasense_driver_node/cam{i}/color_corrected/image/compressed" for i in range(1,6)],
                "pattern": "_nuc_alphasense_color_corrected.bag",
            },
            "out": {
                "camera_info_topics": [f"/gt_box/alphasense_driver_node/cam{i}/color_corrected_rect/camera_info" for i in range(1,6)],
                "image_topics": [f"/gt_box/alphasense_driver_node/cam{i}/color_corrected_rect/image_rect/compressed" for i in range(1,6)],
                "pattern": "_nuc_alphasense_color_corrected_rect.bag",
            },
        }
    }

    tasks = {**tasks_hdr, **tasks_alphasense}    
    for name, task in tasks.items():

        bags = [str(s) for s in Path(MISSION_DATA).rglob("*" + task["in"]["pattern"])]
        print(f"\nProcess for {name} the following bags: \n", bags)

        for input_bag in bags:
            process_rosbag(str(input_bag), task["in"]["image_topics"], task["in"]["camera_info_topics"], 
                           str(input_bag).replace(task["in"]["pattern"],task["out"]["pattern"]), task["out"]["image_topics"], task["out"]["camera_info_topics"])
