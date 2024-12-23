import rosbag
import rerun as rr
import numpy as np
from pathlib import Path
from box_auto.utils import get_bag, ARTIFACT_FOLDER
from cv_bridge import CvBridge
import ros_numpy

# Constants
ROS_BAG_PATHS = [get_bag("*_jetson_hdr_left.bag"), get_bag("*_nuc_livox.bag")]  # List of bag files
OUTPUT_VIDEO_PATH = Path(ARTIFACT_FOLDER) / "rerun" / "output_video.mp4"
FRAME_SAVE_DIR = Path(ARTIFACT_FOLDER) / "rerun" / "frames"
FRAME_SAVE_DIR.mkdir(parents=True, exist_ok=True)

# Initialize Rerun
rr.init("rosbag_visualizer", spawn=True)

# Frame index
frame_idx = 0
bridge = CvBridge()
# Process each ROS bag
for bag_path in ROS_BAG_PATHS:
    print(f"Processing ROS bag: {bag_path}")
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages():
            if "CompressedImage" in str(type(msg)) or "Image" in str(type(msg)):
                # Convert ROS Image message to OpenCV format using cv_bridge
                if "CompressedImage" in str(type(msg)):
                    np_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
                else:
                    np_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

                timestamp_ns = t.to_nsec()
                rr.log(topic, rr.Image(np_image))

                # # Save the frame
                # frame_path = os.path.join(FRAME_SAVE_DIR, f"frame_{frame_idx:04d}.png")
                # cv2.imwrite(frame_path, np_image)
                # frame_idx += 1

            elif "PointCloud2" in str(type(msg)):
                print("PointCloud2")
                points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                rr.log(topic, rr.Points3D(np.stack([points["x"], points["y"], points["z"]], axis=1)))

rr.disconnect()
