import rosbag
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from box_auto.utils import get_bag

# Path to your rosbag file
rosbag_file = get_bag("*_nuc_alphasense.bag")  # _fixed_frame_id

# Topics of interest
topics = [
    "/gt_box/alphasense_driver_node/cam1/color/camera_info",
    "/gt_box/alphasense_driver_node/cam1/color/image/compressed",
    "/gt_box/alphasense_driver_node/cam2/color/camera_info",
    "/gt_box/alphasense_driver_node/cam2/color/image/compressed",
    "/gt_box/alphasense_driver_node/cam3/color/camera_info",
    "/gt_box/alphasense_driver_node/cam3/color/image/compressed",
    "/gt_box/alphasense_driver_node/cam4/color/camera_info",
    "/gt_box/alphasense_driver_node/cam4/color/image/compressed",
    "/gt_box/alphasense_driver_node/cam5/color/camera_info",
    "/gt_box/alphasense_driver_node/cam5/color/image/compressed",
]

# Initialize the CV bridge for image conversion
bridge = CvBridge()

# Data containers for images and camera info
images = {}
camera_infos = {}

# Open the rosbag
with rosbag.Bag(rosbag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if "camera_info" in topic and topic not in camera_infos:
            camera_infos[topic] = msg
        elif "image/compressed" in topic and topic not in images:
            images[topic] = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if len(images) + len(camera_infos) == len(topics):
            break

fig, axes = plt.subplots(1, int(len(topics) / 2), figsize=(20, 8))

for i, cam_idx in enumerate(range(1, 6)):
    image_topic = f"/gt_box/alphasense_driver_node/cam{cam_idx}/color/image/compressed"
    camera_info_topic = f"/gt_box/alphasense_driver_node/cam{cam_idx}/color/camera_info"

    if image_topic in images and camera_info_topic in camera_infos:
        image = images[image_topic]
        camera_info = camera_infos[camera_info_topic]
        axes[i].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        axes[i].axis("off")

        it = image_topic.replace("/gt_box/alphasense_driver_node/", "").replace("/compressed", "")
        subtitle = (
            f"Image frame_id: {camera_info.header.frame_id}\n"
            f"CameraInfo frame_id: {camera_info.header.frame_id}\n"
            f"Camera Topic {it}"
        )
        axes[i].set_title(subtitle, fontsize=10)

plt.tight_layout()
plt.show()
