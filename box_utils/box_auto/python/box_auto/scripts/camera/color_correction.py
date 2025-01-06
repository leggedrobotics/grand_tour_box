import rosbag
from cv_bridge import CvBridge
from pathlib import Path
import yaml
from tqdm import tqdm
from py_raw_image_pipeline import RawImagePipeline
import time

from box_auto.utils import get_bag, upload_bag, WS


def dump_camera_info(camera_info_msg, filename):
    camera_info = {
        "image_width": camera_info_msg.width,
        "image_height": camera_info_msg.height,
        "camera_name": "alphasense_cam1",
        "camera_matrix": {"rows": 3, "cols": 3, "data": camera_info_msg.K},
        "distortion_model": camera_info_msg.distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(camera_info_msg.D), "data": camera_info_msg.D},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": camera_info_msg.R},
        "projection_matrix": {"rows": 3, "cols": 4, "data": camera_info_msg.P},
    }
    with open(filename, "w") as file:
        yaml.dump(camera_info, file, default_flow_style=False)


def process_rosbag(input_bag, output_bag_path, image_topics, camera_info_topics, config_file, use_gpu=False):
    # Initialize ROS bag and CvBridge
    bag = rosbag.Bag(input_bag, "r")
    bridge = CvBridge()

    # Store camera info messages
    camera_info_dict = {}
    proc = {}

    # First, get the camera_info messages
    for camera_info_topic in camera_info_topics:
        for _topic, msg, _t in bag.read_messages(topics=[camera_info_topic]):
            camera_info_dict[camera_info_topic] = msg
            calib_file = camera_info_topic.replace("/", "_") + ".yaml"
            # dump_camera_info(msg, calib_file)
            proc[camera_info_topic] = RawImagePipeline(use_gpu, config_file, calib_file)
            break  # We only need the first camera_info message

    out_bag = rosbag.Bag(output_bag_path, "w", compression="lz4")
    total_messages = bag.get_message_count()
    try:
        for topic, msg, t in tqdm(bag.read_messages(), total=total_messages, desc="Processing Messages"):
            if topic in image_topics:
                compressed = type(msg)._type.find("CompressedImage") != -1

                # Convert ROS Image message to OpenCV image
                if not compressed:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

                st = time.time()
                # Apply color correction using RawImagePipeline
                camera_info_topic = camera_info_topics[image_topics.index(topic)]
                corrected_image = proc[camera_info_topic].process(cv_image, msg.encoding if not compressed else "bgr8")
                print(f"Time: {time.time() - st}")
                # # Display the processed image in a window
                # cv2.imshow("Processed Image", corrected_image)
                # # Wait for a short period to display the image, 1 ms is enough
                # if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit early
                #     break

                # Store Image to Rosbag
                if not compressed:
                    corrected_image_msg = bridge.cv2_to_imgmsg(corrected_image, encoding="passthrough")
                else:
                    corrected_image_msg = bridge.cv2_to_compressed_imgmsg(corrected_image, dst_format="jpg")

                corrected_image_topic = topic.replace("color", "color_corrected")
                corrected_image_msg.header = msg.header
                out_bag.write(corrected_image_topic, corrected_image_msg, t)

            elif topic in camera_info_topics:
                # TBD write message
                out_bag.write(topic, msg, t)

    finally:
        bag.close()
        out_bag.close()


if __name__ == "__main__":
    INPUT_PATTERN = "_nuc_alphasense_calib.bag"
    config_file = str(Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg/color_calib_file.yaml")
    camera_info_topics = [f"/gt_box/alphasense_driver_node/cam{n}/color/camera_info" for n in [1, 2, 3, 4, 5]]
    image_topics = [f"/gt_box/alphasense_driver_node/cam{n}/color/image/compressed" for n in [1, 2, 3, 4, 5]]

    input_bag = get_bag(f"*{INPUT_PATTERN}")

    output_bag_path = input_bag.replace(INPUT_PATTERN, "_nuc_alphasense_color.bag")
    process_rosbag(input_bag, output_bag_path, image_topics, camera_info_topics, config_file, use_gpu=False)
    upload_bag(output_bag_path)

    exit(0)
