import rosbag
from cv_bridge import CvBridge
from pathlib import Path
import os
import yaml
from tqdm import tqdm
from py_raw_image_pipeline import RawImagePipeline
import time

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")


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
        for topic, msg, t in bag.read_messages(topics=[camera_info_topic]):
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

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein mission upload --mission {uuid} --path {output_bag_path}")
        print(f"Color corrected bag uploaded to kleinkram: {output_bag_path}")
    else:
        print(f"Finished processing. Color corrected bag saved as: {output_bag_path}")


if __name__ == "__main__":
    config_file = str(Path(__file__).parent / "color_calib_file.yaml")
    pattern = "_nuc_alphasense.bag"  # "_nuc_alphasense_updated_intrinsics.bag"
    camera_info_topics = [f"/gt_box/alphasense_driver_node/cam{n}/color/camera_info" for n in [1, 2, 3, 4, 5]]
    image_topics = [f"/gt_box/alphasense_driver_node/cam{n}/color/image/compressed" for n in [1, 2, 3, 4, 5]]

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(
            f"klein mission download --mission-uuid {uuid} --local-path /mission_data --pattern *_nuc_alphasense.bag"
        )

    output_pattern = "_nuc_alphasense_cor.bag"
    bags = [str(s) for s in Path(MISSION_DATA).glob("*" + pattern) if output_pattern not in str(s)]
    print("Process bags:", bags)
    for input_bag in bags:
        output_bag_path = input_bag.replace(pattern, output_pattern)
        process_rosbag(input_bag, output_bag_path, image_topics, camera_info_topics, config_file)
