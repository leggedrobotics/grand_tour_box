import rosbag
from cv_bridge import CvBridge
import sys
from pathlib import Path
import os
import yaml
from tqdm import tqdm
from py_raw_image_pipeline import RawImagePipeline
from pytictac import CpuTimer

MISSION_FOLDER = os.environ.get("MISSION_FOLDER", "/mission_data")
MISSION_FOLDER = "/media/jonfrey/T7/2024_05_24_dodo_basel_construction_site/gt_box/1980-01-06-01-40-00"


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


def process_rosbag(input_bag, image_topics, camera_info_topics, config_file, use_gpu=False):
    path = Path(input_bag)
    new_filename = f"{'_'.join(path.stem.split('_')[:-1])}_color_corrected_{path.stem.split('_')[-1]}{path.suffix}"
    output_bag = str(path.with_name(new_filename))

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
            dump_camera_info(msg, calib_file)

            proc[camera_info_topic] = RawImagePipeline(use_gpu, config_file, calib_file)

            break  # We only need the first camera_info message

    out_bag = rosbag.Bag(output_bag, "w")
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

                with CpuTimer("test"):
                    # Apply color correction using RawImagePipeline
                    camera_info_topic = camera_info_topics[image_topics.index(topic)]
                    corrected_image = proc[camera_info_topic].process(
                        cv_image, msg.encoding if not compressed else "bgr8"
                    )

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
                out_bag.write(corrected_image_topic, corrected_image_msg)

            elif topic in camera_info_topics:
                # TBD write message
                out_bag.write(topic, msg)

    finally:
        bag.close()
        out_bag.close()

    print(f"Finished processing. Color corrected bag saved as: {output_bag}")


if __name__ == "__main__":
    # if len(sys.argv) != 6:
    #     print(f"Usage: {sys.argv[0]} <match_args_for_rosbags> <image_topics> <camera_info_topics> <config_file>>")
    #     print(f"Example: {sys.argv[0]} _nuc_alphasense_ '/camera1/image_raw,/camera2/image_raw' '/camera1/camera_info,/camera2/camera_info' /path/to/config.yaml")
    #     sys.exit(1)
    # pattern = sys.argv[1]
    # image_topics = sys.argv[2].split(",")
    # camera_info_topics = sys.argv[3].split(",")
    # config_file = sys.argv[4]

    config_file = "/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/color_correction/calib_file.yaml"
    pattern = "*_nuc_alphasense_0*"
    image_topics = "/gt_box/alphasense_driver_node/cam1/color/image/compressed".split(",")
    camera_info_topics = "/gt_box/alphasense_driver_node/cam1/color/camera_info".split(",")

    if len(image_topics) != len(camera_info_topics):
        print("Error: The number of image topics and camera info topics must be the same.")
        sys.exit(1)

    bags = [str(s) for s in Path(MISSION_FOLDER).rglob(pattern) if str(s).find("color_corrected") == -1]
    print("Process bags:", bags)
    for input_bag in bags:
        process_rosbag(str(input_bag), image_topics, camera_info_topics, config_file)
