
from __future__ import annotations

import rosbag
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import numpy as np
from pathlib import Path
import os
import shutil
from ultralytics import YOLO
from pathlib import Path
import cv2
import supervision as sv
import numpy as np
from typing import Collection, Sequence
import matplotlib.pyplot as plt

# NOTE: use conda env img_anon 

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

BLUR_SIZE = 25
MODELS_PATH = Path(__file__).parent / "models"
MODEL_PATH = MODELS_PATH / "yolo-v11.pt"
BLUR_CLASS_IDS = (1, 2)  # 0 for sign
MODEL = YOLO(MODEL_PATH)
BATCH_SIZE = 8

def fetch_multiple_files_kleinkram(patterns):
    tmp_dir = "/mission_data/tmp"
    os.makedirs(tmp_dir, exist_ok=True)

    for pattern in patterns:
        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein download --mission {uuid} --dest {tmp_dir} '{pattern}'")

            # Move all files from /mission_data/tmp to /mission_data/
            for file_name in os.listdir(tmp_dir):
                source_file = os.path.join(tmp_dir, file_name)
                destination_file = os.path.join("/mission_data", file_name)
                shutil.move(source_file, destination_file)


def _get_detections_batched(images: Sequence[np.ndarray]) -> list[sv.Detections]:
    return list(map(sv.Detections.from_ultralytics, MODEL.predict(source=list(images))))


def _blur_image(
    image: np.ndarray, dets: sv.Detections, class_ids: Collection[int] = BLUR_CLASS_IDS
) -> np.ndarray:
    for xyxy, _, _, cid, _, _ in dets:
        if cid not in class_ids:
            continue
        x1, y1, x2, y2 = map(int, xyxy)
        image[y1:y2, x1:x2] = cv2.GaussianBlur(
            image[y1:y2, x1:x2], (BLUR_SIZE, BLUR_SIZE), 0
        )
    return image


def _anonymize_images(images: Sequence[np.ndarray]) -> list[np.ndarray]:
    detections = _get_detections_batched(images)
    ret = []
    for image, dets in zip(images, detections):
        image = _blur_image(image, dets)
        ret.append(image)
    return ret


def process_rosbag(input_bag, image_topics, out_bag_path, out_image_topics):
    # Initialize ROS bag and CvBridge
    bag = rosbag.Bag(input_bag, "r")

    bridge = CvBridge()

    print("Camera info obtained for all topics. Starting image anonymization...")

    out_bag = rosbag.Bag(out_bag_path, "w", compression="lz4")
    try:
        batch = []
        msg_timestamps = []
        for topic, msg, t in bag.read_messages():
            if topic in image_topics:
                compressed = type(msg)._type.find("CompressedImage") != -1

                # Get the corresponding camera info
                idx = image_topics.index(topic)

                if not compressed:
                    # Convert ROS Image message to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

                batch.append(cv_image)
                msg_timestamps.append(t)

                if len(batch) == BATCH_SIZE:
                    anonymized_images = _anonymize_images(batch)
                    for i, anonymized_image in enumerate(anonymized_images):
                        if not compressed:
                            # Convert back to ROS Image message
                            anonymized_image_msg = bridge.cv2_to_imgmsg(anonymized_image, encoding="passthrough")
                        else:
                            anonymized_image_msg = bridge.cv2_to_compressed_imgmsg(anonymized_image, dst_format="jpg")

                        anonymized_image_msg.header = msg.header
                        out_bag.write(out_image_topics[idx], anonymized_image_msg, msg_timestamps[i])

                    batch = []
                    msg_timestamps = []
            else:
                out_bag.write(topic, msg, t)

    finally:
        bag.close()
        out_bag.close()

    # Reindex the bag
    print(f"Reindexing bag: {out_bag_path}")
    os.system(f"rosbag reindex {out_bag_path}")

    # if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
    #     uuid = os.environ["MISSION_UUID"]
    #     os.system(f"klein upload --mission {uuid} {out_bag}")
    #     print(f"Anonymized bag uploaded to kleinkram: {out_bag}")
    # else:
    print(f"Finished processing. Anonymized bag saved as: {out_bag_path}")


if __name__ == "__main__":
    tasks_hdr = {
        "hdr_front": {
            "in": {
                "image_topics": ["/gt_box/hdr_front/image_raw/compressed"],
                "pattern": "_jetson_hdr_front.bag",
            },
            "out": {
                "image_topics": ["/gt_box/hdr_front_anon/image_rect/compressed"],
                "pattern": "_jetson_hdr_front_anon.bag",
            },
        },
        # "hdr_left": {
        #     "in": {
        #         "camera_info_topics": ["/gt_box/hdr_left/camera_info"],
        #         "image_topics": ["/gt_box/hdr_left/image_raw/compressed"],
        #         "pattern": "_jetson_hdr_left.bag",
        #     },
        #     "out": {
        #         "camera_info_topics": ["/gt_box/hdr_left_rect/camera_info"],
        #         "image_topics": ["/gt_box/hdr_left_rect/image_rect/compressed"],
        #         "pattern": "_jetson_hdr_left_rect.bag",
        #     },
        # },
        # "hdr_right": {
        #     "in": {
        #         "camera_info_topics": ["/gt_box/hdr_right/camera_info"],
        #         "image_topics": ["/gt_box/hdr_right/image_raw/compressed"],
        #         "pattern": "_jetson_hdr_right.bag",
        #     },
        #     "out": {
        #         "camera_info_topics": ["/gt_box/hdr_right_rect/camera_info"],
        #         "image_topics": ["/gt_box/hdr_right_rect/image_rect/compressed"],
        #         "pattern": "_jetson_hdr_right_rect.bag",
        #     },
        # },
    }

    tasks_alphasense = {
        # "alphasense": {
        #     "in": {
        #         "camera_info_topics": [f"/gt_box/alphasense_driver_node/cam{i}/color/camera_info" for i in range(1, 6)],
        #         "image_topics": [
        #             f"/gt_box/alphasense_driver_node/cam{i}/color_corrected/image/compressed" for i in range(1, 6)
        #         ],
        #         "pattern": "_nuc_alphasense_color_corrected.bag",
        #     },
        #     "out": {
        #         "camera_info_topics": [
        #             f"/gt_box/alphasense_driver_node/cam{i}/color_corrected_rect/camera_info" for i in range(1, 6)
        #         ],
        #         "image_topics": [
        #             f"/gt_box/alphasense_driver_node/cam{i}/color_corrected_rect/image_rect/compressed"
        #             for i in range(1, 6)
        #         ],
        #         "pattern": "_nuc_alphasense_color_corrected_rect.bag",
        #     },
        # }
    }

    tasks = {**tasks_hdr, **tasks_alphasense}

    # Make this folder with partents if exists okay
    patterns = ["*" + task["in"]["pattern"] for task in tasks.values()]
    # fetch_multiple_files_kleinkram(patterns)

    tmp_dir = "/mission_data/tmp"
    os.makedirs(tmp_dir, exist_ok=True)

    for name, task in tasks.items():
        bags = [str(s) for s in Path(MISSION_DATA).rglob("*" + task["in"]["pattern"])]
        print(f"\nProcess for {name} the following bags: \n", bags)

        for input_bag in bags:
            process_rosbag(
                str(input_bag),
                task["in"]["image_topics"],
                str(input_bag).replace(task["in"]["pattern"], task["out"]["pattern"]),
                task["out"]["image_topics"],
            )
