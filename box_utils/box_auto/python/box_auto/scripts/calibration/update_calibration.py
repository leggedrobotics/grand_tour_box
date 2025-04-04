import rosbag
import yaml
from typing import Dict, Any
from datetime import datetime
from pathlib import Path
import rospy
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm
import numpy as np
from box_auto.utils import WS, get_bag, upload_bag

CALIBRATION_DATA_PATH = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration"
CAMERA_INFO_PATTERNS = [
    "*_jetson_hdr_right_updated.bag",
    "*_jetson_hdr_left_updated.bag",
    "*_jetson_hdr_front_updated.bag",
    "*_nuc_alphasense_updated.bag",
    "*_jetson_zed2i_images.bag",
]


def load_calibration_data() -> Dict[str, Dict[str, Any]]:
    bag_paths = [str(s) for s in Path(CALIBRATION_DATA_PATH).glob("*tf_static_metadata_new_*.bag")]
    calibration_data = {}
    # Iterate through all bag files
    for bag_path in bag_paths:
        # Open the bag file
        calibration_dict = {}
        print(bag_path)
        with rosbag.Bag(bag_path, "r") as bag:
            # Iterate through all messages in the bag
            for topic, msg, _t in bag.read_messages():
                # Process calibration data topic
                if "/calibration_data" in topic:
                    # Parse YAML and transform to CameraInfo message
                    cal_data = yaml.safe_load(msg.data)
                    msg = CameraInfo()
                    msg.height = cal_data["image_height"]
                    msg.width = cal_data["image_width"]
                    msg.distortion_model = cal_data["distortion_model"]
                    msg.D = cal_data["distortion_coefficients"]["data"]
                    msg.K = cal_data["camera_matrix"]["data"]
                    msg.R = cal_data["rectification_matrix"]["data"]
                    msg.P = cal_data["projection_matrix"]["data"]

                    calibration_dict[cal_data["rostopic"]] = msg

                # Process calibration metadata
                elif topic == "/calibration_metadata":
                    calibration_dict["metadata"] = yaml.safe_load(msg.data)

                elif topic == "/tf_static":
                    calibration_dict["tf_static"] = msg

        calibration_data[calibration_dict["metadata"]["validity_start"]] = calibration_dict

    return calibration_data


def update_tf_static(tf_static_msg, start_end=False):
    reference_bag_path = get_bag("*_jetson_utils.bag")

    # Get start and end time from reference bag
    with rosbag.Bag(reference_bag_path, "r") as ref_bag:
        start_time = ref_bag.get_start_time()
        end_time = ref_bag.get_end_time()

    tf_static_bag_path = reference_bag_path.replace("_jetson_utils", "_tf_static")
    tf_static_start_end_bag_path = reference_bag_path.replace("_jetson_utils", "_tf_static_start_end")

    if tf_static_msg is None:
        raise ValueError("No tf_static message found in the tf_static bag")

    def write(tf_static_msg, output_bag_path, start_time, start_end):
        # Create output bag with repeated tf_static messages
        with rosbag.Bag(output_bag_path, "w", compression="lz4") as out_bag:
            current_time = start_time

            while current_time <= end_time:
                t = rospy.Time.from_sec(current_time)
                for i in range(len(tf_static_msg.transforms)):
                    tf_static_msg.transforms[i].header.stamp = t
                out_bag.write("/tf_static", tf_static_msg, t)
                if current_time == end_time:
                    break

                if not start_end:
                    current_time = current_time + 1.0
                else:
                    current_time = end_time

    write(tf_static_msg, tf_static_bag_path, start_time, False)
    write(tf_static_msg, tf_static_start_end_bag_path, start_time, True)

    upload_bag([tf_static_bag_path, tf_static_start_end_bag_path])


def update_camera_info(calibration):
    for pattern in CAMERA_INFO_PATTERNS:
        bag_path = get_bag(pattern)

        with rosbag.Bag(bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()

        if "_updated.bag" in bag_path:
            out_bag = bag_path.replace("_updated.bag", "_calib.bag")
        else:
            out_bag = bag_path.replace(".bag", "_calib.bag")

        with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
            with tqdm(total=total_messages, desc=f"Processing {Path(bag_path).name}", unit="msgs") as pbar:
                for topic, msg, t in rosbag.Bag(bag_path).read_messages():
                    if str(type(msg)).find("CameraInfo") != -1:
                        key = topic.replace("/camera_info", "").replace("/color", "")
                        found = False
                        for k in calibration.keys():
                            if key in k:
                                new_msg = calibration[k]
                                found = True
                                break
                            else:
                                try:
                                    key1 = key.split("/")[2]  # camera identification
                                    key2 = key.split("/")[4]  # left right identification
                                    if key1 in k and key2 in k and "zed" in key:
                                        new_msg = calibration[k]
                                        found = True
                                        break
                                except:
                                    pass
                        if not found:
                            raise ValueError(f"Key {key} not found in: " + str(list(calibration.keys())))

                        new_msg.header = msg.header
                        outbag.write(topic, new_msg, t)

                    pbar.update(1)

        upload_bag(out_bag)


if __name__ == "__main__":
    # Step 1: Load calibration data from bag file
    calibration_data = load_calibration_data()

    # Step 2: Match calibration data to based on mission timestamp
    reference_bag_path = get_bag("*_jetson_utils.bag")
    reference_date = reference_bag_path.split("/")[-1].split("_")[0]
    reference_date = datetime.strptime(reference_date, "%Y-%m-%d-%H-%M-%S")

    calibration = None
    best_date = None
    best_s = np.inf
    for date, v in calibration_data.items():
        calibration = v
        current_date = datetime.strptime(date, "%Y-%m-%d-%H-%M-%S")

        delta_s = (reference_date - current_date).total_seconds()
        if delta_s > 0 and (delta_s < best_s):
            best_s = delta_s
            calibration = v
            best_date = date
    print(f"Closest date: {best_date} to reference date: {reference_date}")

    # Step 2: Update tf_static
    update_tf_static(calibration["tf_static"])

    # Step 3: Update camera intrinsics
    update_camera_info(calibration)
    exit(0)
