import rosbag
import yaml
from typing import Dict, Any
from datetime import datetime
import os
from pathlib import Path
import rospy
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")
CALIBRATION_DATA_PATH = "/home/rsl/git/grand_tour_box/box_calibration/box_calibration/calibration"
CAMERA_INFO_PATTERNS = [
    "*_jetson_hdr_right_updated.bag",
    "*_jetson_hdr_left_updated.bag",
    "*_jetson_hdr_front_updated.bag",
    "*_nuc_alphasense.bag",
    "*_jetson_zed2i_images.bag",
]


def get_bag(directory, pattern):
    # Get reference bag path
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein download --mission {uuid} --dest {MISSION_DATA} '{pattern}'")

    files = [str(s) for s in Path(directory).rglob(pattern)]
    if len(files) != 1:
        print(f"Error: More or less matching bag files found: {pattern} in directory {directory}")
        return [], False
    return files[0], True


def load_calibration_data() -> Dict[str, Dict[str, Any]]:
    bag_paths = [str(s) for s in Path(CALIBRATION_DATA_PATH).rglob("*tf_static_with_metadata.bag")]

    calibration_data = {}
    # Iterate through all bag files
    for bag_path in bag_paths:
        # Open the bag file
        calibration_dict = {}

        with rosbag.Bag(bag_path, "r") as bag:
            # Iterate through all messages in the bag
            for topic, msg, t in bag.read_messages():
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
                elif topic == "calibration_metadata":
                    calibration_dict["metadata"] = yaml.safe_load(msg.data)

                elif topic == "/tf_static":
                    calibration_dict["tf_static"] = msg

        calibration_data[calibration_dict["metadata"]["validity_start"]] = calibration_dict
    return calibration_data


def update_tf_static(tf_static_msg, start_end=False):
    reference_bag_path, suc = get_bag(MISSION_DATA, "*_jetson_utils.bag")
    if not suc:
        raise ValueError("No reference bag found")

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
        with rosbag.Bag(output_bag_path, "w") as out_bag:
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

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein upload --mission {uuid} {tf_static_bag_path} {tf_static_start_end_bag_path}")
        print(f"TF Static bag uploaded to kleinkram: {tf_static_bag_path} {tf_static_start_end_bag_path}")
    else:
        print(f"Finished processing. TF Static bag saved as: {tf_static_bag_path} {tf_static_start_end_bag_path}")


def update_camera_info(calibration):
    for pattern in CAMERA_INFO_PATTERNS:
        bag_path, suc = get_bag(MISSION_DATA, pattern)
        if not suc:
            raise ValueError("Pattern {pattern}")

        with rosbag.Bag(bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()

        out_bag = bag_path.replace(".bag", "_calib.bag")
        with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
            with tqdm(total=total_messages, desc=f"Processing {Path(bag_path).name}", unit="msgs") as pbar:
                for topic, msg, t in rosbag.Bag(bag_path).read_messages():
                    if str(type(msg)).find("CameraInfo") != -1:
                        key = topic.replace("/camera_info", "")
                        for k in calibration.keys():
                            if k.find(key) != -1:
                                new_msg = calibration[k]
                                break

                        new_msg.header = msg.header
                        outbag.write(topic, new_msg, t)
                    else:
                        outbag.write(topic, msg, t)
                    pbar.update(1)

        if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
            uuid = os.environ["MISSION_UUID"]
            os.system(f"klein upload --mission {uuid} {out_bag}")
            print(f"TF Static bag uploaded to kleinkram: {out_bag}")
        else:
            print(f"Finished processing. TF Static bag saved as: {out_bag}")


if __name__ == "__main__":
    # Step 1: Load calibration data from bag file
    calibration_data = load_calibration_data()

    # Step 2: Match calibration data to based on mission timestamp
    reference_bag_path, suc = get_bag(MISSION_DATA, "*_jetson_utils.bag")
    reference_date = reference_bag_path.split("/")[-1].split("_")[0]
    reference_date = datetime.strptime(reference_date, "%Y-%m-%d-%H-%M-%S")

    closest_date = None
    calibration = None
    for date, v in calibration_data.items():
        current_date = datetime.strptime(date, "%Y-%m-%d-%H-%M-%S")
        if current_date < reference_date and (
            closest_date is None or reference_date - current_date < reference_date - closest_date
        ):
            closest_date = current_date
            calibration = v
    print(f"Closest date: {closest_date} to reference date: {reference_date}")

    # Step 2: Update tf_static
    update_tf_static(calibration["tf_static"])

    # Step 3: Update camera intrinsics
    update_camera_info(calibration)
