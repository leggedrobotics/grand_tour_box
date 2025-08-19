import rosbag
import yaml
from typing import Dict, Any
from datetime import datetime
from pathlib import Path
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm
import numpy as np
from box_auto.utils import WS, get_bag, upload_simple
import shutil
from box_auto.utils import get_uuid_mapping, read_sheet_data
import kleinkram


CALIBRATION_DATA_PATH = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration"

CAMERA_INFO_PATTERNS = [
    "*_jetson_zed2i_images_anon.bag",
]


def load_calibration_data(pattern="*tf_static_metadata_new_*.bag") -> Dict[str, Dict[str, Any]]:
    bag_paths = [str(s) for s in Path(CALIBRATION_DATA_PATH).glob(pattern)]
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
        print(calibration_dict["metadata"]["validity_start"], bag_path)
    return calibration_data


def update_camera_info(calibration, tmp_folder, mission_name):
    for pattern in CAMERA_INFO_PATTERNS:
        bag_path = get_bag(pattern, directory=tmp_folder)

        with rosbag.Bag(bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()

            out_bag = bag_path.replace("_jetson_zed2i_images_anon.bag", "_jetson_zed2i_calib.bag")

            with rosbag.Bag(out_bag, "w", compression="lz4") as outbag:
                with tqdm(total=total_messages, desc=f"Processing {Path(bag_path).name}", unit="msgs") as pbar:
                    for topic, msg, t in inbag.read_messages():
                        if str(type(msg)).find("CameraInfo") != -1:

                            # According to my understanding we also have to flip the camera info here when matching -> this was not done in the bag before
                            if topic == "/boxi/zed2i/left/camera_info":
                                key = "/gt_box/zed2i_driver_node/left_raw/image_raw_color"

                            elif topic == "/boxi/zed2i/right/camera_info":
                                key = "/gt_box/zed2i_driver_node/right_raw/image_raw_color"
                            else:
                                raise ValueError(f"Unexpected topic {topic} for CameraInfo message")

                            new_msg = calibration[key]
                            new_msg.header = msg.header

                            outbag.write(topic, new_msg, t)

                        pbar.update(1)
            upload_simple("GrandTour", "pub_" + mission_name, out_bag, delete=True)


if __name__ == "__main__":
    SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
    uuid_mappings = get_uuid_mapping()
    topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)

    for name, data in uuid_mappings.items():
        try:
            tmp_folder = Path("/data") / (name + "_zed2i")
            tmp_folder.mkdir(parents=True, exist_ok=True)

            if mission_data[name]["GOOD_MISSION"] != "TRUE":
                print("Skip processing mission - no good mission")
                continue

            res = kleinkram.list_files(
                mission_ids=[data["uuid_pub"]],
                file_names=["*_jetson_zed2i_images_anon.bag"],
            )
            if len(res) != 1:
                print("No zed2i bag found for mission", name)
                continue
            if res[0].size < 500 * 1024 * 1024:  # less than 500 MB
                print(f"Skipping mission {name} as the bag is too small: {res[0].size / (1024 * 1024):.2f} MB")
                continue

            print(f"Processing mission {name} with bag {res[0].name}  {res[0].size}")
            kleinkram.download(
                mission_ids=[data["uuid_pub"]],
                file_names=["*_jetson_zed2i_images_anon.bag", "*_jetson_adis.bag"],
                dest=tmp_folder,
                verbose=True,
                overwrite=True,
            )

            # Step 1: Load calibration data from bag file
            calibration_data = load_calibration_data()

            # Step 2: Match calibration data to based on mission timestamp
            reference_bag_path = get_bag("*_jetson_adis.bag", directory=tmp_folder)
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
            # update_tf_static(calibration["tf_static"])

            # Step 3: Update camera intrinsics
            update_camera_info(calibration, tmp_folder, name)
            # exit(0)
        except Exception as e:
            raise RuntimeError(f"Error processing mission {name}: {e}") from e
        finally:
            # Clean up temporary folder
            try:
                if tmp_folder.exists():
                    shutil.rmtree(tmp_folder)
            except Exception as e:
                print(e)
