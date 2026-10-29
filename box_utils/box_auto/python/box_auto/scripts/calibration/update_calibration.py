import copy
import rosbag
import yaml
from typing import Dict, Any, Optional, List
from datetime import datetime
from pathlib import Path

import rospy
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm
import numpy as np

from box_auto.utils import WS, get_bag, upload_bag, MISSION_DATA
from box_auto.utils.atomic_file import atomic_output

CALIBRATION_DATA_PATH = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration"
CAMERA_INFO_PATTERNS = [
    "*_jetson_hdr_right_encoding.bag",
    "*_jetson_hdr_left_encoding.bag",
    "*_jetson_hdr_front_encoding.bag",
    "*_nuc_alphasense_color.bag",
]


def check_expected_bags_present(patterns: List[str], search_dir: Path) -> Optional[Dict[str, str]]:
    """
    Check that each pattern matches exactly one *.bag in search_dir.
    If any missing or ambiguous, print and return None.
    """
    search_dir = Path(search_dir)
    resolved: Dict[str, str] = {}
    missing: List[str] = []
    ambiguous: Dict[str, List[str]] = {}

    for pat in patterns:
        matches = sorted([str(p) for p in search_dir.glob(pat)])
        if len(matches) == 0:
            missing.append(pat)
        elif len(matches) > 1:
            ambiguous[pat] = matches
        else:
            resolved[pat] = matches[0]

    if missing or ambiguous:
        if missing:
            print("\033[91m" + "=" * 60)
            print("[ERROR] Missing expected bag(s):")
            for pat in missing:
                print("  - %s" % pat)
            print("=" * 60 + "\033[0m")
        if ambiguous:
            print("\033[91m" + "=" * 60)
            print("[ERROR] Ambiguous pattern(s) (matched > 1 file):")
            for pat, files in ambiguous.items():
                print("  - %s" % pat)
                for f in files:
                    print("      %s" % f)
            print("=" * 60 + "\033[0m")
        return None

    return resolved


def load_calibration_data() -> Dict[str, Dict[str, Any]]:
    bag_paths = [str(s) for s in Path(CALIBRATION_DATA_PATH).glob("*tf_static_metadata_new_*.bag")]
    calibration_data: Dict[str, Dict[str, Any]] = {}

    for bag_path in bag_paths:
        calibration_dict: Dict[str, Any] = {}
        print(bag_path)

        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, _t in bag.read_messages():
                if "/calibration_data" in topic:
                    cal_data = yaml.safe_load(msg.data)

                    ci = CameraInfo()
                    ci.height = cal_data["image_height"]
                    ci.width = cal_data["image_width"]
                    ci.distortion_model = cal_data["distortion_model"]
                    ci.D = cal_data["distortion_coefficients"]["data"]
                    ci.K = cal_data["camera_matrix"]["data"]
                    ci.R = cal_data["rectification_matrix"]["data"]
                    ci.P = cal_data["projection_matrix"]["data"]

                    calibration_dict[cal_data["rostopic"]] = ci

                elif topic == "/calibration_metadata":
                    calibration_dict["metadata"] = yaml.safe_load(msg.data)

                elif topic == "/tf_static":
                    calibration_dict["tf_static"] = msg

        calibration_data[calibration_dict["metadata"]["validity_start"]] = calibration_dict

    return calibration_data


def update_tf_static(tf_static_msg, start_end=False):
    reference_bag_path = get_bag("*_nuc_livox.bag")

    with rosbag.Bag(reference_bag_path, "r") as ref_bag:
        start_time = ref_bag.get_start_time()
        end_time = ref_bag.get_end_time()

    tf_static_bag_path = reference_bag_path.replace("_nuc_livox", "_tf_static")
    tf_static_start_end_bag_path = reference_bag_path.replace("_nuc_livox", "_tf_static_start_end")

    if tf_static_msg is None:
        raise ValueError("No tf_static message found in the calibration bag(s)")

    def write_repeated(output_bag_path: str, only_start_end: bool):
        with rosbag.Bag(output_bag_path, "w", compression="lz4") as out_bag:
            if only_start_end:
                times = [start_time, end_time] if end_time != start_time else [start_time]
            else:
                tcur = start_time
                times = []
                while tcur < end_time:
                    times.append(tcur)
                    tcur += 1.0
                if not times or times[-1] != end_time:
                    times.append(end_time)

            for ts in times:
                t_ros = rospy.Time.from_sec(ts)
                for i in range(len(tf_static_msg.transforms)):
                    tf_static_msg.transforms[i].header.stamp = t_ros
                out_bag.write("/tf_static", tf_static_msg, t_ros)

    write_repeated(tf_static_bag_path, only_start_end=False)
    write_repeated(tf_static_start_end_bag_path, only_start_end=True)

    upload_bag([tf_static_bag_path, tf_static_start_end_bag_path])


def is_camera_info(msg) -> bool:
    if getattr(msg, "_type", "") == "sensor_msgs/CameraInfo":
        return True
    return msg.__class__.__name__ == "CameraInfo" or ("CameraInfo" in str(type(msg)))


def calib_key_from_topic(topic: str) -> str:
    if topic == "/boxi/zed2i/left/camera_info":
        return "/gt_box/zed2i_driver_node/left_raw/image_raw_color"
    if topic == "/boxi/zed2i/right/camera_info":
        return "/gt_box/zed2i_driver_node/right_raw/image_raw_color"
    return topic.replace("/camera_info", "").replace("/color", "")


def lookup_calib_camera_info(calibration: Dict[str, Any], key: str) -> CameraInfo:
    for k in calibration.keys():
        if key in k:
            return calibration[k]

        try:
            parts = key.split("/")
            key1 = parts[2]
            key2 = parts[4]
            if key1 in k and key2 in k and "zed" in key:
                return calibration[k]
        except Exception:
            pass

    raise ValueError("Key %s not found in: %s" % (key, list(calibration.keys())))


def update_camera_info_inplace(calibration: Dict[str, Any], resolved_bags: Dict[str, str]):
    """
    For each bag matched by CAMERA_INFO_PATTERNS:
      - Rewrite the entire bag with LZ4 compression
      - Replace every existing CameraInfo message with the updated calibrated one
      - Atomically replace the original file with the updated one
    """
    for pattern in CAMERA_INFO_PATTERNS:
        if pattern not in resolved_bags:
            print("\033[91m[ERROR] Skipping pattern (bag not found): %s\033[0m" % pattern)
            continue
        bag_path = resolved_bags[pattern]
        in_path = Path(bag_path)

        if bag_path.endswith("_encoding.bag"):
            calib_path = Path(bag_path.replace("_encoding.bag", "_calib.bag"))
        elif bag_path.endswith("_color.bag"):
            calib_path = Path(bag_path.replace("_color.bag", "_calib.bag"))
        else:
            calib_path = in_path.with_suffix("").with_name(in_path.stem + "_calib.bag")

        with atomic_output(in_path) as tmp_path, atomic_output(calib_path) as tmp_calib_path, rosbag.Bag(str(in_path), "r") as inbag:
            total_messages = inbag.get_message_count()
            updated_count = 0

            with rosbag.Bag(str(tmp_path), "w", compression="lz4") as outbag:
                with rosbag.Bag(str(tmp_calib_path), "w", compression="lz4") as calibbag:
                    with tqdm(total=total_messages, desc="Updating %s" % in_path.name, unit="msgs") as pbar:
                        for topic, msg, t in inbag.read_messages():
                            if is_camera_info(msg):
                                key = calib_key_from_topic(topic)
                                calib_ci = lookup_calib_camera_info(calibration, key)

                                new_msg = copy.deepcopy(calib_ci)
                                new_msg.header = msg.header  # preserve original header/stamp
                                outbag.write(topic, new_msg, t)
                                calibbag.write(topic, new_msg, t)
                                updated_count += 1
                            else:
                                outbag.write(topic, msg, t)
                            pbar.update(1)
            if updated_count == 0:
                raise ValueError(f"No CameraInfo messages found in {in_path}; existing bags are preserved")

        upload_bag([str(in_path), str(calib_path)])


if __name__ == "__main__":
    # Pre-check: all expected target bags must exist locally (in the current directory).
    resolved_bags = check_expected_bags_present(CAMERA_INFO_PATTERNS, Path(MISSION_DATA))
    if not resolved_bags:
        print("\033[91m[ERROR] Required camera bags are missing or ambiguous — cannot continue.\033[0m")
        raise SystemExit(1)

    # Step 1: Load calibration data from bag file(s)
    calibration_data = load_calibration_data()

    # Step 2: Match calibration data based on mission timestamp
    reference_bag_path = get_bag("*_nuc_livox.bag")
    reference_date_str = Path(reference_bag_path).name.split("_")[0]
    reference_date = datetime.strptime(reference_date_str, "%Y-%m-%d-%H-%M-%S")

    calibration = None
    best_date = None
    best_s = np.inf

    for date_str, v in calibration_data.items():
        current_date = datetime.strptime(date_str, "%Y-%m-%d-%H-%M-%S")
        delta_s = (reference_date - current_date).total_seconds()

        if delta_s > 0 and delta_s < best_s:
            best_s = delta_s
            calibration = v
            best_date = date_str

    if calibration is None:
        raise RuntimeError("No suitable calibration found (no calibration date earlier than reference date).")

    print("Closest date: %s to reference date: %s" % (best_date, reference_date))

    # Step 3: Update tf_static
    update_tf_static(calibration["tf_static"])

    # Step 4: Update camera intrinsics IN-PLACE in the original bags
    update_camera_info_inplace(calibration, resolved_bags)
