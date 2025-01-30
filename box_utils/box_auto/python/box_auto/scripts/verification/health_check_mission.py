import rosbag
import yaml
from pathlib import Path
from typing import Optional, Dict, Union
import logging
from colorama import init, Fore, Style
import statistics
import sys
import os
from box_auto.utils import WS, MISSION_DATA, ARTIFACT_FOLDER

YAML_FILE = str(Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg/health_check_reference_data.yaml")
LOG_FILE = Path(ARTIFACT_FOLDER) / "health_check_mission" / "result.log"
LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
LOG_FILE = str(LOG_FILE)

# Initialize colorama for colored output
init()

# Configure logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.propagate = False

class ColoredFormatter(logging.Formatter):
    """Custom formatter for colored logging"""
    format_str = "%(levelname)s - %(message)s"
    FORMATS = {
        logging.DEBUG: Fore.CYAN + format_str + Style.RESET_ALL,
        logging.INFO: Fore.GREEN + format_str + Style.RESET_ALL,
        logging.WARNING: Fore.YELLOW + format_str + Style.RESET_ALL,
        logging.ERROR: Fore.RED + format_str + Style.RESET_ALL,
        logging.CRITICAL: Fore.RED + Style.BRIGHT + format_str + Style.RESET_ALL,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class FileFormatter(logging.Formatter):
    """Plain text formatter for file logging"""

    def __init__(self):
        super().__init__("%(asctime)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")


# Set up console handler with colored output
console_handler = logging.StreamHandler()
console_handler.setFormatter(ColoredFormatter())


# Set up file handler with plain text output
file_handler = logging.FileHandler(LOG_FILE)
file_handler.setFormatter(FileFormatter())

# Clear any existing handlers and add our new ones
logger.handlers = []
logger.addHandler(console_handler)
logger.addHandler(file_handler)
logger.propagate = False


def validate_bags(
    reference_folder: Optional[str] = None,
    yaml_file: Optional[str] = None,
    mission_folder: str = None,
    time_tolerance: float = 1.0
) -> bool:
    if reference_folder and yaml_file:
        raise ValueError("Cannot specify both reference_folder and yaml_file")
    if not reference_folder and not yaml_file:
        raise ValueError("Must specify either reference_folder or yaml_file")
    if not mission_folder:
        raise ValueError("Must specify mission_folder")

    if reference_folder:
        logger.info("Generating reference data from existing bags...")
        reference_data = generate_reference_data(reference_folder)
        with open(YAML_FILE, "w") as f:
            yaml.dump(reference_data, f)
    else:
        if yaml_file == "default":
            yaml_file = YAML_FILE
        with open(yaml_file, "r") as f:
            reference_data = yaml.safe_load(f)

    return validate_mission_folder(reference_data, mission_folder, time_tolerance)

def generate_reference_data(folder_path: str) -> Dict:
    """
    Autogenerate reference data from existing bags in a reference folder.
    You may manually edit the resulting YAML to specify freq_tolerance_percent, 
    dropped_frames_threshold_percent, etc.
    """
    reference_data = {}
    for bag_file in Path(folder_path).glob("*.bag"):
        bag_key = bag_file.name[bag_file.name.find("_") :]
        with rosbag.Bag(str(bag_file), "r") as bag:
            info = bag.get_type_and_topic_info()
            if bag_key not in reference_data:
                reference_data[bag_key] = {"topics": {}}
            for topic, topic_info in info.topics.items():
                reference_data[bag_key]["topics"][topic] = {
                    "msg_type": topic_info.msg_type,
                    "freq": topic_info.frequency,  # from ROS Bag's metadata
                    "checking": ["freq", "dropped_frames"],
                    "freq_tolerance_percent": 10.0,
                    "dropped_frames_threshold_percent": 5.0,
                }
    return reference_data

def validate_mission_folder(reference_data: Dict, mission_folder: str, time_tolerance: float) -> bool:
    """
    Core validation function that checks:
      1) Missing bag files
      2) Missing topics or type mismatches
      3) (Optionally) Frequency checks (from bag metadata)
      4) (Optionally) Dropped frames checks
      5) Timing alignment across bags
    """
    logger.info("-" * 90)
    logger.info("Checking for missing bag files...")
    logger.info("-" * 90)

    mission_bags = {
        bag_file.name[bag_file.name.find("_") :]: bag_file
        for bag_file in Path(mission_folder).glob("*.bag")
    }

    missing_bags = [ref_key for ref_key in reference_data if ref_key not in mission_bags]
    if missing_bags:
        for missing in missing_bags:
            logger.error(f"❌ Missing bag file for key '{missing}' in mission folder")
    else:
        logger.info("✅ No missing bag files.")

    logger.info("-" * 90)
    logger.info("Checking available bags for topic and timing issues...")
    logger.info("-" * 90)

    validation_passed = True
    timing_info = {"start_times": {}, "end_times": {}}

    for ref_key, ref_info in reference_data.items():
        if ref_key not in mission_bags:
            continue  # Already reported as missing
        mission_bag_path = mission_bags[ref_key]
        logger.info("-" * 90)
        logger.info(f"Checking {mission_bag_path.name}")
        with rosbag.Bag(str(mission_bag_path), "r") as bag:
            timing_info["start_times"][ref_key] = bag.get_start_time()
            timing_info["end_times"][ref_key] = bag.get_end_time()
            info = bag.get_type_and_topic_info()

            for topic, ref_topic_info in ref_info["topics"].items():
                # Check topic existence and type
                if topic not in info.topics:
                    logger.error(f"❌ Missing topic {topic} in {mission_bag_path.name}")
                    validation_passed = False
                    continue
                elif info.topics[topic].msg_type != ref_topic_info["msg_type"]:
                    logger.error(f"❌ Type mismatch for topic {topic} in {mission_bag_path.name}")
                    validation_passed = False
                    continue

                # Now perform checks based on "checking"
                checks = ref_topic_info.get("checking", [])
                # Allow user to specify as a string or list
                if isinstance(checks, str):
                    checks = [checks]

                # If "freq" is requested => check bag's own topic_info.frequency
                if "freq" in checks:
                    freq_ok = check_topic_frequency(info, topic, ref_topic_info, mission_bag_path.name)
                    if not freq_ok:
                        validation_passed = False

                # If "dropped_frames" is requested => check message gaps
                if "dropped_frames" in checks:
                    dropped_ok = check_dropped_frames(bag, topic, ref_topic_info, mission_bag_path.name)
                    if not dropped_ok:
                        validation_passed = False

    # Finally, check timing alignment across all bags
    logger.info("-" * 90)
    logger.info("Timing analysis of available bags...")
    logger.info("-" * 90)

    if timing_info["start_times"]:
        median_start = statistics.median(timing_info["start_times"].values())
        # median_end = statistics.median(timing_info["end_times"].values())  # Potentially used if needed

        for bag_key, start_time in timing_info["start_times"].items():
            if abs(start_time - median_start) > time_tolerance:
                type = "early" if start_time < median_start else "late"
                logger.error(f"❌ Start time deviation too large for {bag_key}. Bag started {type}.")
                logger.error(f"    Deviation: {(start_time - median_start):.2f}s")
                validation_passed = False
    if timing_info["end_times"]:
        median_end = statistics.median(timing_info["end_times"].values())
        for bag_key, end_time in timing_info["end_times"].items():
            if abs(end_time - median_end) > time_tolerance:
                type = "early" if end_time < median_end else "late"
                logger.error(f"❌ End time deviation too large for {bag_key}. Bag ended {type}.")
                logger.error(f"    Deviation: {(end_time - median_end):.2f}s")
                validation_passed = False

    if validation_passed:
        logger.info("-" * 90)
        logger.info("✅ All checks passed.")
    else:
        logger.info("-" * 90)
        logger.error("❌ Some checks failed. See logs for details.")
    return validation_passed

def check_topic_frequency(
    info,
    topic: str,
    ref_topic_info: Dict[str, Union[float, str, list]],
    bag_name: str
) -> bool:
    """
    Frequency check using the bag's own metadata (topic_info.frequency).
    We compare it to the reference freq from the YAML and ensure 
    it is within freq_tolerance_percent.
    """
    actual_freq = info.topics[topic].frequency
    ref_freq = ref_topic_info.get("freq", None)
    freq_tolerance_percent = ref_topic_info.get("freq_tolerance_percent", 10.0)

    # If we have no reference freq, skip
    if not ref_freq or ref_freq <= 0:
        return True

    # If the bag does not have a frequency for this topic (rare, but can happen)
    if actual_freq is None or actual_freq <= 0:
        logger.warning(f"⚠️  No valid frequency metadata for {topic} in {bag_name}. Skipping freq check.")
        return True

    # Calculate how far off we are from the reference, in percent
    difference_percent = abs(actual_freq - ref_freq) / ref_freq * 100.0
    if difference_percent > freq_tolerance_percent:
        logger.error(
            f"❌ Frequency too far off for {topic} in {bag_name}. "
            f"Expected ~{ref_freq:.2f} Hz, got {actual_freq:.2f} Hz "
            f"({difference_percent:.2f}% off > {freq_tolerance_percent:.2f}%)."
        )
        return False
    else:
        logger.info(
            f"✅ Frequency check passed for {topic} in {bag_name}. "
            f"Expected ~{ref_freq:.2f} Hz, got {actual_freq:.2f} Hz "
            f"({difference_percent:.2f}% off <= {freq_tolerance_percent:.2f}%)."
        )
    return True

def check_dropped_frames(
    bag: rosbag.Bag,
    topic: str,
    ref_topic_info: Dict[str, Union[float, str, list]],
    bag_name: str
) -> bool:
    """
    Dropped frames check:
      1) Gather all timestamps for the topic
      2) Calculate the nominal period = 1 / freq
      3) Count how many consecutive gaps are > 1.5 * nominal period
      4) Compare count to the threshold in the yaml, e.g. dropped_frames_threshold_percent
    """
    ref_freq = ref_topic_info.get("freq", None)
    if not ref_freq or ref_freq <= 0:
        logger.warning(f"⚠️  No valid reference freq to check dropped_frames for {topic} in {bag_name}. Skipping.")
        return True

    threshold_percent = ref_topic_info.get("dropped_frames_threshold_percent", 5.0)
    nominal_period = 1.0 / ref_freq

    # Gather timestamps
    timestamps = []
    for _, _, t in bag.read_messages(topics=[topic]):
        timestamps.append(t.to_sec())

    if len(timestamps) < 2:
        # Not enough data to do gap analysis
        logger.warning(f"⚠️  Not enough messages on {topic} in {bag_name} for dropped_frames check.")
        return True

    # Sort timestamps (just in case)
    timestamps.sort()

    big_gaps = 0
    for i in range(1, len(timestamps)):
        gap = timestamps[i] - timestamps[i - 1]
        # Check if gap is > 1.5 * nominal_period
        if gap > 1.5 * nominal_period:
            big_gaps += 1

    # Compare big_gaps with total gaps => a percent
    total_gaps = len(timestamps) - 1
    big_gaps_percent = (big_gaps / total_gaps) * 100.0 if total_gaps > 0 else 0

    if big_gaps_percent > threshold_percent:
        logger.error(
            f"❌ Dropped frames too high for {topic} in {bag_name}: "
            f"{big_gaps} large gaps out of {total_gaps} ({big_gaps_percent:.2f}%), "
            f"threshold={threshold_percent:.2f}%"
        )
        return False
    elif big_gaps > 0:
        logger.warning(
            f"⚠️  Some dropped frames for {topic} in {bag_name}: "
            f"{big_gaps} large gaps out of {total_gaps} ({big_gaps_percent:.2f}%)."
        )
    else:
        logger.info(f"✅ Dropped frames check passed for {topic} in {bag_name}.")

    return True

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ROS Bag Validation Script")
    parser.add_argument("--reference_folder", type=str, default=None, help="Path to reference folder")
    parser.add_argument("--yaml_file", type=str, default=None, help="Path to YAML file")
    args = parser.parse_args()

    validation_passed = validate_bags(
        reference_folder=args.reference_folder,
        yaml_file=args.yaml_file,
        mission_folder=MISSION_DATA,
        time_tolerance=20,
    )
    sys.exit(0 if validation_passed else -1)
