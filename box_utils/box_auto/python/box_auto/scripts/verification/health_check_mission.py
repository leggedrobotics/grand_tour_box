import rosbag
import yaml
from pathlib import Path
from typing import Optional, Dict
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


def validate_bags(
    reference_folder: Optional[str] = None,
    yaml_file: Optional[str] = None,
    mission_folder: str = None,
    time_tolerance: float = 1.0,
) -> bool:
    """
    Validate ROS bag files between reference and mission folders.

    Args:
        reference_folder: Path to reference folder containing .bag files
        yaml_file: Path to yaml file with reference information
        mission_folder: Path to folder containing bags to validate
        time_tolerance: Tolerance in seconds for timing comparisons

    Returns:
        bool: True if validation passes, False otherwise
    """
    if reference_folder and yaml_file:
        raise ValueError("Cannot specify both reference_folder and yaml_file")
    if not reference_folder and not yaml_file:
        raise ValueError("Must specify either reference_folder or yaml_file")
    if not mission_folder:
        raise ValueError("Must specify mission_folder")

    # Generate or load reference data
    if reference_folder:
        reference_data = generate_reference_data(reference_folder)
        # Optionally save to yaml
        with open(YAML_FILE, "w") as f:
            yaml.dump(reference_data, f)
    else:
        if yaml_file == "default":
            yaml_file = YAML_FILE

        with open(yaml_file, "r") as f:
            reference_data = yaml.safe_load(f)

    # Validate mission folder
    return validate_mission_folder(reference_data, mission_folder, time_tolerance)


def generate_reference_data(folder_path: str) -> Dict:
    """Generate reference data from bag files in folder"""
    logger.info(f"Generating reference data from {folder_path}")
    reference_data = {}

    for bag_file in Path(folder_path).glob("*.bag"):
        logger.info(f"Processing {bag_file.name}")
        bag_key = bag_file.name[bag_file.name.find("_") :]  # Extract key from filename

        with rosbag.Bag(str(bag_file), "r") as bag:
            # Get bag info
            # Get bag info using get_type_and_topic_info()
            info = bag.get_type_and_topic_info()

            # Initialize bag key in reference data if not exists
            if bag_key not in reference_data:
                reference_data[bag_key] = {
                    "topics": {},
                }

            # Process each topic
            for topic, topic_info in info.topics.items():
                reference_data[bag_key]["topics"][topic] = {
                    "msg_type": topic_info.msg_type,
                    "freq": topic_info.frequency,
                    "checking": "freq",  # per default freq must match
                }

    return reference_data


def validate_mission_folder(reference_data: Dict, mission_folder: str, time_tolerance: float) -> bool:
    """Validate mission folder against reference data"""
    logger.info("-" * 90)
    logger.info(f"Validating mission folder: {mission_folder}")
    logger.info("-" * 90)
    validation_passed = True

    # Create a mapping of bag keys to mission bag files
    mission_bags = {}
    # Store timing information for all bags
    timing_info = {"start_times": {}, "end_times": {}}  # key -> start_time  # key -> end_time

    for bag_file in Path(mission_folder).glob("*.bag"):
        bag_key = bag_file.name[bag_file.name.find("_") :]
        mission_bags[bag_key] = bag_file

    # Validate topic information
    for ref_key, ref_info in reference_data.items():
        if ref_key not in mission_bags:
            logger.error(f"❌ Missing bag file for key '{ref_key}' in mission folder")
            validation_passed = False
            continue

        mission_bag_path = mission_bags[ref_key]
        logger.info(f"Checking {mission_bag_path.name}")

        with rosbag.Bag(str(mission_bag_path), "r") as bag:
            # Get bag info
            timing_info["start_times"][ref_key] = bag.get_start_time()
            timing_info["end_times"][ref_key] = bag.get_end_time()

            info = bag.get_type_and_topic_info()

            # Check topics
            for topic, ref_topic_info in ref_info["topics"].items():
                if topic not in info.topics:
                    logger.error(f"❌ Missing topic {topic} in {mission_bag_path.name}")
                    validation_passed = False
                    continue

                mission_topic_info = info.topics[topic]

                # Check message type
                if mission_topic_info.msg_type != ref_topic_info["msg_type"]:
                    logger.error(f"❌ Message type mismatch for {topic} in {mission_bag_path.name}")
                    logger.error(f"   Expected: {ref_topic_info['msg_type']}")
                    logger.error(f"   Got: {mission_topic_info.msg_type}")
                    validation_passed = False

                # Check frequency
                expected_freq = ref_topic_info["freq"]
                if expected_freq is not None:
                    actual_freq = mission_topic_info.frequency
                    freq_diff = abs(actual_freq - expected_freq)

                    if (freq_diff > expected_freq * 0.1) and abs(freq_diff - expected_freq) > 0.1:  # 10% tolerance

                        if ref_topic_info["checking"] == "freq":
                            loggi = logger.error
                            validation_passed = False
                        else:
                            logger.warning("⚠️ Frequency mismatch - No error set!")
                            loggi = logger.warning

                        loggi(f"⚠️ Frequency mismatch for {topic} in {mission_bag_path.name}")
                        loggi(f"   Expected: {expected_freq:.2f} Hz")
                        loggi(f"   Got: {actual_freq:.2f} Hz")

    deviations = {}

    if timing_info["start_times"]:
        median_start = statistics.median(timing_info["start_times"].values())
        median_end = statistics.median(timing_info["end_times"].values())

        logger.info(f"Median start time: {median_start:.2f}s")
        logger.info(f"Median end time: {median_end:.2f}s")

        # Check each bag's timing against medians
        for bag_key, start_time in timing_info["start_times"].items():
            end_time = timing_info["end_times"][bag_key]

            # Check start time deviation
            start_diff = start_time - median_start
            # Check end time deviation
            end_diff = end_time - median_end

            # Record deviations
            deviations[bag_key] = {"start_diff": start_diff, "end_diff": end_diff}

            if abs(start_diff) > 30:  # 30 seconds tolerance
                logger.error(f"❌ Start time deviation too large for {bag_key}")
                logger.error(f"   Deviation from median: {start_diff:.2f}s")
                validation_passed = False

            if abs(end_diff) > 30:  # 30 seconds tolerance
                logger.error(f"❌ End time deviation too large for {bag_key}")
                logger.error(f"   Deviation from median: {end_diff:.2f}s")
                validation_passed = False

    # Print summary of delays if validation passes
    if validation_passed:
        # Calculate max and min deviations
        max_start_dev = max(dev["start_diff"] for dev in deviations.values())
        max_end_dev = max(dev["end_diff"] for dev in deviations.values())
        min_start_dev = min(dev["start_diff"] for dev in deviations.values())
        min_end_dev = min(dev["end_diff"] for dev in deviations.values())

        logger.info("-" * 90)

        # Print table header
        logger.info("Overview of time delays per bag:")
        logger.info("-" * 85)
        logger.info(f"{'Bag Key':<45} {'Start Delay (s)':<20} {'End Delay (s)':<20}")
        logger.info("-" * 85)

        # Print each bag's deviations
        for bag_key, deviation in deviations.items():
            start_diff = deviation["start_diff"]
            end_diff = deviation["end_diff"]

            logger.info(f"{bag_key:<45} {start_diff:<20.2f} {end_diff:<20.2f}")
        logger.info("-" * 85)

        logger.info(f"Maximum start delay: {max_start_dev:.2f}s")
        logger.info(f"Minimum start delay: {min_start_dev:.2f}s")
        logger.info(f"Maximum end delay: {max_end_dev:.2f}s")
        logger.info(f"Minimum end delay: {min_end_dev:.2f}s")

        logger.info("-" * 90)
        logger.info("✅ All validation checks passed successfully!")
        logger.info("-" * 90)
    else:
        logger.error("❌ Some validation checks failed!")


import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script configuration")
    # Add argument for reference_folder
    parser.add_argument(
        "--reference_folder", type=str, default=None, help="Path to the reference folder. Defaults to None."
    )
    parser.add_argument(
        "--yaml_file",
        type=str,
        default=None,
        help="Path to the YAML file. Defaults to None. if default use the default one",
    )
    args = parser.parse_args()

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein download --mission {uuid} --dest {MISSION_DATA} '*.bag'")

    # Comment in to generate new reference data
    validation_passed = validate_bags(
        reference_folder=args.reference_folder,
        yaml_file=args.yaml_file,
        mission_folder=MISSION_DATA,
        time_tolerance=20,
    )

    return_code = 1
    if not validation_passed:
        return_code = -1

    sys.exit(return_code)
