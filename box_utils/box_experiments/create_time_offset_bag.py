"""
Generates data for time_offset experiment.
Adds a fixed offset to the timestamp of the IMU data and creates following folder structure to easily run DLIO.
Copies the original bag files to the new experiment folders.

MISSION_DATA/time_offset:
├── offset_10000_ns
    ├── {MISSION_DATA}/cpt7_raw_imu.bag
    ├── {MISSION_DATA}/nuc_hesai_post_processed.bag
    ├── {MISSION_DATA}/tf_static.bag
├── offset_1000_ns
    ├── {MISSION_DATA}/cpt7_raw_imu.bag
    ├── {MISSION_DATA}/nuc_hesai_post_processed.bag
    ├── {MISSION_DATA}/tf_static.bag
├── offset_100_ns
    ├── {MISSION_DATA}/cpt7_raw_imu.bag
    ├── {MISSION_DATA}/nuc_hesai_post_processed.bag
    ├── {MISSION_DATA}/tf_static.bag

"""

from box_auto.utils import get_bag
from box_auto.utils import MISSION_DATA
import rosbag
from pathlib import Path
import shutil

# Offset from 1us to 100ms -> given in ns
# OFFSETS = [10**4, 10**5, 10**6, 10**7, 10**8, -10**4, -10**5, -10**6, -10**7, -10**8]
# OFFSETS = [10**6, 10**7, -10**6, -10**7]
OFFSETS = [1]


def create_time_offset_bag(offset_time_in_ns, topic_name, output_folder, input_pattern, output_pattern):
    bag_input_path = get_bag("*" + input_pattern, rglob=False)
    output_bag_name = Path(bag_input_path).name.replace(input_pattern, output_pattern)
    output_bag_path = Path(output_folder) / output_bag_name
    output_bag_path.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(bag_input_path, "r") as bag_reader:
        with rosbag.Bag(str(output_bag_path), "w") as bag_writer:
            for topic, msg, t in bag_reader.read_messages():

                if topic == topic_name:
                    new_nsecs = msg.header.stamp.nsecs + offset_time_in_ns
                    # Handle overflow of nanoseconds (>1 second)
                    extra_secs = new_nsecs // 1_000_000_000
                    new_nsecs = new_nsecs % 1_000_000_000

                    msg.header.stamp.secs += extra_secs
                    msg.header.stamp.nsecs = new_nsecs

                    bag_writer.write(topic, msg, msg.header.stamp)
                else:
                    bag_writer.write(topic, msg, t)

    print(f"Generated bag with offset {offset_time_in_ns}ns: {str(output_bag_path)}")


if __name__ == "__main__":
    paths = [get_bag(pattern, rglob=False) for pattern in ["[0-9]*_tf_static.bag", "*_nuc_hesai_post_processed.bag"]]

    for offset_time_in_ns in OFFSETS:
        output_pattern = "_cpt7_raw_imu.bag"
        input_pattern = "_cpt7_raw_imu.bag"
        output_folder = Path(MISSION_DATA) / "time_offset_pm" / f"offset_{offset_time_in_ns}_ns"
        create_time_offset_bag(
            offset_time_in_ns,
            "/gt_box/cpt7/offline_from_novatel_logs/imu",
            output_folder,
            input_pattern,
            output_pattern,
        )

        for bag_path in paths:
            shutil.copy(bag_path, output_folder)

    def print_folder_structure(folder_path):
        """Print a tree-like structure of the given folder."""
        print(f"\nFolder structure of {folder_path}:")
        for path in sorted(Path(folder_path).rglob("*")):
            # Calculate the relative path from the base folder
            relative_path = path.relative_to(folder_path)
            # Calculate the depth for indentation
            depth = len(relative_path.parts) - 1
            # Print with appropriate indentation
            print(f"{'    ' * depth}├── {path.name}")

    print_folder_structure(output_folder.parent)
