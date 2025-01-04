"""
Generates data for extrinsic_offset experiment.
Adds a fixed offset to the tf_extrinsics between Hesai and CPT7 and creates following folder structure to easily run DLIO.
Copies the original bag files to the new experiment folders.

MISSION_DATA/extrinsic_offset:
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
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Offset from 1mm to 10cm
OFFSETS_X_M = [10**-3, 10**-2, 10**-1]
OFFSETS_PITCH_RAD = [np.deg2rad(1), np.deg2rad(10), np.deg2rad(30)]


def create_extrinsic_offset_bag(offset_x_m, offset_pitch_rad, output_folder, input_pattern, output_pattern):

    bag_input_path = get_bag("*" + input_pattern, rglob=False)
    output_bag_name = Path(bag_input_path).name.replace(input_pattern, output_pattern)
    output_bag_path = Path(output_folder) / output_bag_name
    output_bag_path.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(bag_input_path, "r") as bag_reader:
        with rosbag.Bag(str(output_bag_path), "w") as bag_writer:
            for topic, msg, t in bag_reader.read_messages():

                if topic == "/tf_static":
                    for transform in msg.transforms:
                        if transform.header.frame_id == "box_base" and transform.child_frame_id == "hesai_lidar":
                            transform.transform.translation.x += offset_x_m

                            # Extract current orientation quaternion
                            q = transform.transform.rotation
                            current_quat = [q.x, q.y, q.z, q.w]

                            # Convert quaternion to Euler angles
                            roll, pitch, yaw = euler_from_quaternion(current_quat)

                            # Add pitch offset
                            pitch += offset_pitch_rad

                            # Convert back to quaternion
                            new_quat = quaternion_from_euler(roll, pitch, yaw)

                            # Update the transform with the new quaternion
                            q.x, q.y, q.z, q.w = new_quat
                            break

                    bag_writer.write(topic, msg, t)
                else:
                    bag_writer.write(topic, msg, t)

    print(f"Generated bag with offset x{offset_x_m}m - pitch{offset_pitch_rad}rad: {str(output_bag_path)}")


if __name__ == "__main__":
    paths = [get_bag(pattern, rglob=False) for pattern in ["*_nuc_hesai_post_processed.bag", "*_cpt7_raw_imu.bag"]]

    for offset_x_m, offset_pitch_rad in zip(OFFSETS_X_M, OFFSETS_PITCH_RAD):
        output_pattern = "_tf_static.bag"
        input_pattern = "_tf_static.bag"  # Optionally allow to rename bag

        deg = round(np.rad2deg(offset_pitch_rad), 0)
        mm = round(offset_x_m / 0.001, 0)

        output_folder = Path(MISSION_DATA) / "extrinsic_offset" / f"extrinsic_x{mm}mm_pitch{deg}deg"
        create_extrinsic_offset_bag(
            offset_x_m,
            offset_pitch_rad,
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
