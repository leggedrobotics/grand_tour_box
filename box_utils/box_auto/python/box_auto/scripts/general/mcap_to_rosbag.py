import os
import glob
from boxi import shell_run
import rosbag
import rospy
from rosbags.highlevel.anyreader import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
import numpy as np
import yaml
import rospkg
from sensor_msgs.msg import CameraInfo
import copy

from box_auto.utils import MISSION_DATA


def load_camera_info_from_yaml(yaml_path):
    with open(yaml_path, "r") as f:
        calib_data = yaml.safe_load(f)

    # Create a CameraInfo message
    camera_info = CameraInfo()
    camera_info.width = calib_data["image_width"]
    camera_info.height = calib_data["image_height"]
    camera_info.K = calib_data["camera_matrix"]["data"]
    camera_info.D = calib_data["distortion_coefficients"]["data"]
    camera_info.R = calib_data["rectification_matrix"]["data"]
    camera_info.P = calib_data["projection_matrix"]["data"]
    camera_info.distortion_model = calib_data["distortion_model"]
    return camera_info


def get_calibration_file_path(camera_name):
    rospack = rospkg.RosPack()
    try:
        box_calib_path = rospack.get_path("box_calibration")
        calib_file = os.path.join(box_calib_path, "calibration", "hdr", f"{camera_name}.yaml")
        if os.path.exists(calib_file):
            return calib_file
    except rospkg.ResourceNotFound:
        print("Could not find box_calibration package")
    return None


def downgrade_camerainfo_to_rosbag1(src: Path, dst: Path, replace: bool = False):
    typename = "sensor_msgs/msg/CameraInfo"
    typestore = get_typestore(Stores.ROS1_NOETIC)
    if replace:
        camera_info_msg = {}
        seq_per_topic = {}
        with rosbag.Bag(src, "r") as input_bag:
            with rosbag.Bag(dst, "w") as output_bag:
                for topic, msg, t in input_bag.read_messages():
                    if topic not in seq_per_topic:
                        seq_per_topic[topic] = 0

                    if "camera_info" in topic:
                        if topic not in camera_info_msg:
                            # Extract camera name from frame_id
                            camera_name = msg.header.frame_id.split("/")[-1]
                            calib_file = get_calibration_file_path(camera_name)
                            new_camera_info = load_camera_info_from_yaml(calib_file)
                            camera_info_msg[topic] = new_camera_info

                        new_camera_info = copy.deepcopy(camera_info_msg[topic])
                        new_camera_info.header = msg.header
                        new_camera_info.header.seq = seq_per_topic[topic]

                        output_bag.write(topic, new_camera_info, t)

                    else:
                        # Write original message if not replaced
                        msg.header.seq = seq_per_topic[topic]
                        output_bag.write(topic, msg, t)
                    seq_per_topic[topic] += 1

    else:
        with AnyReader([src]) as reader, Writer(dst) as writer:
            conn_map = {}
            for conn in reader.connections:
                if conn.msgtype == "sensor_msgs/msg/CameraInfo":
                    from_typestore = typestore
                else:
                    from_typestore = reader.typestore

                conn_map[conn.id] = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    typestore=from_typestore,
                    callerid=conn.ext.callerid,
                    latching=conn.ext.latching,
                )

            for conn, timestamp, data in reader.messages():
                wconn = conn_map[conn.id]
                if conn.msgtype == "sensor_msgs/msg/CameraInfo":
                    msg = reader.deserialize(data, conn.msgtype)
                    converted_msg = typestore.types[typename](
                        header=msg.header,
                        height=msg.height,
                        width=msg.width,
                        distortion_model=msg.distortion_model,
                        D=msg.d,
                        K=msg.k,
                        R=msg.r,
                        P=msg.p,
                        binning_x=msg.binning_x,
                        binning_y=msg.binning_y,
                        roi=msg.roi,
                    )
                    outdata = typestore.serialize_ros1(converted_msg, conn.msgtype)
                else:
                    outdata = data

                writer.write(wconn, timestamp, outdata)


def split_rosbags(input_bag_path, camera_direction, duration_minutes=5):
    """Splits ROS bags into smaller chunks based on duration."""
    bag = rosbag.Bag(input_bag_path, "r")
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    current_time = start_time
    index = 0

    while current_time < end_time:
        split_filename = input_bag_path.replace(
            f"jetson_hdr_{camera_direction}_downgraded.bag", f"jetson_hdr_{camera_direction}_{index}.bag"
        )
        with rosbag.Bag(split_filename, "w", compression="lz4") as outbag:
            for topic, msg, t in bag.read_messages(
                start_time=rospy.Time.from_sec(current_time),
                end_time=rospy.Time.from_sec(min(current_time + duration_minutes * 60, end_time)),
            ):
                outbag.write(topic, msg, t)
        current_time += duration_minutes * 60
        index += 1


if __name__ == "__main__":
    mcaps = [str(Path(p).parent) for p in glob.glob(f"{MISSION_DATA}/*/*.mcap", recursive=True)]
    mcaps = np.unique(np.array(mcaps)).tolist()
    print(f"Found {len(mcaps)} directories to process: \n {mcaps}")
    for _i, mcap in enumerate(mcaps):
        # We at first convert all missions to ROS1 - we have to merge the bags into a single mission for the hdr cameras given the metdata.yaml
        timestamp = Path(mcap).name.split("_")[0]
        camera_direction = mcap.split("_")[-1]
        converted_bag_path = Path(mcap).parent.joinpath(f"{timestamp}_jetson_hdr_{camera_direction}_raw.bag")
        if not converted_bag_path.exists():
            cmd = f"rosbags-convert {mcap} --dst {str(converted_bag_path)}"
            print(cmd)
            shell_run(cmd)
            print("Converted to ROS1 raw format, types are not yet converted.")
        else:
            print(f"Skipping {mcap} conversion.")

        downgraded_bag_path = str(converted_bag_path).replace("_raw", "")
        if not Path(downgraded_bag_path).exists():
            # Downgrade CameraInfo and write to a new bag
            print("Downgrading CameraInfo to ROS1 format and saving")
            downgrade_camerainfo_to_rosbag1(converted_bag_path, Path(downgraded_bag_path), True)
        else:
            print(f"Skipping {mcap} downgraded.")

        # Split the downgraded bag into 5-minute chunks
        # print("Splitting the downgraded bag into 5-minute chunks.")
        # split_rosbags(downgraded_bag_path, camera_direction=camera_direction)
        # Remove intermediate artifacts
        os.remove(converted_bag_path)
        # os.remove(downgraded_bag_path)

    print("All directories processed. Split bags are available in each directory.")
