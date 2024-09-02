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

def downgrade_camerainfo_to_rosbag1(src: Path, dst: Path):
    typename = "sensor_msgs/msg/CameraInfo"
    typestore = get_typestore(Stores.ROS1_NOETIC)

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
        with rosbag.Bag(split_filename, "w") as outbag:
            for topic, msg, t in bag.read_messages(
                start_time=rospy.Time.from_sec(current_time),
                end_time=rospy.Time.from_sec(min(current_time + duration_minutes * 60, end_time)),
            ):
                outbag.write(topic, msg, t)
        current_time += duration_minutes * 60
        index += 1



if __name__ == "__main__":
    mcaps = [str(Path(p).parent) for p in glob.glob("/mission_data/*/*.mcap", recursive=True)]
    mcaps = np.unique(np.array(mcaps)).tolist()
    print(f"Found {len(mcaps)} directories to process: \n {mcaps}")
    for i, mcap in enumerate(mcaps):

        # We at first convert all missions to ROS1 - we have to merge the bags into a single mission for the hdr cameras given the metdata.yaml
        timestamp = Path(mcap).parent.name.split("_")[0]
        camera_direction = mcap.split("_")[-1]
        converted_bag_path = Path(mcap).parent.joinpath(f"{timestamp}_jetson_hdr_{camera_direction}_raw.bag")
        if not converted_bag_path.exists():
            cmd = f"rosbags-convert {mcap} --dst {str(converted_bag_path)}"
            shell_run(cmd)
            print("Converted to ROS1 raw format, types are not yet converted.")
        else:
            print(f"Skipping {mcap} conversion.")

        downgraded_bag_path = str(converted_bag_path).replace("raw", "downgraded")
        if not Path(downgraded_bag_path).exists():
            # Downgrade CameraInfo and write to a new bag
            print("Downgrading CameraInfo to ROS1 format and saving")
            downgrade_camerainfo_to_rosbag1(converted_bag_path, Path(downgraded_bag_path))
        else:
            print(f"Skipping {mcap} downgraded.")

        # Split the downgraded bag into 5-minute chunks
        print("Splitting the downgraded bag into 5-minute chunks.")
        split_rosbags(downgraded_bag_path, camera_direction=camera_direction)
        # Remove intermediate artifacts
        os.remove(converted_bag_path)
        os.remove(downgraded_bag_path)

    print("All directories processed. Split bags are available in each directory.")