import os
import glob
from boxi import shell_run
import rosbag
import rospy
from rosbags.highlevel.anyreader import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from pathlib import Path


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument(
        "--pattern", type=str, required=True, help="Glob pattern to match run directories, eg '2024-06-27*'"
    )
    return parser


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


def split_rosbags(input_bag_path, output_run_id, duration_minutes=5):
    """Splits ROS bags into smaller chunks based on duration."""
    bag = rosbag.Bag(input_bag_path, "r")
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    current_time = start_time
    index = 0

    while current_time < end_time:
        split_filename = f"{output_run_id}/{output_run_id}_jetson_hdr_{index}.bag"
        with rosbag.Bag(split_filename, "w") as outbag:
            for topic, msg, t in bag.read_messages(
                start_time=rospy.Time.from_sec(current_time),
                end_time=rospy.Time.from_sec(min(current_time + duration_minutes * 60, end_time)),
            ):
                outbag.write(topic, msg, t)
        current_time += duration_minutes * 60
        index += 1


def main(args):
    dirs = glob.glob(args.pattern)
    total_dirs = len(dirs)
    counter = 1

    for dir in dirs:
        if os.path.isdir(dir):
            run_id = os.path.basename(dir)
            print(f"Processing directory {counter} of {total_dirs}: {run_id}")
            converted_bag_path = f"./{run_id}/{run_id}_jetson_hdr_raw.bag"
            cmd = f"rosbags-convert {run_id}/hdr/ --dst {converted_bag_path}"
            shell_run(cmd)
            print("Converted to ROS1 raw format, types are not yet converted.")

            # Downgrade CameraInfo and write to a new bag
            output_path = f"./{run_id}/{run_id}_jetson_hdr_downgraded.bag"
            print(f"Downgrading CameraInfo to ROS1 format and saving to {output_path}")
            downgrade_camerainfo_to_rosbag1(Path(converted_bag_path), Path(output_path))

            # Split the downgraded bag into 5-minute chunks
            print("Splitting the downgraded bag into 5-minute chunks.")
            split_rosbags(output_path, run_id)

            counter += 1

    print("All directories processed. Split bags are available in each directory.")
