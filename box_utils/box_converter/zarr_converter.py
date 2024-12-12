import os
import rosbag
import zarr
import numpy as np
from typing import Dict, Any, List
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()


# Recursively create Zarr arrays for message fields
def process_recursive(msg: Any, field_name: str, parent_group: zarr.hierarchy.Group, topic_info: Dict):
    """
    Recursively process message fields and create/populate Zarr arrays.

    Args:
        msg (Any): Value of the current field
        field_name (str): Name of the current field
        parent_group (zarr.hierarchy.Group): Parent Zarr group to add arrays to
    """
    is_image = False
    try:
        if msg._type == "sensor_msgs/CompressedImage" or msg._type == "sensor_msgs/Image":
            is_image = True
    except:
        pass

    if is_image:
        subgroup = parent_group

        if msg._type == "sensor_msgs/Image":
            cv_img = bridge.imgmsg_to_cv2(msg)
            nested_field_names = ["height", "width", "header"]

        else:
            nested_field_names = ["header"]
            nparray_img = np.fromstring(msg.data, np.uint8)
            if msg.format == "8UC1; jpeg compressed" or msg.format == "mono8; jpeg compressed":
                cv_img = cv2.imdecode(nparray_img, cv2.IMREAD_GRAYSCALE)
            elif msg.format == "rgb8; jpeg compressed bgr8":
                cv_img = cv2.imdecode(nparray_img, cv2.IMREAD_COLOR)
            elif msg.format == "bgr8; jpeg compressed bgr8" or msg.format == "jpg":
                cv_img = cv2.imdecode(nparray_img, cv2.IMREAD_COLOR)
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        process_recursive(cv_img, "image", subgroup, topic_info)

        # Only process selected subset
        for nested_field_name in nested_field_names:
            nested_msg = getattr(msg, nested_field_name)
            process_recursive(nested_msg, nested_field_name, subgroup, topic_info)

    # Handle basic types
    elif isinstance(msg, (int, float, str, bool)):
        # Create array on first message
        if topic_info["current_count"] == 0:
            dtype = type(msg)
            if dtype == str:
                # For strings, use object dtype with fixed max length
                max_length = 100
                parent_group.create_dataset(
                    field_name,
                    shape=(topic_info["message_count"],),
                    dtype=f"S{max_length}",
                    object_codec=zarr.VLenUTF8(),
                )
            else:
                parent_group[field_name] = parent_group.create_dataset(
                    field_name, shape=(topic_info["message_count"],), dtype=dtype
                )

        # Write the value
        parent_group[field_name][topic_info["current_count"]] = msg

    # Handle lists/arrays
    elif isinstance(msg, (tuple, list, np.ndarray)):
        # Create array on first message
        if topic_info["current_count"] == 0:
            # Determine dtype
            if len(msg) > 0:
                chunks = None
                if isinstance(msg, (np.ndarray)):
                    dtype = msg.dtype
                    s = (topic_info["message_count"],) + msg.shape

                    if len(msg.shape) == 3 and msg.shape[2] == 3:
                        # found image
                        chunks = (1, 256, 256, 3)
                else:
                    dtype = type(msg[0])
                    s = (topic_info["message_count"], len(msg))

                # Create dataset with predefined shape
                parent_group.create_dataset(field_name, shape=s, dtype=dtype, chunks=chunks)

        # Write the array/list
        parent_group[field_name][topic_info["current_count"]] = msg

    # Handle nested messages (recursively process)
    elif hasattr(msg, "__slots__"):
        # Create subgroup if it doesn't exist
        if field_name != "":
            # Avoids nesting empty key -> required to pass in full message
            if field_name not in parent_group:
                subgroup = parent_group.create_group(field_name)
            else:
                subgroup = parent_group[field_name]
        else:
            subgroup = parent_group

        # Recursively process nested message fields
        for nested_field_name in msg.__slots__:
            nested_msg = getattr(msg, nested_field_name)
            process_recursive(nested_msg, nested_field_name, subgroup, topic_info)
    else:
        print("Not implemented")


def convert(bag_path: str, zarr_path: str, topics: List[str] = None, chunk_size: int = 10000):
    """
    Convert specified topics from ROS bag to Zarr

    Args:
        topics (List[str], optional): List of topics to convert.
                                        Converts all topics if None.
        chunk_size (int, optional): Number of messages to store in each Zarr chunk
    """
    os.makedirs(zarr_path, exist_ok=True)

    def topic_to_key(t):
        return topic.replace("/", "_")[1:]

    with rosbag.Bag(bag_path) as bag:

        info = bag.get_type_and_topic_info()
        # TODO: add a filter options which topics to convert

        # Create all the groups
        topic_mapping = {}
        with zarr.open(zarr_path, mode="w") as zar_file:
            for topic, topic_info_bag in info.topics.items():
                g = zar_file.create_group(topic_to_key(topic))
                topic_mapping[topic] = {
                    "zarr_group": g,
                    "message_count": topic_info_bag.message_count,
                    "msg_type": topic_info_bag.msg_type,
                    "current_count": 0,
                }

            for topic, msg, arrival_time in bag.read_messages(topics=topics):
                zarr_group = topic_mapping[topic]["zarr_group"]

                # Process all fields in the message
                process_recursive(msg, "", zarr_group, topic_mapping[topic])

                # Update current count
                topic_mapping[topic]["current_count"] += 1
                print(topic, topic_mapping[topic]["current_count"])

                if topic_mapping[topic]["current_count"] == 100:
                    break

            print(f"Conversion complete. Zarr files saved to {zarr_path}")


def main():
    import argparse

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Convert ROS bag to Zarr")
    parser.add_argument(
        "--bag_path",
        type=str,
        default="/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/2024-10-01-11-29-55_nuc_alphasense_cor.bag",
        help="Path to input ROS bag file",
    )
    parser.add_argument(
        "--zarr_path",
        type=str,
        default="/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/2024-10-01-11-29-55_nuc_alphasense_cor.zarr",
        help="Path to output Zarr directory",
    )
    parser.add_argument("--topics", type=str, nargs="*", help="Optional list of topics to convert")
    parser.add_argument("--chunk_size", type=int, default=10000, help="Chunk size for Zarr storage", required=False)

    # Parse arguments
    args = parser.parse_args()

    # Create and run converter
    convert(zarr_path=args.zarr_path, bag_path=args.bag_path, topics=args.topics, chunk_size=args.chunk_size)


if __name__ == "__main__":
    main()
