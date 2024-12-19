import rosbag
from pathlib import Path
import os

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")


def process_rosbag_imu(input_bag, imu_topic, new_frame_id, out_bag_path):
    """
    Process a rosbag by copying all messages and updating the frame_id of specified IMU topic.

    Args:
        input_bag (str): Path to input bag file
        imu_topic (str): IMU topic to modify
        new_frame_id (str): New frame_id to set
        out_bag_path (str): Path to output bag file
    """
    print(f"Processing bag: {input_bag}")
    print(f"Modifying frame_id to '{new_frame_id}' for topic: {imu_topic}")

    # Open input and output bags
    in_bag = rosbag.Bag(input_bag, "r")
    out_bag = rosbag.Bag(out_bag_path, "w", compression="lz4")

    try:
        # Iterate through all messages
        for topic, msg, t in in_bag.read_messages():
            if topic == imu_topic:
                # Update frame_id for IMU messages
                msg.header.frame_id = new_frame_id

            # Write message to output bag
            out_bag.write(topic, msg, t)

    finally:
        in_bag.close()
        out_bag.close()

    print(f"Finished processing. Modified bag saved as: {out_bag_path}")


if __name__ == "__main__":
    # Configuration
    config = {
        "in_pattern": "_nuc_alphasense.bag",
        "new_frame_id": "alphasense_imu",
        "imu_topic": "/gt_box/alphasense_driver_node/imu",  # Replace with your IMU topic
        "out_pattern": "_nuc_alphasense_frame_id.bag",
    }

    # Find all matching input bags
    bags = [str(s) for s in Path(MISSION_DATA).rglob("*" + config["in_pattern"])]
    print("\nProcessing the following bags:\n", bags)

    # Process each bag
    for input_bag in bags:
        output_bag = str(input_bag).replace(config["in_pattern"], config["out_pattern"])
        process_rosbag_imu(input_bag, config["imu_topic"], config["new_frame_id"], output_bag)
