import rosbag
import numpy as np
from box_auto.utils import get_bag, upload_bag

"""
Exit Codes:

EXIT CODE 0: Successful Verification
EXIT CODE 1: Totalstation position or prism moved
"""


def process_prism_position_bag(input_bag_path, output_bag_path, skip_check):
    # Read the input bag file
    input_bag = rosbag.Bag(input_bag_path)
    output_bag = rosbag.Bag(output_bag_path, "w")

    # Extract prism positions
    prism_positions = []
    for _, msg, t in input_bag.read_messages(topics=["/gt_box/ap20/prism_position"]):
        prism_positions.append((msg, t))

    # Get the first position as reference
    if not prism_positions:
        print("No prism position messages found!")
        exit(1)

    first_position = prism_positions[0][0]

    # Write filtered messages to new bag
    valid_positions = []
    for msg, t in prism_positions:
        # Calculate distance from the first position
        distance = np.sqrt(
            (msg.point.x - first_position.point.x) ** 2
            + (msg.point.y - first_position.point.y) ** 2
            + (msg.point.z - first_position.point.z) ** 2
        )

        # Only write messages beyond 20 cm (0.2 m) from the first position
        if distance > 0.2 or not skip_check:
            output_bag.write("/gt_box/ap20/prism_position", msg, t)

        valid_positions.append(msg)

    # Verify median of first and last 20 messages
    if len(prism_positions) < 40:
        input_bag.close()
        output_bag.close()
        exit(2)

    # Extract first and last 20 messages
    first_20_msgs = [msg for msg, t in prism_positions[:20]]
    last_20_msgs = [msg for msg, t in prism_positions[-20:]]

    # Compute medians for x, y, and z
    first_20_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in first_20_msgs], axis=0)
    last_20_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in last_20_msgs], axis=0)

    # Calculate absolute differences
    median_diff = np.abs(first_20_medians - last_20_medians)
    print(f"Median difference (x, y, z): {median_diff} -  Number of Prism Positions: ", len(prism_positions))

    # Check if median differences are smaller than 3 mm (0.003 m)
    if not np.all(median_diff < 0.003):
        if skip_check:
            print("Warning: Median differences exceed 3 mm! - But we will write everything to robot.bag")
            print("Skipping check - Exit 0")

            for msg, t in prism_positions:
                output_bag.write("/gt_box/ap20/prism_position", msg, t)

            input_bag.close()
            output_bag.close()
            upload_bag(output_bag_path)
            exit(3)
        else:
            print("Warning: Median differences exceed 3 mm! - Nothing will be written to robot.bag")
            # Close bags
            input_bag.close()
            output_bag.close()
            exit(4)
    else:
        print("Writing normally to robot bag")
        for msg, t in prism_positions:
            # Calculate distance from the first position
            distance = np.sqrt(
                (msg.point.x - first_position.point.x) ** 2
                + (msg.point.y - first_position.point.y) ** 2
                + (msg.point.z - first_position.point.z) ** 2
            )

            # Only write messages beyond 20 cm (0.2 m) from the first position
            if distance > 0.2:
                output_bag.write("/gt_box/ap20/prism_position", msg, t)

        # Close bags
        input_bag.close()
        output_bag.close()
        upload_bag(output_bag_path)
        exit(0)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Perform verification.")
    parser.add_argument(
        "--skip_check",
        action="store_true",  # This makes it a boolean flag
        help="If set, skip the validation checks.",
    )
    args = parser.parse_args()

    input_bag_path = get_bag("*_jetson_ap20_synced.bag")
    output_bag_path = input_bag_path.replace("_synced.bag", "_robot.bag")
    # Process the bag
    process_prism_position_bag(input_bag_path, output_bag_path, args.skip_check)
