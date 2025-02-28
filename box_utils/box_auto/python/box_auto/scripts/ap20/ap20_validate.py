import rosbag
import numpy as np
from box_auto.utils import get_bag, upload_bag, MISSION_DATA


"""
Exit Codes:

EXIT CODE 0: Successful Verification
EXIT CODE 1: Totalstation position or prism moved
"""


def process_prism_position_bag(input_bag_path, output_bag_path, skip_check, remove_start, return_check_only):
    # Read the input bag file
    with rosbag.Bag(input_bag_path) as input_bag:
        # Extract prism positions
        prism_positions = []
        for _, msg, t in input_bag.read_messages(topics=["/gt_box/ap20/prism_position"]):
            prism_positions.append((msg, t))

        # Get the first position as reference
        if not prism_positions:
            print("No prism position messages found!")
            return 1, [np.inf, np.inf, np.inf]

        first_position = prism_positions[0][0]

        # Verify median of first and last 20 messages
        if len(prism_positions) < 40:
            return 2, [np.inf, np.inf, np.inf]

        # Extract first and last 20 messages
        first_20_msgs = [msg for msg, t in prism_positions[:20]]
        last_20_msgs = [msg for msg, t in prism_positions[-20:]]

        # Compute medians for x, y, and z
        first_20_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in first_20_msgs], axis=0)
        last_20_medians = np.median([[msg.point.x, msg.point.y, msg.point.z] for msg in last_20_msgs], axis=0)

        # Calculate absolute differences
        median_diff = np.abs(first_20_medians - last_20_medians)
        print(f"Median difference (x, y, z): {median_diff} -  Number of Prism Positions: ", len(prism_positions))

        if return_check_only:
            return 0, median_diff

        # Check if median differences are smaller than 3 mm (0.003 m)
        if not np.all(median_diff < 0.003) and not skip_check:
            print(f"Error {MISSION_DATA}: Median differences exceed 3 mm! - Nothing will be written to robot.bag")
            # Close bags
            return 4, median_diff

        print("Writing normally to robot bag")
        with rosbag.Bag(output_bag_path, "w", compression="lz4") as output_bag:
            for msg, t in prism_positions:
                # Calculate distance from the first position
                distance = np.sqrt(
                    (msg.point.x - first_position.point.x) ** 2
                    + (msg.point.y - first_position.point.y) ** 2
                    + (msg.point.z - first_position.point.z) ** 2
                )
                if remove_start:
                    if distance > 0.2:
                        # Only write messages beyond 20 cm (0.2 m) from the first position
                        output_bag.write("/gt_box/ap20/prism_position", msg, t)
                else:
                    output_bag.write("/gt_box/ap20/prism_position", msg, t)

        # Close bags
        upload_bag(output_bag_path)
        return 0, median_diff


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Perform verification.")
    parser.add_argument(
        "--skip_check",
        action="store_true",  # This makes it a boolean flag
        help="If set, skip the validation checks.",
    )
    parser.add_argument(
        "--remove_start",
        action="store_true",  # This makes it a boolean flag
        default=True,
        help="If set, skip the validation checks.",
    )
    parser.add_argument(
        "--use_metadata",
        action="store_true",  # This makes it a boolean flag
        default=True,
        help="If set, skip the validation checks.",
    )

    args = parser.parse_args()

    input_bag_path = get_bag("*_jetson_ap20_synced.bag")
    output_bag_path = input_bag_path.replace("_synced.bag", "_robot.bag")
    # Process the bag

    args.use_metadata = True
    if args.use_metadata:
        import os
        import yaml
        from pathlib import Path
        from box_auto.utils import BOX_AUTO_DIR

        yaml_path = os.path.join(BOX_AUTO_DIR, "cfg/mission_metadata.yaml")
        with open(yaml_path, "r") as yaml_file:
            metadata = yaml.safe_load(yaml_file)
            mission_date = Path(input_bag_path).stem.replace("_jetson_ap20_synced", "")
            info = metadata[mission_date]
            prism = info["prism"]
            if prism == "Working":
                args.skip_check = False
                args.remove_start = True

            if prism == "Not at end":
                args.skip_check = True
                args.remove_start = True

            if prism == "Not at start and end":
                args.skip_check = True
                args.remove_start = False

    exit_code, media_diff = process_prism_position_bag(
        input_bag_path, output_bag_path, args.skip_check, args.remove_start, return_check_only=False
    )

    exit(exit_code)
