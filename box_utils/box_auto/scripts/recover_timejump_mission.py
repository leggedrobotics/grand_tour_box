import os
from pathlib import Path
import argparse
import re


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Validate ROS bag files")
    parser.add_argument(
        "--mission_data", default=os.environ.get("MISSION_DATA", "/mission_data"), help="Mission folder path"
    )
    parser.add_argument("--valid_rosbags", type=int, default=10, help="Maximum number of valid rosbags")
    return parser.parse_args()


def extract_bag_number(filename):
    """Extract the number between last underscore and .bag extension."""
    match = re.search(r"_(\d+)\.bag$", filename)
    if match:
        return int(match.group(1))
    return None


def validate_bags(mission_path, max_valid_bags):
    """Validate ROS bag files based on number and size criteria."""
    bag_files = [str(s) for s in Path(mission_path).rglob("*.bag")]
    invalid_bags = []
    empty_bags = []

    for bag_file in bag_files:
        bag_number = extract_bag_number(bag_file)
        file_size = Path(bag_file).stat().st_size

        # Check if bag number is greater than allowed
        if bag_number is not None and bag_number > max_valid_bags:
            invalid_bags.append(bag_file)

        # Check if file is empty or exactly 4,117 bytes
        elif file_size <= 4117:
            empty_bags.append(bag_file)
    invalid_bags.sort(key=extract_bag_number)
    empty_bags.sort(key=extract_bag_number)
    return invalid_bags, empty_bags


def print_results(invalid_bags, empty_bags):
    """Print the validation results in a formatted way."""
    print("\nValidation Results:")
    print("-" * 50)

    print("\nInvalid Bags (number > maximum allowed):")
    if invalid_bags:
        for bag in invalid_bags:
            print(f"  - {bag}")
    else:
        print("  None found")

    print("\nEmpty or Small Bags (≤ 4,117 bytes):")
    if empty_bags:
        for bag in empty_bags:
            print(f"  - {bag}")
    else:
        print("  None found")


def main():
    args = parse_arguments()

    try:
        invalid_bags, empty_bags = validate_bags(args.mission_data, args.valid_rosbags)
        print_results(invalid_bags, empty_bags)

        for b in invalid_bags + empty_bags:
            print(b)
            os.system(f"rm {b}")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
