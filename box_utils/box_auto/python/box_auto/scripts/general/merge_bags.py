"""Legacy CLI for the shared fail-closed, atomic bag merger."""

import argparse
from box_auto.scripts.general.merge_mission import merge_bags_single

# write the following code.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix, reindex, and merge ROS bag files.")
    parser.add_argument("--input", type=str, required=True, help="Comma seperated list of bags.")
    parser.add_argument("--output", type=str, required=True, help="Output bag file path ending with .bag")
    parser.add_argument("--filter", type=str, default="*", help="Whitespace seperated list of topics or *")
    parser.add_argument("--upload", default="No", help="(default: No or Yes).")

    args = parser.parse_args()
    bag_files = args.input.split(",")

    print("Found files: ", bag_files)
    merge_bags_single(bag_files, args.output, args.filter, args.upload, verbose=True)
