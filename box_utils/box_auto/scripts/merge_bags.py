from pathlib import Path
from fnmatch import fnmatchcase
from rosbag import Bag
import re
import argparse
import tqdm
import os

def merge_bags_single(input_bag, output_bag, topics="*", verbose=False):
    # From https://www.clearpathrobotics.com/assets/downloads/support/merge_bag.py
    topics = topics.split(" ")

    total_included_count = 0
    total_skipped_count = 0

    if verbose:
        print("Writing bag file: " + output_bag)
        print("Matching topics against patters: '%s'" % " ".join(topics))

    with Bag(output_bag, "w", compression='lz4') as o:
        with tqdm.tqdm(total=len(input_bag), desc=f"Merging", unit="subbags", colour = "green") as pbar:
            for ifile in input_bag:
                matchedtopics = []
                included_count = 0
                skipped_count = 0
                if verbose:
                    print("> Reading bag file: " + ifile)
                try:
                    with Bag(ifile, "r") as ib:
                        for topic, msg, t in ib:
                            if any(fnmatchcase(topic, pattern) for pattern in topics):
                                if topic not in matchedtopics:
                                    matchedtopics.append(topic)
                                    if verbose:
                                        print("Including matched topic '%s'" % topic)
                                o.write(topic, msg, t)
                                included_count += 1
                            else:
                                skipped_count += 1
                except Exception as e:
                    print("Failed to read bag file: %s" % str(e))
                    print("Skipping this bag ")
                total_included_count += included_count
                total_skipped_count += skipped_count
                if verbose:
                    print(
                        "< Included %d messages and skipped %d"
                        % (included_count, skipped_count)
                    )
                pbar.update(1)

    if verbose:
        print(
            "Total: Included %d messages and skipped %d"
            % (total_included_count, total_skipped_count)
        )

    if os.environ.get("KLEINKRAM_ACTIVE", False):
        os.system(f"klein mission upload --mission {os.environ['MISSION_UUID']} --path {output_bag}")
        os.system(f"cp {output_bag} /out")

    return total_included_count, total_skipped_count

# write the following code. 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix, reindex, and merge ROS bag files.")
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="Comma seperated list of bags."
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output bag file path ending with .bag"
    )
    args = parser.parse_args()
    bag_files = args.input.split(",")

    print("Found files: ", bag_files)
    merge_bags_single(bag_files, args.output, "*", verbose=True)