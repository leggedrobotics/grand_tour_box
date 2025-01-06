from pathlib import Path
from fnmatch import fnmatchcase
from rosbag import Bag
import re
import argparse
import tqdm
import os
from box_auto.utils import MISSION_DATA


def merge_bags_single(input_bag, output_bag, topics="*", upload="No", verbose=False):
    # From https://www.clearpathrobotics.com/assets/downloads/support/merge_bag.py
    topics = topics.split(" ")

    total_included_count = 0
    total_skipped_count = 0

    if verbose:
        print("Writing bag file: " + output_bag)
        print("Matching topics against patters: '%s'" % " ".join(topics))

    with Bag(output_bag, "w", compression="lz4") as o:
        with tqdm.tqdm(total=len(input_bag), desc="Merging", unit="subbags", colour="green") as pbar:
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
                    print("< Included %d messages and skipped %d" % (included_count, skipped_count))
                pbar.update(1)

    if verbose:
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE" and upload == "Yes":
        uuid = os.environ["MISSION_UUID"]
        os.system(f"klein upload --mission {uuid} {output_bag}")

    return total_included_count, total_skipped_count


# write the following code.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix, reindex, and merge ROS bag files.")
    parser.add_argument(
        "--overwrite", type=bool, default=True, help="Whether to overwrite existing bag files (default: True)."
    )
    parser.add_argument("--upload", default="No", help="(default: No or Yes).")
    args = parser.parse_args()
    bag_files = sorted(Path(MISSION_DATA).rglob("*.bag"))
    print("Found files: ", bag_files)
    # Group bags
    grouped_files = {}
    pattern = re.compile(r"(.*?_\d+)\.bag$")

    for file in bag_files:
        match = pattern.search(file.name)
        if match:
            prefix = match.group(1).rsplit("_", 1)[0]  # Extract prefix before the last '_'
            if prefix not in grouped_files:
                grouped_files[prefix] = []
            grouped_files[prefix].append(file)

    # Merge bags
    with tqdm.tqdm(total=len(grouped_files), desc="Merging", unit="bags") as pbar:

        for prefix, files in grouped_files.items():

            output_bag = Path(MISSION_DATA) / f"{prefix}.bag"

            # If overwrite is False, check if the output bag already exists
            if not args.overwrite and output_bag.exists():
                print(f"Output file {output_bag} already exists. Skipping merge...")
                continue

            filelist = [str(file) for file in files]
            filelist.sort(key=lambda x: int(x.split(".")[-2].split("_")[-1]))
            print(f"Merging files: {[str(file) for file in files]} into {output_bag}")

            merge_bags_single(filelist, str(output_bag), "*", upload=args.upload, verbose=True)
            pbar.update(1)
