import argparse
from boxi import shell_run
from pathlib import Path
import os

MISSION_DATA = os.environ.get("MISSION_DATA", "/tmp_disk")


def main(overwrite: bool, directory: str):
    # Get all .bag.active files
    bag_files = [
        str(s) for s in Path(directory).rglob("*.bag") if str(s).find(".orig") == -1 and str(s).find("zed2i") != -1
    ]

    if len(bag_files) == 0:
        print("No bag files found. --- Finished cleanly!")
        return

    for p in bag_files:
        shell_run(f"rosbag reindex {p}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reindex all ROS bag files.")
    parser.add_argument(
        "--overwrite", type=bool, default=True, help="Whether to overwrite existing bag files (default: True)."
    )
    args = parser.parse_args()

    # Run main function with the parsed argument
    main(True, MISSION_DATA)
