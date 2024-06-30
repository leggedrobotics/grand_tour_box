from boxi import shell_run, LOCAL_HOSTNAME
from pathlib import Path
import os

SVO_CHECKER_SCRIPT = "/home/rsl/git/grand_tour_box/box_utils/box_recording/scripts/check_svo_file.py"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--folder", default="./", help="Path to the mission folder - defaults to current directory.")
    parser.add_argument(
        "--latest", action="store_true", help="Path to the mission folder - defaults to current directory."
    )
    return parser


def get_latest_modified_folder(directory):
    # Get all entries in the directory
    all_entries = os.listdir(directory)

    # Filter for directories only
    dirs = [entry for entry in all_entries if os.path.isdir(os.path.join(directory, entry))]

    # Check if there are any directories
    if not dirs:
        return None

    # Get the most recently modified directory
    latest_modified_folder = max(dirs, key=lambda x: os.path.getmtime(os.path.join(directory, x)))

    return os.path.join(directory, latest_modified_folder)


def main(args):
    if args.latest:
        args.folder = get_latest_modified_folder("/data")

    bag_files = [str(s) for s in Path(args.folder).rglob("*.bag")]
    print("Found bags: ")
    for bag in bag_files:
        print(f"   --{bag}")
    for bag in bag_files:
        shell_run(f"rosbag info --freq {bag}")
        print("\n\n")

    if LOCAL_HOSTNAME == "jetson":
        svo_files = [str(s) for s in Path(args.folder).rglob("*.svo2")]
        print("Found svo (Zed2i) files: ")
        for svo_file in svo_files:
            print(f"    --{svo_file}")
        for svo_file in svo_files:
            shell_run(f"python {SVO_CHECKER_SCRIPT} --input_svo_file {svo_file}")
            print("\n\n")

    if args.latest:
        print(f"\nLatest found mission {args.folder}\n")
