from boxi import shell_run
from pathlib import Path


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--folder", default="./", help="Path to the mission folder - defaults to current directory.")
    return parser


def main(args):
    rsync = ""
    bag_files = [str(s) for s in Path(args.folder).rglob("*.bag")]
    print("Found bags: ")
    for bag in bag_files:
        print(f"   --{bag}")
    for bag in bag_files:
        shell_run(f"rosbag info --freq {bag}")
        print("\n\n")
