from boxi import shell_run, LOCAL_HOSTNAME
from pathlib import Path
import os


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--folder", default="./", help="Path to the mission folder - defaults to current directory.")
    return parser


def main(args):
    bag_files = [str(s) for s in Path(args.folder).rglob("*.bag.active")]
    for p in bag_files:
        shell_run(f"rosbag fix {p} {p.replace('.active', '')}")
        shell_run(f"rosbag reindex {p.replace('.active', '')} ")
