from boxi import BOX_ROOT_DIR, shell_run
import argparse
import os


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    Path(BOX_ROOT_DIR).parent

    for p in ["box_utils", "box_core", "box_bringup"]:
        pa = os.path.join(BOX_ROOT_DIR, p)
        black = f"black --line-length 120 {pa}"
        shell_run(black)
