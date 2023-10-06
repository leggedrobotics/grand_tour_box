from boxi import BOX_ROOT_DIR, shell_run
import argparse
import os
from pathlib import Path
from time import sleep


def add_arguments(parser):
    yamls = [s.stem for s in Path(BOX_ROOT_DIR).joinpath("box_bringup/box_launch/tmux").rglob("*.yaml")]
    parser.set_defaults(main=main)
    parser.add_argument("-c", choices=yamls, help="tmux configuration file name", default="box_opc")
    return parser

def main(args):
    cfg = os.path.join(BOX_ROOT_DIR, "box_bringup/box_launch/tmux", args.c + ".yaml")
    shell_run(f"tmuxp load  {cfg} -d")
