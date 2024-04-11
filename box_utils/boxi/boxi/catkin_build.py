from boxi import BOX_ROOT_DIR, shell_run
import argparse
import os
import socket

def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    host = socket.gethostname()
    cmd = f"cd ~/catkin_ws; catkin build launch_{host}"
    shell_run(cmd)