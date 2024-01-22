from boxi import BOX_ROOT_DIR, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    cmd = f"remote_time=$(ssh rsl@opc date --iso=ns) && sudo -S date -s $remote_time && sudo /usr/sbin/hwclock -w"
    shell_run(cmd)
