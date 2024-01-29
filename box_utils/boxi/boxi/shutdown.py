from boxi import BOX_ROOT_DIR, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser

def main(args):
    print("shutdown jetson")
    cmd = f"ssh -t rsl@jetson 'sudo shutdown -h now'"
    shell_run(cmd)

    print("shutdown nuc")
    cmd = f"ssh -t rsl@nuc 'sudo shutdown -h now'"
    shell_run(cmd)

    print("shutdown jetson")
    cmd = f"ssh -t rsl@pi 'sudo shutdown -h now'"
    shell_run(cmd)
