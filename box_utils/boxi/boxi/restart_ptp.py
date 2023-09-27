from boxi import BOX_ROOT_DIR, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Get data from Jetson")
    parser.add_argument("--nuc", action="store_true", help="Get data from Nuc")
    return parser


def main(args):

    hosts = []
    if args.jetson:
        cmd = f"sudo systemctl restart ptp4l_mgbe0.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart ptp4l_mgbe1.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart phc2sys_mgbe0.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart phc2sys_mgbe1.service"
        shell_run(cmd)

    if args.nuc:
        cmd = f"systemctl restart ptp4l_enp45s0.service"
        shell_run(cmd)
        cmd = f"systemctl restart ptp4l_enp46s0.service"
        shell_run(cmd)
        cmd = f"systemctl restart phc2sys_system.service"
        shell_run(cmd)
        cmd = f"systemctl restart phc2sys_NIC.service"
        shell_run(cmd)
