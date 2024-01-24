from boxi import LOCAL_HOSTNAME, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    if LOCAL_HOSTNAME == "jetson":
        cmd = f"sudo systemctl restart ptp4l_mgbe0.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart ptp4l_mgbe1.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart phc2sys_mgbe0.service"
        shell_run(cmd)
        cmd = f"sudo systemctl restart phc2sys_mgbe1.service"
        shell_run(cmd)

    if LOCAL_HOSTNAME == "nuc":
        cmd = f"systemctl restart ptp4l_enp45s0.service"
        shell_run(cmd)
        cmd = f"systemctl restart ptp4l_enp46s0.service"
        shell_run(cmd)
        cmd = f"systemctl restart phc2sys_system.service"
        shell_run(cmd)
        cmd = f"systemctl restart phc2sys_NIC.service"
        shell_run(cmd)

    if LOCAL_HOSTNAME == "pi":
        cmd = f"systemctl restart ptp4l.service"
        shell_run(cmd)
        cmd = f"systemctl restart phc2sys.service"
        shell_run(cmd)
