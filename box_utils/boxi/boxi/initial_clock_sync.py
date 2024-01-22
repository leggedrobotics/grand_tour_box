from boxi import BOX_ROOT_DIR, shell_run
import argparse

USERNAME = "rsl"


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    # set jetson time to opc time
    print("set jetson time to opc time")
    cmd = f"ssh -t rsl@jetson /home/rsl/.local/bin/boxi set_time_to_opc"
    shell_run(cmd)

    # restart jetson ptp & phc2sys
    print("restart jetson ptp & phc2sys")
    cmd = f"ssh -t rsl@jetson /home/rsl/.local/bin/boxi restart_ptp"
    shell_run(cmd)

    # restart nuc ptp & phc2sys
    print("restart nuc ptp & phc2sys")
    cmd = f"ssh -t rsl@nuc /home/rsl/.local/bin/boxi restart_ptp"
    shell_run(cmd)

    # restart pi ptp & phc2sys
    print("restart pi ptp & phc2sys")
    cmd = f"ssh -t rsl@pi /home/rsl/.local/bin/boxi restart_ptp"
    shell_run(cmd)

    # restart jetson chrony
    print("restart jetson chrony")
    cmd = f"ssh -t rsl@jetson sudo /usr/bin/systemctl restart chrony"
    shell_run(cmd)

    # restart opc chrony (or rather change to box_chrony conf and then restart)
    print("restart opc chrony")
    cmd = f"sudo /usr/bin/systemctl restart chrony"
    shell_run(cmd)
