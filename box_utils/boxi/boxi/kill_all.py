from boxi import BOX_ROOT_DIR, shell_run
import socket
import logging as log


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc"]
    hostname = socket.gethostname()

    for host in hosts:
        print("Killing ROS on", host)
        if host == hostname:
            cmd = f"tmux kill-server"
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@" + host + " -t tmux kill-server"
        try:
            shell_run(cmd)
        except:
            print("ROS wasn't running on", host)
