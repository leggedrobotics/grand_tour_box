from boxi import BOX_ROOT_DIR, shell_run
import socket
import logging as log


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--all", action="store_true", help="Launch on all PCs")

    return parser


def main(args):
    if args.all:
        hosts = ["opc", "jetson", "nuc", "pi"]
    else:
        hosts = [socket.gethostname()]
    
    hostname = socket.gethostname()
    for host in hosts:
        print("Killing ROS on", host)
        if host == hostname:
            cmd = f"tmux kill-server"
        else:
            cmd = f"ssh -o ConnectTimeout=2 rsl@" + host + " -t tmux kill-server"
        try:
            shell_run(cmd)
        except:
            print("ROS wasn't running on", host)
