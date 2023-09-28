from boxi import BOX_ROOT_DIR, shell_run
import socket

def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc"]
    hostname = socket.gethostname()

    for host in hosts:
        print("start ros on", host)

        if host == hostname:
            cmd = f"tmuxp load $(rospack find box_launch)/tmux/box_" + host + ".yaml"
        else:
            cmd = f"ssh -o ConnectTimeout=4 -t tmuxp load $(rospack find box_launch)/tmux/box_" + host + ".yaml"
        try:
            shell_run(cmd)
        except:
            print("ros couldn't be started on", host)
