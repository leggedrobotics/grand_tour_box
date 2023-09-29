from boxi import BOX_ROOT_DIR, shell_run
import socket

def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc"]
    hostname = socket.gethostname()

    for host in hosts:
        print("stop ros on", host)

        if host == hostname:
            cmd = f"/usr/bin/tmux kill-server"
        else:
            cmd = f"ssh -o ConnectTimeout=4 -t rsl@" + host + " /usr/bin/tmux kill-server"
        try:
            shell_run(cmd)
        except:
            print("ros couldn't be stopped on", host)
