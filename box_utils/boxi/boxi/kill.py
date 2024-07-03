from boxi import shell_run, LOCAL_HOSTNAME


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--all", action="store_true", help="Launch on all PCs")

    return parser


def main(args):
    if args.all:
        hosts = ["opc", "jetson", "nuc", "pi"]
    else:
        hosts = [LOCAL_HOSTNAME]

    hostname = LOCAL_HOSTNAME
    for host in hosts:
        print("Killing ROS on", host)
        if host == hostname:
            cmd = "tmux kill-server"
        else:
            cmd = f"ssh -o ConnectTimeout=2 rsl@{host} -t tmux kill-server"
        try:
            shell_run(cmd)
        except Exception as e:
            print(f"Unable to kill tmux server on {host}: {e}")
