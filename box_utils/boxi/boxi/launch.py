from boxi import shell_run
import socket


def add_arguments(parser):
    modes = [
        "recording",
        "autonomy",
        "replay",
        "calib_camera",
        "calib_lidar",
        "calib_imu",
    ]
    parser.set_defaults(main=main)
    parser.add_argument(
        "-m",
        "--mode",
        choices=modes,
        help="calibration mode of the box",
        default="recording",
    )
    parser.add_argument("--all", action="store_true", help="Launch on all PCs")
    parser.add_argument(
        "--restart",
        action="store_true",
        help="First shutdown everything and then launch",
    )
    return parser


def main(args):
    if args.restart:
        if args.all:
            shell_run("boxi kill --all")
        else:
            shell_run("boxi kill")

    if args.all:
        hosts = ["opc", "jetson", "nuc", "pi"]
    else:
        hosts = [socket.gethostname()]

    hostname = socket.gethostname()

    for host in hosts:
        print(f'Start ros in calibration mode "{args.mode}" on {host}')
        if host == hostname:
            cmd = f"boxi launch_on_host -c {host}_{args.mode}"
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@{host} -t /home/rsl/.local/bin/boxi launch_on_host -c {host}_{args.mode}"

        try:
            shell_run(cmd)
        except:
            print(f"ROS couldn't be started on {host}")
