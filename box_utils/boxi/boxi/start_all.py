from boxi import BOX_ROOT_DIR, shell_run
import socket
import logging as log

def add_arguments(parser):
    modes = ["", "camera", "lidar", "imu"]
    parser.set_defaults(main=main)
    parser.add_argument("-m", choices=modes, help="calibration mode of the box", default="")
    parser.add_argument("--sync_clocks", action="store_true", help="Sync clocks before starting ros")
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc"]
    hostname = socket.gethostname()

    if args.sync_clocks:
        cmd = f'boxi initial_clock_sync'
        shell_run(cmd)

    for host in hosts:
        print("start ros in mode \"" + str(args.m) + "\" on", host)
        if host == hostname:
            cmd = f"tmuxp load $(rospack find box_launch)/tmux/box_" + host + "_calib_" + args.m + ".yaml -d"
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@" + host + " -t /home/rsl/.local/bin/boxi start --kill -c box_" + host + "_calib_" + args.m
        try:
            shell_run(cmd)
        except:
            print("ros couldn't be started on", host)
