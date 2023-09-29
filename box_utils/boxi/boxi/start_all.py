from boxi import BOX_ROOT_DIR, shell_run
import socket

def add_arguments(parser):
    modes = ["no", "camera", "lidar", "imu"]
    parser.set_defaults(main=main)
    parser.add_argument("-m", choices=modes, help="calibration mode of the box", default="no")
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
            cmd = f"tmuxp load $(rospack find box_launch)/tmux/box_" + host + ".yaml"
        else:
            cmd = f"ssh -o ConnectTimeout=4 -t tmuxp load $(rospack find box_launch)/tmux/box_" + host + "_calib_" + args.m + ".yaml"
        try:
            shell_run(cmd)
        except:
            print("ros couldn't be started on", host)
