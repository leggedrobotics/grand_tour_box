from boxi import BOX_ROOT_DIR, shell_run
import socket

def add_arguments(parser):
    modes = ["no", "camera", "lidar", "imu"]
    parser.set_defaults(main=main)
    parser.add_argument("-m", choices=modes, help="calibration mode of the box", default="no")
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc"]
    hostname = socket.gethostname()

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
