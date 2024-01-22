from boxi import BOX_ROOT_DIR, shell_run
import socket
import logging as log


def add_arguments(parser):
    modes = ["", "camera", "lidar", "imu"]
    parser.set_defaults(main=main)
    parser.add_argument("-m", choices=modes, help="calibration mode of the box", default="")
    return parser


def main(args):
    hosts = ["opc", "jetson", "nuc", "pi"]
    hostname = socket.gethostname()

    mode = ""
    if args.m == "no":
        mode = ""
    elif args.m:
        mode = "_calib_" + args.m

    for host in hosts:
        print('Start ros in calibration mode "' + str(args.m) + '" on', host)
        if host == hostname:
            cmd = f"boxi start -c box_" + host + mode
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@" + host + " -t /home/rsl/.local/bin/boxi start -c box_" + host + mode
        try:
            shell_run(cmd)
        except:
            print("ROS couldn't be started on", host)
