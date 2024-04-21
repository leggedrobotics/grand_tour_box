from boxi import BOX_ROOT_DIR, shell_run, bcolors
import argparse
import os
import socket


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Build on Jetson")
    parser.add_argument("--opc", action="store_true", help="Build on OPC")
    parser.add_argument("--nuc", action="store_true", help="Build on Nuc")
    parser.add_argument("--pi", action="store_true", help="Build on Pi")
    parser.add_argument("--all", action="store_true", help="Build All")
    parser.add_argument("--clean", action="store_true", help="Clean before building")
    return parser


def main(args):
    if args.all:
        hosts = ["opc", "jetson", "nuc", "pi"]
    else:
        hosts = []
        if args.jetson:
            hosts.append("jetson")
        if args.opc:
            hosts.append("opc")
        if args.nuc:
            hosts.append("nuc")
        if args.pi:
            hosts.append("pi")
        if len(hosts) == 0:
            hosts.append(socket.gethostname())

    hostname = socket.gethostname()
    for host in hosts:
        cmd = ""
        if host == hostname:
            if args.clean:
                if host != "jetson":
                    # Never fully clean on jetson otherwise this takes 20 minutes to rebuild OpenCV with CUDA support
                    cmd += f"cd ~/catkin_ws; catkin clean --all -y; "
                else:
                    print(f"{bcolors.WARNING}{bcolors.BOLD}catkin clean --all -- will be not executed on the jetson - given that building opencv takes ages! Clean by hand if needed.{bcolors.ENDC}")
            if host == "jetson":
                cmd += f"cd ~/catkin_ws; catkin build opencv_catkin --cmake-args -DCUDA_ARCH_BIN=8.7; "

            cmd += f"cd ~/catkin_ws; catkin build launch_{host}"

        else:
            cmd += f"ssh -o ConnectTimeout=4 rsl@{host} -t /home/rsl/.local/bin/boxi catkin_build --{host}"
            if args.clean:
                cmd += " --clean"

        try:
            shell_run(cmd)
        except:
            print(f"Failed to exectued command on {host}")
