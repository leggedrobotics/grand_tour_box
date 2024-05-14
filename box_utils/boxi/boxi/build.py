from boxi import shell_run, bcolors
import socket


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Build on Jetson")
    parser.add_argument("--opc", action="store_true", help="Build on OPC")
    parser.add_argument("--nuc", action="store_true", help="Build on Nuc")
    parser.add_argument("--pi", action="store_true", help="Build on Pi")
    parser.add_argument("--all", action="store_true", help="Build All")
    parser.add_argument("--clean", action="store_true", help="Clean before building")
    parser.add_argument("--package", default="", help="Optional List of packages seperated by whitespaces")

    return parser


def main(args):
    """
    Usage:
        If not package is specified will always build launch_{hostname}
        boxi build --all --clean
        boxi build --jetson --clean
        boxi build --jetson --package box_health --clean
        boxi build --jetson --package box_health box_recording --clean
    """

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
                if args.package != "":
                    cmd += f"cd ~/catkin_ws; catkin clean {args.package} -y; "
                else:
                    cmd += "cd ~/catkin_ws; catkin clean --all -y; "

            if args.package != "":
                cmd += f"cd ~/catkin_ws; catkin build {args.package}"
            else:
                cmd += f"cd ~/catkin_ws; catkin build launch_{host}"

        else:
            if args.package == "":
                args.package = "launch_" + host

            cmd += f"ssh -o ConnectTimeout=4 rsl@{host} -t /home/rsl/.local/bin/boxi build --{host} --package {args.package}"

            if args.clean:
                cmd += " --clean"

        try:
            shell_run(cmd)
        except:
            print(f"Failed to exectued command on {host}")
