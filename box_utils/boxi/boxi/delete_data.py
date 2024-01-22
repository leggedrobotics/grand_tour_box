from boxi import USERNAME, shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Get data from Jetson")
    parser.add_argument("--nuc", action="store_true", help="Get data from Nuc")
    return parser


def main(args):
    hosts = []
    if args.jetson:
        hosts.append("jetson")
    if args.nuc:
        hosts.append("nuc")
    if len(hosts) == 0:
        print("No host specified. Specify host with --hostname")
        exit()
    for host in hosts:
        cmd = (
            f"ssh -o ConnectTimeout=4 "
            + USERNAME
            + "@"
            + host
            + " rm ~/catkin_ws/src/grand_tour_box/box_utils/box_recording/data/*"
        )
        print(cmd)
        user_is_sure = input("Are you sure you want to delete data on " + host + "? y/n \n")
        if user_is_sure == "y":
            shell_run(cmd)
