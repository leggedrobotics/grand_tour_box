from boxi import USERNAME, shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Get data from Jetson")
    parser.add_argument("--nuc", action="store_true", help="Get data from Nuc")
    return parser


def main(args):
    rsync_part1 = "rsync --progress " + USERNAME + "@"
    rsync_part2 = ":~/git/grand_tour_box/box_utils/box_recording/data/* ."
    hosts = []
    if args.jetson:
        hosts.append("jetson")
    if args.nuc:
        hosts.append("nuc")
    if len(hosts) == 0:
        print("No host specified. Specify host with --hostname")
    for host in hosts:
        cmd = f"{rsync_part1}{host}{rsync_part2}"
        print(cmd)
        shell_run(cmd)
