from boxi import USERNAME, shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Get data from Jetson")
    parser.add_argument("--nuc", action="store_true", help="Get data from Nuc")
    parser.add_argument("--directory", type=str, help="Specific directory inside /data to sync")
    return parser


def main(args):
    rsync_part1 = "rsync -r --progress " + USERNAME + "@"
    rsync_exclusions = ""
    hosts = []
    if args.jetson:
        hosts.append("jetson")
        rsync_exclusions = '--exclude="workspaces"'
        rsync_exclusions += ' --exclude="docker"'
    if args.nuc:
        hosts.append("nuc")

    if args.directory:
        rsync_part2 = f":/data/{args.directory} . {rsync_exclusions}"
    else:
        rsync_part2 = ":/data/* . " + rsync_exclusions

    if len(hosts) == 0:
        print("No host specified. Specify host with --hostname")
    for host in hosts:
        cmd = f"{rsync_part1}{host}{rsync_part2}"
        print(cmd)
        shell_run(cmd)
