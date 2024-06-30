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
        cmd1 = f"ssh -o ConnectTimeout=4 {USERNAME}@{host}" + " rm -f /data/*"
        print(cmd1)
        cmd2 = f"ssh -o ConnectTimeout=4 {USERNAME}@{host} rm -rf ~/.ros"
        print(cmd2)
        user_is_sure = input("Are you sure you want to delete recorded and logging data on " + host + "? y/n \n")
        if user_is_sure == "y":
            shell_run(cmd1)
            shell_run(cmd2)
