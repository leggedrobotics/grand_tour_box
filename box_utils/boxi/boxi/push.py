from boxi import BOX_ROOT_DIR, shell_run, bcolors

USERNAME = "rsl"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Sync to Jetson")
    parser.add_argument("--nuc", action="store_true", help="Sync to Nuc")
    parser.add_argument("--pi", action="store_true", help="Sync to Pi")
    parser.add_argument("--pi2", action="store_true", help="Sync to Pi")
    parser.add_argument("--lpc", action="store_true", help="Sync to ANYmal LPC")
    parser.add_argument("--npc", action="store_true", help="Sync to ANYmal NPC")
    parser.add_argument("--all", action="store_true", help="Sync to All")
    return parser


def main(args):
    rsync = "rsync -a --delete -z -h -r -v --exclude '*.git/*' --exclude '__pycache__' --exclude '*.pyc' --exclude '*boxi.egg-info*' --exclude 'box_drivers/adis16475_driver/adis16475_driver/build*' --exclude '*box_drivers/novatel_oem7_driver/build*'  --exclude '*box_utils/box_recording/data*'  --out-format=\"[%t]:%o:%f:Last Modified %M\""
    hosts = []
    if args.jetson:
        hosts.append("jetson")
    if args.nuc:
        hosts.append("nuc")
    if args.pi:
        hosts.append("pi")
    if args.pi2:
        hosts.append("pi2")
    if args.lpc:
        hosts.append("integration@192.168.0.141")
    if args.npc:
        hosts.append("integration@192.168.0.142")

    if args.all:
        hosts = ["jetson", "nuc", "pi"]

    if len(hosts) == 0:
        print(f"{bcolors.WARNING}{bcolors.BOLD}No host provided. Please provide a host name.{bcolors.ENDC}")
        return

    for host in hosts:
        if host.find("integration") != -1:
            cmd = f"{rsync} {BOX_ROOT_DIR} {host}:/home/integration/git"
        else:
            cmd = f"{rsync} {BOX_ROOT_DIR} {host}:/home/rsl/git"
        shell_run(cmd)
