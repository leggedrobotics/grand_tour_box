from boxi import BOX_ROOT_DIR, shell_run, bcolors
import argparse

USERNAME = "rsl"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Sync to Jetson")
    parser.add_argument("--nuc", action="store_true", help="Sync to Nuc")
    parser.add_argument("--pi", action="store_true", help="Sync to Pi")
    parser.add_argument("--all", action="store_true", help="Sync to All")
    return parser


def main(args):
    print(BOX_ROOT_DIR)
    rsync = "rsync -a --delete -z -h -r -v --exclude '*.git/*' --exclude '__pycache__' --exclude '*.pyc' --exclude '*.bag' --out-format=\"[%t]:%o:%f:Last Modified %M\""
    hosts = []
    if args.jetson:
        hosts.append("jetson")
    if args.nuc:
        hosts.append("nuc")
    if args.pi:
        hosts.append("pi")
    if args.all:
        hosts = ["jetson", "nuc", "pi"]

    if len(hosts) == 0:
        print(f"{bcolors.WARNING}{bcolors.BOLD}No host provided. Please provide a host name.{bcolors.ENDC}")
        return

    for host in hosts:
        cmd = f"{rsync} {BOX_ROOT_DIR} {host}:/home/rsl/git"
        shell_run(cmd)
