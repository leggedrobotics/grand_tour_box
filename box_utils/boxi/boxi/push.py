from boxi import BOX_ROOT_DIR, shell_run
import argparse

USERNAME = "rsl"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Sync to Jetson")
    parser.add_argument("--nuc", action="store_true", help="Sync to Nuc")
    parser.add_argument("--pi", action="store_true", help="Sync to Pi")
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

    for host in hosts:
        cmd = f"{rsync} {BOX_ROOT_DIR} {host}:/home/rsl/git"
        shell_run(cmd)
