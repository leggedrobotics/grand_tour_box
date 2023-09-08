from boxi import BOX_ROOT_DIR, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", type=bool, help="Store data")
    parser.add_argument(
        "--nuc",
        type=str,
        help="Sync to NUC",
    )
    return parser


def main(args):
    rsync = "rsync -a -z -h -r -v --exclude '*.git/*' --exclude '__pycache__' --exclude '*.pyc' --exclude '*.bag' --out-format=\"[%t]:%o:%f:Last Modified %M\" "
    hosts = []
    if args.jetson:
        hosts.append("jetson")
    if args.nuc:
        hosts.append("nuc")

    for host in hosts:
        cmd = f"{rsync} {BOX_ROOT_DIR} {USERNAME}@{host}:{BOX_ROOT_DIR}"
        shell_run(cmd)
