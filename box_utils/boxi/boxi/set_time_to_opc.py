from boxi import shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    cmd = "remote_time=$(ssh rsl@opc date --iso=ns) && sudo -S date -s $remote_time && sudo /usr/sbin/hwclock -w"
    shell_run(cmd)
