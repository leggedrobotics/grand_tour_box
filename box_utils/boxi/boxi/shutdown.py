from boxi import shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    print("shutdown jetson")
    cmd = "ssh -t rsl@jetson 'sudo shutdown -h now'"
    shell_run(cmd)

    print("shutdown nuc")
    cmd = "ssh -t rsl@nuc 'sudo shutdown -h now'"
    shell_run(cmd)

    print("shutdown jetson")
    cmd = "ssh -t rsl@pi 'sudo shutdown -h now'"
    shell_run(cmd)
