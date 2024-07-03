from boxi import shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--install", action="store_true", help="Install precommit")
    return parser


def main(args):
    if args.install:
        shell_run("pip3 install pre-commit")
        shell_run("cd ~/git/grand_tour_box && pre-commit install")

    shell_run("cd ~/git/grand_tour_box && python3 -m pre_commit run")
