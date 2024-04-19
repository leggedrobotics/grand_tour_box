from boxi import BOX_ROOT_DIR, shell_run
import argparse


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--turcan", action="store_true", help="Sets git for turcan")
    parser.add_argument("--jonas", action="store_true", help="Sets git for jonas")
    parser.add_argument("--kappi", action="store_true", help="Sets git for kappi")
    parser.add_argument("--beni", action="store_true", help="Sets git for beni")
    return parser


def main(args):
    if args.turcan:
        print("Setting git for turcan")
        cmd_mail = f"git config --global user.email 'tutuna@leggedrobotics.com'"
        cmd_usr_name=f"git config --global user.name 'Turcan Tuna'"
        
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)

    if args.jonas:
        print("Setting git for Jonas")
        cmd_mail = f"git config --global user.email 'jonfrey@leggedrobotics.com'"
        cmd_usr_name=f"git config --global user.name 'Jonas Frey'"
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)

    if args.kappi:
        print("Setting git for kappi")
        cmd_mail = f"git config --global user.email 'xyz'"
        cmd_usr_name=f"git config --global user.name 'kappi'"
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)

    if args.beni:
        print("Setting git for beni")
        cmd_mail = f"git config --global user.email '.com'"
        cmd_usr_name=f"git config --global user.name 'Benjamin'"
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)

