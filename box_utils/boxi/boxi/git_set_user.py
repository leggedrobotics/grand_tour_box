from boxi import shell_run


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
        cmd_mail = "git config --global user.email 'tutuna@leggedrobotics.com'"
        cmd_usr_name = "git config --global user.name 'Turcan Tuna'"
        novatel_icon = "sed -i '$ s/planet.png/novatelSuite.png/' /home/rsl/.local/share/applications/novatelSuite.desktop"


        shell_run(cmd_mail)
        shell_run(cmd_usr_name)
        shell_run(novatel_icon)

    if args.jonas:
        print("Setting git for Jonas")
        cmd_mail = "git config --global user.email 'jonfrey@ethz.ch'"
        cmd_usr_name = "git config --global user.name 'Jonas Frey'"
        chrome_icon = "sudo sed -i 's|/kappi.png|/05_jonas.png|g' /usr/share/applications/google-chrome.desktop"
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)
        shell_run(chrome_icon)

    if args.kappi:
        print("Setting git for kappi")
        cmd_mail = "git config --global user.email 'kpatterson@ethz.ch'"
        cmd_usr_name = "git config --global user.name 'kappi'"
        novatel_icon = "sed -i '$ s/novatelSuite.png/planet.png/' /home/rsl/.local/share/applications/novatelSuite.desktop"
        chrome_icon = "sudo sed -i 's|/05_jonas.png|/kappi.png|g' /usr/share/applications/google-chrome.desktop"

        shell_run(cmd_mail)
        shell_run(cmd_usr_name)
        shell_run(novatel_icon)
        shell_run(chrome_icon)

    if args.beni:
        print("Setting git for Beni")
        cmd_mail = "git config --global user.email 'bkrumme@ethz.ch'"
        cmd_usr_name = "git config --global user.name 'Benjamin Krummenacher'"
        shell_run(cmd_mail)
        shell_run(cmd_usr_name)
