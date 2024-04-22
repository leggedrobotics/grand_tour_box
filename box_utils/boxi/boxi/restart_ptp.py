from boxi import LOCAL_HOSTNAME, shell_run


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    if LOCAL_HOSTNAME == "jetson":
        cmd = "sudo systemctl restart ptp4l_mgbe0.service"
        shell_run(cmd)
        cmd = "sudo systemctl restart ptp4l_mgbe1.service"
        shell_run(cmd)
        cmd = "sudo systemctl restart phc2sys_mgbe0.service"
        shell_run(cmd)
        cmd = "sudo systemctl restart phc2sys_mgbe1.service"
        shell_run(cmd)

    if LOCAL_HOSTNAME == "nuc":
        cmd = "systemctl restart ptp4l_enp45s0.service"
        shell_run(cmd)
        cmd = "systemctl restart ptp4l_enp46s0.service"
        shell_run(cmd)
        cmd = "systemctl restart phc2sys_enp45s0.service"
        shell_run(cmd)
        cmd = "systemctl restart phc2sys_enp46s0.service"
        shell_run(cmd)

    if LOCAL_HOSTNAME == "pi":
        cmd = "systemctl restart ptp4l_eth0.service"
        shell_run(cmd)
        cmd = "systemctl restart phc2sys_eth0.service"
        shell_run(cmd)
