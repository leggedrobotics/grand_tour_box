from boxi import LOCAL_HOSTNAME, shell_run
import socket
import time

def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Restart PTP and phc2sys on Jetson")
    parser.add_argument("--nuc", action="store_true", help="Restart PTP and phc2sys on Nuc")
    parser.add_argument("--all", action="store_true", help="Build All")
    return parser


def main(args):
    if args.all:
        hosts = ["jetson", "nuc"]
    else:
        hosts = []
        if args.jetson:
            hosts.append("jetson")
        if args.nuc:
            hosts.append("nuc")

    for host in  hosts:   
        print("Restarting PTP and phc2sys on ", host)
        if host == LOCAL_HOSTNAME:
            cmd = "sync-clocks"
            print(cmd)
            shell_run(cmd)
                
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@{host} -t bash -ci sync-clocks"
            print(cmd)
            shell_run(cmd)
            
    if LOCAL_HOSTNAME == "opc":
        chrony_restart = "sudo systemctl restart chrony.service"
        shell_run(chrony_restart)
    else:
        chrony_restart = f"ssh -o ConnectTimeout=4 rsl@opc -t bash -ci 'sudo systemctl restart chrony.service'"
        shell_run(chrony_restart)            