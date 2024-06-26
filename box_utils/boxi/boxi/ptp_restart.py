from boxi import LOCAL_HOSTNAME, shell_run
import subprocess

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
        if len(hosts) == 0:
            hosts.append(LOCAL_HOSTNAME)

    for host in  hosts:   
        print("Restarting PTP and phc2sys on", host)
        if host == LOCAL_HOSTNAME:
            cmd = "sync-clocks"
            print(cmd)
            shell_run(cmd)
                
        else:
            cmd = f"ssh -o ConnectTimeout=4 rsl@{host} -t bash -ci 'sync-clocks; sleep 1'"
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, text=True)
            output, errors = proc.communicate(input='rsl\n')
            print(output)
            