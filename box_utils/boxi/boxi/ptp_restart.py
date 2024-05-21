from boxi import LOCAL_HOSTNAME, shell_run
import socket
import time

def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--jetson", action="store_true", help="Build on Jetson")
    parser.add_argument("--opc", action="store_true", help="Build on OPC")
    parser.add_argument("--nuc", action="store_true", help="Build on Nuc")
    parser.add_argument("--pi", action="store_true", help="Build on Pi")
    parser.add_argument("--all", action="store_true", help="Build All")
    return parser


def main(args):
    if args.all:
        hosts = ["jetson", "nuc", "pi"]
    else:
        hosts = []
        if args.jetson:
            hosts.append("jetson")
        if args.opc:
            hosts.append("opc")
        if args.nuc:
            hosts.append("nuc")
        if args.pi:
            hosts.append("pi")
        if len(hosts) == 0:
            hosts.append(LOCAL_HOSTNAME)

    for host in  hosts:    
        if host == LOCAL_HOSTNAME:
            cmds = []
            if host == "jetson":                
                cmds.append("/usr/bin/systemctl restart ptp4l_mgbe0.service; sleep 1")
                cmds.append("/usr/bin/systemctl restart phc2sys_mgbe0.service")          
                
                cmds.append("/usr/bin/systemctl restart ptp4l_mgbe1.service; sleep 1")
                cmds.append("/usr/bin/systemctl restart phc2sys_mgbe1.service")

            if host == "nuc":
                cmds.append("/usr/bin/systemctl restart ptp4l_enp45s0.service; sleep 1")
                cmds.append("/usr/bin/systemctl restart phc2sys_enp45s0.service")

                cmds.append("/usr/bin/systemctl restart ptp4l_enp46s0.service; sleep 1")
                cmds.append("/usr/bin/systemctl restart phc2sys_enp46s0.service")
            if host == "pi":
                cmds.append("/usr/bin/systemctl restart ptp4l_eth0.service; sleep 1")
                cmds.append("/usr/bin/systemctl restart phc2sys_eth0.service")
            

            for cmd in cmds:
                cmd = "echo rsl | sudo -S " + cmd
                print(cmd)
                shell_run(cmd)
             
            if host == "jetson":
                print ("Waiting for jetson to restart - 10 seconds")
                time.sleep(10)
                
        else:
            cmd = f"ssh -o ConnectTimeout=4 -t rsl@{host} /home/rsl/.local/bin/boxi ptp_restart --{host} --grandmaster={args.grandmaster} "
            print(cmd)
            shell_run(cmd)
            