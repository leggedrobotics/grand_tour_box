from boxi import BOX_ROOT_DIR, shell_run, bcolors
import time

USERNAME = "rsl"


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    shell_run("boxi push --jetson --nuc")
    shell_run("boxi launch -m=recording --restart --all")
    
    time.sleep(20)
    shell_run("rosservice call /gt_box/rosbag_record_coordinator/start_recording \"yaml_file: \'box_default\'\" ")
    
    time.sleep(30)
    shell_run("rosservice call /gt_box/rosbag_record_coordinator/stop_recording \"verbose: false \" ")

    for host in ["jetson", 'nuc']:
        cmd = (
                f"ssh -o ConnectTimeout=4 rsl@{host} -t /home/rsl/.local/bin/boxi mission_summary --latest"
            )
        try:
            shell_run(cmd)
        except:
            print(f"ROS couldn't be started on {host}")