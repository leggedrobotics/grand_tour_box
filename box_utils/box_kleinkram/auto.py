import argparse
import os
from pathlib import Path
import glob
import copy
import subprocess
import socket

LOCAL_HOSTNAME = socket.gethostname()


def shell_run(cmd, cwd=None, env={}, time=True, continue_on_error=True):
    """Execute shell command."""
    # Support both list and str format for commands
    # if isinstance(cmd, str):
    #     cmd = shlex.split(cmd)
    # Set up environmental variables
    env_variables = copy.deepcopy(os.environ)
    env_variables.update(env)

    # Execute command
    try:
        p = subprocess.Popen(cmd, cwd=cwd, env=env_variables, shell=True, executable="/bin/bash")
    except Exception as e:
        raise RuntimeError(f"{e} --- while executing {cmd}")

    if p.wait() != 0:
        print()
        if not continue_on_error:
            raise RuntimeError(f"Error Return non 0 --- while executing {cmd}")


def process_mission_data(data_folder, mission_name, local_hostname, mode):
    MISSION_DATA = os.path.join(data_folder, mission_name)
    cmds = []

    if local_hostname == "jetson":
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/repair_ros2_jetson.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/repair_bags.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/mcap_to_rosbag.py"
        )
        zed_svo = sorted(glob.glob(os.path.join(MISSION_DATA, "*_jetson_zed2i.svo2")))[0]
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; source /home/rsl/catkin_ws/devel/setup.bash; roslaunch zed_wrapper zed2i_replay.launch svo_file:={zed_svo}"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_mission.py"
        )
        keys = [
            "_jetson_utils.bag",
            "_jetson_stim.bag",
            "_jetson_ap20_aux.bag",
            "_jetson_adis.bag",
            "_jetson_zed2i_tf.bag",
            "_jetson_zed2i_prop.bag",
            "_jetson_zed2i_images.bag",
            "_jetson_zed2i_depth.bag",
            "_jetson_hdr_right_raw.bag",
            "_jetson_hdr_left_raw.bag",
            "_jetson_hdr_front_raw.bag",
        ]

    elif local_hostname == "nuc":
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/repair_bags.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_mission.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/create_tf_static.py"
        )
        keys = [
            "_tf_static.bag",
            "_nuc_hesai_post_processed.bag",
            "_nuc_utils.bag",
            "_nuc_tf.bag",
            "_nuc_livox.bag",
            "_nuc_hesai.bag",
            "_nuc_cpt7.bag",
            "_nuc_alphasense.bag",
        ]
    elif local_hostname == "anymal-d039-lpc" or local_hostname == "anymal-d039-npc":
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/repair_bags.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_mission.py"
        )
        keys = [
            "_npc_depth_cameras.bag",
            "_npc_elevation_mapping.bag",
            "_npc_velodyne.bag",
            "_lpc_anymal_imu.bag",
            "_lpc_depth_cameras.bag",
            "_lpc_general.bag",
            "_lpc_locomotion.bag",
            "_lpc_state_estimator.bag",
            "_lpc_tf.bag",
        ]
    elif local_hostname == "opc" or local_hostname == "mavt-rsl-ws":
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/repair_bags.py"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/merge_mission.py"
        )
        cmds.append(
            f"python3 /home/rsl/git/grand_tour_box/box_utils/boxi/boxi/move_cpt7_files.py --data_folder={data_folder}"
        )
        cmds.append(
            f"export MISSION_DATA={MISSION_DATA}; python3 /home/rsl/catkin_ws/src/grand_tour_box/box_utils/box_auto/scripts/export_raw_imu_bag.py"
        )

        keys = [
            "_npc_depth_cameras.bag",
            "_npc_elevation_mapping.bag",
            "_npc_velodyne.bag",
            "_lpc_anymal_imu.bag",
            "_lpc_depth_cameras.bag",
            "_lpc_general.bag",
            "_lpc_locomotion.bag",
            "_lpc_state_estimator.bag",
            "_lpc_tf.bag",
            "_tf_static.bag",
            "_nuc_hesai_post_processed.bag",
            "_nuc_utils.bag",
            "_nuc_tf.bag",
            "_nuc_livox.bag",
            "_nuc_hesai.bag",
            "_nuc_cpt7.bag",
            "_nuc_alphasense.bag",
            "_jetson_utils.bag",
            "_jetson_stim.bag",
            "_jetson_ap20_aux.bag",
            "_jetson_adis.bag",
            "_jetson_zed2i_tf.bag",
            "_jetson_zed2i_prop.bag",
            "_jetson_zed2i_images.bag",
            "_jetson_zed2i_depth.bag",
            "_jetson_hdr_right.bag",
            "_jetson_hdr_left.bag",
            "_jetson_hdr_front.bag",
        ]

    print(local_hostname)

    for cmd in cmds:
        if mode == "process" or mode == "upload_and_process":
            os.system(cmd)

    folder = Path(MISSION_DATA)
    files_to_upload = [str(s) for s in folder.rglob("*.bag")]

    upload = []
    for f in files_to_upload:
        for k in keys:
            if k in f:
                upload.append(f)

    upload.sort()

    project = "GrandTour"
    files_to_upload_str = " ".join(upload)

    metadata = ""
    p = os.path.join(folder, folder.name + ".yaml")
    if os.path.exists(p):
        metadata = "--metadata " + str(p)

    if mode == "upload" or mode == "upload_and_process":
        os.system(
            f"klein upload --ignore-missing-tags --project {project} --mission {mission_name} --create {metadata} {files_to_upload_str}"
        )
        # os.system(
        #     f"klein verify --project {project} --mission {mission_name} {files_to_upload_str}"
        # )

    if mode == "verify":
        os.system(f"klein verify --project {project} --mission {mission_name} {files_to_upload_str}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process and upload mission data.")
    parser.add_argument("--local_hostname", default=LOCAL_HOSTNAME, help="T the hostname of the local machine.")
    parser.add_argument("--data_folder", default="/data", help="Base path")
    parser.add_argument("--mode", default="verify", choices=["upload", "process", "verify", "upload_and_process"])
    parser.add_argument("--mission_names", nargs="+", help="One or more mission names to process.")

    args = parser.parse_args()

    for mission_name in args.mission_names:
        process_mission_data(args.data_folder, mission_name, args.local_hostname, args.mode)
