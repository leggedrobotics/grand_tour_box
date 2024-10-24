from boxi import shell_run, LOCAL_HOSTNAME
import os
from pathlib import Path
import glob

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

cmds = []
if LOCAL_HOSTNAME == "jetson":
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
        "_jetson_zed2i_proprioceptive.bag",
        "_jetson_zed2i_images.bag",
        "_jetson_zed2i_depth.bag",
        "_jetson_hdr_right_raw.bag",
        "_jetson_hdr_left_raw.bag",
        "_jetson_hdr_front_raw.bag",
    ]

elif LOCAL_HOSTNAME == "nuc":
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

for cmd in cmds:
    shell_run(cmd)

folder = Path(MISSION_DATA)
files_to_upload = [str(s) for s in folder.rglob("*.bag")]

# Bags that need to be added manually:
# keys = [
#     "_lpc_",
#     "_npc_",
#     "_cpt7_raw_imu.bag",
#     "_cpt7_gps_optimized_trajectory.bag",
# ]

upload = []
for f in files_to_upload:
    for k in keys:
        if f.find(k) != -1:
            upload.append(f)

upload.sort()

mission_name = folder.name
project = "GrandTourDev"
files_to_upload = " --path " + " --path ".join(files_to_upload)
print(f"klein upload {files_to_upload} --project {project} --mission {mission_name}")
