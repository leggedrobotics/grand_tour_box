from pathlib import Path
import os

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

folder = Path(MISSION_DATA)
files_to_upload = [str(s) for s in folder.rglob("*.bag")]
keys = [
    "_lpc_",
    "_npc_",
    "_nuc_hesai.bag",
    "_jetson_ap20.bag",
    "_cpt7_raw_imu.bag",
    "_cpt7_gps_optimized_trajectory.bag",
    "_tf_static.bag",
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
    "_jetson_hdr_right_raw.bag",
    "_jetson_hdr_left_raw.bag",
    "_jetson_hdr_front_raw.bag",
]
upload = []
for f in files_to_upload:
    for k in keys:
        if f.find(k) != -1:
            upload.append(f)

upload.sort()

mission_name = folder.name
project = "GrandTourDev"
files_to_upload = " --path " + " --path ".join(files_to_upload)
os.system(f"klein upload {files_to_upload} --project {project} --mission {mission_name}")
