from box_auto.utils import get_uuid_mapping, deployments, get_bag
import kleinkram
import os

for m, data in get_uuid_mapping().items():
    print(m, data["uuid"], data["uuid_pub"])

    patterns_delete = [
        "*cpt7_ie*",
        "*ap20_robot.*",
        "*ap20_synced.*",
        "*_cpt7_raw_imu.*",
        "*_cpt7_gps_ie*",
        # "*_jetson_zed2i_*",
        # "*_gt_pose.*",
        # "*_gt_tf.*",
        # "*_hesai_dlio.*",
        # "*_nuc_alphasense_cor.*",
        # "*_tf*",
        # "*_jetson_hdr_left*",
        # "*_jetson_hdr_right*",
        # "*_jetson_hdr_front*"
    ]

    patterns_upload = [
        "*_cpt7_ie_dgps.bag",
        "*_cpt7_ie_lc.bag",
        "*_cpt7_ie_ppp.bag",
        "*_cpt7_ie_tc.bag",
        "*_cpt7_raw_imu.bag",
        "*_jetson_ap20_robot.bag",
        "*_jetson_ap20_synced.bag",
    ]

    files = []
    for pattern in patterns_delete:
        files += kleinkram.list_files(mission_ids=[data["uuid"]], file_names=[pattern])

    if len(files) > 0:
        kleinkram.delete_files(file_ids=[f.id for f in files])

    for d in deployments:
        if m in d["mission_names"]:
            MISSION_DATA = os.path.join(d["data_folder"], m)
            paths = []
            for pattern_upload in patterns_upload:
                try:
                    paths.append(get_bag(pattern_upload, MISSION_DATA))
                except Exception as e:
                    print(e)
                    pass

            kleinkram.upload(mission_id=data["uuid"], files=paths)

    break
