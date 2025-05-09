from box_auto.utils import get_uuid_mapping, deployments, get_bag
import kleinkram
import os
from pathlib import Path
import time

UUID_MAPPING = get_uuid_mapping()
for d in deployments:
    for m in d["mission_names"]:
        MISSION_DATA = os.path.join(d["data_folder"], m)

        print(f"Mission: {m}")

        mission_good = {}
        for tag in ["left", "right", "front"]:
            files = list(Path("/media/jonfrey/Untitled/jetson_data_save/").rglob(f"*{m}_jetson_hdr_{tag}*.mcap"))

            if len(files) == 1:
                file_size = os.path.getsize(files[0])
                print(f"   - {tag}: {file_size}")
                mission_good[tag] = "MCAP in Jetson Folder"

                tmp = str(Path(files[0]).parent.parent)
                if tag == "left":
                    os.system(
                        f"export MISSION_DATA={tmp}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/general/mcap_to_rosbag.py; sleep 5; python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_auto/python/box_auto/scripts/camera/hdr_timestamp_adjuster.py"
                    )

                s = get_bag(f"*{m}_jetson_hdr_{tag}_updated.bag", tmp)

                time.sleep(2)
                kleinkram.upload(mission_id=UUID_MAPPING[m]["uuid_pub"], files=[s])
                try:
                    hdr_file = get_bag(f"*{m}_jetson_hdr_{tag}.bag", tmp)
                    os.system(f"rm {s}")
                except Exception as e:
                    print(e)

                try:
                    hdr_file = get_bag(f"*{m}_jetson_hdr_{tag}_updated.bag", tmp)
                    os.system(f"rm {s}")
                except Exception as e:
                    print(e)

            else:
                # Try in local mission folder
                files = list(Path(MISSION_DATA).rglob(f"*{m}_jetson_hdr_{tag}*.mcap"))
                if len(files) == 1:
                    file_size = os.path.getsize(files[0])
                    print(f"   - {tag}: {file_size}")
                    mission_good[tag] = "MCAP in Mission Folder"

                else:

                    files = list(Path(MISSION_DATA).glob(f"*{m}_jetson_hdr_{tag}.bag"))
                    if len(files) == 1:
                        file_size = os.path.getsize(files[0])
                        print(f"   - {tag}: {file_size}")
                        mission_good[tag] = "BAG in Mission Folder"
                    else:
                        mission_good[tag] = "Failed"
                        print(f"   - {tag}: No file found")
