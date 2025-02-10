from pathlib import Path
import os

base = Path("/media/jonfrey/Untitled/box_paper_dataset_v2")
missions = [
    # "2024-11-02-17-18-32_sphinx_walking_stairs_2",
    "2024-11-14-13-45-37_heap_testsite",
    "2024-11-03-13-51-43_eigergletscher_hike_down",
]

for j, p in enumerate(missions):
    print(j)
    mission_data = base / p
    output_file = str(mission_data / "output_create_euroc_all.log")
    cmd = f"export MISSION_DATA={mission_data}; python3 /home/jonfrey/git/grand_tour_box/box_utils/box_converter/euroc_dataset_converter.py > {output_file} 2>&1"
    print(cmd)
    os.system(cmd)

# "2024-11-02-17-43-10_sphinx_walking_on_flat" -> Fails
# ap20_python_online_matcher  | FileNotFoundError: No matching bags found: *_jetson_ap20_aux.bag in directory /tmp_disk.
# "2024-11-11-13-06-23_pilatus_kulm_hike2" -> Fails service "hdr_timestamp_adjuster" didn't complete successfully: exit 2 hdr_timestamp_adjuster
