from pathlib import Path
import os

base = Path("/media/jonfrey/Untitled/box_paper_dataset_v2")
missions = [
    # "2024-10-01-11-29-55_polybahn",
    # "2024-10-01-11-47-44_main_building_hall",
    "2024-11-02-17-18-32_sphinx_walking_stairs_2",
    "2024-11-03-13-51-43_eigergletscher_hike_down",
    "2024-11-14-13-45-37_heap_testsite",
    "2024-11-25-16-36-19_leica_warehouse_groundfloor",
    "2024-11-02-17-43-10_sphinx_walking_on_flat",
    "2024-11-11-13-06-23_pilatus_kulm_hike2",
    "2024-11-18-13-22-14_arche_demolished",
]

for j, p in enumerate(missions):
    print(j)
    mission_data = base / p
    output_file = str(mission_data / "output_ap20_rerun.log")
    cmd = f"export MISSION_DATA={mission_data}; docker compose -f /home/jonfrey/git/grand_tour_box/box_utils/box_auto/compose/rerun_ap20.yaml up > {output_file} 2>&1"
    print(cmd)
    os.system(cmd)

# "2024-11-02-17-43-10_sphinx_walking_on_flat" -> Fails
# ap20_python_online_matcher  | FileNotFoundError: No matching bags found: *_jetson_ap20_aux.bag in directory /tmp_disk.
# "2024-11-11-13-06-23_pilatus_kulm_hike2" -> Fails service "hdr_timestamp_adjuster" didn't complete successfully: exit 2 hdr_timestamp_adjuster
