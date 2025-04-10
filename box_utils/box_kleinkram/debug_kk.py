import kleinkram
from box_auto.utils import get_uuid_mapping, read_sheet_data

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

for name, data in uuid_mappings.items():
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        files = kleinkram.list_files(mission_ids=[data["uuid_pub"]], file_names=["*_npc_velodyne_processed.bag"])
        for f in files:
            print(f)
            # pass
            # kleinkram.delete_files(file_ids=[f.id])

print("test")

missions = [m for m in kleinkram.list_missions(project_names=["GrandTour"]) if "pub_" in m.name]
patterns = [
    # "_hesai_dynablox.bag",
    # "_hesai_dlio.bag",
    # "_rect.bag",
    # "_hdr_front_updated.bag",
    # "_hdr_left_updated.bag",
    # "_hdr_right_updated.bag",
    ".mcap",
]
for m in missions:
    files = kleinkram.list_files(mission_ids=[m.id])
    files_to_delete = []
    for i, f in enumerate(files):
        for p in patterns:
            if f.name[-4:] == p:
                files_to_delete.append(f)
                print(f)
                break
    if len(files_to_delete) > 0:
        for f in files_to_delete:
            print(f"Deleting {f.name} from {m.name}")
            # kleinkram.delete_files(file_ids=[f.id])
