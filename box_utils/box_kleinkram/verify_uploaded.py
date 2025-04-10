from box_auto.utils import get_uuid_mapping, read_sheet_data
import kleinkram

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

j = 0
for name, data in uuid_mappings.items():
    j += 1

    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_pub"]

        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*_calib.bag"])
        files = [f for f in files if round(f.size / 1024 / 1024, 2) > 0.005]

        print(f"MISSION NAME      {name}:                                                 {len(files)}")
        for f in files:
            print("   " + f.name + " : " + str(round(f.size / 1024 / 1024, 2)) + " MB")
