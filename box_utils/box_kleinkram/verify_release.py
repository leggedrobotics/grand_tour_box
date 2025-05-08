from box_auto.utils import get_uuid_mapping, read_sheet_data
import kleinkram

uuid_mappings = get_uuid_mapping()
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

j = 0
for name, data in uuid_mappings.items():
    if name not in MISSION_DATA.keys():
        print("Mission not found in sheet: ", name)
        continue

    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        uuid = data["uuid_release"]
        j += 1
        files = kleinkram.list_files(mission_ids=[uuid], file_names=["*"])

        if len(files) == 34:
            processed = "processed"
        else:
            processed = "open     "

        files = [f for f in files if f.size / 1024 / 1024 < 0.1]

        print(f"MISSION NAME      {name}:                                 {processed} : {len(files)}")

        for f in files:
            print(("   " + f.name).ljust(50) + " : " + str(round(f.size / 1024, 2)) + " KB")
            #   --- - GNSS was {suc}")

        print("")
        # if len(files) == 0:
        #     print("   " + f" GNSS was {suc}")

print(j)
