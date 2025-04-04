import kleinkram
from box_auto.utils import get_uuid_mapping

for k, v in get_uuid_mapping().items():

    uuid_pub = v["uuid"]
    # print(uuid_pub)
    files = kleinkram.list_files(mission_ids=[uuid_pub])
    total_hits_pet_mission = 0
    for tag in ["_nuc_hesai.bag"]:
        for file in files:
            if tag in file.name:
                # Larger then 500 MB
                if file.size > 500000000:
                    total_hits_pet_mission += 1

    if total_hits_pet_mission == 1:
        print("available")
    else:
        print(k, " failed")

    # 2024-11-02-17-43-10  failed
