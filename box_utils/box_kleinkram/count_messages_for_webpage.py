import kleinkram
from box_auto.utils import get_uuid_mapping, read_sheet_data
from pathlib import Path
import rosbag

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

Path("/data/tmp_dir").mkdir(parents=True, exist_ok=True)
for j, (name, data) in enumerate(get_uuid_mapping().items()):
    if MISSION_DATA[name]["GOOD_MISSION"] == "TRUE":
        m = data["uuid_release"]
        print(f"Downloading mission {m}")
        kleinkram.download(mission_ids=[m], file_names=["*hdr_front.bag"], dest="/data/tmp_dir", verbose=True)
# TODO open rosbag and count messages for /boxi/hdr/front/image/compressed
# Print summary per bag and across all bags

total_messages = 0
for file in Path("/data/tmp_dir").glob("*hdr_front.bag"):
    with rosbag.Bag(file) as bag:
        msg_count = bag.get_message_count()
        total_messages += msg_count
        print(f"{file.name}: {msg_count} messages")

print(f"Total messages across all bags: {total_messages}")
