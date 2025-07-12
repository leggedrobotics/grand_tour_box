from pathlib import Path
from box_auto.utils import get_uuid_mapping, read_sheet_data
import rosbag
import subprocess
import kleinkram
import os

raise ValueError(
    "This script is not needed anymore if you are working with pub or release data. Everything should be correct on kleinkram. If applied again everything will be swapped again."
)

uuid_mappings = get_uuid_mapping()
# Define the spreadsheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
# Read the data and print the list
topic_data, mission_data = read_sheet_data(SPREADSHEET_ID)

for name, data in uuid_mappings.items():
    # Create a dictionary with bag_name_out as the key and a list of missions as the value

    if name != "2024-10-01-11-47-44" and name != "2024-10-01-12-00-49":
        continue

    tmp_folder = Path("/data") / (name + "_release_with_delete")
    tmp_folder.mkdir(parents=True, exist_ok=True)

    out_dir_bags = tmp_folder / "publish_bags"
    out_dir_bags.mkdir(exist_ok=True, parents=True)

    files = kleinkram.list_files(mission_ids=[data["uuid_pub"]], file_names=["*hdr_front*", "*hdr_right*"])

    kleinkram.download(
        mission_ids=[data["uuid_pub"]],
        file_names=["*hdr_front*", "*hdr_right*"],
        dest=tmp_folder,
        verbose=True,
        overwrite=True,
    )

    # Do a quick sorting operation
    bags_to_upload = []

    kleinkram.delete_files(file_ids=[f.id for f in files])

    for f in files:
        fname = f.name
        bag_path_in = tmp_folder / fname
        bag_path_out = str(out_dir_bags / fname)

        if "hdr_front" in bag_path_out:
            bag_path_out = bag_path_out.replace("hdr_front", "hdr_right")
        elif "hdr_right" in bag_path_out:
            bag_path_out = bag_path_out.replace("hdr_right", "hdr_front")

        with rosbag.Bag(bag_path_out, "w", compression="lz4") as bag_out:
            with rosbag.Bag(bag_path_in, "r") as bag_in:
                for topic, msg, t in bag_in.read_messages():
                    if "hdr_front" in topic:
                        topic = topic.replace("hdr_front", "hdr_right")
                        try:
                            msg.header.frame_id = msg.header.frame_id.replace("hdr_front", "hdr_right")
                        except AttributeError:
                            pass
                    elif "hdr_right" in topic:
                        topic = topic.replace("hdr_right", "hdr_front")
                        try:
                            msg.header.frame_id = msg.header.frame_id.replace("hdr_right", "hdr_front")
                        except AttributeError:
                            pass

                    bag_out.write(topic, msg, t)

        print(f)
        print("Write to output ", bag_path_out)

        subprocess.run(["rosbag", "reindex", str(bag_path_out)])
        tmp_path = str(bag_path_out).replace(".bag", ".orig.bag")
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

        res = kleinkram.upload(
            mission_name="pub_" + name, project_name="GrandTour", files=[bag_path_out], create=True, verbose=True
        )
