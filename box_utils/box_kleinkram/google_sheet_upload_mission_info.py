import gspread
import os
from box_auto.utils import get_bag, BOX_AUTO_DIR, MISSION_DATA
import rosbag
import pathlib
from pathlib import Path
import yaml

UPLOAD = "all"


def get_bag_info(bag_list):
    """
    Extract topic, message type, and frame_id information from a ROS bag file.

    Args:
        bag_path: Path to the ROS bag file

    Returns:
        Dictionary with bag name as key and list of (topic, message_type, frame_id) tuples as value
    """
    bag_info = {}
    for bag_path in bag_list:
        bag_name = os.path.basename(bag_path)
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                # Get topic info from bag
                topic_info = bag.get_type_and_topic_info()
                topics_and_types_and_frame_id = []

                for topic_name, topic_data in topic_info.topics.items():
                    message_type = topic_data.msg_type
                    frame_id = None

                    # Read the first message from this topic to get frame_id
                    for _, msg, _ in bag.read_messages(topics=[topic_name]):
                        try:
                            # Try to get frame_id from header
                            if hasattr(msg, "header"):
                                frame_id = msg.header.frame_id
                            # For TF messages, which don't have a standard header
                            elif message_type in ["tf2_msgs/TFMessage", "tf/tfMessage"]:
                                if msg.transforms:
                                    frame_id = msg.transforms[0].header.frame_id
                        except AttributeError:
                            # If we can't find frame_id, keep it as None
                            pass
                        break
                        print(frame_id)
                    topics_and_types_and_frame_id.append((topic_name, message_type, frame_id))

                bag_info[bag_name] = topics_and_types_and_frame_id

        except Exception as e:
            print(f"Error processing bag {bag_path}: {str(e)}")

    return bag_info


def upload_to_sheets(bag_info, sheet_name, spreadsheet_id):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Google Sheets setup

    sheet = gc.open_by_key(spreadsheet_id)
    print(dir(sheet))
    sheet = getattr(sheet, sheet_name)  # Open first sheet
    sheet.clear()

    # Write data
    cells_to_update = []
    row = 1

    for bag_name, topics in bag_info.items():
        # Prepare bag name update
        cells_to_update.append({"range": f"A{row}", "values": [[f"Bag: {bag_name}"]]})
        row += 1

        # Prepare headers update
        cells_to_update.append({"range": f"A{row}:C{row}", "values": [["Topic Name", "Message Type", "Frame ID"]]})
        row += 1

        # Prepare topic information updates
        for topic, msg_type, frame_id in topics:
            print(frame_id)
            cells_to_update.append({"range": f"A{row}:C{row}", "values": [[topic, msg_type, frame_id]]})
            row += 1

        row += 1  # Add space between bag sections

    # Perform batch update
    sheet.batch_update(cells_to_update)


if UPLOAD == "reference_data":
    yaml_file = str(pathlib.Path(BOX_AUTO_DIR) / "cfg/health_check_reference_raw_data.yaml")
    with open(yaml_file, "r") as f:
        reference_data = yaml.safe_load(f)
    bag_list = [get_bag("*" + key) for key in reference_data.keys()]
elif UPLOAD == "all":
    bag_list = [str(s) for s in Path(MISSION_DATA).glob("*.bag")]


bag_list.sort()
bag_info = get_bag_info(bag_list)

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"  # From your provided sheet link
SHEET_NAME = "sheet1"
# upload_to_sheets(bag_info, SHEET_NAME, SPREADSHEET_ID)
# import numpy as np
# msgs = []
# for k,v in bag_info.items():
#     for _v in v:
#         msgs.append(_v[1])

# print( np.unique( np.array(msgs)) )
