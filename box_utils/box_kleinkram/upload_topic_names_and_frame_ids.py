import gspread
import os
from box_auto.utils import get_bag, BOX_AUTO_DIR
import rosbag
import pathlib
import yaml


def get_bag_info(bag_list):
    """
    Extract topic and message type information from a ROS bag file.

    Args:
        bag_path: Path to the ROS bag file

    Returns:
        Dictionary with bag name as key and list of (topic, message_type) tuples as value
    """
    bag_info = {}
    for bag_path in bag_list:
        bag_name = os.path.basename(bag_path)
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                # Get topic info from bag
                topic_info = bag.get_type_and_topic_info()
                topics_and_types = []

                for topic_name, topic_data in topic_info.topics.items():
                    message_type = topic_data.msg_type
                    topics_and_types.append((topic_name, message_type))

                bag_info[bag_name] = topics_and_types

        except Exception as e:
            print(f"Error processing bag {bag_path}: {str(e)}")

    return bag_info


def upload_to_sheets(bag_info, sheet_name, spreadsheet_id):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Google Sheets setup

    sheet = gc.open_by_key(spreadsheet_id)
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
        cells_to_update.append({"range": f"A{row}:B{row}", "values": [["Topic Name", "Message Type"]]})
        row += 1

        # Prepare topic information updates
        for topic, msg_type in topics:
            cells_to_update.append({"range": f"A{row}:B{row}", "values": [[topic, msg_type]]})
            row += 1

        row += 1  # Add space between bag sections

    # Perform batch update
    sheet.batch_update(cells_to_update)


yaml_file = str(pathlib.Path(BOX_AUTO_DIR) / "cfg/health_check_reference_raw_data.yaml")
with open(yaml_file, "r") as f:
    reference_data = yaml.safe_load(f)


bag_list = [get_bag("*" + key) for key in reference_data.keys()]

bag_list.sort()

bag_info = get_bag_info(bag_list)

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"  # From your provided sheet link
SHEET_NAME = "sheet1"
upload_to_sheets(bag_info, SHEET_NAME, SPREADSHEET_ID)
