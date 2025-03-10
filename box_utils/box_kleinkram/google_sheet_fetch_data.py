import gspread
import pathlib
from box_auto.utils import BOX_AUTO_DIR


def read_sheet_data(spreadsheet_id, sheet_name):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Open the Google Sheet
    sheet = gc.open_by_key(spreadsheet_id)
    worksheet = sheet.worksheet(sheet_name)

    # Read data from A1 to G500
    data = worksheet.get_all_values("A1:G500")

    # Define the keys for the dictionary
    keys = ["topic_name_orig", "type", "frame_id_orig", "convert", "topic_name_out", "frame_id_out", "bag_name_out"]

    # Store the data in a list of dictionaries
    data_list = []
    for row in data:
        data_list.append({str(k): str(v) for k, v in zip(keys, row)})

    current_bag_name = None
    data_list_raw = []
    for _data in data_list:
        if "Bag: " in _data["topic_name_orig"]:
            current_bag_name = _data["topic_name_orig"].replace("Bag: 2024-11-11-12-42-47_", "")
        if _data["topic_name_orig"] != "":
            if _data["topic_name_orig"][0] == "/" and _data["convert"] in ["Yes", "ROSBAG"]:
                _data["bag_name_orig"] = current_bag_name
                data_list_raw.append(_data)

    return data_list_raw


# Define the spreadsheet ID and sheet name
SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
SHEET_NAME = "topic_overview"

# Read the data and print the list
data_list_raw = read_sheet_data(SPREADSHEET_ID, SHEET_NAME)

from collections import defaultdict

# Create a dictionary with bag_name_out as the key and a list of missions as the value
data_dict_by_bag_name = defaultdict(list)
for entry in data_list_raw:
    data_dict_by_bag_name[entry["bag_name_out"]].append(entry)

# Do a quick sorting operation
for k, v in data_dict_by_bag_name.items():
    print(len(v))
