import gspread
import pathlib
from box_auto.utils import BOX_AUTO_DIR


def read_sheet_data(spreadsheet_id, sheet_name_topic="topic_overview", sheet_name_mission="mission_overview"):
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Open the Google Sheet
    sheet = gc.open_by_key(spreadsheet_id)
    worksheet = sheet.worksheet(sheet_name_topic)

    # Read data from A1 to G500
    data = worksheet.get_all_values("A1:G500")

    # Define the keys for the dictionary
    keys = ["topic_name_orig", "type", "frame_id_orig", "convert", "topic_name_out", "frame_id_out", "bag_name_out"]

    # Store the data in a list of dictionaries
    data_list = []
    for row in data:
        data_list.append({str(k): str(v) for k, v in zip(keys, row)})

    current_bag_name = None
    topic_data = []
    for _data in data_list:
        if "Bag: " in _data["topic_name_orig"]:
            current_bag_name = _data["topic_name_orig"].replace("Bag: 2024-11-11-12-42-47_", "")
        if _data["topic_name_orig"] != "":
            if _data["topic_name_orig"][0] == "/" and _data["convert"] in ["Yes", "ROSBAG"]:
                _data["bag_name_orig"] = current_bag_name
                topic_data.append(_data)

    worksheet = sheet.worksheet(sheet_name_mission)
    data = worksheet.get_all_values("A1:Y71")

    data_list = []

    keys = data[0][1:]
    mission_data = {}
    for row in data[1:]:
        mission_data[row[0]] = {str(k): str(v) for k, v in zip(keys, row[1:])}

    return topic_data, mission_data
