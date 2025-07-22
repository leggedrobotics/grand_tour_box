import gspread
import pathlib
from box_auto.utils import BOX_AUTO_DIR


def read_sheet_data(spreadsheet_id, sheet_name_topic="topic_overview", sheet_name_mission="mission_overview"):
    """
    Reads and processes data from a Google Sheet containing topic and mission information.

    Args:
        spreadsheet_id (str): The ID of the Google Spreadsheet.
        sheet_name_topic (str): Name of the worksheet containing topic data.
        sheet_name_mission (str): Name of the worksheet containing mission data.

    Returns:
        tuple: (topic_data, mission_data) - Processed topic and mission information.
    """
    # Authenticate using service account credentials
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    # Open the Google Sheet using the provided spreadsheet ID
    sheet = gc.open_by_key(spreadsheet_id)
    worksheet = sheet.worksheet(sheet_name_topic)

    # Read data from A1 to G545 from the topic worksheet
    data = worksheet.get_all_values("A1:G545")

    # Define column headers for the topic data
    keys = ["topic_name_orig", "type", "frame_id_orig", "convert", "topic_name_out", "frame_id_out", "bag_name_out"]

    # Convert raw data to list of dictionaries with proper keys
    data_list = []
    for row in data:
        data_list.append({str(k): str(v) for k, v in zip(keys, row)})

    # Process topic data to extract relevant information
    current_bag_name = None
    topic_data = []
    for _data in data_list:
        # Extract bag name when a bag entry is found
        if "Bag: " in _data["topic_name_orig"]:
            current_bag_name = _data["topic_name_orig"].replace("Bag: 2024-11-11-12-42-47_", "")
        if _data["topic_name_orig"] != "":
            # Only include topics that start with '/' and are marked for conversion
            if _data["topic_name_orig"][0] == "/" and _data["convert"] in ["Yes", "ROSBAG"]:
                _data["bag_name_orig"] = current_bag_name
                topic_data.append(_data)

    # Read data from the mission worksheet
    worksheet = sheet.worksheet(sheet_name_mission)
    data = worksheet.get_all_values("A1:Z72")

    data_list = []

    # Process mission data using the first row as keys
    keys = data[0][1:]
    mission_data = {}
    for row in data[1:]:
        mission_data[row[0]] = {str(k): str(v) for k, v in zip(keys, row[1:])}

    return topic_data, mission_data
