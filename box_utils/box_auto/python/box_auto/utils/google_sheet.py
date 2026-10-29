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

    topic_data = parse_topic_rows(worksheet.get_all_values("A:G"))
    worksheet = sheet.worksheet(sheet_name_mission)
    mission_data = parse_mission_rows(worksheet.get_all_values("A:AD"))
    return topic_data, mission_data


def parse_topic_rows(rows):
    """Pad sparse spreadsheet rows; never silently truncate a late mission."""
    keys = ["topic_name_orig", "type", "frame_id_orig", "convert", "topic_name_out", "frame_id_out", "bag_name_out"]
    current_bag_name = None
    result = []
    for row in rows:
        values = list(row) + [""] * max(0, len(keys) - len(row))
        item = dict(zip(keys, map(str, values)))
        name = item["topic_name_orig"].strip()
        if name.startswith("Bag: "):
            current_bag_name = name[len("Bag: ") :]
            prefix = "2024-11-11-12-42-47_"
            if current_bag_name.startswith(prefix):
                current_bag_name = current_bag_name[len(prefix) :]
        if name.startswith("/") and item["convert"] in ("Yes", "ROSBAG"):
            if not current_bag_name:
                raise ValueError("Converted topic has no preceding bag: " + name)
            item["bag_name_orig"] = current_bag_name
            result.append(item)
    return result


def parse_mission_rows(rows):
    if not rows:
        raise ValueError("Mission worksheet is empty")
    keys = [str(k).strip() for k in rows[0][1:]]
    populated = [k for k in keys if k]
    if len(populated) != len(set(populated)):
        raise ValueError("Mission worksheet has duplicate headers")
    result = {}
    for row in rows[1:]:
        if not row or not any(str(v).strip() for v in row):
            continue
        mission = str(row[0]).strip()
        if not mission or mission in result:
            raise ValueError("Missing or duplicate mission identifier: " + mission)
        values = list(row[1:]) + [""] * len(keys)
        result[mission] = {key: str(value) for key, value in zip(keys, values) if key}
    return result
