import json
import yaml
import gspread
import pathlib
from box_auto.utils import BOX_AUTO_DIR

youtube = False
add_tags_from_google_sheet = True

if youtube:
    # Paths to the files
    json_file_path = "/home/jonfrey/git/anymal-grand-tour-webpage/frontend/public/data/missions.json"
    yaml_file_path = "/home/jonfrey/Videos/video_ids_fist_upload_run.yaml"
    output_file_path = "/home/jonfrey/git/anymal-grand-tour-webpage/frontend/public/data/missions_with_links.json"

    # Load the JSON data
    with open(json_file_path, "r") as json_file:
        missions = json.load(json_file)

    # Load the YAML data
    with open(yaml_file_path, "r") as yaml_file:
        video_ids = yaml.safe_load(yaml_file)

    # Add youtube_link to missions if date_time matches
    for mission in missions:
        date_time = mission.get("date_time")
        if date_time and date_time in video_ids:
            mission["youtube_link"] = video_ids[date_time]

    # Save the updated missions to a new JSON file
    with open(output_file_path, "w") as output_file:
        json.dump(missions, output_file, indent=4)

    print(f"Updated missions saved to {output_file_path}")

if add_tags_from_google_sheet:
    gc = gspread.service_account(
        filename=pathlib.Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json"
    )

    spreadsheet_id = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
    sheet_name_topic = "mission_tags"
    # Open the Google Sheet
    sheet = gc.open_by_key(spreadsheet_id)
    worksheet = sheet.worksheet(sheet_name_topic)
    data = worksheet.get_all_values("A1:AH71")

    data_store = {}
    if data:
        headers = data[0]  # First row contains the column names (tags)
        for row in data[1:]:  # Iterate over the remaining rows
            mission_name = row[0]  # First column is the mission name
            if mission_name:  # Skip empty rows
                tags = {headers[i]: row[i] for i in range(1, len(headers)) if row[i]}  # Map tags to values
                data_store[mission_name] = tags

    print(data_store)

    # Paths to the files
    json_file_path = "/home/jonfrey/git/anymal-grand-tour-webpage/frontend/public/data/missions.json"
    output_file_path = "/home/jonfrey/git/anymal-grand-tour-webpage/frontend/public/data/missions_with_labels.json"

    # Load the JSON data
    with open(json_file_path, "r") as json_file:
        missions = json.load(json_file)

    # Add tags from the Google Sheet to the missions
    for mission in missions:
        mission_name = mission.get("date_time")
        if mission_name in data_store:
            # Add tags from the Google Sheet to the "labels" field
            sheet_tags = list(data_store[mission_name].keys())

            mission["labels"] = mission.get("labels", []) + sheet_tags

    # Save the updated missions to a new JSON file
    with open(output_file_path, "w") as output_file:
        json.dump(missions, output_file, indent=4)

    print(f"Updated missions saved to {output_file_path}")
